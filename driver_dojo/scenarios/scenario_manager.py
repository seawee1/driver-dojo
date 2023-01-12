from threading import Thread, Semaphore
from queue import Queue
from gym.utils import seeding
from os.path import join as pjoin
import numpy as np

from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.intersection2 import IntersectionScenario  # TODO: Delete old 'intersection.py', rename new 'intersection2.py' scenario python file


def create_scenario(q, lock, scenario_cls, scenario_args):  # Used for multi-threaded scenario generation
    scenario = scenario_cls(*scenario_args, lock)
    if scenario.task_realisable:
        q.put(scenario)


class ScenarioManager:
    def __init__(self, config):
        self._config = config
        self._scenario_config = config.scenario

        self._num_networks = self._scenario_config.num_maps
        self._num_traffics = self._scenario_config.num_traffic
        self._num_tasks = self._scenario_config.num_tasks
        self._test_seeding = self._scenario_config.seeding_mode

        self._network_seed_sampler = np.random.default_rng(self._config.simulation.seed)
        self._traffic_seed_sampler = np.random.default_rng(self._config.simulation.seed)
        self._task_seed_sampler = np.random.default_rng(self._config.simulation.seed)

        self._scenario_queue = None
        self._lock = None
        self._threads = []
        #self._lock = None
        if self._config.scenario.generation_threading:
            self.step(no_ret=True)  # Already start generation ahead of time, during environment init

    def _sample_scenario_args(self):
        max_int = 2147483647  # np.iinfo(np.int_).max # Numpy max_int bigger than allowed max_int for SUMO engine seeding
        offset = self._scenario_config.seed_offset  # Draw scenario-specific seeds
        net_seed = self._network_seed_sampler.integers(offset, self._num_networks + offset)
        traffic_seed = self._traffic_seed_sampler.integers(offset, self._num_traffics + offset)
        task_seed = self._task_seed_sampler.integers(offset, self._num_tasks + offset)
        assert self._scenario_config.seeding_mode in ['train', 'test_maps', 'test_traffic']
        if self._scenario_config.seeding_mode == 'test_maps':  # Allows for easy evaluating the agent on guaranteed disjunct test scenario set
            net_seed = self._network_seed_sampler.integers(self._num_networks + offset, max_int)
        elif self._scenario_config.seeding_mode == 'test_traffic':
            traffic_seed = self._traffic_seed_sampler.integers(self._num_traffics + offset, max_int)
            #task_seed = self._task_seed_sampler.integers(self._num_tasks + offset, max_int) if self._num_tasks + offset < max_int else task_seed

        return self._config.simulation.work_path, self._scenario_config, self._config.simulation, net_seed, traffic_seed, task_seed

    def step(self, no_ret=False) -> BasicScenario:
        if self._config.scenario.generation_threading:
            if self._scenario_queue is None:
                self._scenario_queue = Queue()
                self._lock = Semaphore()

            while self._scenario_queue.qsize() == 0:
                # Cleanup threads
                for i, t in enumerate(self._threads):  # Not sure if we need this though
                    if not t.is_alive():
                        t.join()
                self._threads = [x for x in self._threads if x.is_alive()]

                # Start new threads
                self._start_new_threads()
                import time
                time.sleep(2.0)

            if not no_ret:
                next_scenario = self._scenario_queue.get(block=True)
                return next_scenario
        else:
            task_not_real = False
            while not task_not_real:
                scenario_args = self._sample_scenario_args()
                scenario_cls = IntersectionScenario  # TODO: Mapping to
                scenario = scenario_cls(*scenario_args, None)
                task_not_real = scenario.task_realisable
            if not no_ret:
                return scenario


    def _start_new_threads(self):
        num_gen = self._scenario_config.generation_num_buffer - self._scenario_queue.qsize()  # How many left until buffer is full
        max_start = self._scenario_config.generation_num_threads - len(self._threads)  # How many more threads can we start
        num_start = min(num_gen, max_start)
        for _ in range(num_start):
            scenario_args = self._sample_scenario_args()  # TODO: Map Config.ScenarioConfig.name to 'scenario' class
            t = Thread(target=create_scenario, args=(self._scenario_queue, self._lock, IntersectionScenario, scenario_args))
            t.start()
            self._threads.append(t)
