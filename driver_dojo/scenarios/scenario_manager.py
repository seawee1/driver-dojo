from threading import Thread, Semaphore
from queue import Queue
from gym.utils import seeding
from os.path import join as pjoin

from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.intersection2 import IntersectionScenario  # TODO: Delete old 'intersection.py', rename new 'intersection2.py' scenario python file


def create_scenario(q, lock, scenario_cls, scenario_args):  # Used for multi-threaded scenario generation
    scenario = scenario_cls(*scenario_args)
    if scenario.is_valid:
        with lock:
            q.put(scenario)

class ScenarioManager:
    def __init__(self, config):
        self._config = config
        self._scenario_config = config.scenario

        self._num_networks = self._scenario_config.num_maps
        self._num_traffics = self._scenario_config.num_traffic
        self._num_tasks = self._scenario_config.num_tasks
        self._test_seeding = self._scenario_config.test_seeding

        self._network_seed_sampler, _ = seeding.np_random(self._config.simulation.seed)
        self._traffic_seed_sampler, _ = seeding.np_random(self._config.simulation.seed)
        self._task_seed_sampler, _ = seeding.np_random(self._config.simulation.seed)

        self._scenario_queue = Queue()
        self._threads = []
        self._lock = Semaphore()
        self.step(no_ret=True)  # Already start generation ahead of time, during environment init

    def _sample_scenario_args(self):
        offset = self._scenario_config.seed_offset  # Draw scenario-specific seeds
        net_seed = self._network_seed_sampler.integers(offset, self._num_networks + offset)
        traffic_seed = self._traffic_seed_sampler.integers(offset, self._num_traffics + offset)
        task_seed = self._task_seed_sampler.integers(offset, self._num_tasks + offset)
        if self._scenario_config.test_seeding:  # Allows for easy evaluating the agent on guaranteed disjunct test scenario set
            max_int = np.iinfo(np.int_).max
            net_seed = self._network_seed_sampler.integers(self._num_networks + offset, max_int) if self._num_networks + offset < max_int else net_seed
            traffic_seed = self._traffic_seed_sampler.integers(self._num_traffics + offset, max_int) if self._num_traffics + offset < max_int else traffic_seed
            task_seed = self._task_seed_sampler.integers(self._num_tasks + offset, max_int) if self._num_tasks + offset < max_int else task_seed

        return self._config.simulation.work_path, self._scenario_config, self._config.simulation, net_seed, traffic_seed, task_seed

    def step(self, no_ret=False) -> BasicScenario:
        if self._config.scenario.generation_threading:
            # Cleanup threads
            for t in self._threads:  # Not sure if we need this though
                if not t.is_alive():
                    t.join()
            self._threads = [x for x in self._threads if x.is_alive()]

            # Start new threads
            self._start_new_threads()

            if not no_ret:
                next_scenario = self._scenario_queue.get(block=True)
                return next_scenario
        else:
            while True:
                scenario_args = self._sample_scenario_args()
                scenario_cls = IntersectionScenario  # TODO: Mapping to
                scenario = scenario_cls(*scenario_args)
                if scenario.is_valid:
                    if no_ret:
                        return
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
