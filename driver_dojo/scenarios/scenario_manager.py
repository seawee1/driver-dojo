from threading import Thread, Semaphore
from queue import Queue
from gym.utils import seeding
from os.path import join as pjoin

from driver_dojo.scenarios.basic_scenarios import BasicScenario
from driver_dojo.scenarios.intersection2 import IntersectionScenario


def create_scenario(q, lock, scenario_cls, scenario_args):
    scenario = scenario_cls(*scenario_args)
    with lock:
        q.put(scenario)


class ScenarioManager:
    def __init__(self, config):
        self._config = config

        self._network_seed_sampler, _ = seeding.np_random(self._config.simulation.seed)
        self._traffic_seed_sampler, _ = seeding.np_random(self._config.simulation.seed)
        self._num_networks = self._config.scenario.num_road_scenarios
        self._num_traffics = self._config.scenario.num_traffic_scenarios
        self._inverse_seeding = self._config.scenario.inverse_seeding

        self._scenario_queue = Queue()
        self._threads = []
        self._lock = Semaphore()
        self.step(no_ret=True)  # Already start generation ahead of time

    def _sample_params(self):
        offset = self._config.scenario.seed_offset  # Draw seeds
        net_seed = self._network_seed_sampler.integers(offset, self._num_networks + offset)
        traffic_seed = self._traffic_seed_sampler.integers(offset, self._num_traffics + offset)
        if self._config.scenario.inverse_seeding:  # Used for easy testing on out-of-distribution data
            net_seed = self._network_seed_sampler.integers(self._num_networks + offset, 2147483647) if self._num_networks + offset < 2147483647 else net_seed
            traffic_seed = self._traffic_seed_sampler.integers(self._num_traffics + offset, 2147483647) if self._num_traffics + offset < 2147483647 else traffic_seed

        def get_file_path(seed, ext):
            base_path = self._config.simulation.work_path
            return pjoin(base_path, f'{self._config.scenario.name}_{seed}.{ext}')

        sumocfg_path = get_file_path(net_seed, 'sumocfg')
        sumo_net_path = get_file_path(net_seed, 'net.xml')
        xodr_path = get_file_path(net_seed, 'xodr')
        sumo_vType_path = get_file_path(f'{net_seed}_vTypes', 'add.xml')
        sumo_add_path = self._config.scenario.add_path  # These are only relevant if given from the outside
        sumo_rou_path = self._config.scenario.route_path
        scenario_name = f'{self._config.scenario.name}'
        return self._config, net_seed, traffic_seed, sumocfg_path, sumo_net_path, sumo_rou_path, sumo_add_path, sumo_vType_path, xodr_path, scenario_name

    def step(self, no_ret=False) -> BasicScenario:
        self._threads = [x for x in self._threads if x.is_alive()]
        num_gen = self._config.scenario.generation_num_buffer - self._scenario_queue.qsize()  # How many left until buffer is full
        max_start = self._config.scenario.generation_num_threads - len(self._threads)  # How many more threads can we start
        num_start = min(num_gen, max_start)

        for i in range(num_start):
            scenario_args = self._sample_params()  # TODO: switch-case between scenario type names
            t = Thread(target=create_scenario, args=(self._scenario_queue, self._lock, IntersectionScenario, scenario_args))
            t.start()
            self._threads.append(t)

        if not no_ret:
            next_scenario = self._scenario_queue.get(block=True)
            return next_scenario
