import os
from typing import List
import traci
import logging
from sumolib import checkBinary
from os.path import join as pjoin

from driver_dojo.scenarios.basic_scenarios import BasicScenario


class SUMOEngine:
    def __init__(self, config, sumo_label):
        self._running = False
        self.config = config
        self.sumo_label = sumo_label
        # self._carla_simulation = None
        # self._carla_actors = {}
        # self._carla_sensors = []
        # self._carla_process = None
        # self._additional_paths = ""
        self.traci = None

    def _get_command(self, scenario: BasicScenario) -> List[str]:
        # Have a look at https://manned.org/sumo-gui for more commandline arguments
        # Might be interesting:
        # --device.rerouting.probability
        # --threads
        # --ignore-junction-blocker
        #   Decrease --lateral-resolution for better performance?
        sumo_cmd = [
            "-S",
            "-c",
            scenario.sumocfg_path,
            "-W",
            "true",
            #'-v',
            #'true',
            # "--xml-validation",
            # "never"
            "--no-step-log",
            "true",
            "--step-length",
            str(self.config["simulation"]["dt"]),
            "--lateral-resolution",
            str(
                self.config["simulation"]["lateral_resolution"]
            ),  # Need to be set smooth lane changes of SUMO-controlled vehicle. Otherwise, cars teleport.
            "--collision.action",
            "none",  # Collisions are dealt with by driver_dojo.common.collision_detection
            "--scale",
            str(self.config["simulation"]["demand_scale"]),
            "--time-to-impatience",
            str(self.config["simulation"]["time_to_impatience"]),
            "--gui-settings-file",
            str(self.config["simulation"]["gui_xml_path"]),
            "--seed",
            str(scenario.traffic_seed),
            "--default.carfollowmodel",
            self.config["simulation"]["car_following_model"].value,
            "--lanechange.overtake-right",
            "false",
            "--xml-validation", "never",
            "--xml-validation.net", "always",
            '--log', pjoin(self.config.simulation.work_path, f'{self.sumo_label}_log.txt'),
            '--message-log', pjoin(self.config.simulation.work_path, f'{self.sumo_label}_msg-log.txt'),
            '--error-log', pjoin(self.config.simulation.work_path, f'{self.sumo_label}_error-log.txt')
        ]
        sumo_cmd += (
            ["--tls.all-off", "true"]
            if self.config["simulation"]["tls_off"]
            else []
        )
        sumo_cmd += (
            ["--additional-files", scenario.sumo_add_path]
            if scenario.sumo_add_path != ''
            else []
        )
        sumo_cmd += (
            [
                "--time-to-teleport",
                str(self.config["simulation"]["time_to_teleport"]),
            ]
            if self.config["simulation"]["time_to_teleport"] is not None
            else []
        )
        if self.config.rendering.sumo_gui:
            sumo_cmd += ["--delay", str(1000)]

        return sumo_cmd

    def reset(self, scenario):
        if self.config.rendering.sumo_gui:
            sumo_binary = (
                checkBinary("sumo-gui.exe")
                if os.name == "nt"
                else checkBinary("sumo-gui")
            )
        else:
            # Do not use GUI
            sumo_binary = (
                checkBinary("sumo.exe") if os.name == "nt" else checkBinary("sumo")
            )

        if not self._running:
            # Logging
            command_str = " ".join(self._get_command(scenario))
            logging.info(f"Calling sumo with sumo commandline arguments:")
            for arg in self._get_command(scenario):
                logging.info(arg)
            logging.info(f"Command string:")
            logging.info(command_str)

            # Start a SUMO process
            if os.getenv('LIBSUMO_AS_TRACI'):
                logging.info(f"Using libsumo instead of traci...")
                print(self._get_command(scenario))
                traci.simulation.start([sumo_binary] + self._get_command(scenario), label=self.sumo_label)
                self.traci = traci
            else:
                logging.info(f"Using standard traci...")
                traci.start([sumo_binary] + self._get_command(scenario), label=self.sumo_label)
                self.traci = traci.getConnection(self.sumo_label)

            self._running = True
            logging.info(
                f"Initialized SUMO instance with label {self.sumo_label}..."
            )
        else:
            # Resets simulator to initial state
            self.traci.load(self._get_command(scenario))
            logging.info(
                f"Reset SUMO instance with label {self.sumo_label}..."
            )
        return self.traci

    def close(self):
        # Closes the simulator
        if self._running:
            self.traci.close()
            self.traci = None
            self._running = False
            logging.info(
                f"Closed SUMO instance with label {self.sumo_label}..."
            )

    def simulationStep(self):
        try:
            self.traci.simulationStep()
        except traci.FatalTraCIError as e:
            logging.error(e)
            logging.info(
                f"An error occurred during simulation step of SUMO instance with label {self.sumo_label}..."
            )
            self.close()
            return True
        return False
