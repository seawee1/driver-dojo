import sys
import string
import os
import glob
import time
import traci
import sumolib.net
import logging
import subprocess
from os.path import join as pjoin
import xml.etree.cElementTree as ET
from sumolib import checkBinary

import driver_dojo.common.runtime_vars as runtime_vars


class SUMOEngine:
    def __init__(self):
        self.running = False
        self.additional_paths = ""

    def init_engine(self):
        # Have a look at https://manned.org/sumo-gui for more commandline arguments
        # Might be interesting:
        # --device.rerouting.probability
        # --threads
        # --ignore-junction-blocker
        #   Decrease --lateral-resolution for better performance?
        sumoCmd = [
            "-c",
            runtime_vars.sumocfg_path,
            "-W",
            "true",
            "--no-step-log",
            "true",
            "--step-length",
            str(runtime_vars.config["simulation"]["dt"]),
            "--lateral-resolution",
            str(
                runtime_vars.config["simulation"]["lateral_resolution"]
            ),  # Need to be set smooth lane changes of SUMO-controlled vehicle. Otherwise, cars teleport.
            "--collision.action",
            "none",  # Collisions are dealt with by driver_dojo.common.collision_detection
            "--scale",
            str(runtime_vars.config["simulation"]["demand_scale"]),
            "--xml-validation",
            "never",
            "--time-to-impatience",
            str(runtime_vars.config["simulation"]["time_to_impatience"]),
            "--gui-settings-file",
            str(runtime_vars.config["simulation"]["gui_xml_path"]),
            "--seed",
            str(runtime_vars.traffic_seed),
            "--default.carfollowmodel",
            runtime_vars.config["simulation"]["car_following_model"].value,
            "--lanechange.overtake-right",
            "false",
        ]
        sumoCmd += (
            ["--tls.all-off", "true"]
            if runtime_vars.config["simulation"]["tls_off"]
            else []
        )
        sumoCmd += (
            ["--additional-files", runtime_vars.config["simulation"]["add_path"]]
            if runtime_vars.config["simulation"]["add_path"] is not None
            else []
        )
        sumoCmd += (
            [
                "--time-to-teleport",
                str(runtime_vars.config["simulation"]["time_to_teleport"]),
            ]
            if runtime_vars.config["simulation"]["time_to_teleport"] is not None
            else []
        )
        if runtime_vars.config["simulation"]["render"]:
            sumoCmd += ["--delay", str(1000)]
            # Use GUI
            sumoBinary = (
                checkBinary("sumo-gui.exe")
                if os.name == "nt"
                else checkBinary("sumo-gui")
            )
            sumoCmd.insert(2, "-S")
        else:
            # Do not use GUI
            sumoBinary = (
                checkBinary("sumo.exe") if os.name == "nt" else checkBinary("sumo")
            )

        # TODO: Include again. However, has no effect with EIDM!
        # driverstate_config = runtime_vars.config['driver_state']
        # if driverstate_config['enabled']:
        #     for k, v in driverstate_config.items():
        #         driverstate_config[k] = str(v)
        #     sumoCmd += [
        #         "--device.driverstate.probability", driverstate_config["probability"],
        #         "--device.driverstate.deterministic", "true",
        #         "--device.driverstate.initialAwareness", driverstate_config["initialAwareness"],
        #         "--device.driverstate.errorTimeScaleCoefficient", driverstate_config["errorTimeScaleCoefficient"],
        #         "--device.driverstate.errorNoiseIntensityCoefficient", driverstate_config["errorNoiseIntensityCoefficient"],
        #         "--device.driverstate.speedDifferenceErrorCoefficient", driverstate_config["speedDifferenceErrorCoefficient"],
        #         "--device.driverstate.headwayErrorCoefficient", driverstate_config["headwayErrorCoefficient"],
        #         "--device.driverstate.speedDifferenceChangePerceptionThreshold", driverstate_config["speedDifferenceChangePerceptionThreshold"],
        #         "--device.driverstate.headwayChangePerceptionThreshold", driverstate_config["headwayChangePerceptionThreshold"],
        #         "--device.driverstate.minAwareness", driverstate_config["minAwareness"],
        #         "--device.driverstate.maximalReactionTime", driverstate_config["maximalReactionTime"],
        #     ]

        if not self.running:
            # Logging
            args_str = " ".join(sumoCmd)
            logging.info(f"Calling 'sumo {args_str}'")

            # Start a SUMO process
            traci.start([sumoBinary] + sumoCmd, label=runtime_vars.sumo_label)
            runtime_vars.traci = traci.getConnection(runtime_vars.sumo_label)
            self.running = True
            logging.info(
                f"Initialized SUMO instance with label {runtime_vars.sumo_label}..."
            )
        else:
            # Resets simulator to initial state
            runtime_vars.traci.load(sumoCmd)

        # self.generator_thread = Thread(target=self.generate, args=())
        # self.generator_thread.start()
        return runtime_vars.traci

    def close_engine(self):
        # Closes the simulator
        if self.running:
            runtime_vars.traci.close()
            traci.switch(runtime_vars.sumo_label)
            traci.close()
            runtime_vars.traci = None
            self.running = False
            logging.info(
                f"Closed SUMO instance with label {runtime_vars.sumo_label}..."
            )
        sys.stdout.flush()

    def simulationStep(self):
        # Add whatever you want to do before/after simulationStep
        runtime_vars.traci.simulationStep()
