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

from driver_dojo.carla_integration.bridge_helper import BridgeHelper


class SUMOEngine:
    def __init__(self):
        self.running = False
        self.carla_simulation = None
        self.carla_actors = {}
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
            "--xml-validation", "never",
            "--xml-validation.net", "always"
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


        if runtime_vars.config.simulation.co_sim_to_carla:
            if self.carla_simulation is None:
                # Start a Carla server
                carla_executable = runtime_vars.config.simulation.carla_path
                carla_start_cmd = [
                    carla_executable,
                    '-carla-rpc-port=3000',
                    #'-carla-server',
                    #-benchmark -fps=<framerate>'
                ]
                p = subprocess.Popen(
                   carla_start_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE
                )
                import time
                time.sleep(10)

                # Create the CarlaSimulation object
                from driver_dojo.carla_integration.carla_simulation import CarlaSimulation
                self.carla_simulation = CarlaSimulation('127.0.0.1', 3000, 0.2)


                BridgeHelper.blueprint_library = self.carla_simulation.world.get_blueprint_library()

                # Configuring carla simulation in sync mode.
                settings = self.carla_simulation.world.get_settings()
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = runtime_vars.config.simulation.dt
                self.carla_simulation.world.apply_settings(settings)

            self.carla_actors = {}
            # We have to grab this again and again
            BridgeHelper.offset = runtime_vars.net.getLocationOffset()
            # Load map
            xodr_path = os.path.join(runtime_vars.temp_path, runtime_vars.sumo_label + '.xodr')
            if os.path.exists(xodr_path):
                with open(xodr_path, encoding='utf-8') as od_file:
                    try:
                        data = od_file.read()
                    except OSError:
                        print('file could not be readed.')
                        sys.exit()
                vertex_distance = 2.0  # in meters
                max_road_length = 500.0 # in meters
                wall_height = 0.0      # in meters
                extra_width = 0.0      # in meters
                import carla
                world = self.carla_simulation.client.generate_opendrive_world(
                    data, carla.OpendriveGenerationParameters(
                        vertex_distance=vertex_distance,
                        max_road_length=max_road_length,
                        wall_height=wall_height,
                        additional_width=extra_width,
                        smooth_junctions=False,
                        enable_mesh_visibility=True))

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
        if runtime_vars.config.simulation.co_sim_to_carla:
            spawned_actors = set(runtime_vars.traci.simulation.getDepartedIDList())
            destroyed_actors = set(runtime_vars.traci.simulation.getArrivedIDList())

            # Spawn new
            sumo_spawned_actors = spawned_actors - set(self.carla_actors.values())
            for sumo_actor_id in sumo_spawned_actors:
                sumo_actor = self.get_actor(sumo_actor_id)

                carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, True)
                if carla_blueprint is not None:
                    carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                       sumo_actor.extent)

                    carla_actor_id = self.carla_simulation.spawn_actor(carla_blueprint, carla_transform)
                    from driver_dojo.carla_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
                    if carla_actor_id != INVALID_ACTOR_ID:
                        self.carla_actors[sumo_actor_id] = carla_actor_id
                else:
                    print("No blueprint")

            # Destroy old
            for sumo_actor_id in destroyed_actors:
                if sumo_actor_id in self.carla_actors:
                    self.carla_simulation.destroy_actor(self.carla_actors.pop(sumo_actor_id))

            # Update current

            # Updating sumo actors in carla.
            for sumo_actor_id in self.carla_actors:
                carla_actor_id = self.carla_actors[sumo_actor_id]

                sumo_actor = self.get_actor(sumo_actor_id)
                carla_actor = self.carla_simulation.get_actor(carla_actor_id)

                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)
                #if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(), sumo_actor.signals)
                #else:
                #    carla_lights = None

                self.carla_simulation.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)
            self.carla_simulation.tick()

    @staticmethod
    def get_actor(actor_id):
        """
        Accessor for sumo actor.
        """
        results = traci.vehicle.getSubscriptionResults(actor_id)

        type_id = traci.vehicle.getTypeID(actor_id)
        from driver_dojo.carla_integration.sumo_simulation import SumoActorClass
        vclass = SumoActorClass("passenger")
        color = traci.vehicle.getColor(actor_id)

        length = traci.vehicle.getLength(actor_id)
        width = traci.vehicle.getWidth(actor_id)
        height = traci.vehicle.getHeight(actor_id)

        location = list(traci.vehicle.getPosition3D(actor_id))
        rotation = [traci.vehicle.getSlope(actor_id), traci.vehicle.getAngle(actor_id), 0.0]
        import carla
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = traci.vehicle.getSignals(actor_id)
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)
        from driver_dojo.carla_integration.sumo_simulation import SumoActor
        return SumoActor(type_id, vclass, transform, signals, extent, color)
