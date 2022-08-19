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
        self.carla_simulation = None
        self.carla_actors = {}
        self.carla_sensors = []
        self.carla_process = None
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
            if os.getenv('LIBSUMO_AS_TRACI'):
                traci.simulation.start([sumoBinary] + sumoCmd, label=runtime_vars.sumo_label)
                runtime_vars.traci = traci
            else:
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
            from driver_dojo.carla_integration.bridge_helper import BridgeHelper
            import carla
            if self.carla_simulation is None:
                import random
                import psutil
                def is_used(port):
                    """Checks whether or not a port is used"""
                    return port in [conn.laddr.port for conn in psutil.net_connections()]

                server_port = random.randint(15000, 32000)
                while is_used(server_port) or is_used(server_port+1): server_port += 2

                # Start a Carla server
                carla_executable = runtime_vars.config.simulation.carla_path
                if runtime_vars.config.simulation.render:
                    carla_start_cmd = [
                        carla_executable,
                        "-windowed",
                        "-ResX={}".format(600),
                        "-ResY={}".format(600),
                    ]
                else:
                    carla_start_cmd = [
                        carla_executable,
                        '-RenderOffScreen',
                    ]

                carla_start_cmd += [
                    f"--carla-rpc-port={server_port}",
                    "-quality-level=Low"
                ]

                if os.name != "nt":
                    self.carla_process = subprocess.Popen(
                        carla_start_cmd,
                        shell=True,
                        preexec_fn=os.setsid,
                        stdout=open(os.devnull, "w"),
                    )
                else:
                    self.carla_process = subprocess.Popen(
                        carla_start_cmd,
                        shell=True,
                    )

                import time
                time.sleep(10)

                # Create the CarlaSimulation object
                from driver_dojo.carla_integration.carla_simulation import CarlaSimulation
                self.carla_simulation = CarlaSimulation('127.0.0.1', server_port, runtime_vars.config.simulation.dt)

                BridgeHelper.blueprint_library = self.carla_simulation.world.get_blueprint_library()

            for x in self.carla_sensors:
                x.destroy()
            for k, v in self.carla_actors.items():
                self.carla_simulation.destroy_actor(v)
            if self.carla_simulation: self.carla_simulation.tick()

            self.carla_actors = {}
            self.carla_sensors = []
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
                extra_width = 1.0      # in meters
                world = self.carla_simulation.client.generate_opendrive_world(
                    data, carla.OpendriveGenerationParameters(
                        vertex_distance=vertex_distance,
                        max_road_length=max_road_length,
                        wall_height=wall_height,
                        additional_width=extra_width,
                        smooth_junctions=True,
                        enable_mesh_visibility=True))

            # Configuring carla simulation in sync mode.
            settings = self.carla_simulation.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = runtime_vars.config.simulation.dt
            settings.substepping = False
            self.carla_simulation.world.apply_settings(settings)

        return runtime_vars.traci

    def close_engine(self):
        # Closes the simulator
        if self.running:
            runtime_vars.traci.close()
            runtime_vars.traci = None
            self.running = False
            logging.info(
                f"Closed SUMO instance with label {runtime_vars.sumo_label}..."
            )

            if self.carla_process:
                self.carla_simulation.close()

                if os.name != "nt":
                    os.killpg(os.getpgid(self.carla_process.pid), signal.SIGTERM)
                else:
                    import signal
                    #os.kill(self.carla_process.pid, signal.CTRL_C_EVENT)
                    #subprocess.call(['taskkill', '/F', '/T', '/PID', str(self.carla_process.pid)])
                    #signal.CTRL_BREAK_EVENT
                    self.carla_process.terminate()
                    self.carla_process.wait()

                self.carla_simulation = None
                self.carla_process = None
                self.carla_actors = {}

        sys.stdout.flush()

    def simulationStep(self):
        try:
            # Add whatever you want to do before/after simulationStep
            runtime_vars.traci.simulationStep()
        except traci.FatalTraCIError as e:
            logging.error(e.message)
            self.close_engine()
            return True


        if runtime_vars.config.simulation.co_sim_to_carla:
            from driver_dojo.carla_integration.bridge_helper import BridgeHelper
            import carla
            spawned_actors = set(runtime_vars.traci.simulation.getDepartedIDList())
            destroyed_actors = set(runtime_vars.traci.simulation.getArrivedIDList())

            # Spawn new
            sumo_spawned_actors = spawned_actors - set(self.carla_actors.values())
            for sumo_actor_id in sumo_spawned_actors:
                sumo_actor = self.get_actor(sumo_actor_id)
                if sumo_actor_id == runtime_vars.config.simulation.egoID:
                    carla_blueprint = self.carla_simulation.world.get_blueprint_library().find("vehicle.audi.a2")
                else:
                    carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, False)

                if carla_blueprint is not None:
                    carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                       sumo_actor.extent)

                    carla_actor_id = self.carla_simulation.spawn_actor(carla_blueprint, carla_transform)
                    from driver_dojo.carla_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
                    if carla_actor_id != INVALID_ACTOR_ID:
                        self.carla_actors[sumo_actor_id] = carla_actor_id
                        carla_actor = self.carla_simulation.get_actor(carla_actor_id)
                else:
                    print("No blueprint")
                    continue

                if runtime_vars.config.simulation.egoID == sumo_actor_id and runtime_vars.observer is not None:
                    from driver_dojo.core.types import CarlaSensor, Observer
                    from driver_dojo.observer.carla_observer import CarlaCameraObserver
                    for carla_sensor_type in runtime_vars.config.observations.carla_sensors:
                        if carla_sensor_type == CarlaSensor.RGBCamera:
                            bp_name = 'sensor.camera.rgb'
                        elif carla_sensor_type == CarlaSensor.DepthCamera:
                            bp_name = 'sensor.camera.depth'
                        else:
                            raise NotImplementedError

                        camera_bp = self.carla_simulation.world.get_blueprint_library().find(bp_name)
                        # Modify the attributes of the blueprint to set image resolution and field of view.
                        camera_bp.set_attribute('image_size_x', '84')
                        camera_bp.set_attribute('image_size_y', '84')
                        camera_bp.set_attribute('fov', '110')
                        camera_bp.set_attribute('sensor_tick', '0.0')
                        carla_actor = self.carla_simulation.get_actor(self.carla_actors[runtime_vars.config.simulation.egoID])
                        camera = self.carla_simulation.world.spawn_actor(
                            camera_bp,
                            carla.Transform(carla.Location(x=1.6, z=1.7)),
                            attach_to=carla_actor,
                            attachment_type=carla.AttachmentType.Rigid
                        )
                        observer = [x for x in runtime_vars.observer.observation_members['image'] if isinstance(x,CarlaCameraObserver)][0]
                        camera.listen(observer.listen_rgb)
                        self.carla_sensors.append(camera)
                        print("New camera")





            # Destroy old
            for sumo_actor_id in destroyed_actors:
                if sumo_actor_id in self.carla_actors:
                    self.carla_simulation.destroy_actor(self.carla_actors.pop(sumo_actor_id))

            # Updating sumo actors in carla.
            for sumo_actor_id in self.carla_actors:
                carla_actor_id = self.carla_actors[sumo_actor_id]

                sumo_actor = self.get_actor(sumo_actor_id)
                carla_actor = self.carla_simulation.get_actor(carla_actor_id)

                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(), sumo_actor.signals)

                self.carla_simulation.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)


            #camera_bp = blueprint_library.find('sensor.camera.rgb')
            #camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)
            #camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))


            # Update spectator cam
            # if runtime_vars.config.simulation.egoID in self.carla_actors:
            #     spectator = self.carla_simulation.world.get_spectator()
            #     ego_actor_carla = self.carla_simulation.world.get_actors().find(self.carla_actors[runtime_vars.config.simulation.egoID])
            #     ego_transform = ego_actor_carla.get_transform()
            #     spectator_transform = carla.Transform(
            #         carla.Location(
            #             ego_transform.location.x - 5 * ego_transform.get_forward_vector().x,
            #             ego_transform.location.y - 5 * ego_transform.get_forward_vector().y,
            #             ego_transform.location.z + 3
            #         ),
            #         ego_transform.rotation,
            #     )
            #     spectator.set_transform(spectator_transform)


            self.carla_simulation.tick()

        return False

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
