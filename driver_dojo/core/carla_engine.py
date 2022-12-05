import logging
import os
import signal
import subprocess
import sys
import random
import time
import psutil
import carla

from driver_dojo.core.types import CarlaSensor
from driver_dojo.carla_integration.bridge_helper import BridgeHelper
from driver_dojo.carla_integration.carla_simulation import CarlaSimulation
from driver_dojo.carla_integration.constants import INVALID_ACTOR_ID
from driver_dojo.carla_integration.sumo_simulation import SumoActorClass, SumoActor
from driver_dojo.observer import BaseObserver
from driver_dojo.observer.carla import CarlaCameraObserver


class CarlaEngine:
    def __init__(self, config, traffic_manager, carla_observers):
        self._running = False
        self.config = config
        self._traffic_manager = traffic_manager
        self._carla_process = None
        self._carla_simulation = None
        self._carla_sensors = []
        self._carla_actors = {}
        self._carla_observers = carla_observers
        self.world = None

    def reset(self, scenario):
        if not self._running:  # Start Carla server
            def is_used(port):
                return port in [conn.laddr.port for conn in psutil.net_connections()]
            server_port = random.randint(15000, 32000)
            while is_used(server_port) or is_used(server_port + 1):
                server_port += 2

            # Start a Carla server
            carla_executable = self.config.simulation.carla_path
            if self.config.rendering.human_mode:
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
                "-quality-level=Epic"  # Low
            ]

            if os.name != "nt":
                self._carla_process = subprocess.Popen(
                    carla_start_cmd,
                    shell=True,
                    preexec_fn=os.setsid,
                    stdout=open(os.devnull, "w"),
                )
            else:
                self._carla_process = subprocess.Popen(
                    carla_start_cmd,
                    # shell=True,
                )

            time.sleep(10)  # Wait, then connect
            self._carla_simulation = CarlaSimulation('127.0.0.1', server_port, self.config.simulation.dt)

            BridgeHelper.blueprint_library = self._carla_simulation.world.get_blueprint_library()
            self._running = True

        for x in self._carla_sensors:  # Destroy past actors and sensors
            x.destroy()
        for k, v in self._carla_actors.items():
            self._carla_simulation.destroy_actor(v)
        self._carla_simulation.tick()
        self._carla_actors = {}
        self._carla_sensors = []

        BridgeHelper.offset = scenario.sumo_net.getLocationOffset()  # Load the scenario map
        if os.path.exists(scenario.xodr_path):
            # TODO: We might be able to create nicer street textures using the PythonAPI
            # See https://carla.readthedocs.io/en/latest/tuto_G_texture_streaming/
            with open(scenario.xodr_path, encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    print('file could not be read.')
                    sys.exit()
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0  # in meters
            wall_height = 0.0  # in meters
            extra_width = 1.0  # in meters
            self.world = self._carla_simulation.client.generate_opendrive_world(
                data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True
                )
            )
        else:
            raise ValueError

        # TODO: In the near future (probably Carla 0.9.14), it will be possible to spawn static meshes using the Carla Python API
        # See https://github.com/carla-simulator/carla/pull/5723
        # world = self._carla_simulation.world
        # bp = world.get_blueprint_library().find("static.prop.mesh")
        # bp.set_attribute('mesh_path', "/Game/Carla/Static/Vehicles/4Wheeled/ParkedVehicles/TeslaM3/SM_TeslaM3_parked.SM_TeslaM3_parked")
        # bp.set_attribute('mass', "1000")
        # bp.set_attribute('scale', "0.9")
        # spawn_point = carla.Transform(carla.Location(0.0, 0.0, 5.0), carla.Rotation(0.0, 0.0, 0.0))
        # prop = world.spawn_actor(bp, spawn_point)

        # Configuring carla simulation in sync mode
        settings = self._carla_simulation.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.config.simulation.dt
        settings.substepping = False
        self._carla_simulation.world.apply_settings(settings)

    def close(self):
        if self._running:
            self._carla_simulation.close()
            del self._carla_simulation

            if os.name != "nt":
                os.killpg(os.getpgid(self._carla_process.pid), signal.SIGTERM)
            else:
                subprocess.call(['taskkill', '/F', '/T', '/PID', str(self._carla_process.pid)])

            self._carla_simulation = None
            self._carla_process = None
            self._carla_actors = {}
            self._carla_sensors = []
            self._running = False
            logging.info("Closed Carla instance...")

    def simulationStep(self):
        spawned_actors = [x for x in self._traffic_manager.actor_ids if x not in self._carla_actors.keys()]
        destroyed_actors = [x for x in self._carla_actors.keys() if x not in self._traffic_manager.actor_ids]

        for sumo_actor_id in spawned_actors:
            sumo_actor = self._get_sumo_actor(sumo_actor_id)
            if sumo_actor_id == self.config.simulation.egoID:
                carla_blueprint = self._carla_simulation.world.get_blueprint_library().find("vehicle.audi.a2")
            else:
                carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, False)

            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform, sumo_actor.extent)
                carla_actor_id = self._carla_simulation.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    #carla_actor = self._carla_simulation.get_actor(carla_actor_id)
                    self._carla_actors[sumo_actor_id] = carla_actor_id
            else:
                print("No blueprint")
                continue

            # if self.config.simulation.egoID == sumo_actor_id and world.world.observer is not None:
            #     from driver_dojo.core.types import CarlaSensor, Observer
            #     from driver_dojo.observer.carla_observer import CarlaCameraObserver
            #     for carla_sensor_type in world.world.config.observations.carla_sensors:
            #         if carla_sensor_type == CarlaSensor.RGBCamera:
            #             bp_name = 'sensor.camera.rgb'
            #         elif carla_sensor_type == CarlaSensor.DepthCamera:
            #             bp_name = 'sensor.camera.depth'
            #         else:
            #             raise NotImplementedError
            #
            #         camera_bp = self._carla_simulation.world.get_blueprint_library().find(bp_name)
            #         # Modify the attributes of the blueprint to set image resolution and field of view.
            #         camera_bp.set_attribute('image_size_x', '84')
            #         camera_bp.set_attribute('image_size_y', '84')
            #         camera_bp.set_attribute('fov', '110')
            #         camera_bp.set_attribute('sensor_tick', '0.0')
            #         carla_actor = self._carla_simulation.get_actor(self._carla_actors[world.world.config.simulation.egoID])
            #         camera = self._carla_simulation.world.spawn_actor(
            #             camera_bp,
            #             carla.Transform(carla.Location(x=1.6, z=1.7)),
            #             attach_to=carla_actor,
            #             attachment_type=carla.AttachmentType.Rigid
            #         )
            #         observer = [x for x in world.world.observer.observation_members['image'] if isinstance(x, CarlaCameraObserver)][0]
            #         camera.listen(observer.listen_rgb)
            #         self._carla_sensors.append(camera)

        for sumo_actor_id in destroyed_actors:
            if sumo_actor_id in self._carla_actors:
                self._carla_simulation.destroy_actor(self._carla_actors.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        for sumo_actor_id in self._carla_actors:
            sumo_actor = self._get_sumo_actor(sumo_actor_id)
            carla_actor_id = self._carla_actors[sumo_actor_id]
            carla_actor = self._carla_simulation.get_actor(carla_actor_id)
            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform, sumo_actor.extent)
            carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(), sumo_actor.signals)
            self._carla_simulation.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        if self.config.simulation.egoID in spawned_actors:
            for observer in self._carla_observers:  # Attach sensors to ego
                sensor = self._setup_observer(observer)
                self._carla_sensors.append(sensor)
        elif self.config.simulation.egoID in destroyed_actors:
            for x in self._carla_sensors:  # Destroy past actors and sensors
                x.destroy()
            self._carla_sensors = []

        try:
            self._carla_simulation.tick()
        except Exception:
            logging.info("Error ticking Carla instance...")
            self._running = False
            return True
        return False

    def _get_sumo_actor(self, actor_id):
        """
        Accessor for sumo actor.
        """
        actor_state = self._traffic_manager.get_actor_state(actor_id).to_sumo()
        type_id = actor_state.veh_type
        vclass = SumoActorClass("passenger")  # TODO: For the future, we might need to take other classes into consideration
        #color = actor_state.color
        location = actor_state.location
        rotation = actor_state.rotation
        transform = carla.Transform(
            carla.Location(location[0], location[1], location[2]),
            carla.Rotation(rotation[0], rotation[1], rotation[2])
        )
        signals = actor_state.signals
        extent = carla.Vector3D(*actor_state.extent)
        return SumoActor(type_id, vclass, transform, signals, extent, color)

    def _setup_observer(self, observer: CarlaCameraObserver):
        if observer.sensor_type == CarlaSensor.RGBCamera:
            bp_name = 'sensor.camera.rgb'
        elif observer.sensor_type == CarlaSensor.DepthCamera:
            bp_name = 'sensor.camera.depth'
        else:
            raise NotImplementedError

        camera_bp = self._carla_simulation.world.get_blueprint_library().find(bp_name)
        camera_bp.set_attribute('image_size_x', str(observer.width))
        camera_bp.set_attribute('image_size_y', str(observer.height))
        camera_bp.set_attribute('sensor_tick', '0.0')
        for k, v in observer.attributes.items():
            camera_bp.set_attribute(k, str(v))

        carla_actor = self._carla_simulation.get_actor(self._carla_actors[self.config.simulation.egoID])
        loc = observer.location
        rot = observer.rotation

        camera_transform = carla.Transform(
            carla.Location(loc[0], loc[1], loc[2]),
            carla.Rotation(rot[0], rot[1], rot[2])
        )
        camera = self._carla_simulation.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=carla_actor,
            attachment_type=carla.AttachmentType.Rigid
        )
        camera.listen(observer.listen)
        return camera

#### RESET
# if world.world.config.simulation.co_sim_to_carla:
#     from driver_dojo.carla_integration.bridge_helper import BridgeHelper
#     import carla
#     if self._carla_simulation is None:
#         import random
#         import psutil
#         def is_used(port):
#             """Checks whether or not a port is used"""
#             return port in [conn.laddr.port for conn in psutil.net_connections()]
#
#         server_port = random.randint(15000, 32000)
#         while is_used(server_port) or is_used(server_port + 1): server_port += 2
#
#         # Start a Carla server
#         carla_executable = world.world.config.simulation.carla_path
#         if world.world.config.simulation.render:
#             carla_start_cmd = [
#                 carla_executable,
#                 "-windowed",
#                 "-ResX={}".format(600),
#                 "-ResY={}".format(600),
#             ]
#         else:
#             carla_start_cmd = [
#                 carla_executable,
#                 '-RenderOffScreen',
#             ]
#
#         carla_start_cmd += [
#             f"--carla-rpc-port={server_port}",
#             "-quality-level=Low"
#         ]
#
#         if os.name != "nt":
#             self._carla_process = subprocess.Popen(
#                 carla_start_cmd,
#                 shell=True,
#                 preexec_fn=os.setsid,
#                 stdout=open(os.devnull, "w"),
#             )
#         else:
#             self._carla_process = subprocess.Popen(
#                 carla_start_cmd,
#                 # shell=True,
#             )
#
#         import time
#         time.sleep(10)
#
#         # Create the CarlaSimulation object
#         from driver_dojo.carla_integration.carla_simulation import CarlaSimulation
#         self._carla_simulation = CarlaSimulation('127.0.0.1', server_port, world.world.config.simulation.dt)
#
#         BridgeHelper.blueprint_library = self._carla_simulation.world.get_blueprint_library()
#
#     for x in self._carla_sensors:
#         x.destroy()
#     for k, v in self._carla_actors.items():
#         self._carla_simulation.destroy_actor(v)
#     if self._carla_simulation: self._carla_simulation.tick()
#
#     self._carla_actors = {}
#     self._carla_sensors = []
#     # We have to grab this again and again
#     BridgeHelper.offset = world.world.net.getLocationOffset()
#     # Load map
#     xodr_path = os.path.join(world.world.tmp_path, world.world.sumo_label + '.xodr')
#     if os.path.exists(xodr_path):
#         with open(xodr_path, encoding='utf-8') as od_file:
#             try:
#                 data = od_file.read()
#             except OSError:
#                 print('file could not be readed.')
#                 sys.exit()
#         vertex_distance = 2.0  # in meters
#         max_road_length = 500.0  # in meters
#         wall_height = 0.0  # in meters
#         extra_width = 1.0  # in meters
#         world = self._carla_simulation.client.generate_opendrive_world(
#             data, carla.OpendriveGenerationParameters(
#                 vertex_distance=vertex_distance,
#                 max_road_length=max_road_length,
#                 wall_height=wall_height,
#                 additional_width=extra_width,
#                 smooth_junctions=True,
#                 enable_mesh_visibility=True))
#
#     # Configuring carla simulation in sync mode.
#     settings = self._carla_simulation.world.get_settings()
#     settings.synchronous_mode = True
#     settings.fixed_delta_seconds = world.world.config.simulation.dt
#     settings.substepping = False
#     self._carla_simulation.world.apply_settings(settings)
#
# return world.world.traci


### CLOSE
#     if self._carla_process:
#         self._carla_simulation.close()
#
#         if os.name != "nt":
#             os.killpg(os.getpgid(self._carla_process.pid), signal.SIGTERM)
#         else:
#             import signal
#             # os.kill(self.carla_process.pid, signal.CTRL_C_EVENT)
#             del (self._carla_simulation)
#             # os.kill(self.carla_process.pid, signal.CTRL_BREAK_EVENT)
#             subprocess.call(['taskkill', '/F', '/T', '/PID', str(self._carla_process.pid)])
#             # signal.CTRL_BREAK_EVENT
#             # self.carla_process.terminate()
#
#         self._carla_simulation = None
#         self._carla_process = None
#         self._carla_actors = {}
#
# sys.stdout.flush()

### STEP
# if world.world.config.simulation.co_sim_to_carla:
#     from driver_dojo.carla_integration.bridge_helper import BridgeHelper
#     import carla
#     spawned_actors = set(world.world.traci.simulation.getDepartedIDList())
#     destroyed_actors = set(world.world.traci.simulation.getArrivedIDList())
#
#     # Spawn new
#     sumo_spawned_actors = spawned_actors - set(self._carla_actors.values())
#     for sumo_actor_id in sumo_spawned_actors:
#         sumo_actor = self.get_actor(sumo_actor_id)
#         if sumo_actor_id == world.world.config.simulation.egoID:
#             carla_blueprint = self._carla_simulation.world.get_blueprint_library().find("vehicle.audi.a2")
#         else:
#             carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, False)
#
#         if carla_blueprint is not None:
#             carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
#                                                                sumo_actor.extent)
#
#             carla_actor_id = self._carla_simulation.spawn_actor(carla_blueprint, carla_transform)
#             from driver_dojo.carla_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
#             if carla_actor_id != INVALID_ACTOR_ID:
#                 self._carla_actors[sumo_actor_id] = carla_actor_id
#                 carla_actor = self._carla_simulation.get_actor(carla_actor_id)
#         else:
#             print("No blueprint")
#             continue
#
#         if world.world.config.simulation.egoID == sumo_actor_id and world.world.observer is not None:
#             from driver_dojo.core.types import CarlaSensor, Observer
#             from driver_dojo.observer.carla_observer import CarlaCameraObserver
#             for carla_sensor_type in world.world.config.observations.carla_sensors:
#                 if carla_sensor_type == CarlaSensor.RGBCamera:
#                     bp_name = 'sensor.camera.rgb'
#                 elif carla_sensor_type == CarlaSensor.DepthCamera:
#                     bp_name = 'sensor.camera.depth'
#                 else:
#                     raise NotImplementedError
#
#                 camera_bp = self._carla_simulation.world.get_blueprint_library().find(bp_name)
#                 # Modify the attributes of the blueprint to set image resolution and field of view.
#                 camera_bp.set_attribute('image_size_x', '84')
#                 camera_bp.set_attribute('image_size_y', '84')
#                 camera_bp.set_attribute('fov', '110')
#                 camera_bp.set_attribute('sensor_tick', '0.0')
#                 carla_actor = self._carla_simulation.get_actor(self._carla_actors[world.world.config.simulation.egoID])
#                 camera = self._carla_simulation.world.spawn_actor(
#                     camera_bp,
#                     carla.Transform(carla.Location(x=1.6, z=1.7)),
#                     attach_to=carla_actor,
#                     attachment_type=carla.AttachmentType.Rigid
#                 )
#                 observer = [x for x in world.world.observer.observation_members['image'] if isinstance(x, CarlaCameraObserver)][0]
#                 camera.listen(observer.listen_rgb)
#                 self._carla_sensors.append(camera)
#
#     # Destroy old
#     for sumo_actor_id in destroyed_actors:
#         if sumo_actor_id in self._carla_actors:
#             self._carla_simulation.destroy_actor(self._carla_actors.pop(sumo_actor_id))
#
#     # Updating sumo actors in carla.
#     for sumo_actor_id in self._carla_actors:
#         carla_actor_id = self._carla_actors[sumo_actor_id]
#
#         sumo_actor = self.get_actor(sumo_actor_id)
#         carla_actor = self._carla_simulation.get_actor(carla_actor_id)
#
#         carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
#                                                            sumo_actor.extent)
#
#         carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(), sumo_actor.signals)
#
#         self._carla_simulation.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

# self._carla_simulation.tick()

### OTHER
# @staticmethod
# def get_actor(actor_id):
#     """
#     Accessor for sumo actor.
#     """
#     results = traci.vehicle.getSubscriptionResults(actor_id)
#
#     type_id = traci.vehicle.getTypeID(actor_id)
#     from driver_dojo.carla_integration.sumo_simulation import SumoActorClass
#     vclass = SumoActorClass("passenger")
#     color = traci.vehicle.getColor(actor_id)
#
#     length = traci.vehicle.getLength(actor_id)
#     width = traci.vehicle.getWidth(actor_id)
#     height = traci.vehicle.getHeight(actor_id)
#
#     location = list(traci.vehicle.getPosition3D(actor_id))
#     rotation = [traci.vehicle.getSlope(actor_id), traci.vehicle.getAngle(actor_id), 0.0]
#     import carla
#     transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
#                                 carla.Rotation(rotation[0], rotation[1], rotation[2]))
#
#     signals = traci.vehicle.getSignals(actor_id)
#     extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)
#     from driver_dojo.carla_integration.sumo_simulation import SumoActor
#     return SumoActor(type_id, vclass, transform, signals, extent, color)
