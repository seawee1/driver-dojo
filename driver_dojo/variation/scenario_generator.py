import shutil
import xml.etree.cElementTree as ET
from os.path import join as pjoin
import sumolib
import subprocess
from threading import Thread
import psutil
import os

from driver_dojo.common import state_variables
import driver_dojo.core.types as types
from driver_dojo.variation import (
    JunctionSample,
    create_junction,
    create_highway_entry,
    create_highway_drive,
    create_highway_exit,
    HighwaySample
)
from scenariogeneration import xodr
from driver_dojo.navigation.sumo_map import parse_route
from driver_dojo.variation.sumolib_generators import create_roundabout, RoundaboutSample


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


class ScenarioGenerator:
    def __init__(self):
        self.generator_thread = None

    def step(self):
        # Start the generator thread
        self.generator_thread = Thread(target=self._generate, args=())
        self.generator_thread.start()

    def join(self):
        # Wait until it's finished
        self.generator_thread.join()

        # Read in all the important things
        state_variables.net = sumolib.net.readNet(
            state_variables.config["simulation"]["net_path"], withInternal=True
        )
        if not state_variables.config.simulation.init_ego:
            state_variables.route_edges = parse_route(
                state_variables.config["simulation"]["route_path"],
                state_variables.config["simulation"]["routeID"],
                standalone=True,
            )
        state_variables.net_bbox = state_variables.net.getBBoxXY()

        store_path = state_variables.config.simulation.store_for_plotting
        if store_path:
            os.makedirs(store_path, exist_ok=True)
            shutil.copyfile(
                state_variables.config.simulation.net_path,
                os.path.join(store_path, f"{state_variables.episode_count}.net.xml"),
            )
            print(
                os.path.abspath(
                    os.path.join(store_path, f"{state_variables.episode_count}.net.xml")
                )
            ),
            xodr_path = pjoin(
                state_variables.temp_path, f"{state_variables.sumo_label}.xodr"
            )
            if os.path.exists(pjoin(state_variables.temp_path, xodr_path)):
                shutil.copyfile(
                    xodr_path,
                    os.path.join(store_path, f"{state_variables.episode_count}.xodr"),
                )

    def _generate(self):
        if state_variables.config["variation"]["network"]:
            self._generate_network()
        if state_variables.config["variation"]["network_netgenerate"]:
            self._generate_network_netgenerate()
            self._generate_traffic_routing_randomTrips()
        if state_variables.config["variation"]["traffic_routing_randomTrips"]:
            self._generate_traffic_routing_randomTrips()
        if state_variables.config["variation"]["traffic_vTypes"]:
            self._generate_vType_distribution()

    def _generate_network_netgenerate(self):
        netgenerate_cmd = [
            "-c",
            state_variables.config["variation"]["netgenerate_xml_path"],
            "-o",
            state_variables.config["simulation"]["net_path"],
            "--seed",
            str(state_variables.maps_seed),
        ]

        netgenerate_cmd = (
            ["netgenerate.exe"] + netgenerate_cmd
            if os.name == "nt"
            else ["netgenerate"] + netgenerate_cmd
        )
        subprocess.run(netgenerate_cmd)

    def _generate_network(self):
        netconvert_cmd = [
            "netconvert",
            "--opendrive",
            pjoin(state_variables.temp_path, f"{state_variables.sumo_label}.xodr"),
            "--output-file",
            pjoin(state_variables.temp_path, f"{state_variables.sumo_label}.net.xml"),
            "--junctions.internal-link-detail",
            "10",
            "--junctions.corner-detail",
            "20",
            "--opendrive.import-all-lanes",
            "false",
            "--opendrive.curve-resolution",
            "1.0",
            "--opendrive.internal-shapes",
            "false",
            "--opendrive.lane-shapes",
            "false",
            "--geometry.max-grade.fix",
            "true",
            "--plain.extend-edge-shape",
            "false",
            "--no-turnarounds",
            "true",  # Who need them anyways?
            "--default.junctions.keep-clear",
            "false",
            "--opendrive.advance-stopline",
            "15",  # For more natural intersections
            "--keep-nodes-unregulated",
            "false",
            "--offset.disable-normalization",
            "false",
            "--geometry.min-radius.fix",
            "true",
            "--check-lane-foes.all",
            "true",
            #'--numerical-ids', 'true'
        ]

        road_id = 0
        internal_id = 100
        junction_id = 0
        roads = []
        junctions = []
        for road_segment in state_variables.config["variation"]["network"]:
            # TODO: Composing
            r = None
            j = None
            # Roundabout
            if road_segment == types.GeneratableRoad.Roundabout:
                sample = RoundaboutSample()
                create_roundabout(sample.radius, sample.num_lanes, sample.internal_lanes, sample.rad_incident, sample.angles, sample.road_cs, sample.lengths, sample.squeeze)
                r = []
                j = []
            # Generate intersection
            if road_segment == types.GeneratableRoad.Intersection:
                zero_cloth = True
                while zero_cloth:
                    road_id = 0
                    internal_id = 100
                    junction_id = 0
                    roads = []
                    js = JunctionSample()
                    (
                        r,
                        j,
                        glueable_roads,
                        road_id,
                        internal_id,
                        junction_id,
                        zero_cloth,
                    ) = create_junction(
                        js.radius,
                        js.lengths,
                        js.n_lanes,
                        js.angles,
                        js.cs_inner,
                        js.cs_outer,
                        road_id,
                        internal_id,
                        junction_id,
                    )

            # Generate highway scenario
            highway_entry = road_segment == types.GeneratableRoad.HighwayEntry
            highway_drive = road_segment == types.GeneratableRoad.HighwayDrive
            highway_exit = road_segment == types.GeneratableRoad.HighwayExit
            if highway_entry or highway_drive or highway_exit:

                highway_sample = HighwaySample()
                n_lanes = highway_sample.n_lanes
                entry = highway_sample.entry
                drive = highway_sample.drive
                exit = highway_sample.exit

                if highway_entry:
                    (
                        r,
                        j,
                        glueable_roads,
                        road_id,
                        internal_id,
                        junction_id,
                    ) = create_highway_entry(
                        entry.c1,
                        entry.c2,
                        entry.c3,
                        entry.c_merge,
                        n_lanes,
                        road_id,
                        internal_id,
                        junction_id,
                    )
                elif highway_drive:
                    (
                        r,
                        j,
                        glueable_roads,
                        road_id,
                        internal_id,
                        junction_id,
                    ) = create_highway_drive(
                        drive.c1,
                        drive.c2,
                        drive.c3,
                        drive.c4,
                        n_lanes,
                        road_id,
                        internal_id,
                        junction_id,
                    )
                elif highway_exit:
                    (
                        r,
                        j,
                        glueable_roads,
                        road_id,
                        internal_id,
                        junction_id,
                    ) = create_highway_exit(
                        exit.c1,
                        exit.c2,
                        exit.c3,
                        exit.c_exit,
                        n_lanes,
                        road_id,
                        internal_id,
                        junction_id,
                    )

                # We overwrite the allowed speed.
                netconvert_cmd += [
                    "--default.speed",
                    "33.33",
                ]

            roads += r
            junctions += j

        odr = xodr.OpenDrive("Scenario")
        for road in roads:
            odr.add_road(road)
        for junction in junctions:
            odr.add_junction(junction)
        odr.adjust_roads_and_lanes()
        odr.write_xml(
            pjoin(state_variables.temp_path, f"{state_variables.sumo_label}.xodr")
        )

        # Sometimes netconvert can get stuck in an infinite loop.
        # In case this happens, we kill the process and generate a new map which it hopefully can translate.
        p = subprocess.Popen(
            netconvert_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )  # , stderr=subprocess.PIPE)
        try:
            p.wait(2.0)
        except subprocess.TimeoutExpired:
            try:
                kill(p.pid)
            except psutil.NoSuchProcess:
                pass
            p.wait()
            self._generate_network()

    def _generate_traffic_routing_randomTrips(self):
        trips_path = pjoin(
            state_variables.temp_path, f"{state_variables.sumo_label}.trips.xml"
        )

        randomTrips_cmd = [
            "python",
            state_variables.config["variation"]["randomTrips_py_path"],
            "-n",
            state_variables.config["simulation"]["net_path"],
            "-r",
            state_variables.config["simulation"]["route_path"],
            "-o",
            trips_path,
            "--seed",
            str(state_variables.traffic_seed),
            "--random-depart",
            "--random",
            "--period",
            str(state_variables.config.variation.randomTrips_period),
            "--validate",
        ]
        randomTrips_cmd += (
            state_variables.config["variation"]["randomTrips_args"].split(" ")
            if state_variables.config["variation"]["randomTrips_args"]
            else []
        )

        p = subprocess.Popen(
            randomTrips_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )
        p.wait()

    def _generate_vType_distribution(self):
        import subprocess

        vTypeDistribution_cmd = [
            "python",
            state_variables.config["variation"]["vTypeDistribution_py_path"],
            state_variables.config["variation"]["vTypeDistribution_config_path"],
            "--output",
            pjoin(
                state_variables.temp_path, f"{state_variables.sumo_label}.add.xml"
            ),  # We write to this file. See driver_dojo.core.env_train:_init_sumo_paths() method.
            "--name",
            "vehDist",
            "--seed",
            str(state_variables.traffic_seed),
            "--size",
            str(state_variables.config.variation.vTypeDistribution_num),
        ]

        if (
            state_variables.config["variation"]["vTypeDistribution_args"]
            and state_variables.config["variation"]["vTypeDistribution_args"] != ""
        ):
            vTypeDistribution_cmd += [
                state_variables.config["variation"]["vTypeDistribution_args"]
            ]

        p = subprocess.Popen(
            vTypeDistribution_cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE
        )
        p.wait()
