import os
import numpy as np

import pyclothoids as pcloth
from scenariogeneration import xodr
from scenariogeneration.xodr.lane import Lane, RoadMark, LaneSection, Lanes, RoadLine
from scenariogeneration.xodr.enumerations import (
    RoadMarkType,
    ContactPoint,
)
from scenariogeneration.xodr.geometry import Line, Spiral, PlanView
from scenariogeneration.xodr.opendrive import Road
from scenariogeneration.xodr.links import Junction, Connection
import logging

from driver_dojo.common import state_variables

STD_ROADMARK_SOLID = RoadMark(RoadMarkType.solid, 0.2)

STD_ROADMARK_BROKEN = RoadMark(RoadMarkType.broken, 0.2)

STD_ROADMARK_BROKEN_BROKEN = RoadMark(RoadMarkType.broken_broken)
STD_ROADMARK_BROKEN_BROKEN.add_specific_road_line(RoadLine(0.2, 3, 3, 0.2, 0))
STD_ROADMARK_BROKEN_BROKEN.add_specific_road_line(RoadLine(0.2, 3, 3, -0.2, 0))

STD_ROADMARK_SOLID_SOLID = RoadMark(RoadMarkType.solid_solid)
STD_ROADMARK_SOLID_SOLID.add_specific_road_line(RoadLine(0.2, 0, 0, 0.2, 0))
STD_ROADMARK_SOLID_SOLID.add_specific_road_line(RoadLine(0.2, 0, 0, -0.2, 0))

STD_ROADMARK_SOLID_BROKEN = RoadMark(RoadMarkType.solid_broken)
STD_ROADMARK_SOLID_BROKEN.add_specific_road_line(RoadLine(0.2, 0, 0, 0.2, 0))
STD_ROADMARK_SOLID_BROKEN.add_specific_road_line(RoadLine(0.2, 3, 3, -0.2, 0))

STD_ROADMARK_BROKEN_SOLID = RoadMark(RoadMarkType.broken_solid)
STD_ROADMARK_BROKEN_SOLID.add_specific_road_line(RoadLine(0.2, 0, 0, -0.2, 0))
STD_ROADMARK_BROKEN_SOLID.add_specific_road_line(RoadLine(0.2, 3, 3, 0.2, 0))


def standard_lane(offset=3.6, rm=STD_ROADMARK_BROKEN):
    lc = Lane(a=offset)
    lc.add_roadmark(rm)
    return lc


def create_road_start(
    road_id,
    n_lanes,
    x_start=None,
    y_start=None,
    h_start=None,
    length=100,
    junction=-1,
    lane_offset=3.6,
    geometry=None,
):
    if not geometry:
        geometry = [Line(length)]

    # create planviews
    planview = PlanView(x_start, y_start, h_start)
    # Add geometry
    for geom in geometry:
        planview.add_geometry(geom)

    # create lanesections
    lanesec = LaneSection(0, standard_lane())
    for i in range(n_lanes[0]):
        lanesec.add_left_lane(standard_lane(lane_offset))
    for i in range(n_lanes[1]):
        lanesec.add_right_lane(standard_lane(lane_offset))

    # create lanes
    lanes = Lanes()
    lanes.add_lanesection(lanesec)

    # finally create the roads
    return Road(road_id, planview, lanes, road_type=junction)


def create_3cloths_right_lane(
    cloth1_start,
    cloth1_end,
    cloth1_length,
    cloth2_start,
    cloth2_end,
    cloth2_length,
    cloth3_start,
    cloth3_end,
    cloth3_length,
    r_id,
    x_start=None,
    y_start=None,
    h_start=None,
    junction=1,
    n_lanes=1,
    lane_offset=3.6,
    road_marks=STD_ROADMARK_BROKEN,
):
    pv = PlanView(x_start, y_start, h_start)

    spiral1 = Spiral(cloth1_start, cloth1_end, length=cloth1_length)
    spiral2 = Spiral(cloth2_start, cloth2_end, length=cloth2_length)
    spiral3 = Spiral(cloth3_start, cloth3_end, length=cloth3_length)

    pv.add_geometry(
        spiral1
    )  # if abs(spiral1.curvstart - spiral1.curvend) != 0.0 else 0
    pv.add_geometry(
        spiral2
    )  # if abs(spiral2.curvstart - spiral2.curvend) != 0.0 else 0
    pv.add_geometry(
        spiral3
    )  # if abs(spiral3.curvstart - spiral3.curvend) != 0.0 else 0
    zero_cloth = (
        abs(spiral1.curvstart - spiral1.curvend) == 0.0
        or abs(spiral2.curvstart - spiral2.curvend) == 0.0
        or abs(spiral3.curvstart - spiral3.curvend) == 0.0
    )
    center_lane = Lane()
    if road_marks:
        center_lane.add_roadmark(road_marks)
    lsec = LaneSection(0, center_lane)

    for i in range(n_lanes):
        rl = Lane(a=lane_offset)
        if road_marks:
            rl.add_roadmark(road_marks)
        lsec.add_right_lane(rl)

    lanes = Lanes()
    lanes.add_lanesection(lsec)
    return Road(r_id, pv, lanes, road_type=junction), zero_cloth


def create_junction(
    radius,
    lengths,
    n_lanes,
    angles,
    cs_inner,
    cs_outer,
    road_id,
    internal_id,
    junction_id,
):
    junction = Junction(f"Junction {junction_id}", junction_id)
    roads = []
    junction_roads = []
    zero_cloth = False
    # Generate incoming lanes
    roadIDs = []
    for i in range(len(n_lanes)):
        geometry = [Spiral(cs_outer[i], cs_inner[i], lengths[i])]

        road = create_road_start(
            road_id,
            n_lanes[i],
            radius[i] * np.cos(angles[i]),
            radius[i] * np.sin(angles[i]),
            angles[i],
            geometry=geometry,
        )
        road.add_predecessor(xodr.ElementType.junction, 1)
        roads.append(road)
        roadIDs.append(road_id)
        road_id += 1

    for i, _ in enumerate(roadIDs):
        for j, _ in enumerate(roadIDs):
            if i == j:
                continue

            an1 = angles[j] - angles[i] - np.pi
            if an1 > np.pi:
                an1 = -(2 * np.pi - an1)

            n_in = n_lanes[i][0]
            n_out = n_lanes[j][1]
            n_conn = min(n_in, n_out)

            in_offset = 0
            out_offset = 0
            # left turn
            if i - j == 1 or (i == 0 and j == len(n_lanes) - 1):
                if n_in >= n_out:
                    pass
                else:
                    pass
            # right turn
            if i - j == -1 or (i == len(n_lanes) - 1 and j == 0):
                if n_in >= n_out:
                    in_offset = int(n_in - n_out)
                else:
                    out_offset = int(n_out - n_in)
            # print(i, j, an1, n_in, n_out, in_offset, out_offset, jd.n_lanes[i], jd.n_lanes[j])

            for k in range(n_conn):
                from_x = -radius[i]
                from_y = -(k + in_offset) * 3.6
                to_x = radius[j] * np.cos(an1) + np.sin(an1) * (k + out_offset) * 3.6
                to_y = radius[j] * np.sin(an1) - np.cos(an1) * (k + out_offset) * 3.6

                clothoids = pcloth.SolveG2(
                    from_x, from_y, 0, 1 / 1000000000, to_x, to_y, an1, 1 / 1000000000,
                )

                junc_road, zero_cloth_ = create_3cloths_right_lane(
                    clothoids[0].KappaStart,
                    clothoids[0].KappaEnd,
                    clothoids[0].length,
                    clothoids[1].KappaStart,
                    clothoids[1].KappaEnd,
                    clothoids[1].length,
                    clothoids[2].KappaStart,
                    clothoids[2].KappaEnd,
                    clothoids[2].length,
                    internal_id,
                    x_start=radius[i] * np.cos(angles[i])
                    - np.sin(angles[i]) * (k + in_offset) * 3.6,
                    y_start=radius[i] * np.sin(angles[i])
                    + np.cos(angles[i]) * (k + in_offset) * 3.6,
                    h_start=angles[i] - np.pi,
                    n_lanes=1,
                    lane_offset=3.6,
                    junction=1,
                )
                internal_id += 1
                zero_cloth = zero_cloth or zero_cloth_

                junc_road.add_successor(
                    xodr.ElementType.road,
                    roadIDs[j],
                    xodr.ContactPoint.start,
                    lane_offset=-k - out_offset,
                )
                junc_road.add_predecessor(
                    xodr.ElementType.road,
                    roadIDs[i],
                    xodr.ContactPoint.start,
                    lane_offset=k + in_offset,
                )
                junction_roads.append(junc_road)

                conne1 = Connection(
                    junc_road.successor.element_id, junc_road.id, ContactPoint.end
                )
                conne1.add_lanelink(-1, -(1 + k + out_offset))
                conne2 = Connection(
                    junc_road.predecessor.element_id, junc_road.id, ContactPoint.start
                )
                conne2.add_lanelink(1 + k + in_offset, -1)

                junction.add_connection(conne1)
                junction.add_connection(conne2)

                internal_id += 1

    glueable_roads = [(road, xodr.ContactPoint.end) for road in roads]
    return (
        roads + junction_roads,
        [junction],
        glueable_roads,
        road_id,
        internal_id,
        junction_id + 1,
        zero_cloth,
    )


def create_highway_drive(
    c1,
    c2,
    c3,
    c4,
    n_lanes,
    road_id,
    internal_id,
    junction_id,
    l1=150,
    l2=150,
    l3=150,
    before_road_tuple=None,
):
    # The road
    road = xodr.create_road(
        [xodr.Spiral(c1, c2, l1), xodr.Spiral(c2, c3, l2), xodr.Spiral(c3, c4, l3),],
        id=road_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
    )

    if before_road_tuple:
        before_road, before_road_cp = before_road_tuple
        road.add_predecessor(xodr.ElementType.road, before_road.id, before_road_cp)
        if before_road_cp == xodr.ContactPoint.start:
            before_road.add_predecessor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )
        else:
            before_road.add_successor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )

    glueable_roads = [(road, xodr.ContactPoint.start), (road, xodr.ContactPoint.end)]
    return [road], [], glueable_roads, road_id + 1, internal_id, junction_id


def create_highway_entry(
    c1,
    c2,
    c3,
    c_merge,
    n_lanes,
    road_id,
    internal_id,
    junction_id,
    l1=200,
    l2=300,
    l_merge=150,
    before_road_tuple=None,
):
    assert l2 >= 200
    # The road
    road = xodr.create_road(
        [
            xodr.Spiral(
                c1, c2, l1 - 35
            ),  # We compensate for intermittend road and internal junction road lengths.
        ],
        id=road_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
        lane_width=3.6,
    )

    # Merge road
    merge_id = road_id + 1
    merge_road = xodr.create_road(
        xodr.Spiral(c_merge, c2, l_merge),
        id=merge_id,
        left_lanes=0,
        right_lanes=1,
        lane_width=3.6,
    )

    intermittend_id = merge_id + 1
    intermittend_road = xodr.create_road(
        [xodr.Arc(c2, 30),],
        id=intermittend_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1]
        + 1,  # [xodr.LaneDef(1, 189, n_lanes[1] + 1, n_lanes[1], -(n_lanes[1] + 1))],
        lane_width=3.6,
    )

    merging_id = intermittend_id + 1
    merging_road = xodr.create_road(
        [xodr.Spiral(c2, c3, 200),],
        id=merging_id,
        left_lanes=n_lanes[0],
        right_lanes=[
            xodr.LaneDef(1, 150, n_lanes[1] + 1, n_lanes[1], -(n_lanes[1] + 1))
        ],
        lane_width=3.6,
    )

    # Continue road
    continue_id = merging_id + 1
    continue_road = xodr.create_road(
        [xodr.Spiral(c2, c3, l2 - 200),],
        id=continue_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
        lane_width=3.6,
    )

    # Junction roads
    continue_internal = xodr.create_road(
        xodr.Arc(c2, 5),
        id=internal_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
        road_type=2,
        lane_width=3.6,
    )
    merge_internal = xodr.create_road(
        xodr.Arc(c2, 5),
        id=internal_id + 1,
        left_lanes=0,
        right_lanes=1,
        road_type=2,
        lane_width=3.6,
    )

    # Link normal roads
    road.add_successor(xodr.ElementType.junction, junction_id)
    # merge_road.add_successor(xodr.ElementType.junction, junction_id)
    intermittend_road.add_predecessor(xodr.ElementType.junction, junction_id)
    intermittend_road.add_successor(
        xodr.ElementType.road, merging_id, xodr.ContactPoint.start
    )
    merging_road.add_predecessor(
        xodr.ElementType.road, intermittend_id, xodr.ContactPoint.end
    )
    merging_road.add_successor(
        xodr.ElementType.road, continue_id, xodr.ContactPoint.start
    )
    continue_road.add_predecessor(
        xodr.ElementType.road, merging_id, xodr.ContactPoint.end
    )

    # Link internal roads
    merge_internal.add_predecessor(
        xodr.ElementType.road, merge_id, xodr.ContactPoint.end, lane_offset=-1
    )
    merge_internal.add_successor(
        xodr.ElementType.road,
        intermittend_id,
        xodr.ContactPoint.start,
        lane_offset=-n_lanes[1],
    )
    continue_internal.add_predecessor(
        xodr.ElementType.road, road_id, xodr.ContactPoint.end
    )
    continue_internal.add_successor(
        xodr.ElementType.road, intermittend_id, xodr.ContactPoint.start
    )

    # Create junction
    merge_junction = xodr.create_junction(
        [continue_internal, merge_internal],
        junction_id,
        [road, intermittend_road, merge_road],
        name=f"Junction {junction_id}",
    )

    # Add before road
    if before_road_tuple:
        before_road, before_road_cp = before_road_tuple
        road.add_predecessor(xodr.ElementType.road, before_road.id, before_road_cp)
        if before_road_cp == xodr.ContactPoint.start:
            before_road.add_predecessor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )
        else:
            before_road.add_successor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )
    glueable_roads = [
        (road, xodr.ContactPoint.start),
        (merge_road, xodr.ContactPoint.start),
        (continue_road, xodr.ContactPoint.end),
    ]
    return (
        [
            road,
            merging_road,
            merge_road,
            intermittend_road,
            continue_road,
            continue_internal,
            merge_internal,
        ],
        [merge_junction],
        glueable_roads,
        continue_id + 1,
        internal_id + 2,
        junction_id + 1,
    )


def create_highway_exit(
    c1,
    c2,
    c3,
    c_exit,
    n_lanes,
    road_id,
    internal_id,
    junction_id,
    l1=200,
    l2=100,
    l_exit=150,
    before_road_tuple=None,
):
    assert l1 >= 200
    # # The road
    # road = xodr.create_road(
    #     [
    #         xodr.Spiral(c1, c2, l1 - 160),
    #     ],
    #     id=road_id,
    #     left_lanes=n_lanes[0],
    #     right_lanes=n_lanes[1],
    # )

    splitting_id = road_id  # + 1
    splitting_road = xodr.create_road(
        [xodr.Spiral(c2, c3, 200),],
        id=splitting_id,
        left_lanes=n_lanes[0],
        right_lanes=[
            xodr.LaneDef(50, 150, n_lanes[1], n_lanes[1] + 1, -n_lanes[1] - 1)
        ],
    )

    intermittend_id = splitting_id + 1
    intermittend_road = xodr.create_road(
        [xodr.Arc(c2, 1),],
        id=intermittend_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1]
        + 1,  # [xodr.LaneDef(1, 189, n_lanes[1] + 1, n_lanes[1], -(n_lanes[1] + 1))],
    )

    # Merge road
    exit_id = intermittend_id + 1
    exit_road = xodr.create_road(
        xodr.Spiral(c2, c_exit, l_exit), id=exit_id, left_lanes=0, right_lanes=1
    )

    # Continue road
    continue_id = exit_id + 1
    continue_road = xodr.create_road(
        [xodr.Spiral(c2, c3, l2),],
        id=continue_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
    )

    # Junction roads
    continue_internal = xodr.create_road(
        xodr.Arc(c2, 1),
        id=internal_id,
        left_lanes=n_lanes[0],
        right_lanes=n_lanes[1],
        road_type=2,
    )
    split_internal = xodr.create_road(
        xodr.Arc(c2, 1), id=internal_id + 1, left_lanes=0, right_lanes=1, road_type=2
    )

    # Link normal roads
    # road.add_successor(xodr.ElementType.road, splitting_id, xodr.ContactPoint.start)
    # splitting_road.add_predecessor(xodr.ElementType.road, road_id, xodr.ContactPoint.end)
    splitting_road.add_successor(
        xodr.ElementType.road, intermittend_id, xodr.ContactPoint.start
    )
    intermittend_road.add_predecessor(
        xodr.ElementType.road, splitting_id, xodr.ContactPoint.end
    )
    intermittend_road.add_successor(xodr.ElementType.junction, junction_id)
    continue_road.add_predecessor(xodr.ElementType.junction, junction_id)
    # exit_road.add_predecessor(xodr.ElementType.junction, junction_id)
    # Link internal roads
    split_internal.add_predecessor(
        xodr.ElementType.road,
        intermittend_id,
        xodr.ContactPoint.end,
        lane_offset=-n_lanes[1],
    )
    split_internal.add_successor(
        xodr.ElementType.road, exit_id, xodr.ContactPoint.start, lane_offset=-1
    )
    continue_internal.add_predecessor(
        xodr.ElementType.road, intermittend_id, xodr.ContactPoint.end
    )
    continue_internal.add_successor(
        xodr.ElementType.road, continue_id, xodr.ContactPoint.start
    )

    # Create junction
    exit_junction = xodr.create_junction(
        [continue_internal, split_internal],
        junction_id,
        [continue_road, intermittend_road, exit_road],
        name=f"Junction {junction_id}",
    )

    # Add before road
    if before_road_tuple:
        before_road, before_road_cp = before_road_tuple
        splitting_road.add_predecessor(
            xodr.ElementType.road, before_road.id, before_road_cp
        )
        if before_road_cp == xodr.ContactPoint.start:
            before_road.add_predecessor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )
        else:
            before_road.add_successor(
                xodr.ElementType.road, road_id, xodr.ContactPoint.start
            )

    glueable_roads = [
        (splitting_road, xodr.ContactPoint.start),
        (exit_road, xodr.ContactPoint.end),
        (continue_road, xodr.ContactPoint.end),
    ]
    return (
        [
            splitting_road,
            intermittend_road,
            exit_road,
            continue_road,
            continue_internal,
            split_internal,
        ],
        [exit_junction],
        glueable_roads,
        continue_id + 1,
        internal_id + 2,
        junction_id + 1,
    )


class JunctionSample:
    def __init__(self):
        self.odr = xodr.OpenDrive("scenario")
        self.n_roads = state_variables.np_random_maps.choice(
            [3, 4, 5], p=[0.3, 0.5, 0.2]
        )
        self.junction_name = "junction 1"
        self.junction_id = 1
        self.internal_id = 100
        self.radius = [
            state_variables.np_random_maps.randint(15, 31) for _ in range(self.n_roads)
        ]
        self.lengths = [
            state_variables.np_random_maps.randint(40, 80) for _ in range(self.n_roads)
        ]
        self.cs_outer = [
            state_variables.np_random_maps.uniform(low=-0.02, high=0.02)
            for _ in range(self.n_roads)
        ]
        self.cs_inner = [
            state_variables.np_random_maps.uniform(low=-0.02, high=0.02)
            for _ in range(self.n_roads)
        ]
        self.n_lanes = [
            [
                state_variables.np_random_maps.choice([1, 2, 3], p=[0.45, 0.45, 0.1]),
                state_variables.np_random_maps.choice([1, 2, 3], p=[0.45, 0.45, 0.1]),
            ]
            for _ in range(self.n_roads)
        ]
        self.angles = [
            np.radians(
                i * 360 / self.n_roads
                + state_variables.np_random_maps.randint(
                    -250.0 / self.n_roads / 2.0, 250.0 / self.n_roads / 2.0
                )
            )
            for i in range(self.n_roads)
        ]


class HighwayEntrySample:
    def __init__(self):
        curvature_range = [-0.03, 0.03]
        self.c1 = state_variables.np_random_maps.uniform(low=-0.01, high=0.01)
        self.c2 = state_variables.np_random_maps.uniform(low=-0.005, high=0.005)
        self.c3 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )
        self.c_merge = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=self.c1 - 0.01
        )


class HighwayDriveSample:
    def __init__(self):
        curvature_range = [-0.015, 0.015]
        self.c1 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )
        self.c2 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )
        self.c3 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )
        self.c4 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )


class HighwayExitSample:
    def __init__(self):
        curvature_range = [-0.03, 0.03]
        self.c1 = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=curvature_range[1]
        )
        self.c2 = state_variables.np_random_maps.uniform(low=-0.005, high=0.005)
        self.c3 = state_variables.np_random_maps.uniform(low=-0.01, high=0.01)
        self.c_exit = state_variables.np_random_maps.uniform(
            low=curvature_range[0], high=self.c3 - 0.01
        )


class HighwaySample:
    def __init__(self):
        self.n_lanes = [
            state_variables.np_random_maps.choice([1, 2, 3, 4], p=[0.1, 0.4, 0.4, 0.1]),
            state_variables.np_random_maps.choice([1, 2, 3, 4], p=[0.1, 0.4, 0.4, 0.1]),
        ]
        self.entry = HighwayEntrySample()
        self.drive = HighwayDriveSample()
        self.exit = HighwayExitSample()


if __name__ == "__main__":
    odr = xodr.OpenDrive("scenario")
    jd = JunctionSample()
    odr, roads, internal_id, zero_cloth = create_junction(jd)
    odr.write_xml(os.path.basename(__file__).replace(".py", ".xodr"))
    os.system(
        f"{os.path.join('C:/Users/seegras/Portables/esmini-2.0.15-1067/bin/odrviewer.exe')} --odr driver_dojo_junction.xodr"
    )
