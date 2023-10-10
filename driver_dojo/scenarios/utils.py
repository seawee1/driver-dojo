import ntpath
import xml.etree.cElementTree as ET

from scenariogeneration import xodr
from scenariogeneration.xodr.lane import Lane, RoadMark, LaneSection, Lanes, RoadLine
from scenariogeneration.xodr.enumerations import (
    RoadMarkType,
)
from scenariogeneration.xodr.geometry import Line, Spiral, PlanView
from scenariogeneration.xodr.opendrive import Road

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
        lane = standard_lane(lane_offset)
        lane.add_roadmark(xodr.STD_ROADMARK_BROKEN)
        lanesec.add_left_lane(lane)
    for i in range(n_lanes[1]):
        lane = standard_lane(lane_offset)
        lane.add_roadmark(xodr.STD_ROADMARK_SOLID)
        lanesec.add_right_lane(lane)

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


def write_sumocfg(sumocfg_path, net_path, rou_path=None):
    # TODO
    net_path = ntpath.basename(net_path)
    root = ET.Element("configuration")
    inp = ET.SubElement(root, "input")
    ET.SubElement(
        inp, "net-file", value=net_path
    )
    if rou_path != '':
        rou_path = ntpath.basename(rou_path)
        ET.SubElement(
            inp, "route-files", value=rou_path
        )
    tree = ET.ElementTree(root)
    tree.write(sumocfg_path)


def write_scenariogen_xodr(roads, junctions, xodr_path):
    odr = xodr.OpenDrive("scenario")
    for road in roads:
        odr.add_road(road)
    for junction in junctions:
        odr.add_junction(junction)
    odr.adjust_roads_and_lanes()
    odr.write_xml(xodr_path)
