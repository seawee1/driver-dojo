import argparse
import shapely.geometry
import sumolib
from shapely import geometry, affinity
from pathlib import Path
from shapely.ops import unary_union
import matplotlib.pyplot as plt

from driver_dojo.common.utils import lane_to_polygon


# This is an adapted version of driver_dojo.common.utils.net_to_polygon
def net_to_polygons(
        net,
        fuse_lanes=True,
        fuse_all=False,
        debug_plt=False,
        with_internal=True,
        show=False,
):
    fig = plt.figure(figsize=(20, 20))
    # fig.set_dpi(300)
    ax = fig.add_subplot(111)

    # Edges first
    edges = net.getEdges(withInternal=True)
    polys = []
    for i, e in enumerate(edges):
        # Get all the lanes of an edge
        lanes = e.getLanes()
        # Create polygon for each lane
        lane_polys = []
        for lane in lanes:
            lane_polys.append(lane_to_polygon(lane.getShape(), lane.getWidth()))

        if fuse_lanes:
            # Dilate each lane polygon by 2 meter
            lane_polys = [
                lane_poly.buffer(3) for lane_poly in lane_polys if lane_poly.is_valid
            ]
            # Fuse all lanes into one polygon
            edge_poly = unary_union(lane_polys)
            if isinstance(edge_poly, geometry.Polygon):
                polys.append(edge_poly.buffer(-3))  # Erode again by 1 meter
            else:
                polys += [poly.buffer(-3) for poly in edge_poly.geoms]
        else:
            polys += [lane_poly for lane_poly in lane_polys]

    # Get the hull of each junction, create polygon
    nodes = net.getNodes()
    for n in nodes:
        if len(n.getShape()) < 3:
            continue
        polys.append(geometry.Polygon(n.getShape()))

    # Fuse into one polygon
    if fuse_all:
        # Dilate each lane polygon by 1 meter
        polys = [poly.buffer(2) for poly in polys if poly.is_valid]
        # Fuse all lanes into one polygon
        polys = [unary_union(polys)]
        polys = [poly.buffer(-2) for poly in polys if poly.is_valid]

    # Plot the street network polygons
    for p in polys:
        if isinstance(p, shapely.geometry.MultiPolygon):
            return
        xs, ys = zip(*p.exterior.coords)
        ax.plot(xs, ys, color="black", linewidth=2)
        ax.fill(xs, ys, color=(0.78125, 0.78125, 0.78125, 0.5))

    def plot_lane_left_marking(lane, linestyle, linewidth=1):
        left_marking, right_marking = lane_to_polygon(
            lane.getShape(), lane.getWidth(), left_right=True
        )
        xs, ys = zip(*left_marking)
        ax.plot(
            xs, ys, linestyle=linestyle, color="black", marker="", linewidth=linewidth
        )

    def plot_lane_right_marking(lane, linestyle, linewidth=1):
        left_marking, right_marking = lane_to_polygon(
            lane.getShape(), lane.getWidth(), left_right=True
        )
        xs, ys = zip(*right_marking)
        ax.plot(
            xs, ys, linestyle=linestyle, color="black", marker="", linewidth=linewidth
        )

    # Plot the dashed lane markings
    # Main roads
    edges = net.getEdges(withInternal=with_internal)
    for i, e in enumerate(edges):
        for j, lane in enumerate(e.getLanes()):
            if lane.getID()[0] == ":":
                plot_lane_left_marking(lane, "--", linewidth=1)
                plot_lane_right_marking(lane, "--", linewidth=1)
            else:
                if "-" in e.getID():
                    if lane.getIndex() == 0:
                        plot_lane_right_marking(lane, "-", linewidth=1)
                    else:
                        plot_lane_right_marking(lane, "--")
                else:
                    if lane.getIndex() == len(e.getLanes()) - 1:
                        plot_lane_left_marking(lane, "-", linewidth=1)
                    else:
                        plot_lane_left_marking(lane, "--")

    plt.axis("off")
    fig.gca().set_aspect("equal", adjustable="box")
    if show:
        plt.show()
    return polys


if __name__ == "__main__":
    # TODO: Buggy for roundabout
    parser = argparse.ArgumentParser("Plots an .net.xml file.")
    parser.add_argument(
        "net_dir", type=str, help="Directory with a bunch of .net.xml files."
    )
    parser.add_argument(
        "--withInternal", action="store_true", help="With internal lanes or not."
    )
    parser.add_argument("--show", action="store_true", help="Show instead of save.")
    args = parser.parse_args()

    for file in Path(args.net_dir).glob("*.net.xml"):
        net = sumolib.net.readNet(str(file), withInternal=True)
        net_to_polygons(
            net, fuse_all=True, with_internal=args.withInternal, show=args.show,
        )
        print(f"Save figure to {str(file).split('.')[0]}.pdf")
        if not args.show:
            plt.savefig(
                f"{str(file).split('.')[0]}.pdf",
                bbox_inches="tight",
                pad_inches=0,
                transparent=True,
            )
