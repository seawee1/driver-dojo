from scenariogeneration import xodr
from scenariogeneration.xodr import Junction, Spiral, Connection, ContactPoint
import pyclothoids as pcloth
import numpy as np

from driver_dojo.scenarios.basic_scenarios import ScenarioGenScenario
from driver_dojo.scenarios.utils import create_road_start, create_3cloths_right_lane


class IntersectionScenario(ScenarioGenScenario):
    def _generate_map(self, parameters, road_id=1, internal_id=100, junction_id=1):
        n_roads, radius, lengths, cs_outer, cs_inner, n_lanes, angles = parameters

        junction = Junction(f"junction {junction_id}", junction_id)
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

        #glueable_roads = [(road, xodr.ContactPoint.end) for road in roads]
        return roads + junction_roads, [junction], zero_cloth

    def sample_map_parameters(self, rng):
        n_roads = rng.choice(
            [3, 4, 5], p=[0.3, 0.5, 0.2]
        )
        # junction_name = "junction 1"
        # self.junction_id = 1
        # self.internal_id = 100
        radius = [
            rng.integers(15, 31) for _ in range(n_roads)
        ]
        lengths = [
            rng.integers(40, 80) for _ in range(n_roads)
        ]
        cs_outer = [
            rng.uniform(low=-0.02, high=0.02)
            for _ in range(n_roads)
        ]
        cs_inner = [
            rng.uniform(low=-0.02, high=0.02)
            for _ in range(n_roads)
        ]
        n_lanes = [
            [
                rng.choice([1, 2, 3], p=[0.45, 0.45, 0.1]),
                rng.choice([1, 2, 3], p=[0.45, 0.45, 0.1]),
            ]
            for _ in range(n_roads)
        ]
        angles = [
            np.radians(
                i * 365 / n_roads
                + rng.integers(
                    -250.0 / n_roads / 2.0, 250.0 / n_roads / 2.0
                )
            )
            for i in range(n_roads)
        ]

        return n_roads, radius, lengths, cs_outer, cs_inner, n_lanes, angles


if __name__ == '__main__':
    import numpy as np

    sec = IntersectionScenario()
    rng = np.random.default_rng()
    y = sec.sample_map_parameters(rng)
    sec.generate(y)
    print(y)
