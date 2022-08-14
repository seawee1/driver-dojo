import numpy as np
import sumolib
from sumolib.net.generator.network import *
from driver_dojo.common import runtime_vars
from pyclothoids import Clothoid


def build(net, netName="net.net.xml"):
    # Taken from https://github.com/eclipse/sumo/blob/main/tools/sumolib/net/generator/network.py
    connections = []
    nodesFile = tempfile.NamedTemporaryFile(mode="w", delete=False)
    print("<nodes>", file=nodesFile)
    for nid in net._nodes:
        n = net._nodes[nid]
        print(
            '    <node id="%s" x="%s" y="%s" type="%s"/>'
            % (n.nid, n.x, n.y, n.nodeType),
            file=nodesFile,
        )
    print("</nodes>", file=nodesFile)
    nodesFile.close()

    edgesFile = tempfile.NamedTemporaryFile(mode="w", delete=False)
    print("<edges>", file=edgesFile)
    for eid in net._edges:
        e = net._edges[eid]
        print(
            '    <edge id="%s" from="%s" to="%s" numLanes="%s" speed="%s" shape="%s">'
            % (e.eid, e.fromNode.nid, e.toNode.nid, e.numLanes, e.maxSpeed, e.shapes,),
            file=edgesFile,
        )
        for s in e.splits:
            print(
                '        <split pos="%s" lanes="%s"/>'
                % (-s.distance, " ".join(map(str, s.lanes))),
                file=edgesFile,
            )

        """
    for i,l in enumerate(e.lanes):
        if l.allowed==None and l.disallowed==None:
            continue
        ls =  '        <lane index="%s" ' % (i)
        if l.allowed!=None:
            ls = ls + 'allow="%s"' % l.allowed
        if l.disallowed!=None:
            ls = ls + 'disallow="%s"' % l.disallowed
        print >> edgesFile, ls+'/>'
    """

        connections.extend(e.getConnections(net))
        print("    </edge>", file=edgesFile)

        hadConstraints = False
        for i, l in enumerate(e.lanes):
            if l.allowed is None and l.disallowed is None:
                continue
            hadConstraints = True
        if hadConstraints:
            for s in e.splits:
                eid = e.eid
                if s.distance != 0:
                    eid = eid + ".%s" % -s.distance
                print('    <edge id="%s">' % (eid), file=edgesFile)
                for i, l in enumerate(e.lanes):
                    # if i not in s.lanes:
                    #    continue
                    if l.allowed is None and l.disallowed is None:
                        continue
                    ls = '        <lane index="%s" ' % (i)
                    if l.allowed is not None:
                        ls = ls + 'allow="%s"' % l.allowed
                    if l.disallowed is not None:
                        ls = ls + 'disallow="%s"' % l.disallowed
                    print(ls + "/>", file=edgesFile)
                print("    </edge>", file=edgesFile)

    print("</edges>", file=edgesFile)
    edgesFile.close()

    connectionsFile = tempfile.NamedTemporaryFile(mode="w", delete=False)
    print("<connections>", file=connectionsFile)
    for c in connections:
        eid = c.fromEdge.eid
        if len(c.fromEdge.splits) > 1:
            eid = eid + ".-" + str(c.fromEdge.splits[-1].distance)
        print(
            '    <connection from="%s" to="%s" fromLane="%s" toLane="%s"/>'
            % (eid, c.toEdge.eid, c.fromLane, c.toLane),
            file=connectionsFile,
        )
    for n in net._nodes:
        if len(net._nodes[n].crossings) == 0:
            continue
        for c in net._nodes[n].crossings:
            print(
                '    <crossing node="%s" edges="%s"/>' % (n, " ".join(c)),
                file=connectionsFile,
            )
    print("</connections>", file=connectionsFile)
    connectionsFile.close()

    netconvert = sumolib.checkBinary("netconvert")

    subprocess.call(
        [
            netconvert,
            "-v",
            "-n",
            nodesFile.name,
            "-e",
            edgesFile.name,
            "-x",
            connectionsFile.name,
            "-o",
            netName,
            #"--default.lanewidth",
            #"3.6",
            #"--geometry.max-grade.fix",
            #"true",
            #"--plain.extend-edge-shape",
            #"true",
            "--no-turnarounds",
            "true",  # Who need them anyways?
            #"--default.junctions.keep-clear",
            #"false",
            #"--keep-nodes-unregulated",
            #"false",
            #"--geometry.min-radius.fix",
            #"true",
            #"--check-lane-foes.all",
            #"true",
            #'--edges.join', 'true',
            #'--junctions.join', 'true',
            #'--rectangular-lane-cut', 'true',
        ]
    )
    import time
    time.sleep(1)
    os.remove(nodesFile.name)
    os.remove(edgesFile.name)
    os.remove(connectionsFile.name)
    net.netName = netName
    return netName


def build_edge(e_id, n1, n2, numLanes, lanes, maxSpeed=13.333):
    e = Edge(e_id, n1, n2, numLanes=numLanes, maxSpeed=maxSpeed, lanes=lanes)
    return e


def create_roundabout(radius, num_lanes, internal_lanes, rads_incident, angles, road_cs, lengths, squeeze):
    defaultEdge = Edge(numLanes=2, maxSpeed=13.0)
    net = Net(None, defaultEdge)

    perturb_std = 0.000
    nodes = []

    def sample_arc(rad_a, rad_b):
        while rad_a < 0:
            rad_a += 2 * np.pi
        while rad_b < 0:
            rad_b += 2 * np.pi

        rad_amount = abs(np.arctan2(np.sin(rad_a - rad_b), np.cos(rad_a - rad_b)))

        num = int(rad_amount / (2*np.pi / 60.0))
        rads = np.linspace(rad_a, rad_a + rad_amount, endpoint=True, num=num)
        xs, ys = [], []
        for rad in rads:
            x = radius * np.cos(rad) #+ runtime_vars.np_random_maps.normal(
                #0.0, perturb_std
            #)
            x *= squeeze[0]
            y = radius * np.sin(rad) #+ runtime_vars.np_random_maps.normal(
                #0.0, perturb_std
            #)
            y *= squeeze[1]
            xs.append(x), ys.append(y)

        return xs, ys

    # Create inner nodes
    for i, rad in enumerate(rads_incident):
        x = radius * np.cos(rad)
        y = radius * np.sin(rad)
        x *= squeeze[0]
        y *= squeeze[1]
        node = Node(f"Node{i}", x, y, "priority")
        nodes.append(node)
        net.addNode(node)

    # Inner edges
    edges = []
    for i in range(len(nodes)):
        j = 0 if i + 1 == len(nodes) else i + 1
        lanes = [Lane() for i in range(internal_lanes)]
        e = build_edge(f"{i}", nodes[i], nodes[j], internal_lanes, lanes)
        # e = net.buildEdge(nodes[i], nodes[j])
        edges.append(e)

    in_edges = []
    out_edges = []
    for i in range(len(nodes)):
        # Edges leading to outside
        clothoid = Clothoid.StandardParams(
            nodes[i].x, nodes[i].y, rads_incident[i] + angles[i], 0.0, road_cs[i], lengths[i]
        )
        xs, ys = clothoid.SampleXY(10)

        xs = [x * squeeze[0] for x in xs]
        ys = [y * squeeze[1] for y in ys]

        # x = (radius + lengths[i]) * np.cos(rads_incident[i] + angles[i])
        # y = (radius + lengths[i]) * np.sin(rads_incident[i] + angles[i])

        x = xs[-1]
        y = ys[-1]

        in_node = Node(f"InNode{i}", x, y, "priority")
        net.addNode(in_node)

        if num_lanes[i][0] > 0:
            lanes = [Lane() for lane in range(num_lanes[i][0])]
            e = build_edge(f"in{i}", in_node, nodes[i], num_lanes[i][0], lanes)
            e.shapes = [
                f"{round(x, 2)},{round(y, 2)}" for x, y in zip(xs[::-1], ys[::-1])
            ]
            e.shapes = " ".join(e.shapes)
            in_edges.append(e)
            net.addEdge(e)

        if num_lanes[i][1] > 0:
            lanes = [Lane() for lane in range(num_lanes[i][0])]
            e = build_edge(f"out{i}", nodes[i], in_node, num_lanes[i][1], lanes)
            e.shapes = [f"{round(x, 2)},{round(y, 2)}" for x, y in zip(xs, ys)]
            e.shapes = " ".join(e.shapes)
            out_edges.append(e)
            net.addEdge(e)

    for i, e in enumerate(edges):
        j = i + 1
        if j == len(rads_incident):
            j = 0

        xs, ys = sample_arc(rads_incident[i], rads_incident[j])
        e.shapes = [f"{x},{y}" for x, y in zip(xs, ys)]
        e.shapes = " ".join(e.shapes)
        net.addEdge(e)

    build(net, netName=runtime_vars.config.simulation.net_path)


class RoundaboutSample:
    def __init__(self):
        self.num_incident = runtime_vars.np_random_maps.randint(3, 6)
        self.radius = runtime_vars.np_random_maps.uniform(10.0, 30.0)  # TODO: This and next changed
        self.radius = max(self.num_incident*11.0, self.radius)
        self.num_lanes = [
            [
                runtime_vars.np_random_maps.randint(1, 3),
                runtime_vars.np_random_maps.randint(1, 3),
            ]
            for _ in range(self.num_incident)
        ]
        angle_mean = 0.0
        angle_std = 0.0 #0.15  # TODO: This changed
        self.angles = [
            runtime_vars.np_random_maps.normal(angle_mean, angle_std)
            for i in range(self.num_incident)
        ]
        incident_std = 0.15  # This changed
        def draw_rads():
            rads_incident = [
                i * 2 * np.pi / self.num_incident
                + runtime_vars.np_random_maps.normal(0.0, incident_std)
                for i in range(self.num_incident)
            ]
            return rads_incident
        def check_rads(rads):
            for i in range(len(rads)):
                j = i+1
                if i == len(rads) - 1:
                    j = 0
                if np.abs(rads[i] - rads[j]) < (np.pi / 4):
                    return False
            return True
        while True:
            self.rads_incident = draw_rads()
            if check_rads(self.rads_incident):
                break

        self.internal_lanes = runtime_vars.np_random_maps.randint(1, 3)
        self.lengths = [
            runtime_vars.np_random_maps.randint(70, 120) for i in range(self.num_incident)
        ]
        self.squeeze = [
            runtime_vars.np_random_maps.uniform(0.8, 1.2), # This changed
            runtime_vars.np_random_maps.uniform(0.8, 1.2),
        ]
        self.road_cs = [runtime_vars.np_random_maps.uniform(-0.0003, 0.0003) for i in range(self.num_incident)]
