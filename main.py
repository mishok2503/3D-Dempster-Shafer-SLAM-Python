import sys

import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map, get_score
from lidar import LidarPoint, LidarPointType


np.random.seed(0)
world = Map((800, 800, 180), 0.1, True, 1)
robot = Robot(world.get_center())

# world.store("world.json")

vis = o3d.visualization.Visualizer()
vis.create_window()

t = world.cell_size

# robot_box = o3d.geometry.TriangleMesh.create_sphere(0.1)
# robot_box.paint_uniform_color([1, 0, 0])
# vis.add_geometry(robot_box)
# helper = copy.deepcopy(robot_box)
# helper.translate(robot.position - [0, 3, 0 ])
# vis.add_geometry(helper)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([])
vis.add_geometry(pcd)

e = 0
N = 1000
T = 100
rot, pos = None, None
keep_running = True
with open(sys.argv[1], "r") as file:
    data = json.load(file)["data"]

    lines = [[0, i + 1] for i in range(len(data["measurements"][0]["lidar_data"]))]
    colors = [[0, 1, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(line_set)

    for measurement in data["measurements"]:
        e += 1
        print(e)

        if e < T:
            continue
        lidar_data = []
        for point in measurement["lidar_data"]:
            point_type = LidarPointType.UNKNOWN
            if point["type"] == "point":
                point_type = LidarPointType.POINT
            elif point["type"] == "maximum":
                point_type = LidarPointType.MAX
            lidar_data.append(LidarPoint(np.array(point["coordinates"]), point_type))
        lidar_data = np.array(lidar_data)

        if pos is not None:
            robot.apply_odometry(pos, rot, world, lidar_data, 0 if e < T + 100 else 300)

        pos = np.array(measurement["odometry"]["position"])
        rot = np.array(measurement["odometry"]["euler_angles"])

        world.update(robot, lidar_data)

        if e < N + T and e % 50 != 10:
            continue
        # world.store("world.json")

        ###############################

        # robot_box.translate(robot.position, relative=False)
        # vis.update_geometry(robot_box)

        points = [robot.position] + [robot.lidar2world(i.point) for i in lidar_data]
        line_set.points = o3d.utility.Vector3dVector(points)
        vis.remove_geometry(line_set)
        vis.add_geometry(line_set)

        res, colors = [], []
        for x in range(world.size[0]):
            for y in range(world.size[1]):
                for z in range(world.size[2]):
                    p = get_score(world.get_cell((x, y, z)))
                    if p <= 0.5:
                        continue
                    res.append([x * t, y * t, z * t])
                    c = 1 - p
                    colors.append([c, c, c])

        pcd.points = o3d.utility.Vector3dVector(res)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        vis.remove_geometry(pcd)
        vis.add_geometry(pcd)

        keep_running = vis.poll_events()
        vis.update_renderer()

        if e > N + T:
            break

    while keep_running:
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()
