import sys

import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map
from lidar import LidarPoint, LidarPointType

np.random.seed(0)
world = Map((1000, 1000, 4), 0.02, False, 5)
robot = Robot(world.get_center())

vis = o3d.visualization.Visualizer()
vis.create_window()

t = world.cell_size

robot_box = o3d.geometry.TriangleMesh.create_sphere(0.1)
robot_box.paint_uniform_color([0.9, 0.1, 0.1])
robot_box.compute_vertex_normals()
vis.add_geometry(robot_box)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([])
vis.add_geometry(pcd)

e = 0
N = 50
rot, pos = None, None
keep_running = True
with open(sys.argv[1], "r") as file:
    data = json.load(file)["data"]
    for measurement in data["measurements"]:
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
            robot.apply_odometry(pos, rot, world, lidar_data)

        pos = np.array(measurement["odometry"]["position"])
        rot = np.array(measurement["odometry"]["euler_angles"])

        world.update(robot, lidar_data)

        e += 1
        print(e)

        # if e < N:
        #     continue

        ###############################

        robot_box.translate(robot.position, relative=False)
        vis.update_geometry(robot_box)

        res, colors = [], []
        for x in range(world.size[0]):
            for y in range(world.size[1]):
                for z in range(world.size[2]):
                    p = world.get_cell((x, y, z)).get_p()
                    if p < 0.6:
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

        if e > N:
            break

    while keep_running:
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()
