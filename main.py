import time

import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map
from lidar import LidarPoint, LidarPointType

world = Map((250, 250, 120), 0.15)
robot = Robot(world.get_center())
gRobot = Robot(world.get_center())

# vis = o3d.visualization.Visualizer()
# vis.create_window()

t = world.cell_size
x, y, z = world.world2map(robot.position)
# tr = [robot.position]
# lines = []
# l_colors = [[0, 1, 0]]

mse_pos = 0
mse_rot = 0
N = 0

start = time.time()

# line_set = o3d.geometry.LineSet()
# vis.add_geometry(line_set)
#
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector([])
# vis.add_geometry(pcd)
#
# robot_box = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
# robot_box.compute_vertex_normals()
# robot_box.paint_uniform_color([1, 0, 0])
# vis.add_geometry(robot_box)
#
# gRobot_box = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
# gRobot_box.compute_vertex_normals()
# gRobot_box.paint_uniform_color([0, 0, 1])
# vis.add_geometry(gRobot_box)

e = 0
rot, pos = None, None
with open("data.json", "r") as file:
    with open("result.json") as gFile:
        data = json.load(file)["data"]
        gData = json.load(gFile)["data"]
        stt = time.time()
        for measurement, gMeasurement in zip(data["measurements"], gData["measurements"]):
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
                gRobot.apply_true(gPos, gRot)
                N += 1
                mse_pos += np.linalg.norm(robot.position - gRobot.position) ** 2

            pos = np.array(measurement["odometry"]["position"])
            rot = np.array(measurement["odometry"]["euler_angles"])

            gPos = np.array(gMeasurement["odometry"]["position"])
            gRot = np.array(gMeasurement["odometry"]["euler_angles"])

            world.update(robot, lidar_data)

            ###############################

            e += 1
            print(e)
            if e > 100:
                break

            # res, colors = [], []
            # tr.append(robot.position)
            # lines.append([len(tr) - 2, len(tr) - 1])
            # l_colors.append([0, 1, 0])
            # robot_box.translate(tr[-1], relative=False)
            # gRobot_box.translate(gRobot.position, relative=False)
            # for x in range(world.size[0]):
            #     for y in range(world.size[1]):
            #         for z in range(world.size[2]):
            #             p = world.get_cell((x, y, z)).p[(0, 1)]
            #             if p < 0.6:
            #                 continue
            #             res.append([x * t, y * t, z * t])
            #             c = 1 - p
            #             colors.append([c, c, c])
            #
            # pcd.points = o3d.utility.Vector3dVector(res)
            # pcd.colors = o3d.utility.Vector3dVector(colors)
            #
            # line_set = o3d.geometry.LineSet(
            #     points=o3d.utility.Vector3dVector(tr),
            #     lines=o3d.utility.Vector2iVector(lines),
            # )
            # line_set.colors = o3d.utility.Vector3dVector(l_colors)
            #
            # vis.remove_geometry(pcd)
            # vis.remove_geometry(line_set)
            # pcd.points = o3d.utility.Vector3dVector(res)
            # vis.add_geometry(pcd)
            # # vis.add_geometry(line_set)
            # vis.update_geometry(robot_box)
            # # vis.update_geometry(gRobot_box)
            #
            # keep_running = vis.poll_events()
            # vis.update_renderer()
            #
            # if e > 100:
            #     while keep_running:
            #         keep_running = vis.poll_events()
            #         vis.update_renderer()
            #
            # e += 1
            # print(e)
            #
            # if not keep_running:
            #     break

rmse = np.sqrt(mse_pos / N)
print(rmse)
print(time.time() - start)
print(time.time() - stt)
# vis.destroy_window()
