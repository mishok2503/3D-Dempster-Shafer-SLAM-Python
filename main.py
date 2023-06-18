import copy
import sys

import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map
from lidar import LidarPoint, LidarPointType

world = Map((250, 250, 200), 0.1)
robot = Robot(world.get_center())

vis = o3d.visualization.Visualizer()
vis.create_window()

t = world.cell_size
x, y, z = world.world2map(robot.position)
tr = [robot.position]
lines = []
l_colors = [[0, 1, 0]]

line_set = o3d.geometry.LineSet()
vis.add_geometry(line_set)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([])
vis.add_geometry(pcd)

# robot_box = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
robot_box = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.05,
                                                   cone_radius=0.1,
                                                   cylinder_height=0.15,
                                                   cone_height=0.1,
                                                   resolution=30,
                                                   cylinder_split=1,
                                                   cone_split=1)
robot_box.rotate(robot_box.get_rotation_matrix_from_xyz((np.pi / 2, np.pi / 2, 0)))
robot_box.compute_vertex_normals()
robot_box.paint_uniform_color([1, 0, 0])
# vis.add_geometry(robot_box)

e = 0
N = 200
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

        # samples = []
        if pos is not None:
            robot.apply_odometry(pos, rot, world, lidar_data)
            # samples = robot.apply_odometry(pos, rot, world, lidar_data)
            # robot.apply_true(pos, rot)

        pos = np.array(measurement["odometry"]["position"])
        rot = np.array(measurement["odometry"]["euler_angles"])

        world.update(robot, lidar_data)

        e += 1
        print(e)

        if e < N:
            continue

        ###############################

        res, colors = [], []
        tr.append(robot.position)
        lines.append([len(tr) - 2, len(tr) - 1])
        l_colors.append([0, 1, 0])
        # rts = [copy.deepcopy(robot_box).translate(r.position, relative=False).rotate(r.get_rotation_matrix()) for r in samples]
        rt = copy.deepcopy(robot_box).translate(tr[-1], relative=False).rotate(robot.get_rotation_matrix())
        # robot_box.translate(tr[-1], relative=False)
        # robot_box.rotate(robot.get_rotation_matrix())
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

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(tr),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(l_colors)

        vis.remove_geometry(pcd)
        vis.remove_geometry(line_set)
        pcd.points = o3d.utility.Vector3dVector(res)
        vis.add_geometry(pcd)
        vis.add_geometry(rt)
        # for qw in rts:
        #     vis.add_geometry(qw)
        vis.add_geometry(line_set)
        # vis.update_geometry(robot_box)

        keep_running = vis.poll_events()
        vis.update_renderer()
        vis.remove_geometry(rt)
        # for qw in rts:
        #     vis.remove_geometry(qw)

        if e > N:
            break

    while keep_running:
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()
