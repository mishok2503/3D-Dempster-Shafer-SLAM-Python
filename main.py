import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map
from lidar import LidarPoint, LidarPointType

world = Map((250, 250, 200), 0.3)
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

robot_box = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
robot_box.compute_vertex_normals()
robot_box.paint_uniform_color([1, 0, 0])
vis.add_geometry(robot_box)

e = 0
M = 0
rot, pos = None, None
with open("ros.json", "r") as file:
    data = json.load(file)["data"]
    for measurement in data["measurements"]:

        e += 1

        ne = 0
        if e < ne:
            continue
        # if e > 5700:
        #     break

        lidar_data = []
        for point in measurement["lidar_data"]:
            tp = np.array(point["coordinates"])
            point_type = LidarPointType.UNKNOWN
            if point["type"] == "point":
                point_type = LidarPointType.POINT
            elif point["type"] == "maximum":
                point_type = LidarPointType.MAX
                print("MAXIMUM")
            lidar_data.append(LidarPoint(tp, point_type))
        lidar_data = np.array(lidar_data)

        if pos is not None:
            rot = np.zeros(3)
            pos = np.zeros(3)
            if e > ne + 32:
                robot.apply_odometry(pos, rot, world, lidar_data)
            else:
                robot.apply_true(pos, rot)

        pos = np.array(measurement["odometry"]["position"])
        rot = np.array(measurement["odometry"]["euler_angles"])

        world.update(robot, lidar_data)

        if e % 64 != 0:
            continue
        print(e)
        print(robot.rotation)
        print(robot.position)

        ###############################

        res, colors = [], []
        tr.append(robot.position)
        lines.append([len(tr) - 2, len(tr) - 1])
        l_colors.append([0, 1, 0])
        robot_box.translate(tr[-1], relative=False)
        for x in range(world.size[0]):
            for y in range(world.size[1]):
                for z in range(world.size[2]):
                    p = world.get_cell((x, y, z)).p[(0, 1)]
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
        vis.add_geometry(line_set)
        vis.update_geometry(robot_box)

        keep_running = vis.poll_events()
        vis.update_renderer()

    while keep_running:
        keep_running = vis.poll_events()
        vis.update_renderer()

vis.destroy_window()
