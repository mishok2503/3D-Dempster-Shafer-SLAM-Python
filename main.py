import numpy as np
import open3d as o3d
import json

from robot import Robot
from map import Map
from lidar import LidarPoint, LidarPointType

world = Map((300, 300, 80), 0.15)
robot = Robot(world.get_center())

e = 0
with open("result.json", "r") as file:
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

        robot.apply_odometry(
            np.array(measurement["odometry"]["position"]),
            np.array(measurement["odometry"]["euler_angles"])
        )

        world.update(robot, lidar_data)
        if e > 34:
            break
        e += 1
        print(e)

res = []
t = world.cell_size
robot_on_map = world.world2map(robot.position)
for x in range(world.size[0]):
    for y in range(world.size[1]):
        for z in range(world.size[2]):
            p = world.get_cell((x, y, z)).p
            if p < 0.6 and robot_on_map != (x, y, z):
                continue
            mesh_box = o3d.geometry.TriangleMesh.create_box(width=t, height=t, depth=t)
            mesh_box.compute_vertex_normals()
            c = 1 - p
            mesh_box.paint_uniform_color([1, 0, 0] if robot_on_map == (x, y, z) else [c, c, c])
            mesh_box.translate((x * t, y * t, z * t), relative=False)
            res.append(mesh_box)
o3d.visualization.draw_geometries(res)
