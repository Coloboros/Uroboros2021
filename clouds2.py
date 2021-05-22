import numpy as np
import copy
import open3d as o3d

import os
files = sorted(os.listdir('clouds2'))

vis = o3d.visualization.Visualizer()
vis.create_window()

geometry = o3d.io.read_point_cloud(f'clouds2\\{files[0]}')
vis.add_geometry(geometry)
vc = vis.get_view_control()
vc.rotate(0,500)
vc.rotate(250,0)
vc.rotate(0,250)
num = 500

for i in range(num):
    print(i)
    cloud = o3d.io.read_point_cloud(f'clouds2\\{files[i]}')
    geometry.points = cloud.points
    vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()