import os
import sys
sys.path.append('C:\\Users\\tiwar\\Desktop\\Project\\ACL_Thesis\\Thesis\\point_cloud_filtering\\Utilities')

import open3d as o3d
from Utilities import csf, cloth, particle, rasterization, point_cloud
import numpy as np
import matplotlib.pyplot as plt

wd = os.getcwd()
print("\nworking directory", wd)
dataset_dir = wd + '\..\..\Datasets\marketplacefeldkirch_station4_intensity_rgb\marketplacefeldkirch_station4_intensity_rgb.txt'      # WINDOWS
print("\n>>>>>>>> DATA set directory.....\n", dataset_dir)

# input point cloud
data = point_cloud.point_cloud_formatting(dataset_dir)
pcd_points = point_cloud.point_cloud_data(data.o3d_format_from_file())

# Inverting Point Cloud
inverted_pcd =  point_cloud.point_cloud_data(pcd_points.invert_point_cloud())

# Initilizing the CSF parameters
csf_params = csf.CSF_PARAMS(False, 0.05, 0.05, 0.5, 2, 1000, 0.5)

csf_filter = csf.CSF(csf_params)

cl = csf_filter.do_filtering(inverted_pcd.pcd, inverted_pcd.get_bounding_XY(), True)

print("\n>>>>>>>>> Filtering Completed.....")   

ground_pcd = o3d.geometry.PointCloud()
ground_pcd = pcd_points.pcd.select_by_index(csf_filter.ground_points)
ground_pcd.paint_uniform_color([1, 0, 0])

non_ground_pcd = o3d.geometry.PointCloud()
non_ground_pcd = pcd_points.pcd.select_by_index(csf_filter.non_ground_points)
non_ground_pcd.paint_uniform_color([0, 1, 0])

o3d.visualization.draw_geometries([ground_pcd, non_ground_pcd])

cloth_pcd_list = []
num_particles = len(cl.cloth_parameters.particles)
for i in range(num_particles):
    cloth_pcd_list.append(cl.cloth_parameters.particles[i].get_position())

cloth_data = point_cloud.point_cloud_formatting(None)
cloth_data.downsample = False
cloth_pcd = cloth_data.o3d_format_from_array(np.asarray(cloth_pcd_list))
cloth_pcd.paint_uniform_color([0, 0, 0])


o3d.visualization.draw_geometries([cloth_pcd, inverted_pcd.pcd])

"""
num_particles = len(cl.cloth_parameters.particles)
nearest_pc_points_id = []
for i in range(num_particles):
    nearest_pc_points_id.append(cl.cloth_parameters.particles[i].nearest_point_index)

ground_id = []
[ground_id.extend(x) for x in nearest_pc_points_id if (x != [] and x not in ground_id)]
                  
data = point_cloud.point_cloud(dataset_dir)

pcd = data.o3d_format_from_file()
ground = pcd.select_by_index(ground_id)
ground.paint_uniform_color([0, 0, 0])

non_ground = pcd.select_by_index(ground_id, invert=True)
non_ground.paint_uniform_color([0.6, 0.6, 0.6])
print(">>>>>>>>> Visualizing data.....")
o3d.visualization.draw_geometries([ground, non_ground])
"""