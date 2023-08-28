import numpy as np
#import open3d as o3d

point_cloud_data = np.loadtxt('Datasets/marketplacefeldkirch_station7_intensity_rgb.txt')

print('Shape of Point_cloud_data', np.size(point_cloud_data))
print(o3d.__version__)
