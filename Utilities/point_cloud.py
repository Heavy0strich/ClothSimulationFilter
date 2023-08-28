import numpy as np
import open3d as o3d

class point_cloud_formatting():
    def __init__(self, data_path):
        # Removing the intensity column from the dataset
        self.format = 3 # (x, y, z, r, g, b)
        self.downsample = True
        if data_path != None:
            self.arr = np.loadtxt(data_path)
        else:
            pass

    def o3d_format_from_file(self):
        pcd = np.zeros((np.shape(self.arr)[0], self.format))
        colors = np.zeros((np.shape(self.arr)[0], self.format))
        max_entries = np.shape(self.arr)[1]
        for i in range(self.format):
            pcd[:, i] = self.arr[:, i] 
            #colors[:, i] = self.arr[:, i+max_entries-2] * 0.00392156862   # Converting pixel values to float [0, 1]
        
        #print('pcd from o3d', np.shape(pcd))
        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(pcd)
        #pcd_combined.colors = o3d.utility.Vector3dVector(colors)
        if self.downsample:
            # Downsample Point cloud
            down_pcd = pcd_combined.voxel_down_sample(voxel_size = 0.05)
            return down_pcd
        else:
            return pcd_combined
        
    def o3d_format_from_array(self, arr):
        pcd = np.zeros((np.shape(arr)[0], self.format))
        colors = np.zeros((np.shape(arr)[0], self.format))
        arr_shape = np.shape(arr)[1]
        if arr_shape == 3:
            for i in range(self.format):
                pcd[:, i] = arr[:, i]
        else:
            for i in range(self.format):
                pcd[:, i] = arr[:, i] 
                colors[:, i] = arr[:, i+4] * 0.00392156862   # Converting pixel values to float [0, 1]
        
        #print('pcd from o3d', np.shape(pcd))
        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(pcd)
        #pcd_combined.colors = o3d.utility.Vector3dVector(colors)
        if self.downsample:
            # Downsample Point cloud
            down_pcd = pcd_combined.voxel_down_sample(voxel_size = 0.03)
            return down_pcd
        else:
            return pcd_combined
    
class point_cloud_data:
    def __init__(self, pcd):
        self.pcd = pcd

    def get_copy(self):
        pcd_copy = o3d.geometry.PointCloud()
        pcd_copy.points = o3d.utility.Vector3dVector(np.asarray(self.pcd.points))
        return pcd_copy
    
    def invert_point_cloud(self):
        R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])  # Finding image about xy plane
        pcd_copy = self.get_copy()
        inverted_pcd = pcd_copy.rotate(R)
        return inverted_pcd
    
    def get_points(self):
        return np.asarray(self.pcd.points)
    
    def get_bounding_XY(self):
        # Get bounds on point cloud data(min-max x and y coordinates to get the xy plane)
        x_min       = np.min(np.asarray(self.pcd.points)[:, 0])
        x_max       = np.max(np.asarray(self.pcd.points)[:, 0])
        y_min       = np.min(np.asarray(self.pcd.points)[:, 1])
        y_max       = np.max(np.asarray(self.pcd.points)[:, 1])
        z_max       = np.max(np.asarray(self.pcd.points)[:, 2])

        return [x_min, x_max, y_min, y_max, z_max]