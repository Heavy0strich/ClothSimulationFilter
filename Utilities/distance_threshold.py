import numpy as np
from math import fabs

class c2c_distance:
    def __init__(self, class_threshold):
        self.threshold = class_threshold

    def calculate_distance(self, cloth, pcd):
        ground_indices = []
        non_ground_indices = []

        point_cloud_size = np.shape(pcd)[0]

        for i in range(point_cloud_size):
            pcd_x = pcd[i][0]
            pcd_y = pcd[i][1]

            delta_X = pcd_x - cloth.origin_pos[0]
            delta_Y = pcd_y - cloth.origin_pos[1]

            col0    = int(delta_X /cloth.step_x)
            row0    = int(delta_Y /cloth.step_y)

            col1    = col0 + 1
            row1    = row0

            col2    = col0 + 1
            row2    = row0 + 1

            col3    = col0
            row3    = row0 + 1

            subdelta_X = (delta_X - col0 * cloth.step_x)/ cloth.step_x
            subdelta_Y = (delta_Y - row0 * cloth.step_y)/ cloth.step_y

            fxy = cloth.getParticle(col0, row0).get_position()[2] * (1- subdelta_X) * (1 - subdelta_Y) + cloth.getParticle(col3, row3).get_position()[2] * (1 - subdelta_X) * subdelta_Y + cloth.getParticle(col2, row2).get_position()[2] * subdelta_X * subdelta_Y + cloth.getParticle(col1, row1).get_position()[2] * subdelta_X * (1- subdelta_Y)

            height_var = fxy - pcd[i][2]

            if fabs(height_var) < self.threshold:
                ground_indices.append(i)
            else:
                non_ground_indices.append(i)

        return ground_indices, non_ground_indices
