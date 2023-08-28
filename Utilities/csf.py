import cloth as cl
import rasterization as rt
import distance_threshold as dist
import numpy as np

class CSF_PARAMS:
    def __init__(self, bSloopSmooth, time_step, class_threshold, cloth_resolution, rigidness, iterations, smooth_threshold):
        self.bSloopSmooth       = bSloopSmooth
        self.time_step          = time_step
        self.height_threshold   = class_threshold
        self.cloth_resolution   = cloth_resolution
        self.rigidness          = rigidness
        self.iterations         = iterations
        self.smooth_threshold   = smooth_threshold

class CSF:
    def __init__(self, params, index = 1):
        self.params = params            # initilizing parameters for CSF algorithm
        self.index  = index
        self.ground_points = []
        self.non_ground_points = []                 

    def do_filtering(self, inverted_pcd, pcd_bounding_XY, export):
        
        print("\n>>>>>>>>> Processing Point Cloud.....")
        [x_min, x_max, y_min, y_max, z_max] = pcd_bounding_XY
        print("\n>>>>>>>>> Cofiguring Terrain.....")
        cloth_z_height = 2 + z_max
        cloth_buffer   = 2

        origin_pos = np.array([
            x_min - cloth_buffer * self.params.cloth_resolution,
            y_min - cloth_buffer * self.params.cloth_resolution,
            z_max + cloth_z_height
        ])

        width_num = int((x_max - x_min)/self.params.cloth_resolution) + 2 * cloth_buffer

        height_num = int((y_max - y_min)/self.params.cloth_resolution) + 2 * cloth_buffer
        print("\n>>>>>>>>> Cloth Resolution:\n", width_num, height_num)
        print("\n>>>>>>>>> Cloth Origin:\n", origin_pos)
        
        print("\n>>>>>>>>> Point Cloud bounding box:\n", x_min, x_max, y_min, y_max, z_max)

        cloth_params = cl.Cloth_Parameters(self.params.iterations, self.params.rigidness, self.params.time_step, self.params.smooth_threshold, self.params.height_threshold)

        cloth = cl.Cloth(cloth_params, origin_pos, self.params.cloth_resolution, self.params.cloth_resolution, width_num, height_num) 
        print("\n>>>>>>>>> Creating Cloth.....\n")
        cloth.create_cloth()
        print(">>>>>>>>> Cloth Bounding box:\n", cloth.get_bounding_box())
        print("\n>>>>>>>>> Rasterizing.....\n")
        raster = rt.Rasterization(cloth, np.asarray(inverted_pcd.points))
        raster.raster_terrain()

        gravity = np.array([0, 0, -9.8])
        print("\n>>>>>>>>> Gravity:\n", gravity)
        print("\n>>>>>>>>> Simulating.....")
        cloth.add_force(gravity)
        EARLY_STOP = False
        for i in range(self.params.iterations):
            max_diff = cloth.timestep()
            cloth.terr_collision()

            if max_diff != 0 and max_diff < 0.00005:
                # early stop
                EARLY_STOP = True
                print("\n>>>>>>>>> No. of iterations: ", i)
                break

        
        if EARLY_STOP:
            print("\n>>>>>>>>> Early Stop Triggered.....")

        if self.params.bSloopSmooth:
            print("\n>>>>>>>>> Smoothing.....")
            cloth.movable_filter()

        thresh = dist.c2c_distance(self.params.height_threshold)
        print("\n>>>>>>>>> Collecting Ground and Non-Ground Points.....")
        self.ground_points, self.non_ground_points = thresh.calculate_distance(cloth, np.asarray(inverted_pcd.points))

        if export:
            print("\n>>>>>>>>> Exporting.....")
            cloth.save_to_file('cloth.txt')
            return cloth