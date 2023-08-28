import numpy as np
from particle import MIN_INF

class Rasterization:
    def __init__(self, cloth, pcd):
        self.cloth  = cloth
        self.pcd    = pcd

    def square_distance(self, p1, p2, p3, p4):
        return (p1 - p3) ** 2 + (p2 - p4) ** 2

    def find_height_value_by_neighbour(self, p):
        nqueue = []
        pbacklist = []

        neighbour_size = len(p.neighbour_list)

        for i in range(neighbour_size):
            p.isvisited = True
            nqueue.insert(0, p.neighbour_list[i])

        # iterate of nqueue
        while len(nqueue) > 0:
            p_neighbour = nqueue.pop()
            pbacklist.append(p_neighbour)

            if p_neighbour.nearest_point_height > MIN_INF:
                for ptc in pbacklist:
                    ptc.isvisited = False
                while len(nqueue) > 0:
                    pp = nqueue.pop()
                    pp.isvisited = False
                return p_neighbour.nearest_point_height
            else:
                n_size = len(p_neighbour.neighbour_list)
                for i in range(n_size):
                    ptmp = p_neighbour.neighbour_list[i]

                    if ptmp.isvisited == False:
                        ptmp.isvisited = True
                        nqueue.insert(0, ptmp)

        return MIN_INF

    def find_height_value_by_scanline(self, p):
        xpos = p.pos_x
        ypos = p.pos_y

        for i in range(xpos + 1, self.cloth.num_particles_width):
            crresHeight = self.cloth.getParticle(i, ypos).nearest_point_height

            if crresHeight > MIN_INF:
                return crresHeight
            
        for i in range(xpos - 1, -1, -1):
            crresHeight = self.cloth.getParticle(i, ypos).nearest_point_height

            if crresHeight > MIN_INF:
                return crresHeight
            
        for i in range(ypos + 1, self.cloth.num_particles_height):
            crresHeight = self.cloth.getParticle(xpos, i).nearest_point_height

            if crresHeight > MIN_INF:
                return crresHeight
            
        for i in range(ypos - 1, -1, -1):
            crresHeight = self.cloth.getParticle(xpos, i).nearest_point_height

            if crresHeight > MIN_INF:
                return crresHeight
        
        return self.find_height_value_by_neighbour(p)
    
    def raster_terrain(self):
        pcd_size = np.shape(self.pcd)[0]

        for i in range(pcd_size):
            pc_x = self.pcd[i][0]
            pc_y = self.pcd[i][1]

            delta_X = pc_x - self.cloth.origin_pos[0]
            delta_Y = pc_y - self.cloth.origin_pos[1]

            col = int(delta_X / self.cloth.step_x + 0.5) 
            row = int(delta_Y / self.cloth.step_y + 0.5)
            # col row are used to get cloth particle near the ith lidar point

            if ((col >= 0) and (row >= 0)):
                pt = self.cloth.getParticle(col, row)
                pt.corresponding_Lidar_point.append(i)
                pc2particle_distance = self.square_distance(pc_x, pc_y, pt.get_position()[0], pt.get_position()[1])

                if pc2particle_distance < pt.tmpDist:
                    pt.tmpDist = pc2particle_distance
                    pt.nearest_point_height = self.pcd[i][2]
                    pt.nearest_point_index.append(i)

        for i in range(self.cloth.get_size()):
            p_cur = self.cloth.getParticle1d(i)
            nearest_height = p_cur.nearest_point_height

            if(nearest_height > MIN_INF):
                self.cloth.height_values.append(nearest_height)
            else:
                self.cloth.height_values.append(self.find_height_value_by_scanline(p_cur))