
import particle as pt
import numpy as np
import multiprocessing as mp
from math import fabs

MAX_PARTICLE_FOR_POSTPROCESSING  = 50

class Cloth_Parameters:
    def __init__(self, constraint_iterations, rigidness, time_step, smooth_threshold, height_threshold):
        """
        particles                   : all particles that are in the cloth
                                     - list that stores particle objects  
        """
        self.constraint_iterations      = constraint_iterations
        self.rigidness                  = rigidness
        self.time_step                  = time_step
        self.particles                  = []
        self.smooth_threshold           = smooth_threshold
        self.height_threshold           = height_threshold


class Cloth:
    def __init__(self, parameters, origin_pos, step_x, step_y, num_particles_width, num_particles_height):
        """
        height_values               : height values of the particles in the cloth
        """
        self.cloth_parameters           = parameters
        self.origin_pos                 = origin_pos
        self.step_x                     = step_x
        self.step_y                     = step_y
        self.height_values              = []
        self.num_particles_width        = num_particles_width               # Number of particles in the width of the cloth
        self.num_particles_height       = num_particles_height              # Number of particles in the height of the cloth

    def getParticle(self, x, y):
        """
        Returns the particle object at the index given by x and y
            - index = y * num_particles_width + x
        """
        return self.cloth_parameters.particles[y * self.num_particles_width + x]
    def makeConstraint(self, p1, p2):
        """
        Adds the particles to each other's neighbour list
        """
        p1.neighbour_list.append(p2)
        p2.neighbour_list.append(p1)
    
    def get_size(self):
        """
        returns total number of particles in the cloth
        """
        return self.num_particles_width * self.num_particles_height
    
    def get1DIndex(self, x, y):
        """
        x : Column number
        y : Row number
        """
        return y * self.num_particles_width + x
    
    def getHeightvals(self):
        return self.height_values
    
    def getParticle1d(self, index):
        return self.cloth_parameters.particles[index]
    
    def create_cloth(self):
        """
        Creating a cloth grid from the origin_pos in the x and y direction
            - origin_pos will be the left bottom corner of the cloth (When viewed from the top)
            - while creating the cloth, we will assume all the particles in the cloth
            are at same height

        particles is a list storing all the particles in the cloth
            - particles[].pos_x and particles[].pos_y will be used to store the 2D index of 
            the particle on the cloth
        """
        #pos = np.meshgrid(np.arange(self.origin_pos[0], self.num_particles_width, self.step_x), np.arange(self.origin_pos[1], self.num_particles_height, self.step_y))



        for j in range(self.num_particles_height):
            for i in range(self.num_particles_width):
                pos = np.array([self.origin_pos[0] + i * self.step_x, self.origin_pos[1] + j * self.step_y, self.origin_pos[2]])
                self.cloth_parameters.particles.append(pt.Particle(pos, self.cloth_parameters.time_step))
                #print("printing particles in cloth", j + self.num_particles_height * i)
                #print("Particle Position:", (i, j))
                self.cloth_parameters.particles[j * self.num_particles_width + i].pos_x = i
                self.cloth_parameters.particles[j * self.num_particles_width + i].pos_y = j

        # Connecting immediate neighbour particles with constraints
        # distance 1[for edge particles] and sqrt(2)[for diagonal particles] in the grid

        for x in range(self.num_particles_width):
            for y in range(self.num_particles_height):
                if (x < self.num_particles_width - 1):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x + 1, y))

                if(y < self.num_particles_height - 1):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x, y + 1))

                if((x < self.num_particles_width - 1) and (y < self.num_particles_height - 1)):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x + 1, y + 1))

                if((x < self.num_particles_width - 1) and (y < self.num_particles_height - 1)):
                    self.makeConstraint(self.getParticle(x + 1, y), self.getParticle(x, y + 1))

        # Connecting secondary neighbour particles with constraints
        # distance 2 and sqrt(4) in the grid
        for x in range(self.num_particles_width):
            for y in range(self.num_particles_height):
                if (x < self.num_particles_width - 2):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x + 2, y))

                if(y < self.num_particles_height - 2):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x, y + 2))

                if((x < self.num_particles_width - 2) and (y < self.num_particles_height - 2)):
                    self.makeConstraint(self.getParticle(x, y), self.getParticle(x + 2, y + 2))

                if((x < self.num_particles_width - 2) and (y < self.num_particles_height - 2)):
                    self.makeConstraint(self.getParticle(x + 2, y), self.getParticle(x, y + 2))
    

    def timestep(self):
        particle_count = len(self.cloth_parameters.particles)
        for i in range(particle_count):
            self.cloth_parameters.particles[i].timeStep()

        for i in range(particle_count):
            self.cloth_parameters.particles[i].satisfy_constraint_self(self.cloth_parameters.constraint_iterations)
        
        maxDiff = 0.0
        for i in range(particle_count):
            if(self.cloth_parameters.particles[i].isMovable()):
                diff = fabs(self.cloth_parameters.particles[i].old_position[2] - self.cloth_parameters.particles[i].current_position[2])

                if(diff > maxDiff):
                    maxDiff = diff

        return maxDiff
    
    def add_force(self, direction):
        for i in range(len(self.cloth_parameters.particles)):
            self.cloth_parameters.particles[i].add_force(direction)

    def terr_collision(self):
        particle_count = len(self.cloth_parameters.particles)
        for i in range(particle_count):
            v = self.cloth_parameters.particles[i].get_position()

            if(v[2] < self.height_values[i]):
                self.cloth_parameters.particles[i].offset_position(np.array([0, 0, self.height_values[i] - v[2]]))
                self.cloth_parameters.particles[i].make_unmovable()

    def find_unmovable_point(self, connected):
        edge_points = []
        for i in range(len(connected)):
            x = connected[i][0]
            y = connected[i][1]
            index = y * self.num_particles_width + x
            ptc = self.getParticle(x, y)

            if(x > 0):
                ptc_x = self.getParticle(x - 1, y)
                if(ptc_x.isMovable()):
                    index_ref = y * self.num_particles_width + x - 1

                    if((fabs(self.height_values[index] - self.height_values[index_ref]) < self.cloth_parameters.smooth_threshold) and 
                       ptc.get_position()[2] - self.height_values[index] < self.cloth_parameters.height_threshold):
                        offset_vec = np.array([0, 0, self.height_values[index] - ptc.get_position()[2]])
                        self.cloth_parameters.particles[index].offset_position(offset_vec)
                        ptc.make_unmovable()
                        edge_points.append(i)
                        continue


            if(x < self.num_particles_width - 1):
                ptc_x = self.getParticle(x + 1, y)
                if(ptc_x.isMovable()):
                    index_ref = y * self.num_particles_width + x + 1

                    if((fabs(self.height_values[index] - self.height_values[index_ref]) < self.cloth_parameters.smooth_threshold) and 
                       ptc.get_position()[2] - self.height_values[index] < self.cloth_parameters.height_threshold):
                        offset_vec = np.array([0, 0, self.height_values[index] - ptc.get_position()[2]])
                        self.cloth_parameters.particles[index].offset_position(offset_vec)
                        ptc.make_unmovable()
                        edge_points.append(i)
                        continue

            if(y > 0):
                ptc_y = self.getParticle(x, y - 1)
                if(ptc_y.isMovable()):
                    index_ref = (y - 1) * self.num_particles_width + x

                    if((fabs(self.height_values[index] - self.height_values[index_ref]) < self.cloth_parameters.smooth_threshold) and 
                       ptc.get_position()[2] - self.height_values[index] < self.cloth_parameters.height_threshold):
                        offset_vec = np.array([0, 0, self.height_values[index] - ptc.get_position()[2]])
                        self.cloth_parameters.particles[index].offset_position(offset_vec)
                        ptc.make_unmovable()
                        edge_points.append(i)
                        continue

            if(y < self.num_particles_height - 1):
                ptc_y = self.getParticle(x, y + 1)
                if(ptc_y.isMovable()):
                    index_ref = (y + 1) * self.num_particles_width + x

                    if((fabs(self.height_values[index] - self.height_values[index_ref]) < self.cloth_parameters.smooth_threshold) and 
                       ptc.get_position()[2] - self.height_values[index] < self.cloth_parameters.height_threshold):
                        offset_vec = np.array([0, 0, self.height_values[index] - ptc.get_position()[2]])
                        self.cloth_parameters.particles[index].offset_position(offset_vec)
                        ptc.make_unmovable()
                        edge_points.append(i)
                        continue

        return edge_points
    
    def handle_slop_connected(self, edge_points, connected, neighbours):
        visited = []
        for i in range(len(connected)):
            visited.append(False)
        
        queue = []
        for i in range(len(edge_points)):
            queue.insert(0, edge_points[i])
            visited[edge_points[i]] = True

        while(len(queue) != 0):
            index = queue.pop()
            index_center = connected[index][1] * self.num_particles_width + connected[index][0]

            for i in range(len(neighbours[index])):
                index_neighbour = connected[neighbours[index][i]][1] * self.num_particles_width + connected[neighbours[index][i]][0]

                if ((fabs(self.height_values[index_center] - self.height_values[index_neighbour]) < self.cloth_parameters.smooth_threshold) and 
                    (fabs(self.cloth_parameters.particles[index_center].get_position()[2] - self.height_values[index_neighbour]) < self.cloth_parameters.height_threshold)):
                    offset_vec = np.array([0, 0, self.height_values[index_neighbour] - self.cloth_parameters.particles[index_neighbour].get_position()[2]])
                    self.cloth_parameters.particles[index_neighbour].offset_position(offset_vec)
                    self.cloth_parameters.particles[index_neighbour].make_unmovable()

                    if(not visited[neighbours[index][i]]):
                        queue.insert(0, neighbours[index][i])
                        visited[neighbours[index][i]] = True

    def movable_filter(self):
        for x in range(self.num_particles_width):
            for y in range(self.num_particles_height):
                ptc = self.getParticle(x, y)

                if(ptc.isMovable() and not ptc.isvisited):
                    que = []
                    connected = []
                    neighbours = []
                    
                    sum = 1
                    index = y * self.num_particles_width + x
                    connected.append(np.array([x, y]))
                    self.cloth_parameters.particles[index].isvisited = True

                    que.insert(0, index)

                    while(len(que)!= 0):
                        ptc_f = self.cloth_parameters.particles[que.pop()]
                        cur_x = ptc_f.pos_x
                        cur_y = ptc_f.pos_y
                        neighbour = []

                        if(cur_x > 0):
                            ptc_left = self.getParticle(cur_x - 1, cur_y)

                            if(ptc_left.isMovable()):
                                if(not ptc_left.isvisited):
                                    sum += 1
                                    ptc_left.isvisited = True
                                    connected.append(np.array([cur_x - 1, cur_y]))
                                    que.insert(0, self.num_particles_width * cur_y + cur_x - 1)
                                    neighbour.append(sum - 1)
                                    ptc_left.c_pos = sum - 1

                                else:
                                    neighbour.append(ptc_left.c_pos)

                        if(cur_x < self.num_particles_width - 1):
                            ptc_right = self.getParticle(cur_x + 1, cur_y)

                            if(ptc_right.isMovable()):
                                if(not ptc_right.isvisited):
                                    sum += 1
                                    ptc_right.isvisited = True
                                    connected.append(np.array([cur_x + 1, cur_y]))
                                    que.insert(0, self.num_particles_width * cur_y + cur_x + 1)
                                    neighbour.append(sum - 1)
                                    ptc_right.c_pos = sum - 1

                                else:
                                    neighbour.append(ptc_right.c_pos)

                        if(cur_y > 0):
                            ptc_down = self.getParticle(cur_x, cur_y - 1)

                            if(ptc_down.isMovable()):
                                if(not ptc_down.isvisited):
                                    sum += 1
                                    ptc_down.isvisited = True
                                    connected.append(np.array([cur_x, cur_y - 1]))
                                    que.insert(0, self.num_particles_width * (cur_y - 1) + cur_x)
                                    neighbour.append(sum - 1)
                                    ptc_down.c_pos = sum - 1

                                else:
                                    neighbour.append(ptc_down.c_pos)

                        if(cur_y < self.num_particles_height - 1):
                            ptc_up = self.getParticle(cur_x, cur_y + 1)

                            if(ptc_up.isMovable()):
                                if(not ptc_up.isvisited):
                                    sum += 1
                                    ptc_up.isvisited = True
                                    connected.append(np.array([cur_x, cur_y + 1]))
                                    que.insert(0, self.num_particles_width * (cur_y + 1) + cur_x)
                                    neighbour.append(sum - 1)
                                    ptc_up.c_pos = sum - 1

                                else:
                                    neighbour.append(ptc_up.c_pos)

                        neighbours.append(neighbour)
                    if sum > MAX_PARTICLE_FOR_POSTPROCESSING:
                        edge_points = self.find_unmovable_point(connected)
                        self.handle_slop_connected(edge_points, connected, neighbours)

    def save_to_file(self, file_name):
        with open(file_name, 'w') as f:
            for ptc in self.cloth_parameters.particles:
                f.write(str(ptc.get_position()[0]) + " " + str(ptc.get_position()[1]) + " " + str(ptc.get_position()[2]) + "\n")

    def get_bounding_box(self):
        max_x = -100000
        max_y = -100000
        max_z = -100000
        min_x = 100000
        min_y = 100000

        for ptc in self.cloth_parameters.particles:
            
            if(ptc.get_position()[0] > max_x):
                max_x = ptc.get_position()[0]
            if(ptc.get_position()[0] < min_x):
                min_x = ptc.get_position()[0]
            if(ptc.get_position()[1] > max_y):
                max_y = ptc.get_position()[1]
            if(ptc.get_position()[1] < min_y):
                min_y = ptc.get_position()[1]
            if(ptc.get_position()[2] > max_z):
                max_z = ptc.get_position()[2]

        
            
        return (max_x, min_x, max_y, min_y, max_z)

    
                    
    
        


        

