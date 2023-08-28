import numpy as np

DAMPING                    = 0.01
MAX_INF                    = 99999999
MIN_INF                    = -99999999

singleMove1                = np.array([0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322])
doubleMove1                = np.array([0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5])


class Particle:
    def __init__(self, pos,time_step):
        """
        neighbour_list      : list of particle objects that are within the radius of the particle

        current_position    : np.array([x, y, z]) of the particle
        """
        self.movable                            = True
        self.mass                               = 1.0
        self.acceleration                       = np.array([0.0, 0.0, 0.0])
        self.time_step2                         = time_step ** 2
    
        self.current_position                   = pos
        self.old_position                       = pos
        self.isvisited                          = False
        self.neighbour_count                    = 0
        self.pos_x                              = 0                 # used to store the column position of the particle in the cloth
        self.pos_y                              = 0                 # used to store the row position of the particle in the cloth
        self.c_pos                              = 0
    
        self.neighbour_list                     = []
        self.corresponding_Lidar_point          = []
        self.nearest_point_index                = []
        self.nearest_point_height               = MIN_INF
        self.tmpDist                            = MAX_INF

    def isMovable(self):
        return self.movable
    
    def add_force(self, force):
        self.acceleration += force / self.mass

    def get_position(self):
        return self.current_position
    
    def make_unmovable(self):
        self.movable = False
    
    def reset_acceleration(self):
        self.acceleration = np.array([0.0, 0.0, 0.0])

    def offset_position(self, offset):
        self.current_position += offset

    def update(self):
        self.neighbour_count = len(self.neighbour_list)
        
    
    def timeStep(self):
        if self.movable:
            temp = self.current_position
            self.current_position = self.current_position + (self.current_position - self.old_position) * (1.0 - DAMPING) + self.acceleration * self.time_step2
            self.old_position = temp

    def satisfy_constraint_self(self, constraint_times):
        self.update()
        for i in range(self.neighbour_count):
            p2 = self.neighbour_list[i]
            correction_vector = np.array([0, 0, p2.current_position[2] - self.current_position[2]])

            if (p2.isMovable() and self.isMovable()):
                if (constraint_times > 14):
                    correction_vector_half = correction_vector * 0.5
                else:
                    correction_vector_half = correction_vector * doubleMove1[constraint_times]

                self.offset_position(correction_vector_half)
                p2.offset_position(-correction_vector_half)

            elif (self.isMovable() and not p2.isMovable()):
                if (constraint_times > 14):
                    correction_vector_half = correction_vector * 1
                else:
                    correction_vector_half = correction_vector * singleMove1[constraint_times]

                self.offset_position(correction_vector_half)
            
            elif (not self.isMovable() and p2.isMovable()):
                if (constraint_times > 14):
                    correction_vector_half = correction_vector * 1
                else:
                    correction_vector_half = correction_vector * singleMove1[constraint_times]

                p2.offset_position(-correction_vector_half)

        