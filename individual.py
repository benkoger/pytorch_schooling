
import numpy as np
from scipy import ndimage 
import math as m

from rotate import rotate

PI_OVER_180 = np.pi/180.0

# Smallest angle to another vector
def smallest_angle_to(vector, other_vector):

    # this function calculates the smaller of the two angles between two unit vectors that begin at the origin
    vec = vector
    vec = vec / np.linalg.norm(vec)  #the speed is unimportant so the vector can be normalised
    
    dot_product = np.dot(vec, other_vector)
    
    if dot_product > 1.0:
        dot_product = 1.0
    if dot_product < 0.0:
        dot_product = 0.0
    
    val = m.acos(dot_product)
    return (val/PI_OVER_180)

class Individual:


    def __init__(self, r_center, direction, max_turning_rate, 
                 speed, zone_of_attraction, zone_of_orientation,
                 zone_of_repulsion, angular_error_sd,  
                 body_size, state, ko = 0.25, ka = 0.25):

        self.r_center = r_center	
        self.direction = direction 
        self.max_turning_rate = max_turning_rate	
        self.speed = speed
        self.zone_of_attraction = zone_of_attraction 
        self.zone_of_orientation = zone_of_orientation
        self.zone_of_repulsion = zone_of_repulsion 
        self.body_size = body_size 
        self.zop_count = 0
        self.angular_error_sd = angular_error_sd 
        self.desired_direction = np.zeros(2)
        self.zor_count = 0
        self.zoa_count = 0
        self.total_zoa = np.zeros(2)
        self.total_zoo = np.zeros(2)
        self.total_zor = np.zeros(2)
        self.desired_direction = np.zeros(2)
        self.ko = ko
        self.ka = ka
        self.state = state
        


    def normalize(self, vec):
	    vec_mag = np.sqrt(vec[0]**2 + vec[1]**2)
	    vec_norm = vec / vec_mag
	    return vec_norm


    def move(self, arena_size, timestep_inc):	# so we have direction (current direction) and a desired direction
        
        if np.sum(np.abs(self.desired_direction)) >.00001:
            self.turn_towards_vector(timestep_inc)
        self.move_myself(timestep_inc, arena_size)

 

    def move_myself(self, timestep_inc, arena_size):

        self.r_center += self.direction * self.speed * timestep_inc
        
        if self.r_center[0] >= arena_size:
            self.r_center[0] -= arena_size
        if self.r_center[1] >= arena_size: 
            self.r_center[1] -= arena_size
        
        if self.r_center[0] < 0.0: 
            self.r_center[0] += arena_size
        if self.r_center[1] < 0.0: 
            self.r_center[1] += arena_size
    


    

    def turn_towards_vector(self, timestep_inc):

        max_degrees = self.max_turning_rate * timestep_inc;
        dev_angle = 360.0 * np.random.normal(0, self.angular_error_sd)
        vector = rotate(self.desired_direction, dev_angle)
        check_angle = smallest_angle_to(self.direction, self.desired_direction)
        
        if check_angle <= max_degrees:
            self.direction = self.desired_direction

        elif check_angle > max_degrees: # should be able to go from the cross product to the dot product
        
            cross_product = np.cross(self.direction, self.desired_direction)
            
            if cross_product > 0:
                self.direction = rotate(self.direction, max_degrees)
            else:
                self.direction = rotate(self.direction, -max_degrees)

        #self.desired_direction = np.copy(self.direction)


