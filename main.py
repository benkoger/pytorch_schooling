
import time
import random
import numpy as np
import math as m
from scipy import ndimage
import cv2

from individual import Individual
from rotate import rotate


def main():

    #Simulation start time
    t1 = time.clock()
    #Random generator engine from a time-based seed
    random.seed()
    
    #Set parameters
    num_timesteps = 1600000
    timestep_inc = 0.2
    arena_size = 1000
    top_left = np.array([0.0,0.0])
    bottom_right = np.array([arena_size,arena_size])

    
    sight_num = 166  #make even

    total_agents = 100
    total_prey_agents = 75
    total_pred_agents = total_agents - total_prey_agents
    max_turning_rate_prey = 3.0
    max_turning_rate_pred = 2.0
    zop_prey = 100.0
    zop_pred = 150.0
    speed_prey = 6.0
    speed_pred = 9.0
    color_prey = [255,135,71]
    color_pred = [255,255,255]
    length_prey = 5.0 
    length_pred = 10.0
    angular_error_sd = 0.0
    
    zop_max = zop_pred if zop_pred > zop_prey else zop_prey
    
    agents = []
    
    
    #Simulation starts HERE
    
    #Set up agents
    for i in range(total_agents):
        set_direction = np.array([1.0, 0.0])    # need to be set to unit vectors
        set_direction = rotate(set_direction, random.random()*360.0)
        set_r_centre = random_bounded_point(bottom_right, top_left)

        if i < total_prey_agents:
            agents.append(Individual(set_r_centre, set_direction, max_turning_rate_prey,
                              speed_prey, zop_prey, angular_error_sd, 1, 
                              color_prey, length_prey, sight_num, i))
        else:
            agents.append(Individual(set_r_centre, set_direction, max_turning_rate_pred, 
                              speed_pred, zop_pred, angular_error_sd, 2, 
                              color_pred, length_pred, sight_num, i))
    
    for j in range(num_timesteps):
        
        #draw to screen
        if j % 1 == 0:
            graphics(arena_size, agents, zop_max)
        
        move_agents(arena_size, zop_max, agents, timestep_inc)
           


def move_agents(arena_size, zop_max, agents, timestep_inc):
    #reset each individuals z buffer, sight_vec, and zop_count
    for i in range(len(agents)):
        agents[i].z_buff = np.ones(agents[i].sight_num) * (zop_max + 1)
        agents[i].sight_vec = np.zeros(agents[i].sight_num)
        agents[i].zop_count = 0

    linked_list(arena_size, zop_max, agents)
    implement_social_forces(agents)

    
    for i in range(len(agents)):
        
        agents[i].move(timestep_inc, arena_size)


def linked_list(arena_size, zop_max, agents):

    # this implementation assumes equal x and y 
    # dimensions i.e. the arena is of size n x n

    
    # Algorithm 1: List construction
    # this algorithm creates headers and linked lists (assigns a list of agents to a particular cell)
    # headers (head): holds index of first atom (or -1 if empty)
    # linked list (lscl): holds the agents index to which the ith agents points

    number_of_cells = int(arena_size // zop_max)   # in a row/column
    cell_size = arena_size / number_of_cells      #width/height

    lscl = np.zeros(len(agents), np.int32)
    head = -1 * np.ones(number_of_cells * number_of_cells, np.int32) 
    cell_x = 0
    cell_y = 0

    for i in range(len(agents)):
        # c_x and c_y determine which cell agents is in
        cell_x = agents[i].r_centre[0] // cell_size
        cell_y = agents[i].r_centre[1] // cell_size

        #if agents[i].r_centre[0] < 0 or agents[i].r_centre[1] < 0:
        #print(str(agents[i].r_centre[0]) + ',' + str(agents[i].r_centre[1]))
        
        c = int(cell_x * number_of_cells + cell_y)  #calculate scalar cell id
        lscl[i] = head[c]                      #link agents to previous occupant or -1 if first to be present there
        head[c] = i                            #the last agents goes to the header
    
    # Algorithm 2: Compute interactions
    for cell_x  in range(number_of_cells):
        for cell_y in range(number_of_cells):
           
            c = cell_x * number_of_cells + cell_y
            if head[c] != -1:

                for scan_neighbours_x in range(cell_x - 1, cell_x + 2):
                    for scan_neighbours_y in range(cell_y - 1, cell_y + 2):
                    
                        pbound_x = scan_neighbours_x;
                        pbound_y = scan_neighbours_y;
                        
                        if scan_neighbours_x < 0:
                            pbound_x += number_of_cells 
                        elif scan_neighbours_x >= number_of_cells:
                            pbound_x -= number_of_cells
                        
                        if scan_neighbours_y < 0:
                            pbound_y += number_of_cells
                        elif scan_neighbours_y >=number_of_cells:
                            pbound_y -= number_of_cells
                        
                        c1 = pbound_x * number_of_cells + pbound_y
                        
                        i = head[c]
                        while i != -1:
                        
                            j = head[c1]
                            while j != -1:
                            
                                if i != j:
                                    #Here is, I think, where to compute lines of sight
                                    compute_social_forces(int(i), int(j), agents, arena_size) #fix this int casting


                                    
                                j = lscl[j]

                            i = lscl[i];


def compute_social_forces(i, j, agents, arena_size):  #takes two ints

    temp_vector = np.zeros(2);
    
    temp_vector = agents[j].r_centre - agents[i].r_centre
    
    if abs(temp_vector[0]) > arena_size / 2:

        if agents[i].r_centre[0] < agents[j].r_centre[0]:
            temp_vector[0] = (agents[j].r_centre[0] 
                             - (agents[i].r_centre[0] + arena_size))
        else:
            temp_vector[0] = (agents[j].r_centre[0]
                             - (agents[i].r_centre[0] - arena_size))

    if abs(temp_vector[1]) > arena_size / 2:

        if (agents[i].r_centre[1] < agents[j].r_centre[1]):
            temp_vector[1] = (agents[j].r_centre[1] 
                             - (agents[i].r_centre[1] + arena_size))
        else:
            temp_vector[1] = (agents[j].r_centre[1] 
                             - (agents[i].r_centre[1] - arena_size))

    
    #check to see if you are interacting


    dist = np.sqrt(temp_vector @ temp_vector)

    if dist < agents[i].zone_of_perception:

        if dist != 0:
            temp_vector = temp_vector / dist # normalize
        
        mod_sight_vector(i, j, dist, temp_vector, agents)
        agents[i].zop_count += 1



def mod_sight_vector(i, j, dist, vector, agents): #int& i, int& j, double dist, CVec2D vector

    if agents[j].length/dist > 1:
        angle_width_half = 90
    else:
        angle_width_half = m.asin(agents[j].length/dist) * 180.0/(m.pi)
    angle = (m.atan2(agents[i].direction[1],agents[i].direction[0]) 
             - m.atan2(vector[1],vector[0])) * 180.0/(m.pi)
    for ind in range(m.ceil((angle - angle_width_half)/2.0),
                     m.floor((angle + angle_width_half)/2.0)):
        if (ind > (-1*(agents[i].sight_num / 2 + 1)) and ind < (agents[i].sight_num / 2 )):
            if(dist < agents[i].z_buff[ind + agents[i].sight_num / 2]):
                agents[i].sight_vec[ind + agents[i].sight_num / 2] = agents[j].type
                agents[i].z_buff[ind + agents[i].sight_num / 2] = dist




def implement_social_forces(agents):

    # now have total_zop calculated for all individuals
    for k in range(len(agents)):
        #Things in individuals line of sight
        if agents[k].zop_count > 0:
            #Maybe this is where the initially random direction should be set
            agents[k].desired_direction = rotate(agents[k].desired_direction, 
                                    agents[k].max_turning_rate * rand_move_direction())

        #nothing in line of sight so just randomly choose
        else:
            agents[k].desired_direction = rotate(agents[k].desired_direction, 
                                    agents[k].max_turning_rate * rand_move_direction())


def rand_move_direction():

    move = random.randint(-1,1)  #Should give -1 or 1
    
    return move;




def random_bounded_point(bottom_right, top_left):

    # Create randomly distributed co-ordinate in the simulated world
    range_x = bottom_right[0] - top_left[0]
    range_y = bottom_right[1] - top_left[1]
    
    random_x = random.random()
    random_y = random.random()
    
    random_x *= range_x
    random_y *= range_y
    random_point = np.array([random_x, random_y])
    
    return random_point



def graphics(arena_size, agents, zop_max):

    number_of_cells = int(arena_size // zop_max)   # in a row/column
    cell_size = arena_size / number_of_cells      #width/height

    
    colors = [[244,66,66],[244,158,66],[244,232,66],[185,244,66],[125,244,66],
              [66,244,200],[66,217,244],[66,101,244],[176,66,244],[244,66,170],
              [255,135,71],[255,255,255]]

    # Draw arena
    visualisation = np.zeros((arena_size, arena_size, 3), np.uint8)
    
    # Draw agents


    for i  in range(len(agents)):


        cv2.circle(visualisation, (int(agents[i].r_centre[0]), 
            int(agents[i].r_centre[1])), int(agents[i].length), 
            agents[i].color, -1, cv2.LINE_AA)

        cv2.line(visualisation, (int(agents[i].r_centre[0]), 
            int(agents[i].r_centre[1])), (int(agents[i].tail[0]), 
            int(agents[i].tail[1])), agents[i].color, 1, cv2.LINE_AA)
        
        if(agents[i].id % 170 == 0):
            cv2.circle(visualisation, (int(agents[i].r_centre[0]), 
                int(agents[i].r_centre[1])), int(agents[i].zone_of_perception), 
                agents[i].color, 1, cv2.LINE_AA)

            for p in range(agents[i].sight_num):
                
                if agents[i].sight_vec[p] != 0:

                    cv2.circle(visualisation, (int(agents[i].r_centre[0] 
                        + m.cos((agents[i].sight_num*m.pi/180.0)-(2*p*m.pi/180.0)
                        + m.atan2(agents[i].direction[1],agents[i].direction[0]))
                        * agents[i].zone_of_perception), int(agents[i].r_centre[1] 
                        + m.sin((agents[i].sight_num*m.pi/180.0)-(2*p*m.pi/180.0)
                        + m.atan2(agents[i].direction[1],agents[i].direction[0]))
                        * agents[i].zone_of_perception)),
                        int(m.pi*agents[i].zone_of_perception/90.0),
                        colors[int(agents[i].sight_vec[p] - 1)], -1, cv2.LINE_AA)
                       
    
    cv2.imshow("decision_making", visualisation)
    cv2.waitKey(1)


if __name__ == "__main__":
    main()