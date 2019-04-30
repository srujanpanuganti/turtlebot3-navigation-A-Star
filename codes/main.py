# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 22:48:32 2019

@author: Srujan Panuganti
"""

import numpy as np
from obstacle_space_offset import is_obstacle
from obstacle_space_offset import obstacle_set_create
from action_set_offset import action_set
import math
np.set_printoptions(threshold=np.inf)
import Queue as queue
# import queue
import argparse
import math
import matplotlib.pyplot as plt


## To read the input from the user
def read(start,goal):

    start_pos = []
    goal_pos = []

    data_start = start.split(",")
    data_goal = goal.split(",")
    for element in data_start:
        start_pos.append(int(element))


    for element in data_goal:
        goal_pos.append(int(element))


    return start_pos,goal_pos

#to get the input from the user
parser = argparse.ArgumentParser(prefix_chars='@')
parser.add_argument('start_position')
parser.add_argument('goal_position')
args = parser.parse_args()
start_position, goal_position = read(args.start_position,args.goal_position)

# print('s',start_position)
# print('g',goal_position)

## to check if the node is visited
def is_visited_check(node, node_check_set):
    return str(node) in node_check_set


def node_info_list(min_x,min_y,max_x,max_y):
    loc = []

    for i in range(int(min_x),int(max_x),1):
        for j in range(int(min_y),int(max_y),1):
            point = [i,j]
            # print(point)

            loc.append([i,j])


    distance = {}
    for node in loc:
        distance[str(node)] = 9999999#math.inf
    #print(distance[str([5,5])])
    return distance



## To check if the entered start and goal position are in the obstacle space or not
def valid_start_goal(start,goal):
    status = True
    if is_obstacle(start) or is_obstacle(goal):
        status = False
        #print('either the start_position or your goal position is in obstacle space, enter a valid input')
    return status

## to make the map in all ones and zeros only
def ceil_map(map_passed):
	map_passed = map_passed/300
	map_passed = np.ceil(map_passed)
	return map_passed

def d(x):
    y = math.floor(x*1000)/1000
    return y

def cost_to_go(current_node,goal_node):

    # print(current_node,goal_node)

    x1 = current_node[0]
    y1 = current_node[0]

    x2 = goal_node[0]
    y2 = goal_node[0]

    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    # print('distance',d)

    return d

def display_map(start_position,goal_position,node_path,visited_node,obs_list):

    node_path_array = np.asarray(node_path)
    print('node path',node_path_array.shape)

    display_arrary = np.asarray(obs_list)
    visited_node_array = np.asarray(visited_node)
    # plt.figure(figsize=(111,101))

    plt.plot(display_arrary[:,0],display_arrary[:,1],'g+')
    plt.plot(start_position[0],start_position[1],'bo')
    plt.plot(goal_position[0],goal_position[1],'go')
    plt.plot(node_path_array[:,0],node_path_array[:,1],'ro')
    # plt.plot(visited_node_array[:,0],visited_node_array[:,1],'b+')

    plt.xlim(min_x,max_x)
    plt.ylim(min_y,max_y)
    plt.grid(True)
    plt.show()
    # plt.savefig('path.png')


# Defining the start and goal locations
# start_position = [2,60]
# start_position = [0,0]
# goal_position = [10,10]

grid_size = [1110,1010]
grid_length = 5
initial_theta = 0
res = 10

x_offset = 11.1/2 * res
y_offset = 10.1/2 * res

max_x = 11.1 * res - x_offset
max_y = 10.1 * res - y_offset

min_x = -x_offset
min_y = -y_offset


## checking if the supplied input is valid or not
is_a_vaid_input = valid_start_goal(start_position,goal_position)

start_position_theta = [start_position[0],start_position[1],initial_theta]

nodes_list = []
nodes_list.append([start_position])

## To capture the parent nodes from the visited node
q = queue.PriorityQueue()
q.put([0, start_position_theta])

node_check_set = set([])            #visited nodes
node_check_set.add(str(start_position))

node_info_dict = node_info_list(min_x,min_y,max_x,max_y)
node_info_dict[str(start_position)] = 0

node_info_parent_dict = {}
node_info_velocities = {}

visited_node = []

iter1 = iter2 = 0
goal_reached = False

rpm1 = 5	#2
rpm2 = 4	#3
len = 0.23*res
rad =0.076*res/2


displayArray = 255 * np.ones((111+1,101+1,3))
displayRes = 1

obs_set, obs_list = obstacle_set_create()

while not q.empty() and is_a_vaid_input == True:# and :

    node = q.get()      ##[0, [15, 20, 0]] --> [cost, [x,y,theta]]
    # print('node in while',node)

    x_new = node[1][0]
    y_new = node[1][1]
    theta_new = node[1][2]
    node_pos = [node[1][0],node[1][1]]

    iter1+=1
    # print(iter1)

    if [int(node[1][0]),int(node[1][1])] == goal_position:
        print('goal reached',iter1)
        goal_obtained = [node[1][0],node[1][1]]
        goal_reached = True
        break
    explored_nodes = action_set(x_new,y_new,max_x,max_y,vel1=rpm1,vel2=rpm2,dt=0.5,node_number=5,node_info=5,theta = theta_new,length=len,radius=rad)

    # action = np.array([new_x,new_y,new_theta,new_cost,action[0],action[1]])
    for action in explored_nodes:

        # print(action)  ##[1.98308572 3.1073315  1.72727273 0.19       0.         5.        ]
        action_pos = [action[0],action[1]]
        action_theta = action[2]
        action_cost = action[3]
        action_velocities = [action[4],action[5],action_cost]

        if is_visited_check(action_pos,node_check_set) == False:

            if is_obstacle(action_pos) == False:
                node_check_set.add(str([int(action_pos[0]),int(action_pos[1])])) ## marked as visited --> added to visited nodes ## str([d(act[0]),d(act[1])])
                visited_node.append(action_pos)
                cost = action_cost + node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] + cost_to_go(action_pos,goal_position)
                node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] = cost
                q.put([cost,[action[0],action[1],action_theta]])
                # node_info_parent_dict[str([int(action_pos[0]),int(action_pos[1])])] = [int(node_pos[0]),int(node_pos[1])] #--> parent is updated to the node info
                node_info_parent_dict[str(action_pos)] = node_pos#--> parent is updated to the node info
                node_info_velocities[str(action_pos)] = action_velocities


        else:
            if is_obstacle(action_pos) == False:
                temp = action_cost + node_info_dict[str(node_pos)]
                if node_info_dict[str(action_pos)] > temp:
                    node_info_dict[str(action_pos)] = temp + cost_to_go(action_pos,goal_position)
                    # node_info_parent_dict[str([int(action_pos[0]),int(action_pos[1])])] = [int(node_pos[0]),int(node_pos[1])]           #--> parent is updated to the node info
                    node_info_parent_dict[str(action_pos)] = node_pos           #--> parent is updated to the node info
                    node_info_velocities[str(action_pos)] = action_velocities

node_path = []
# print(node_info_dict)
# print(node_info_velocities)
node_velocities_list = []

if is_a_vaid_input == True:
    if goal_reached:

        node_path.append(goal_obtained)
        parent = node_info_parent_dict[str(goal_obtained)]

        while parent != start_position:
            node_velocities_list.append(node_info_velocities[str(parent)])
            parent =  node_info_parent_dict[str(parent)]
            node_path.append(parent)
        node_path.reverse()
        node_velocities_list.reverse()

        display_map(start_position,goal_obtained,node_path,visited_node,obs_list)

    else:
        print('goal not reached')
else:
    print('enter a valid input')


node_vel_arr = np.asarray(node_velocities_list)
node_path_arr = np.asarray(node_path)

with open('node_path.txt', 'w') as node_path_file:

    for i in node_path_arr:
        t = np.empty([1,2])
        t[0:,] = i
        np.savetxt(node_path_file,t,delimiter='\t')

velocity_list = []



for velocity in node_vel_arr:
    # print(velocity[0],velocity[1])

    xDot = (rad/2)*(velocity[1]+velocity[0])*math.cos(velocity[2])
    yDot = (rad/2)*(velocity[1]+velocity[0])*math.sin(velocity[2])
    thDot = (rad/len)*(velocity[1]-velocity[0])


    lin_x = np.linalg.norm(np.array([[xDot],[yDot]]))
    lin_y = 0
    lin_z = 0

    ang_x = 0
    ang_y = 0
    ang_z = thDot

    velocity_list.append([lin_x,lin_y,lin_z,ang_x,ang_y,ang_z])

velocity_array = np.asarray(velocity_list)

with open('converted_velocities.txt', 'w') as velocities_file:

    for i in velocity_array:
        p = np.empty([1,6])
        p[0:,] = i
        np.savetxt(velocities_file,p,delimiter='\t')

