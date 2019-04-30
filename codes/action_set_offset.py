import numpy as np
import math
np.set_printoptions(threshold=np.inf)


def calculate_values(cur_x,cur_y,cur_theta,len,rad,u_left,u_right,dt):
    x = cur_x
    y = cur_y
    theta= cur_theta
    l = len
    r = rad
    ul = u_left
    ur = u_right


    dx = (r/2)*(ul+ur)*math.cos(theta)*dt
    dy = (r/2)*(ul+ur)*math.sin(theta)*dt
    d_theta = (r/l)*(ur-ul)*dt

    x_new = x + dx 
    y_new = y + dy 
    theta = theta + d_theta
    euclidean_distante = np.sqrt((x-x_new)**2 + (y-y_new)**2)

    return x_new, y_new, theta, euclidean_distante

def new_position_integration(current_x,current_y,current_theta,length,radius,vel1,vel2,dt):

    cur_x = current_x
    cur_y = current_y
    cur_theta = current_theta
    len = length
    rad = radius
    u_left = vel1
    u_right = vel2
    new_cost = 0
    integration_step  = math.ceil(dt/0.01)
    new_time_step = dt/integration_step

    for i in range(int(integration_step)):
        cur_x , cur_y, cur_theta, cur_cost= calculate_values(cur_x,cur_y,cur_theta,len,rad,u_left,u_right,new_time_step)
        # cur_x = temp_x + cur_x
        # cur_y = temp_y + cur_y
        # cur_theta = temp_theta + cur_theta
        new_cost = cur_cost + new_cost

    return cur_x,cur_y,cur_theta,new_cost

def action_set(current_x,current_y,max_x,max_y,length,radius,vel1,vel2,dt,node_number,node_info,theta):
    
    current_theta = theta
    action_list = []
    action_set = np.array([[0,vel1],[vel1,0],[vel1,vel1],[0,vel2],[vel2,0],[vel2,vel2],[vel1,vel2],[vel2,vel1]])

    for action in action_set:
        new_x,new_y,new_theta,new_cost = new_position_integration(current_x,current_y,current_theta,length,radius,action[0],action[1],dt)
        if new_x >= min_x and new_y >=min_y and new_x <= max_x and new_y <= max_y:

            next_action = np.array([new_x,new_y,new_theta,new_cost,action[0],action[1]])
            action_list.append(next_action)
    return action_list


res = 10

x_offset = 11.1/2 * res
y_offset = 10.1/2 * res

max_x = 11.1 * res - x_offset
max_y = 10.1 * res - y_offset

min_x = -x_offset
min_y = -y_offset


