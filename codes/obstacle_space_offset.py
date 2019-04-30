import numpy as np
import matplotlib.pyplot as plt



def is_point_in_rect(points,rect_coordinates):        # points = [0,0]    ## rect1 = [[1,4],[4,2]]
    x = points[0]
    y = points[1]

    a = rect_coordinates[0][0]
    b = rect_coordinates[0][1]

    c = rect_coordinates[1][0]
    d = rect_coordinates[1][1]

    if x >= a and x <= c and y <= b and y >= d:
        return True
    else:
        return False

def is_point_in_circle(points, circle_center,radius):    ## points = [0,0] ## radi = 5   ##circle_cent = [5,5]
    x = points[0]
    y = points[1]

    h = circle_center[0]
    k = circle_center[1]

    if (x-h)**2 + (y-k)**2 <= radius**2:
        return True
    else:
        return False

def rect_offset(rect_coords):           ## rect1 = [[1,4],[4,2]]

    x1 =  rect_coords[0][0] - x_offset
    x2 =  rect_coords[1][0] - x_offset

    y1 =  rect_coords[0][1] - y_offset
    y2 =  rect_coords[1][1] - y_offset

    return [[x1,y1],[x2,y2]]

def cir_offset(cir_cent):       #circle_cent = [5,5]

    x = cir_cent[0] - x_offset
    y = cir_cent[1] - y_offset

    return [x,y]

def is_border(point,robot_radius):       # point = [1,2]  ## robot_radius = 0.354/2 *res

    # print(robot_radius)
    # print(point)

    x = point[0]
    y = point[1]

    if x <= robot_radius - x_offset or x >= x_max-robot_radius or y <= robot_radius - y_offset or y >= y_max-robot_radius:
        return True
    else:
        False

def is_point_left_semi(points, circle_center,radius):       ## points = [0,0] ## radi = 5   ##circle_cent = [5,5]
    x = points[0]
    y = points[1]

    h = circle_center[0]
    k = circle_center[1]

    if (x-h)**2 + (y-k)**2 <= radius**2  and  x<= h:
        return True
    else:
        return False

def is_point_right_semi(points, circle_center,radius):      ## points = [0,0] ## radi = 5   ##circle_cent = [5,5]
    x = points[0]
    y = points[1]

    h = circle_center[0]
    k = circle_center[1]

    if (x-h)**2 + (y-k)**2 <= radius**2  and  x>= h:
        return True
    else:
        return False

def obstacle_set_create():

    # print(int(grid[0]))
    obstacle_list = []
    obs_set = set([])
    for i in range(int(x_min),int(x_max),1):
        for j in range(int(y_min),int(y_max),1):
            point = [i,j]
            # print(point)

            if is_obstacle(point): 
                obs_set.add(str(point))
                obstacle_list.append(point)

    return obs_set,obstacle_list
# obtained_set = obstacle_set_create(rectangles,circles)

def is_obstacle_set(point,obst_set):
    return str(point) in obst_set
# stat3 = is_obstacle_set()

def is_obstacle(point):

    if is_point_in_rect(point,rect1) or is_point_in_rect(point,rect2) or is_point_in_rect(point,rect3) or \
        is_point_in_rect(point,rect4) or is_point_in_rect(point,rect5) or is_point_in_rect(point,rect6) or \
        is_point_in_rect(point,rect7) or is_point_in_rect(point,rect8) or is_point_in_rect(point,rect9) or \
        is_point_in_rect(point,rect10) or is_point_in_rect(point,rect11) or is_point_in_rect(point,rect12) or \
        is_point_in_rect(point,rect13) or is_point_in_rect(point,rect14) or is_point_in_circle(point,c1,c1_rad) or\
        is_point_in_circle(point,c2,c2_rad) or is_point_in_circle(point,c3,c3_rad) or is_point_in_circle(point,c4,c4_rad) or \
        is_point_in_circle(point,c5,c5_rad) or is_point_in_circle(point,c6,c6_rad) or is_border(point,robot_radius*res):

        return True
    else:
        return False


clearance = 0.05
res = 10
robot_radius = (0.354/2) + clearance

x_offset = 11.1/2 * res
y_offset = 10.1/2 * res

x_max = 11.1 * res - x_offset
y_max = 10.1 * res - y_offset

x_min = -x_offset
y_min = -y_offset


#Rectangles:

#rect 1
rect1 = rect_offset(np.multiply([[1.4995 - robot_radius,9.1 + robot_radius],[3.0985 + robot_radius,7.501 - robot_radius]],res))

rect2 = rect_offset(np.multiply([[4.38 - robot_radius,4.98 + robot_radius],[5.29 + robot_radius,3.15 - robot_radius]], res))

rect3 = rect_offset(np.multiply([[4.74 - robot_radius,1.87 + robot_radius],[7.48 + robot_radius,0.35 - robot_radius]], res))

rect4 = rect_offset(np.multiply([[5.29 - robot_radius,3.41 + robot_radius],[7.12 + robot_radius,2.65 - robot_radius]], res))

rect5 = rect_offset(np.multiply([[6.85 - robot_radius,0.35 + robot_radius],[11.1,0]], res))

rect6 = rect_offset(np.multiply([[7.44 - robot_radius,6.97 + robot_radius],[11.1,6.21 - robot_radius]], res))

rect7 = rect_offset(np.multiply([[7.79 - robot_radius,0.93 + robot_radius],[8.96 + robot_radius,0.35]], res))

rect8 = rect_offset(np.multiply([[7.845 - robot_radius,3.84 + robot_radius],[9.365 + robot_radius,2.67 - robot_radius]], res))

rect9 = rect_offset(np.multiply([[8.32 - robot_radius,10.1],[9.18 + robot_radius,8.27 - robot_radius]], res))

rect10 = rect_offset(np.multiply([[9.27 - robot_radius,1.11 + robot_radius],[11.1,0.35]], res))

rect11 = rect_offset(np.multiply([[9.83 - robot_radius,10.1],[10.26 + robot_radius,9.19 - robot_radius]], res))

rect12 = rect_offset(np.multiply([[10.19 - robot_radius,4.485 + robot_radius],[11.1,3.625 - robot_radius]], res))

rect13 = rect_offset(np.multiply([[10.52 - robot_radius,5.655 + robot_radius],[11.1,4.485]], res))

rect14 = rect_offset(np.multiply([[10.52 - robot_radius,2.9525 + robot_radius],[11.1,1.7825 - robot_radius]], res))

### semi circles
left_semi_center = np.multiply([1.4995,8.3005], res)
left_semi_radius = 0.7995* res

right_semi_center = np.multiply([3.0985,8.3005], res)
right_semi_radius = 0.7995* res

#circles:
c1 = cir_offset(np.multiply([3.9,9.6], res))
c1_rad = 0.405* res + robot_radius

# print(c1)
# print(cir_offset(c1))

c2 = cir_offset(np.multiply([4.38,7.36], res))
c2_rad = 0.405* res + robot_radius

c3 = cir_offset(np.multiply([4.38,2.74], res))
c3_rad = 0.405* res + robot_radius

c4 = cir_offset(np.multiply([3.9,0.405], res))
c4_rad = 0.405* res + robot_radius

c5 = cir_offset(np.multiply([1.4995,8.3005], res))
c5_rad = 0.7995* res + robot_radius

c6 = cir_offset(np.multiply([3.0985,8.3005], res))
c6_rad = 0.7995* res + robot_radius

# grid = [x,y]
grid = np.multiply([11.1,10.1], res)

rectangles = [rect1,rect2,rect3,rect4,rect5,rect6,rect7,rect8,rect9,rect10,rect11,rect12,rect13,rect14]
circles = [[c1,c1_rad],[c2,c2_rad],[c3,c3_rad],[c4,c4_rad],[c5,c5_rad],[c6,c6_rad]]

ob_set, ob_list  = obstacle_set_create()

ob_array = np.asarray(ob_list)

# plt.plot(ob_array[:,0],ob_array[:,1], 'b+')
# plt.show()
# stat = is_obstacle([3.0985,8.3005])
# print(ob_list)

# plt.plot(ob_array[:,0],ob_array[:,1],'b+')
# plt.show()


