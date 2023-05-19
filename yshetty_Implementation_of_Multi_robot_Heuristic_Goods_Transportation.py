import matplotlib.pyplot as plt
import numpy as np
import time

source = [5,5] 
sink = [3,8]

def dist(curr_point, target_point):
    point1 = np.array(target_point)
    point2 = np.array(curr_point)
    return np.linalg.norm(point1 - point2)


def vel(curr_point, target_point):
    point1 = np.array(target_point)
    point2 = np.array(curr_point)
    
    vect = dist(curr_point, target_point)
    vel_x = 0.45*(point1[0] - point2[0])/vect
    vel_y = 0.45*(point1[1] - point2[1])/vect
    return vel_x,vel_y

def goto_source(curr_point, target_point):
    velx,vely = vel(curr_point,target_point)
    curr_point[0] += velx
    curr_point[1] += vely
    return curr_point

def goto_sink(curr_point, target_point):
    velx,vely = vel(curr_point,target_point)
    curr_point[0] += velx
    curr_point[1] += vely
    return curr_point

def goto_depot(curr_point, target_point):
    velx,vely = vel(curr_point,target_point)
    curr_point[0] += velx
    curr_point[1] += vely
    return curr_point

def in_depot(robots):
    if dist(robots.position,robots.init_pos) < 0.5:
        return True
    else :
        return False
    

def T_tos_s(robot) :
    return dist(robot.position,source)/0.45

def T_depot_to_source(robot):
    return dist(robot.init_pos,source)/0.45

class Robot():
    def __init__(self, id, position, init_pos,h,command,goods_carried,energy):
        self.id = id
        self.position = position
        self.init_pos = init_pos
        self.picked_up = False
        self.command = command
        self.h = h
        self.goods_carried = goods_carried
        self.T = [0]
        self.energy = energy

    def append_T(self,T_val):
        self.T.append(T_val)

start_pt = [1,1]
plt.ion()
fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
plt.xlim(0,10)
plt.ylim(0,10)

plt.draw()

robots = []
temp_robot = Robot(0,[1.5,1.5],(1.5,1.5),0,0,0,0)
robots.append(temp_robot)
temp_robot = Robot(1,[0.5,0.5],(0.5,0.5),0,0,0,0)
robots.append(temp_robot)
temp_robot = Robot(2,[1.5,0.5],(1.5,0.5),0,0,0,0)
robots.append(temp_robot)
temp_robot = Robot(3,[0.5,1.5],(0.5,1.5),0,0,0,0)
robots.append(temp_robot)

task_do = [False,False,False,False]
new_pt = start_pt
T_wait = [0]*4
# times = [23, 55, 67, 78, 89, 104, 110, 140, 156, 200, 213 ,214,215,216,217,218,219, 220, 234, 235, 249,276,280, 289,300,310, 340,360.387,
#          400,410,450,467,478,489,490,505,530,568,588,600,631,653,679,699,710,722,743,777,799,809,822,876,899,903,
#           905,917,925,944,955,980 ]
times = np.linspace(0,100,101)
print(times)
t_wait_flag = False
pending_tasks =0
for t in range(1000):
    
    # print(t,task_do)
    # print(pending_tasks)
    if t in times :
        pending_tasks += 1
        
    
           
    for i in range(4):
        if pending_tasks >0 :
            robots[i].h = sum(robots[i].T)/len(robots[i].T)
            # print('h',i,robots[i].h)
            if not task_do[i] :
                if not in_depot(robots[i]):
                    T_to_source = T_tos_s(robots[i]) 
                    if robots[i].h < T_to_source and robots[i].h > 0.5*T_to_source:
                        task_do[i] = True
                        robots[i].command = 1
                        robots[i].append_T(0)
                        print('jere')
                        pending_tasks -= 1
                        i =4
                else:
                    if T_wait[i] <= max(0,robots[i].h - T_depot_to_source(robots[i])):
                        T_wait[i] += 1
                        t_wait_flag = True
                        # print(T_wait)
                    else :
                        task_do[i] = True
                        robots[i].command = 1
                        robots[i].append_T(0)
                        t_wait_flag = False
                        T_wait[i] = 0
                        pending_tasks -= 1
                        # print(i)
                        i =4
        
    for i in range(4):
        if robots[i].command == 1 :
        # Send robot to source
            if dist(robots[i].position,source) > 0.55:
                robots[i].position= goto_source(robots[i].position,source)
                robots[i].picked_up = False
                
            else :
                robots[i].picked_up = True
                robots[i].command = 2

        if robots[i].command == 2 :
        # Send robot to sink
            if dist(robots[i].position,sink) > 0.55:
                robots[i].position= goto_sink(robots[i].position,sink)
                robots[i].picked_up = True
            else :
                robots[i].picked_up = False
                robots[i].command = 3
                task_do[i] = False
                robots[i].goods_carried += 1

        if robots[i].command == 3 :
        # Send robot back to depot
            task_do[i] = False
            robots[i].picked_up = False
            if dist(robots[i].position,robots[i].init_pos) > 0.55:
                robots[i].position= goto_depot(robots[i].position,robots[i].init_pos)
                
            else :
                task_do[i] = False
                robots[i].command = 0
            
        if not robots[i].picked_up and robots[i].goods_carried>0 and not in_depot(robots[i]):
            robots[i].T[robots[i].goods_carried -1] += 1

           
        

    x = [source[0],sink[0],robots[0].position[0],robots[1].position[0],robots[2].position[0],robots[3].position[0]]
    y = [source[1],sink[1],robots[0].position[1],robots[1].position[1],robots[2].position[1],robots[3].position[1]]
 
    
    sc.set_offsets(np.c_[x,y])
    fig.canvas.draw_idle()
    # print(x,y)
    plt.pause(0.1)

plt.waitforbuttonpress()