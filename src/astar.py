#!/usr/bin/env python
from __future__ import division

import rospy
import numpy as np
import matplotlib.pyplot as plt
import heapq
import sys
import os
import math 
import subprocess

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from moveBot import move


def get_alpha(theta):
    while theta>360:
        theta = theta-360
    alpha = int(theta/30)
    while alpha<0:
        alpha +=12

    while alpha>=12:
        alpha -=12
    return alpha    

def plot_curve(Xi,Yi,Thetai,UL,UR,c = "blue"):
    t = 0
    r = 3.8
    L = 35.4
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan

class Map:
    def __init__(self,start,goal,rpm1,rpm2):
        """
        Constructs a new instance.
    
        :param      start:      The start
        :type       start:      Start Node coordinates
        :param      goal:       The goal
        :type       goal:       Goal Node coordinates
        :param      clearence:  The clearence
        :type       clearence:  Int
        :param      radius:     The radius
        :type       radius:     Int
        :param      step_size:  The step size
        :type       step_size:  Int
        """
        self.visited_map = np.zeros((2040,2040,12),np.uint8)
        self.start = start
        self.goal = goal
        # self.step_size = step_size
        self.queue=[]
        self.visited = []
        self.shortest_path = [] 
        self.radius= 30
        self.clearence= 5
        r = self.radius
        c = self.clearence

        self.rpm1 = rpm1
        self.rpm2 = rpm2


        plt.scatter(start[0],start[1],s = 10,c = 'r')
        plt.scatter(goal[0],goal[1],s = 10,c = 'r')
        
        # Circles
        plt.gca().add_patch(plt.Circle((510,510), radius=100, fc='y'))
        plt.gca().add_patch(plt.Circle((710,810), radius=100, fc='y'))
        plt.gca().add_patch(plt.Circle((310,210), radius=100, fc='y'))
        plt.gca().add_patch(plt.Circle((710,210), radius=100, fc='y'))

        # Boundary
        plt.gca().add_patch(plt.Rectangle((0,0),10,1020, color='black'))
        plt.gca().add_patch(plt.Rectangle((0,0),1020,10, color='black'))
        plt.gca().add_patch(plt.Rectangle((0,1010),1020,10,color='black'))
        plt.gca().add_patch(plt.Rectangle((1010,0),10,1020,color='black'))

        # Squares
        plt.gca().add_patch(plt.Rectangle((235,735),150,150))
        plt.gca().add_patch(plt.Rectangle((35,460),150,150))
        plt.gca().add_patch(plt.Rectangle((835,460),150,150))
        plt.axis('scaled')


    def isObstacle(self,x,y):
        """
        Determines if obstacle using half-plane equations.
    
        :param      j:    x-coordinate
        :type       j:    flloat
        :param      i:    y-coordinate
        :type       i:    float
    
        :returns:   True if obstacle, False otherwise.
        :rtype:     boolean
        """
        r=self.radius
        c=self.clearence

        obstacle = False
        # print("Point :",x,y)
        if (x<=(10+r+c)) or (x>=1020-(10+(r+c))) or (y<=(10+r+c)) or (y>=1020-(10+(r+c))):
            # print("Boundary Condition")
            obstacle = True

       # Circle
        if ((x-510)**2 + (y-510)**2 <= ((100+r+c)**2)):
            # print("Circle Condition 1")
            obstacle = True
        
        # Circle
        if ((x-710)**2 + (y-810)**2 <= ((100+r+c)**2)):
            # print("Circle Condition 2")
            obstacle = True
        
        # Circle
        if ((x-310)**2 + (y-210)**2 <= ((100+r+c)**2)):
            # print("Circle Condition 3")
            obstacle = True

        # Circle
        if ((x-710)**2 + (y-210)**2 <= ((100+r+c)**2)):
            # print("Circle Condition 4")
            obstacle = True

        # Rectangle
        if (((x-(35-(r+c)))>=0) and ((x-(185+(r+c)))<=0) and ((y-(460-(r+c)))>=0) and ((y-(660+(r+c)))<=0)):
            # print("Rectangle Condition 1")
            obstacle = True

        # Rectangle
        if (((x-(235-(r+c)))>=0) and ((x-(385+(r+c)))<=0) and ((y-(735-(r+c)))>=0) and ((y-(885+(r+c)))<=0)):
            # print("Rectangle Condition 2")
            obstacle = True

        # Rectangle
        if (((x-(835-(r+c)))>=0) and ((x-(985+(r+c)))<=0) and ((y-(460-(r+c)))>=0) and ((y-(660+(r+c)))<=0)):
            # print("Rectangle Condition 3")
            obstacle = True


        return obstacle
        
    def cost(self,node,step_cost):
        """
        Returns 
    
        :param      node:       The node at which the cost is to be obtained 
        :type       node:       List
        :param      step_cost:  The step cost to reach that node from start node
        :type       step_cost:  int
    
        :returns:   Heuristic cost for A*
        :rtype:     float
        """
        return(step_cost + np.linalg.norm(np.array(node[0:2])-np.array(self.goal[0:2]),2))



    def actionsAvailable(self,x,y,theta,step_cost):
        """
        Returns 5 actions available at the given nodes 
    
        :param      x:          The current node x
        :type       x:          float
        :param      y:          The current node y
        :type       y:          float
        :param      theta:      The theta
        :type       theta:      Int in range (0,12)
        :param      step_cost:  The step cost
        :type       step_cost:  Int
        """
        actions = [[0,self.rpm1],[self.rpm1,0],[self.rpm1,self.rpm1],[0,self.rpm2],[self.rpm2,0],[self.rpm2,self.rpm2],[self.rpm1,self.rpm2],[self.rpm2,self.rpm1]]

        for a in actions:
            xn,yn,thetan = plot_curve(x,y,theta,a[0],a[1])
            # Check obstacle condition for the explored nodes 
            if self.isObstacle(xn,yn) == False:
                if (self.visited_map[int(round(yn*2)),int(round(xn*2)),get_alpha(thetan)])==0:
                    self.visited_map[int(round(yn*2)),int(round(xn*2)),get_alpha(thetan)]=1
                    heapq.heapify(self.queue)
                    heapq.heappush(self.queue,[self.cost((xn,yn),step_cost),xn,yn,thetan,step_cost+1,a[0],a[1],x,y,theta ])

                elif (self.visited_map[int(round(yn*2)),int(round(xn*2)),get_alpha(thetan)])>0:
                    pass



    def backtrack(self):
        n= len(self.visited)
        parent = []
        j = 0
        print("Backtracking")
        while(True):
            popped = self.visited[n-1-j]
            current_node = [popped[1],popped[2],popped[3],popped[-5],popped[-4]]
            parent_node = [popped[-3],popped[-2],popped[-1],popped[-5],popped[-4]]
            parent.append(parent_node)
            self.shortest_path.append([parent_node[0],parent_node[1],parent_node[2],parent_node[3],parent_node[4]])
            if [current_node[0],current_node[1]] == [self.start[0],self.start[1]]:
                break
            cp = np.array(self.visited)[:,1:4]
            for i in range(0,cp.shape[0]):
                if (cp[i][0]==parent_node[0]) and (cp[i][1]==parent_node[1]) and (cp[i][2]==parent_node[2]):
                    j = n-1-i
        for pt in self.shortest_path:
            plot_curve(pt[0],pt[1],pt[2],pt[3],pt[4],c='red')

        self.shortest_path = np.array(self.shortest_path)
        plt.plot(self.shortest_path[:,0],self.shortest_path[:,1])
        
        
        plt.xlim(0,1050)
        plt.ylim(0,1050)
        plt.show()

    def astar(self):
        """
        A Star Alorithm
        """
        heapq.heapify(self.queue)
        heapq.heappush(self.queue,[self.cost(self.start[:2],0),self.start[0],self.start[1],self.start[2],0,0,0,self.start[0],self.start[1],self.start[2]])
        print("Planning...")
        while True:
            # Pop the element with least cost
            current = heapq.heappop(self.queue)
            self.visited.append(current)

            # Check Goal Condition
            if (np.linalg.norm(np.array(current[1:2])- np.array(self.goal[0:1])) <= 5):
                print("Goal Reached! ")

                # Perform backtracking
                self.backtrack()
                break 

            # Search for the available actions in the popped element of the queue
            self.actionsAvailable(current[1],current[2],current[3],current[4])

def cart2map(node):
    """
    Convert 

    :param      node:  The node
    :type       node:  { type_description }

    :returns:   { description_of_the_return_value }
    :rtype:     { return_type_description }
    """
    return((node[0]-510)/100,(node[1]-510)/100,node[2]*np.pi/180)

def angle_between(p1, p2):
    return (math.atan2(p2[1]-p1[1],p2[0]-p1[0]))*(180/np.pi)

def simulateTurtlebot(sp):
    """
    A function to move the turlebot

    :param      sp:   shortest path
    :type       sp:   list of points
    """
    pts = np.array(sp)[:,0:3]
    print("Shortest Path",sp)

    new_pts =[]
    for i in range(pts.shape[0]):
        new_pts.append(cart2map(pts[i]))
    pts = np.array(new_pts)[:,0:3]
    print("pts :", pts)
    prev_angle = 0
    diff = 0
    for j in range(1,pts.shape[0]-1):
        print(j)
        i = pts.shape[0]-j-1
        dist = np.sqrt((pts[i,0]-pts[i-1,0])**2 + (pts[i,1]-pts[i-1,1])**2)
        angle = angle_between(pts[i]-pts[i],pts[i-1]-pts[i])
        diff = angle-prev_angle
        prev_angle = angle
        print("args",dist,diff)
        print("points",pts[i],pts[i-1])
        ret = move(dist,diff)    

def main():
    
    print("Let the origin be at the bottom left corner of the Map!")

    start = []
    print("Enter start node in the format [x,y,theta] in (cm,cm,deg) format (Press enter after passing each element):")
    for i in range(0,3):
        x = input()
        start.append(int(x))

    goal = []
    print("Enter goal node in the format [x,y,theta] in (cm,cm,deg) format(Press enter after passing each element):")
    for i in range(0,3):
        x = input()
        goal.append(int(x))

    rpm1 = input("Enter the RPM1!")
    rpm2 = input("Enter the RPM2!")

    m = Map(start,goal,rpm1,rpm2)
    print("Start :",start)
    print("Goal :",goal)
    if m.isObstacle(start[0],start[1]):
        print("ERROR! Start Node is in the obstacle!")
        sys.exit()
    if m.isObstacle(goal[0],goal[1]):
        print("ERROR! Goal Node is in the obstacle!")
        sys.exit()

    start1 = cart2map(start)
    print("start1",start1)

    quat = [0,0,0,0]
    state_msg = ModelState()
    state_msg.model_name = 'mobile_base'
    state_msg.pose.position.x = start1[0]
    state_msg.pose.position.y = start1[1]
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]

    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state( state_msg )

    m.astar()

    simulateTurtlebot(m.shortest_path)


if __name__ == "__main__":
    main()
        
