#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

#Few things to work
#1.Check the function of polynomial_time_scaling_3rd_order
#2.Change the p_start and p_end to current way point 

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def get_path_from_A_star(self,start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]
    #create open and closed list
        def heuristic_estimate_of_distance(startpoint, goalpoint):
            return sqrt(pow((goalpoint[0] - startpoint[0]), 2) + pow((goalpoint[1] - startpoint[1]), 2))
        open_list = []
        closed_list = []
	boundary = [(0.5,0), (1,0), (1.5,0), (2,0), (2.5,0), (3,0), (3.5,0),(4,0),(4.5,0),(5,0),(5.5,0),(5.5,0.5),(5.5,1),(5.5,1.5),(5.5,2),(5.5,2.5),(0.5,2.5),(1,2.5),(1.5,2.5),(2,2.5),(2.5,2.5),(3,2.5),(3.5,2.5),(4,2.5),(4.5,2.5),(5,2.5)]
	reference_bound = [(0.5 - start[0],0 - start[1]), (1 - start[0],0 - start[1]), (1.5 - start[0],0 - start[1]), (2 - start[0],0 - start[1]), (2.5 - start[0],0 - start[1]), (3 - start[0],0 - start[1]), (3.5 - start[0],0 - start[1]),(4 - start[0],0 - start[1]),(4.5 - start[0],0 - start[1]),(5 - start[0],0 - start[1]),(5.5 - start[0],0 - start[1]),(5.5 - start[0],0.5 - start[1]),(5.5 - start[0],1 - start[1]),(5.5 - start[0],1.5 - start[1]),(5.5 - start[0],2 - start[1]),(5.5 - start[0],2.5 - start[1]),(0.5 - start[0],2.5 - start[1]),(1 - start[0],2.5 - start[1]),(1.5 - start[0],2.5 - start[1]),(2 - start[0],2.5 - start[1]),(2.5 - start[0],2.5 - start[1]),(3 - start[0],2.5 - start[1]),(3.5 - start[0],2.5 - start[1]),(4 - start[0],2.5 - start[1]),(4.5 - start[0],2.5 - start[1]),(5 - start[0],2.5 - start[1])]
	
        #add start to open list
        open_list.append(start)
        g_score = {}
        h_score = {}
        f_score = {}
        came_from = {}
        g_score[start] = 0        #g(n)
        h_score[start] = heuristic_estimate_of_distance(start, goal)    #h(start)
        f_score[start] = h_score[start] #f(n)=h(n)+g(n)
        #create function to compute total cost
        
        def total_cost(currentpoint):
            P_cost = sqrt(pow((currentpoint[0] - start[0]), 2) + pow((currentpoint[1] - start[1]), 2))
            H_cost = sqrt(pow((goal[0] - currentpoint[0]), 2) + pow((goal[1] - currentpoint[1]), 2))
            total = P_cost + H_cost
            return total

        #compute start and goal total cost

        def reconstruct_path(came_from, current_point):
            total_path = []
            while current_point in came_from.keys():
                current_point = came_from[current_point]
                total_path.append(current_point)
            total_path.remove(start)
            total_path.reverse()
            total_path.append(goal)
            return total_path

        #loop until open list is empty
        while len(open_list) > 0:
            #initialize current point cost and current point
            current_point_cost = total_cost(open_list[0])
            current_point = open_list[0]

            #find the current point with lowest total cost
            for i in open_list:
                if total_cost(i) < current_point_cost:
                    current_point = i

            if current_point == goal:
                return  reconstruct_path(came_from, goal)
            #remove current point from open list
            open_list.remove(current_point)

            #add current point to closed list
            closed_list.append(current_point)

            #check if current point have reached to the goal, return path

            
            #get current point x and y position
            (x, y) = (current_point[0], current_point[1])

            #get neighbors
            neighbor_list = [(x-0.5, y), (x+0.5, y), (x, y-0.5), (x, y+0.5)]
            
            def dist_between(pointA, pointB):
                return sqrt(pow((pointA[0] - pointB[0]), 2) + pow((pointA[1] - pointB[1]), 2))

            for n in neighbor_list:
                if (n in closed_list) or (n in obstacles) or (n in reference_bound):
                    continue
                tentative_g_score = g_score[current_point] + dist_between(current_point,n)
                if n not in open_list:
                    tentative_is_better = True
                elif tentative_g_score < g_score[n]:
                    tentative_is_better = True
                else:
                    tentative_is_better = False
                if tentative_is_better == True:
                    came_from[n] = current_point
                    g_score[n] = tentative_g_score    
                    h_score[n] = heuristic_estimate_of_distance(n, goal) 
                    f_score[n] = g_score[n] + h_score[n]
                    open_list.append(n)
        return False    

    def run(self):
	start = (0,0.5)
	goal = (5-start[0],1-start[1])
        obstacles = [(1.5-start[0],0-start[1]),(1.5-start[0],0.5-start[1]),(1.5-start[0],1-start[1]),(1.5-start[0],1.5-start[1]),(3-start[0],1-start[1]),(3-start[0],1.5-start[1]),(3-start[0],2-start[1])]
	
        waypoints = self.get_path_from_A_star(start, goal, obstacles)
        self.posix = 0
        self.posiy = 0
        self.velx = 0
        self.vely = 0
        vel = Twist()
	for i in range(len(waypoints)-1):
            self.move_to_point(list(waypoints[i]), list(waypoints[i+1]))
	    if abs(self.pose.x - goal[0]) < 0.5:
                vel.linear.x = 0
                vel.angular.z = 0
                for i in range(30):
                    self.vel_pub.publish(vel)
                    self.rate.sleep()   #Stop   
                break 

    

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint

        #To get coefficient matrix
        C_x = self.polynomial_time_scaling_3rd_order(self.posix, self.velx, current_waypoint[0], (next_waypoint[0] - self.posix)/3, 3)
        C_y = self.polynomial_time_scaling_3rd_order(self.posiy, self.vely, current_waypoint[1], (next_waypoint[1] - self.posiy)/3, 3)
	self.velx = (next_waypoint[0] - self.posix)/3
	self.vely = (next_waypoint[1] - self.posiy)/3      
	self.posix = current_waypoint[0]
	self.posiy = current_waypoint[1]
        vel = Twist()
        pid = Controller()
        pid.setPD(1,0.01)
        for j in range(31):
            t = j/10.0

            #Position formulation
            P_x = C_x[3,0] + C_x[2,0]*t + C_x[1,0]*pow(t,2) + C_x[0,0]*pow(t,3)
            P_y = C_y[3,0] + C_y[2,0]*t + C_y[1,0]*pow(t,2) + C_y[0,0]*pow(t,3)

            #Velocity formulation
            V_x = C_x[2,0] + 2*C_x[1,0]*t + 3*C_x[0,0]*pow(t,2)
            V_y = C_y[2,0] + 2*C_y[1,0]*t + 3*C_y[0,0]*pow(t,2)

            #PID controller for angular velocity
            pid.setPoint(atan2(V_y, V_x)) #Should check

            vel.linear.x = sqrt(pow(V_x,2)+pow(V_y,2))
            vel.angular.z = pid.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()
            

        #Finish one segment

        pass


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial

        #(Pending)boundary condition matrix
        x = np.array([[p_start],[p_end],[v_start],[v_end]])
        
        #T matrix
        T_matrix = np.array([[0,0,0,1],[pow(T,3),pow(T,2),T,1],[0,0,1,0],[3*pow(T,2),2*T,1,0]])
        
        #coefficients matrics
        C_matrix = np.dot(np.linalg.inv(T_matrix),x)
        

        return C_matrix


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


class Controller:
    def __init__(self, P=0.0, D=0.0, Derivator=0):
        self.Kp = P
        self.Kd = D
        self.Derivator = Derivator
        self.set_point = 0.0
        self.error = 0.0
	self.previous_error = 0
        

    def update(self, current_value):
        # calculate P_value and D_value
	# calculate P_term and D_term
        error = self.set_point - current_value
        if error > pi:
  	# specific design for circular situation
            error = error - 2*pi
        elif error < -pi:
            error = error + 2*pi
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term



    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
    
    def setPD(self, set_P=0.0, set_D=0.0):
        self.Kp = set_P
        self.Kd = set_D


if __name__ == '__main__':
    whatever = Turtlebot()

import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # load csv file and plot trajectory 
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

    plt.xlim(0, 10)
    plt.ylim(0, 5)
    plt.show()

if __name__ == '__main__':
    visualization()