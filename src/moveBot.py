#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
import sys


def move(distance,angle):
    # Starts a new node
    rospy.init_node('turtlebot_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    vel_msg = Twist()

    angle = angle * 3.14/180
    distance = distance
    speed = 0.2
    w = 0.5

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #Setting the current time for distance calculus
    current_distance = 0
    current_angle = 0

    # print("Angle :",angle)
    #Loop to move the turtle in an specified distance
    t0 = rospy.Time.now().to_sec()
    while(current_distance < distance):
        # rospy.loginfo("Distance %s",current_distance)
        #Publish the velocity
        vel_msg.linear.x = speed
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    # #After the loop, stops the robot
    vel_msg.linear.x = 0

    # Loop to move the turtle in an specified angle
    t0 = rospy.Time.now().to_sec()
    while (current_angle < abs(angle)):
        # print("Angle ",angle)
        if angle>=0:
            # print("1")
            vel_msg.angular.z=w
            if angle>180:
                vel_msg.angular.z=-w
                angle = angle-180
        if angle<0:
            # print("2")
            vel_msg.angular.z=-w
        t1=rospy.Time.now().to_sec()
        velocity_publisher.publish(vel_msg)
        current_angle = (w*(t1-t0))
        # rospy.loginfo("Angle %s",current_angle)

    vel_msg.angular.z=0

    #Force the robot to stop
    # velocity_publisher.publish(vel_msg)

    # for t in range(1000):
    #     velocity_publisher.publish(vel_msg)
    return 0
if __name__ == "__main__":
    move(0,90)