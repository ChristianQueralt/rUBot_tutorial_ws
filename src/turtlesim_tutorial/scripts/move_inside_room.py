#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

# Define room limits
ROOM_X_MIN = 1.0
ROOM_X_MAX = 10.0
ROOM_Y_MIN = 1.0
ROOM_Y_MAX = 10.0

robot_x = 5.5  # Initial position (assumed center of the room)
robot_y = 5.5

def pose_callback(pose):
    """Callback function to update the turtle's position"""
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f, Robot Y = %f", pose.x, pose.y)

def move_turtle(lin_vel, ang_vel):
    """Moves the turtle inside the room while staying within boundaries"""
    global robot_x, robot_y

    rospy.init_node('move_inside_room', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rate = rospy.Rate(10)  # 10 Hz
    vel = Twist()

    rospy.loginfo("Moving inside the room...")

    move_direction = random.randint(1, 4)  # Choose a random direction
    
    while not rospy.is_shutdown():
        if ROOM_X_MIN < robot_x < ROOM_X_MAX and ROOM_Y_MIN < robot_y < ROOM_Y_MAX and move_direction == 1:
            vel.linear.x = lin_vel
            vel.linear.y = 0
            vel.angular.z = 0

        elif ROOM_X_MIN < robot_x < ROOM_X_MAX and ROOM_Y_MIN < robot_y < ROOM_Y_MAX and move_direction == 2:
            vel.linear.x = -lin_vel
            vel.linear.y = 0
            vel.angular.z = 0
        
        elif ROOM_X_MIN < robot_x < ROOM_X_MAX and ROOM_Y_MIN < robot_y < ROOM_Y_MAX and move_direction == 3:
            vel.linear.x = 0
            vel.linear.y = lin_vel
            vel.angular.z = 0

        elif ROOM_X_MIN < robot_x < ROOM_X_MAX and ROOM_Y_MIN < robot_y < ROOM_Y_MAX and move_direction == 4:
            vel.linear.x = 0
            vel.linear.y = -lin_vel
            vel.angular.z = 0

        else:
            rospy.logwarn("Robot reached the room limits! Stopping...")
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0

        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        v = rospy.get_param("~v", 2.0)  # Default linear velocity
        w = rospy.get_param("~w", 1.0)  # Default angular velocity
        move_turtle(v, w)
    except rospy.ROSInterruptException:
        pass
