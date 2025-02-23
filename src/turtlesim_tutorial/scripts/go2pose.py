#!/usr/bin/env python3
from cmath import cos, sin
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:
    def __init__(self):
        rospy.init_node('move_turtle', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.goal_pose.theta = rospy.get_param("~theta")  # Get final orientation
        self.distance_tolerance = rospy.get_param("~tol")
        self.angle_tolerance = rospy.get_param("~ang_tol", 0.05)  # Angular tolerance
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function to update current pose."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Calculate the Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Proportional linear velocity control."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculate the angle between the turtle's current position and the goal."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, target_angle, constant=6):
        """Proportional angular velocity control."""
        return constant * (target_angle - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal position first, then adjusts orientation."""
        goal_pose = Pose()
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y

        vel_msg = Twist()

        # **PHASE 1: Move to position**
        while self.euclidean_distance(goal_pose) >= self.distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(self.steering_angle(goal_pose))

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop movement after reaching position
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Reached Position!")

        # **PHASE 2: Adjust final orientation**
        rospy.loginfo("Adjusting orientation to theta=%.2f", self.goal_pose.theta)
        while abs(self.goal_pose.theta - self.pose.theta) >= self.angle_tolerance:
            vel_msg.angular.z = 0.5  # Small constant rotation

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()


        # Stop final movement
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Reached Final Orientation!")
        rospy.logwarn("Stopping Robot")

        rospy.spin()


if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass
