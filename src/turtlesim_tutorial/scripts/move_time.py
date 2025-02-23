#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_rubot(lin_velx, ang_vel, time_duration):
    """Moves the turtle for a given duration with specified velocities."""
    rospy.init_node('move_time', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    vel = Twist()
    vel.linear.x = lin_velx
    vel.angular.z = ang_vel

    time_begin = rospy.Time.now()  # Start time
    duration_s = 0  # Initialize duration

    while not rospy.is_shutdown():
        if duration_s <= time_duration:
            rospy.loginfo("Robot running... Time elapsed: %.2f sec" % duration_s)
            pub.publish(vel)
        else:
            rospy.logwarn("Stopping robot")
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            break  # Exit loop when time is exceeded

        time_end = rospy.Time.now()  # Update end time
        rospy.loginfo("Time_end = " + str(time_end))  # Log time_end
        duration = time_end - time_begin  # Compute elapsed duration
        duration_s = duration.to_sec()  # Convert to seconds
        
        rate.sleep()

if __name__ == '__main__':
    try:
        # Get parameters from launch file or command line
        v = rospy.get_param("~v", 2.0)  # Default linear velocity
        w = rospy.get_param("~w", 1.0)  # Default angular velocity
        t = rospy.get_param("~t", 5.0)  # Default duration (5 seconds)

        move_rubot(v, w, t)
    except rospy.ROSInterruptException:
        pass
