#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import FSMState
import time

class SquareDriver:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.current_mode = None
        rospy.Subscriber('fsm_mode', FSMState, self.fsm_callback)
        rospy.on_shutdown(self.shutdown)

    def fsm_callback(self, msg):
        self.current_mode = msg.state

    def drive_straight(self, speed, duration):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.pub.publish(twist)
        self.pub.publish(Twist())  # Stop the robot after moving straight

    def turn(self, speed, duration):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = speed
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.pub.publish(twist)
        self.pub.publish(Twist())  # Stop the robot after turning

    def execute_square(self):
        if self.current_mode != 'AUTONOMOUS':
            return

        speed = 0.2  # Adjust as needed
        straight_duration = 5  # Adjust as needed
        turn_duration = 2  # Adjust as needed

        for _ in range(4):
            self.drive_straight(speed, straight_duration)
            time.sleep(5)  # Wait for 5 seconds
            self.turn(speed, turn_duration)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.pub.publish(Twist())  # Stop the robot
        rospy.sleep(1)

def main():
    rospy.init_node('square_driver', anonymous=True)
    driver = SquareDriver()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            driver.execute_square()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

