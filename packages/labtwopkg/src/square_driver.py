#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class SquareDriver:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)

        self.is_autonomous = False
        self.last_time = rospy.Time.now()
        self.state = "STOP"

    def fsm_callback(self, msg):
        if msg.state == 'LANE_FOLLOWING':
            if not self.is_autonomous:
                self.is_autonomous = True
                self.last_time = rospy.Time.now()
                self.state = "DRIVE_STRAIGHT"
        else:
            self.is_autonomous = False
            self.send_stop()

    def send_drive_command(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_pub.publish(cmd_msg)

    def send_stop(self):
        self.send_drive_command(0, 0)

def control_loop(self):
    if not self.is_autonomous:
        return

    current_time = rospy.Time.now()
    elapsed = (current_time - self.last_time).to_sec()

    if self.state == "DRIVE_STRAIGHT" and elapsed >= 4.0:  # Time to drive 1m
        self.send_stop()
        self.last_time = current_time
        self.state = "STOP"
    elif self.state == "STOP" and elapsed >= 5.0:
        self.send_drive_command(0, 1.0)  # Turn 90 degrees
        self.last_time = current_time
        self.state = "TURN"
    elif self.state == "TURN" and elapsed >= 1.0:
        self.send_drive_command(0.25, 0)  # Drive straight again
        self.last_time = current_time
        self.state = "DRIVE_STRAIGHT"

if __name__ == '__main__':
    rospy.init_node('square_driver')
    driver = SquareDriver()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        driver.control_loop()
        rate.sleep()

