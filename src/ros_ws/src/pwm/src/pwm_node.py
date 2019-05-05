#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from adafruit_servokit import ServoKit

PWM_FREQ = 50
MOTOR_CHANNEL = 13
SERVO_CHANNEL = 14

kit = ServoKit(channels=16)
kit.servo[MOTOR_CHANNEL].set_pulse_width_range(1000, 2000)
kit.servo[SERVO_CHANNEL].set_pulse_width_range(1000, 2000)

def motor_callback(data):
    value = min(max(data.data, -1.0), 1.0)
    rospy.logdebug(rospy.get_caller_id() + "motor: %f", value)
    kit.continuous_servo[MOTOR_CHANNEL].throttle = value

def servo_callback(data):
    value = min(max(data.data, -1.0), 1.0)
    rospy.logdebug(rospy.get_caller_id() + "servo %f", value)
    kit.continuous_servo[SERVO_CHANNEL].throttle = value
    
def main():
    rospy.init_node('pwm')
    rospy.Subscriber("pwm/motor", Float64, motor_callback)
    rospy.Subscriber("pwm/servo", Float64, servo_callback)
    rospy.spin()
  
if __name__ == '__main__':
    main()
