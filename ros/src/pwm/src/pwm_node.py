#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from adafruit_servokit import ServoKit

def callback(data, args):
    kit, channel, name = args
    value = min(max(data.data, -1.0), 1.0)
    rospy.logdebug(rospy.get_caller_id() + '%s: %f', name, value)
    kit.continuous_servo[channel].throttle = value
    
def main():
    rospy.init_node('pwm_node')

    motor_channel = rospy.get_param('motor_channel', 0)
    servo_channel = rospy.get_param('servo_channel', 1)

    kit = ServoKit(channels=16)
    kit.continuous_servo[motor_channel].set_pulse_width_range(1000, 2000)
    kit.continuous_servo[servo_channel].set_pulse_width_range(1000, 2000)

    rospy.Subscriber("motor", Float64, callback, (kit, motor_channel, 'motor'))
    rospy.Subscriber("servo", Float64, callback, (kit, servo_channel, 'servo'))
    rospy.spin()
  
if __name__ == '__main__':
    main()
