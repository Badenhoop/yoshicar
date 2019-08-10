//
// Created by philipp on 18.04.19.
//

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <asionet/Worker.h>
#include "../include/Receiver.h"

static constexpr int MOTOR_CONTROL_TYPE = 1;
static constexpr int SERVO_CONTROL_TYPE = 2;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remote_control_receiver_node");
	ros::NodeHandle n;
	ros::Publisher motorPublisher = n.advertise<std_msgs::Float64>("pwm/motor", 1);
	ros::Publisher servoPublisher = n.advertise<std_msgs::Float64>("pwm/servo", 1);

	asionet::Context context;
	asionet::Worker worker{context};
	using namespace std::chrono_literals;
	auto receiver = std::make_shared<remoteControl::Receiver>(
		context, 500ms,
		[&]
		{
			std_msgs::Float64 msg;
			msg.data = 0.0;
			motorPublisher.publish(msg);
		},
		[&](int controlType, double controlValue)
		{
			std_msgs::Float64 msg;
			msg.data = controlValue;
			if (controlType == MOTOR_CONTROL_TYPE)
			{
				ROS_DEBUG("received motor value: %f", controlValue);
				motorPublisher.publish(msg);
			}
			else if (controlType == SERVO_CONTROL_TYPE)
			{
				ROS_DEBUG("received servo value: %f", controlValue);
				servoPublisher.publish(msg);
			}
		});
	receiver->run();
	ros::spin();
	return 0;
}