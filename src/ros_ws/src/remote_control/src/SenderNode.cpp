//
// Created by philipp on 18.04.19.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <asionet/Worker.h>
#include "../include/Sender.h"

static constexpr int MOTOR_CONTROL_TYPE = 1;
static constexpr int SERVO_CONTROL_TYPE = 2;

int main(int argc, char ** argv)
{
	asionet::Context context;
	asionet::Worker worker{context};
	auto sender = std::make_shared<remoteControl::Sender>(context, "192.168.178.135");

	ros::init(argc, argv, "remote_control_sender_node");
	ros::NodeHandle n;
	ros::Subscriber motorSubscriber = n.subscribe<std_msgs::Float64>(
		"remote_control/motor", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG("sending motor value: %f", msg->data);
			sender->setControlValue(MOTOR_CONTROL_TYPE, msg->data);
		});
	ros::Subscriber servoSubscriber = n.subscribe<std_msgs::Float64>(
		"remote_control/servo", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG("sending servo value: %f", msg->data);
			sender->setControlValue(SERVO_CONTROL_TYPE, msg->data);
		});
	sender->run();
	ros::spin();
	return 0;
}