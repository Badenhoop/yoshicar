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
	ros::init(argc, argv, "remote_control_sender_node");
	ros::NodeHandle n;

	std::string receiverAddress;
	int receiverPort;
	int senderPort;
	n.param("receiver_address", receiverAddress, std::string{"127.0.0.1"});
	n.param("receiver_port", receiverPort, 10000);
	n.param("sender_port", senderPort, 10001);

	asionet::Context context;
	asionet::Worker worker{context};
	auto sender = std::make_shared<remoteControl::Sender>(context, receiverAddress, receiverPort, senderPort);

	ros::Subscriber motorSubscriber = n.subscribe<std_msgs::Float64>(
		"motor", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG("sending motor value: %f", msg->data);
			sender->setControlValue(MOTOR_CONTROL_TYPE, msg->data);
		});
	ros::Subscriber servoSubscriber = n.subscribe<std_msgs::Float64>(
		"servo", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG("sending servo value: %f", msg->data);
			sender->setControlValue(SERVO_CONTROL_TYPE, msg->data);
		});
	sender->run();
	ros::spin();
	return 0;
}