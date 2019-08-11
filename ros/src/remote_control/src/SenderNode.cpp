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

	ros::NodeHandle nhPrivate{"~"};
	std::string receiverAddress;
	int receiverPort;
	int senderPort;
	nhPrivate.param("receiver_address", receiverAddress, std::string{"127.0.0.1"});
	nhPrivate.param("receiver_port", receiverPort, 10000);
	nhPrivate.param("sender_port", senderPort, 10001);

	asionet::Context context;
	asionet::Worker worker{context};
	auto sender = std::make_shared<remoteControl::Sender>(context, receiverAddress, receiverPort, senderPort);

	ros::NodeHandle nhPublic;
	ros::Subscriber motorSubscriber = nhPublic.subscribe<std_msgs::Float64>(
		"motor", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG_STREAM(ros::this_node::getName() << ": motor=" << msg->data);
			sender->setControlValue(MOTOR_CONTROL_TYPE, msg->data);
		});
	ros::Subscriber servoSubscriber = nhPublic.subscribe<std_msgs::Float64>(
		"servo", 1,
		[sender](const auto & msg)
		{
			ROS_DEBUG_STREAM(ros::this_node::getName() << ": servo=" << msg->data);
			sender->setControlValue(SERVO_CONTROL_TYPE, msg->data);
		});
	sender->run();
	ros::spin();
	return 0;
}
