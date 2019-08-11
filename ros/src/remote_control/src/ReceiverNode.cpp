//
// Created by philipp on 18.04.19.
//

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <asionet/Worker.h>
#include "../include/Receiver.h"

static constexpr int MOTOR_CONTROL_TYPE = 1;
static constexpr int SERVO_CONTROL_TYPE = 2;

auto duration(double seconds)
{
	long long nanoseconds = seconds * 1000000000.0;
	return std::chrono::nanoseconds{nanoseconds};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "remote_control_receiver_node");
	ros::NodeHandle nhPublic;
	ros::Publisher motorPublisher = nhPublic.advertise<std_msgs::Float64>("motor", 1);
	ros::Publisher servoPublisher = nhPublic.advertise<std_msgs::Float64>("servo", 1);

	int receiverPort;
	int senderPort;
	double timeoutSeconds;
	ros::NodeHandle nhPrivate{"~"};
	nhPrivate.param("receiver_port", receiverPort, 10000);
	nhPrivate.param("sender_port", senderPort, 10001);
	nhPrivate.param("receiver_timeout", timeoutSeconds, 0.5);

	auto timeout = duration(timeoutSeconds);

	asionet::Context context;
	asionet::Worker worker{context};
	using namespace std::chrono_literals;
	auto receiver = std::make_shared<remoteControl::Receiver>(
		context, receiverPort, senderPort, timeout,
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
				ROS_DEBUG_STREAM(ros::this_node::getName() << ": motor=" << msg.data);
				motorPublisher.publish(msg);
			}
			else if (controlType == SERVO_CONTROL_TYPE)
			{
				ROS_DEBUG_STREAM(ros::this_node::getName() << ": servo=" << msg.data);
				servoPublisher.publish(msg);
			}
		});
	receiver->run();
	ros::spin();
	return 0;
}
