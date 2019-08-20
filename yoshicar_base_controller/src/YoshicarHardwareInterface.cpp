//
// Created by philipp on 13.08.19.
//

#include "../include/YoshicarHardwareInterface.h"
#include <std_msgs/Float64.h>

namespace yoshicar
{

HardwareInterface::HardwareInterface()
{
	ros::param::get("~velocity_gain", velocityGain);
	ros::param::get("~velocity_offset", velocityOffset);
	ros::param::get("~steering_angle_gain", steeringAngleGain);
	ros::param::get("~steering_angle_offset", steeringAngleOffset);
	ros::param::get("~min_forward_velocity", minForwardVelocity);
	ros::param::get("~min_backward_velocity", minBackwardVelocity);
	ros::param::get("~wheel_base", wheelBase);

	motorPublisher = nh.advertise<std_msgs::Float64>("motor", 1);
	servoPublisher = nh.advertise<std_msgs::Float64>("servo", 1);

	const std::string rearWheelJointName{"rear_wheel_joint"};
	// rear wheel state
	hardware_interface::JointStateHandle rearWheelStateHandle{
		rearWheelJointName, &rearWheelPosition, &rearWheelVelocity, &rearWheelEffort};
	jointStateInterface.registerHandle(rearWheelStateHandle);
	// rear wheel command
	hardware_interface::JointHandle rearWheelCommandHandle{
		rearWheelStateHandle, &rearWheelCommand};
	velocityJointInterface.registerHandle(rearWheelCommandHandle);

	const std::string frontSteerJointName{"front_steer_joint"};
	// front steer state
	hardware_interface::JointStateHandle frontSteerStateHandle{
		frontSteerJointName, &frontSteerPosition, &frontSteerVelocity, &frontSteerEffort};
	jointStateInterface.registerHandle(frontSteerStateHandle);
	// front steer command
	hardware_interface::JointHandle frontSteerCommandHandle{
		frontSteerStateHandle, &rearWheelCommand};
	positionJointInterface.registerHandle(frontSteerCommandHandle);

	registerInterface(&jointStateInterface);
	registerInterface(&velocityJointInterface);
	registerInterface(&positionJointInterface);

}

void HardwareInterface::read()
{
	// TODO: Estimate current velocity based on commands and IMU (using Kalman filter?)
	rearWheelVelocity = processMinVelocities(rearWheelCommand);
	frontSteerPosition = frontSteerCommand;
}

void HardwareInterface::write()
{
	// TODO: handle brake and backwards driving behavior of ESC
	double translationalVelocity = processMinVelocities(rearWheelCommand);
	double rotationalVelocity = frontSteerCommand;

	double motor = (translationalVelocity - velocityOffset) / velocityGain;
	double steeringAngle = convertTransRotVelToSteeringAngle(translationalVelocity, rotationalVelocity);
	double servo = (steeringAngle - steeringAngleOffset) / steeringAngleGain;

	std_msgs::Float64 msg;
	msg.data = motor;
	motorPublisher.publish(msg);
	msg.data = servo;
	servoPublisher.publish(msg);
}

double HardwareInterface::convertTransRotVelToSteeringAngle(double translationalVelocity,
															double rotationalVelocity)
{
	// see http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
	if (translationalVelocity == 0 || rotationalVelocity == 0)
		return 0;

	double radius = translationalVelocity / rotationalVelocity;
	return std::atan(wheelBase / radius);
}

double HardwareInterface::processMinVelocities(double velocity)
{
	if (velocity > 0 && velocity < minForwardVelocity)
		return 0;
	else if (velocity < 0 && velocity > minBackwardVelocity)
		return 0;
	return velocity;
}

}