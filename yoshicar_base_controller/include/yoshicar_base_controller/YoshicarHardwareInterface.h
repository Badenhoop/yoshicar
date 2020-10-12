//
// Created by philipp on 13.08.19.
//

#ifndef YOSHICAR_CONTROL_YOSHICARHARDWAREINTERFACE_H
#define YOSHICAR_CONTROL_YOSHICARHARDWAREINTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "ros/ros.h"

namespace yoshicar
{
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  HardwareInterface();
  void read();
  void write();

private:
  double velocityGain;
  double velocityOffset;
  double steeringAngleGain;
  double steeringAngleOffset;

  double minForwardVelocity;
  double minBackwardVelocity;

  double wheelBase;

  ros::NodeHandle nh;
  ros::Publisher motorPublisher;
  ros::Publisher servoPublisher;

  hardware_interface::JointStateInterface jointStateInterface;
  hardware_interface::VelocityJointInterface velocityJointInterface;
  hardware_interface::PositionJointInterface positionJointInterface;

  double rearWheelPosition;
  double rearWheelVelocity;
  double rearWheelEffort;
  double rearWheelCommand;

  double frontSteerPosition;
  double frontSteerVelocity;
  double frontSteerEffort;
  double frontSteerCommand;

  double convertTransRotVelToSteeringAngle(double translationalVelocity, double rotationalVelocity);
  double processMinVelocities(double velocity);
};

}  // namespace yoshicar

#endif  // YOSHICAR_CONTROL_YOSHICARHARDWAREINTERFACE_H
