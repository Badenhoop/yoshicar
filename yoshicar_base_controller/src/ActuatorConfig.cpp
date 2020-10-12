#include "yoshicar_base_controller/ActuatorConfig.h"
#include <ros/ros.h>

namespace yoshicar
{
ActuatorConfig::ActuatorConfig()
{
  ros::NodeHandle nhPriv;
  velocityGain = nhPriv.param<double>("velocity_gain", 0.);
  velocityOffset = nhPriv.param<double>("velocity_offset", 0.);
  steeringAngleGain = nhPriv.param<double>("steering_angle_gain", 0.);
  steeringAngleOffset = nhPriv.param<double>("steering_angle_offset", 0.);
  minForwardVelocity = nhPriv.param<double>("min_forward_velocity", 0.);
  minReverseVelocity = nhPriv.param<double>("min_reverse_velocity", 0.);
}

double ActuatorConfig::motorCommandToVelocity(double motorCommand) const
{
  return velocityGain * motorCommand + velocityOffset;
}

double ActuatorConfig::velocityToMotorCommand(double velocity) const
{
  if (velocityGain == 0.)
    return 0.;

  return (velocity - velocityOffset) / velocityGain;
}

double ActuatorConfig::servoCommandToSteeringAngle(double servoCommand) const
{
  return steeringAngleGain * servoCommand + steeringAngleOffset;
}

double ActuatorConfig::steeringAngleToServoCommand(double steeringAngle) const
{
  if (steeringAngleGain == 0.)
    return 0.;

  return (steeringAngle - steeringAngleOffset) / steeringAngleGain;
}

}