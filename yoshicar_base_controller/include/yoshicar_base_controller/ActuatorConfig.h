#ifndef YOSHICAR_BASE_CONTROLLER_ACTUATORCONFIG_H
#define YOSHICAR_BASE_CONTROLLER_ACTUATORCONFIG_H

namespace yoshicar
{
class ActuatorConfig
{
public:
  ActuatorConfig();

  double motorCommandToVelocity(double motorCommand) const;
  double velocityToMotorCommand(double velocity) const;

  double servoCommandToSteeringAngle(double servoCommand) const;
  double steeringAngleToServoCommand(double steeringAngle) const;

  double getMinForwardVelocity() const
  {
    return minForwardVelocity;
  }

  double getMinReverseVelocity() const
  {
    return minReverseVelocity;
  }

private:
  double velocityGain;
  double velocityOffset;
  double steeringAngleGain;
  double steeringAngleOffset;
  double minForwardVelocity;
  double minReverseVelocity;
};

}  // namespace yoshicar

#endif  // YOSHICAR_BASE_CONTROLLER_ACTUATORCONFIG_H
