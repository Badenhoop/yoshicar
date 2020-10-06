#ifndef YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H
#define YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H

#include <mutex>
#include <memory>
#include <ros/ros.h>

namespace yoshicar
{
struct MotorState
{
  double targetVelocity;
  double currentVelocity;
  bool isDriving;
  ros::Time startTime;
  double motorCommand;
};

class MotorStateMachine
{
public:
  void setTargetVelocity(double velocity);
  double getCurrentVelocity(double velocity);

private:
  std::mutex stateMutex;
};

}  // namespace yoshicar

#endif  // YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H
