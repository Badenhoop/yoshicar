#ifndef YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H
#define YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <yoshicar_msgs/VelocityEstimate.h>
#include "ActuatorConfig.h"

namespace yoshicar
{
class MotorState;
using MotorStatePtr = std::unique_ptr<MotorState>;

class MotorState
{
public:
  MotorState() : startTime(ros::Time::now())
  {
  }

  virtual MotorStatePtr targetVelocityEvent(double targetVelocity) = 0;
  virtual MotorStatePtr isDrivingEvent(bool isDriving) = 0;
  virtual MotorStatePtr updateEvent() = 0;

  virtual double getMotorCommand() const = 0;
  virtual yoshicar_msgs::VelocityEstimate getVelocityEstimate() const = 0;

protected:
  ros::Time startTime;
};

class StoppedMotorState : public MotorState
{
public:
  StoppedMotorState();

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  double brakeReleaseDuration;
};

class ReleasedBrakeMotorState : public MotorState
{
public:
  ReleasedBrakeMotorState() : MotorState()
  {
  }

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;
};

class ForwardDrivingMotorState : public MotorState
{
public:
  explicit ForwardDrivingMotorState(double currTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  ActuatorConfig config;
  double currTargetVelocity;
};

class ForwardBrakingMotorState : public MotorState
{
public:
  explicit ForwardBrakingMotorState(double prevTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  double prevTargetVelocity;
  double brakeMotorCommand;
};

class ForwardCommandMotorState : public MotorState
{
public:
  explicit ForwardCommandMotorState(double currTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  double currTargetVelocity;
  ActuatorConfig config;
};

class ReverseDrivingMotorState : public MotorState
{
public:
  explicit ReverseDrivingMotorState(double currTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  ActuatorConfig config;
  double currTargetVelocity;
};

class ReverseBrakingMotorState : public MotorState
{
public:
  explicit ReverseBrakingMotorState(double prevTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  double prevTargetVelocity;
  double brakeMotorCommand;
};

class ReverseCommandMotorState : public MotorState
{
public:
  explicit ReverseCommandMotorState(double currTargetVelocity);

  MotorStatePtr targetVelocityEvent(double targetVelocity) override;
  MotorStatePtr isDrivingEvent(bool isDriving) override;
  MotorStatePtr updateEvent() override;

  double getMotorCommand() const override;
  yoshicar_msgs::VelocityEstimate getVelocityEstimate() const override;

private:
  double currTargetVelocity;
  ActuatorConfig config;
};

class MotorStateMachine
{
public:
  MotorStateMachine();

private:
  ros::Subscriber targetVelocitySubscriber;
  ros::Subscriber isDrivingSubscriber;

  ros::Publisher motorCommandPublisher;
  ros::Publisher estimatedVelocityPublisher;

  ros::Timer stateUpdateTimer;

  MotorStatePtr state;

  void targetVelocityCallback(const std_msgs::Float64ConstPtr& msg);
  void isDrivingCallback(const std_msgs::BoolConstPtr& msg);
  void updateCallback();

  void performEvent(const std::function<MotorStatePtr()>& event);
};

}  // namespace yoshicar

#endif  // YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H