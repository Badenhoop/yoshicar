#include "yoshicar_base_controller/MotorStateMachine.h"

namespace yoshicar
{
double varianceDecay(double t, double vStart, double vEnd, double duration)
{
  double vDelta = vStart - vEnd;
  return vStart - (vDelta / (1 + std::exp(-(12. / duration * t - 6.))));
}

MotorStateMachine::MotorStateMachine()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhPriv;

  targetVelocitySubscriber =
      nh.subscribe<std_msgs::Float64>("target_velocity", 1, &MotorStateMachine::targetVelocityCallback, this);
  isDrivingSubscriber = nh.subscribe<std_msgs::Bool>("is_driving", 1, &MotorStateMachine::isDrivingCallback, this);

  motorCommandPublisher = nh.advertise<std_msgs::Float64>("motor", 1);
  estimatedVelocityPublisher = nh.advertise<yoshicar_msgs::VelocityEstimate>("msm_velocity_estimate", 1);

  auto updatePeriod = nhPriv.param<double>("msm_update_period", 0.01);
  stateUpdateTimer = nh.createTimer(
      ros::Duration(updatePeriod), [this] { updateCallback(); }, false);

  state = std::make_unique<StoppedMotorState>();
}

void MotorStateMachine::targetVelocityCallback(const std_msgs::Float64ConstPtr& msg)
{
  performEvent([&] { return state->targetVelocityEvent(msg->data); });
}

void MotorStateMachine::isDrivingCallback(const std_msgs::BoolConstPtr& msg)
{
  performEvent([&] { return state->isDrivingEvent(msg->data); });
}

void MotorStateMachine::updateCallback()
{
  performEvent([&] { return state->updateEvent(); });
  std_msgs::Float64 motorCommand;
  motorCommand.data = state->getMotorCommand();
  motorCommandPublisher.publish(motorCommand);
  estimatedVelocityPublisher.publish(state->getVelocityEstimate());
}

void MotorStateMachine::performEvent(const std::function<MotorStatePtr()>& event)
{
  auto newState = event();
  if (newState == nullptr)
    return;
  state = std::move(newState);
}

StoppedMotorState::StoppedMotorState() : MotorState()
{
  ros::NodeHandle nhPriv{ "~" };
  brakeReleaseDuration = nhPriv.param<double>("brake_release_duration", 0.2);
}

MotorStatePtr StoppedMotorState::targetVelocityEvent(double targetVelocity)
{
  return nullptr;
}

MotorStatePtr StoppedMotorState::isDrivingEvent(bool isDriving)
{
  return nullptr;
}

MotorStatePtr StoppedMotorState::updateEvent()
{
  if (ros::Time::now() - startTime > ros::Duration{ brakeReleaseDuration })
    return std::make_unique<ReleasedBrakeMotorState>();

  return nullptr;
}

double StoppedMotorState::getMotorCommand() const
{
  return 0.;
}

yoshicar_msgs::VelocityEstimate StoppedMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  estimate.variance = 0.01;
  return estimate;
}

MotorStatePtr ReleasedBrakeMotorState::targetVelocityEvent(double targetVelocity)
{
  if (targetVelocity > 0.)
    return std::make_unique<ForwardCommandMotorState>(targetVelocity);

  if (targetVelocity < 0.)
    return std::make_unique<ReverseCommandMotorState>(targetVelocity);

  return nullptr;
}

MotorStatePtr ReleasedBrakeMotorState::isDrivingEvent(bool isDriving)
{
  return nullptr;
}

MotorStatePtr ReleasedBrakeMotorState::updateEvent()
{
  return nullptr;
}

double ReleasedBrakeMotorState::getMotorCommand() const
{
  return 0.;
}

yoshicar_msgs::VelocityEstimate ReleasedBrakeMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  estimate.variance = 0.01;
  return estimate;
}

ForwardDrivingMotorState::ForwardDrivingMotorState(double currTargetVelocity)
  : MotorState(), currTargetVelocity(currTargetVelocity)
{
}

MotorStatePtr ForwardDrivingMotorState::targetVelocityEvent(double targetVelocity)
{
  if (targetVelocity <= 0.)
    return std::make_unique<ForwardBrakingMotorState>(currTargetVelocity);

  double currMotorCommand = config.velocityToMotorCommand(currTargetVelocity);
  double nextMotorCommand = config.velocityToMotorCommand(targetVelocity);
  if (currMotorCommand == nextMotorCommand)
    return nullptr;

  // If the new target velocity results in a new motor command, we perform
  // a self transition with the new target velocity
  return std::make_unique<ForwardDrivingMotorState>(targetVelocity);
}

MotorStatePtr ForwardDrivingMotorState::isDrivingEvent(bool isDriving)
{
  if (isDriving)
    return nullptr;

  // This may only happen if the motor fails for some reason
  return std::make_unique<StoppedMotorState>();
}

MotorStatePtr ForwardDrivingMotorState::updateEvent()
{
  return nullptr;
}

double ForwardDrivingMotorState::getMotorCommand() const
{
  return config.velocityToMotorCommand(currTargetVelocity);
}

yoshicar_msgs::VelocityEstimate ForwardDrivingMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;

  if (currTargetVelocity < config.getMinForwardVelocity())
  {
    estimate.velocity = 0.;
    estimate.variance = 0.01;
    return estimate;
  }

  estimate.velocity = currTargetVelocity;
  double t = (ros::Time::now() - startTime).toSec();
  estimate.variance = varianceDecay(t, currTargetVelocity, currTargetVelocity * 0.1, 1.);
  return estimate;
}

ForwardBrakingMotorState::ForwardBrakingMotorState(double prevTargetVelocity)
  : MotorState(), prevTargetVelocity(prevTargetVelocity)
{
  ros::NodeHandle nhPriv;
  brakeMotorCommand = nhPriv.param<double>("forward_brake_motor_command", -1.);
}

MotorStatePtr ForwardBrakingMotorState::targetVelocityEvent(double targetVelocity)
{
  if (targetVelocity <= 0.)
    return nullptr;

  return std::make_unique<ForwardDrivingMotorState>(targetVelocity);
}

MotorStatePtr ForwardBrakingMotorState::isDrivingEvent(bool isDriving)
{
  if (isDriving)
    return nullptr;

  return std::make_unique<StoppedMotorState>();
}

MotorStatePtr ForwardBrakingMotorState::updateEvent()
{
  if (ros::Time::now() - startTime > ros::Duration{ 5. })
    return std::make_unique<StoppedMotorState>();

  return nullptr;
}

double ForwardBrakingMotorState::getMotorCommand() const
{
  return brakeMotorCommand;
}

yoshicar_msgs::VelocityEstimate ForwardBrakingMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  double t = (ros::Time::now() - startTime).toSec();
  estimate.variance = varianceDecay(t, prevTargetVelocity * prevTargetVelocity, 0.01, 1.);
  return estimate;
}

ForwardCommandMotorState::ForwardCommandMotorState(double currTargetVelocity) : currTargetVelocity(currTargetVelocity)
{
}

MotorStatePtr ForwardCommandMotorState::targetVelocityEvent(double targetVelocity)
{
  currTargetVelocity = targetVelocity;
  return nullptr;
}

MotorStatePtr ForwardCommandMotorState::isDrivingEvent(bool isDriving)
{
  if (!isDriving)
    return nullptr;

  return std::make_unique<ForwardDrivingMotorState>(currTargetVelocity);
}

MotorStatePtr ForwardCommandMotorState::updateEvent()
{
  // In case the ESC is still in the weird state of not having
  // recognized that we released the brakes we go back to the stop state
  if (ros::Time::now() - startTime > ros::Duration{ 1. })
    return std::make_unique<StoppedMotorState>();

  return nullptr;
}

double ForwardCommandMotorState::getMotorCommand() const
{
  return config.velocityToMotorCommand(currTargetVelocity);
}

yoshicar_msgs::VelocityEstimate ForwardCommandMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  estimate.variance = 0.01;
  return estimate;
}

ReverseDrivingMotorState::ReverseDrivingMotorState(double currTargetVelocity)
  : MotorState(), currTargetVelocity(currTargetVelocity)
{
}

MotorStatePtr ReverseDrivingMotorState::targetVelocityEvent(double targetVelocity)
{
  if (targetVelocity >= 0.)
    return std::make_unique<ReverseBrakingMotorState>(currTargetVelocity);

  double currMotorCommand = config.velocityToMotorCommand(currTargetVelocity);
  double nextMotorCommand = config.velocityToMotorCommand(targetVelocity);
  if (currMotorCommand == nextMotorCommand)
    return nullptr;

  // If the new target velocity results in a new motor command, we perform
  // a self transition with the new target velocity
  return std::make_unique<ReverseDrivingMotorState>(targetVelocity);
}

MotorStatePtr ReverseDrivingMotorState::isDrivingEvent(bool isDriving)
{
  if (isDriving)
    return nullptr;

  // This may only happen if the motor fails for some reason
  return std::make_unique<StoppedMotorState>();
}

MotorStatePtr ReverseDrivingMotorState::updateEvent()
{
  return nullptr;
}

double ReverseDrivingMotorState::getMotorCommand() const
{
  return config.velocityToMotorCommand(currTargetVelocity);
}

yoshicar_msgs::VelocityEstimate ReverseDrivingMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;

  if (currTargetVelocity < config.getMinReverseVelocity())
  {
    estimate.velocity = 0.;
    estimate.variance = 0.01;
    return estimate;
  }

  estimate.velocity = currTargetVelocity;
  double t = (ros::Time::now() - startTime).toSec();
  estimate.variance = varianceDecay(t, currTargetVelocity, currTargetVelocity * 0.1, 1.);
  return estimate;
}

ReverseBrakingMotorState::ReverseBrakingMotorState(double prevTargetVelocity)
  : MotorState(), prevTargetVelocity(prevTargetVelocity)
{
  ros::NodeHandle nhPriv;
  brakeMotorCommand = nhPriv.param<double>("reverse_brake_motor_command", 0.);
}

MotorStatePtr ReverseBrakingMotorState::targetVelocityEvent(double targetVelocity)
{
  if (targetVelocity >= 0.)
    return nullptr;

  return std::make_unique<ForwardDrivingMotorState>(targetVelocity);
}

MotorStatePtr ReverseBrakingMotorState::isDrivingEvent(bool isDriving)
{
  if (isDriving)
    return nullptr;

  return std::make_unique<StoppedMotorState>();
}

MotorStatePtr ReverseBrakingMotorState::updateEvent()
{
  if (ros::Time::now() - startTime > ros::Duration{ 5. })
    return std::make_unique<StoppedMotorState>();

  return nullptr;
}

double ReverseBrakingMotorState::getMotorCommand() const
{
  return brakeMotorCommand;
}

yoshicar_msgs::VelocityEstimate ReverseBrakingMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  double t = (ros::Time::now() - startTime).toSec();
  estimate.variance = varianceDecay(t, prevTargetVelocity * prevTargetVelocity, 0.01, 1.);
  return estimate;
}

ReverseCommandMotorState::ReverseCommandMotorState(double currTargetVelocity) : currTargetVelocity(currTargetVelocity)
{
}

MotorStatePtr ReverseCommandMotorState::targetVelocityEvent(double targetVelocity)
{
  currTargetVelocity = targetVelocity;
  return nullptr;
}

MotorStatePtr ReverseCommandMotorState::isDrivingEvent(bool isDriving)
{
  if (!isDriving)
    return nullptr;

  return std::make_unique<ForwardDrivingMotorState>(currTargetVelocity);
}

MotorStatePtr ReverseCommandMotorState::updateEvent()
{
  // In case the ESC is still in the weird state of not having
  // recognized that we released the brakes we go back to the stop state
  if (ros::Time::now() - startTime > ros::Duration{ 1. })
    return std::make_unique<StoppedMotorState>();

  return nullptr;
}

double ReverseCommandMotorState::getMotorCommand() const
{
  return config.velocityToMotorCommand(currTargetVelocity);
}

yoshicar_msgs::VelocityEstimate ReverseCommandMotorState::getVelocityEstimate() const
{
  yoshicar_msgs::VelocityEstimate estimate;
  estimate.velocity = 0.;
  estimate.variance = 0.01;
  return estimate;
}

}  // namespace yoshicar