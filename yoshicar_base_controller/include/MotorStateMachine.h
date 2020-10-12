#ifndef YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H
#define YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H

#include <ros/ros.h>

namespace yoshicar
{
// This will become a message
struct VelocityEstimate
{
    double velocity;
    double variance;
};

class MotorState
{
public:
    using MotorStatePtr = std::unique_ptr<MotorState>;

    MotorState() : startTime(ros::Time::now())
    {
    }

    virtual MotorStatePtr targetVelocityEvent(double targetVelocity) = 0;
    virtual MotorStatePtr isDrivingEvent(bool isDriving) = 0;
    virtual MotorStatePtr updateEvent() = 0;

    virtual double getMotorCommand() const = 0;
    virtual VelocityEstimate getVelocityEstimate() const = 0;

protected:
    ros::Time startTime;
};

class StoppedMotorState : public MotorState 
{
public:
    StoppedMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity)
    {
        return nullptr;
    }

    MotorStatePtr isDrivingEvent(bool isDriving)
    {
        return nullptr;
    }

    MotorStatePtr updateEvent()
    {
        if (ros::Time::now() - startTime > ros::Duration{0.05})
            return std::make_unique<ReleasedBrakeMotorState>();

        return nullptr;
    }

    double getMotorCommand() const
    {
        return 0;
    }

    VelocityEstimate getVelocityEstimate() const
    {
        VelocityEstimate estimate;
        estimate.velocity = 0.;
        estimate.variance = 0.01;
        return estimate;
    }
};

class ReleasedBrakeMotorState : public MotorState 
{
public:
    ReleasedBrakeMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
};

class ForwardDrivingMotorState : public MotorState 
{
public:
    ForwardDrivingMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
};

class ForwardBrakingMotorState : public MotorState 
{
public:
    ForwardBrakingMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
};

class ForwardCommandMotorState : public MotorState 
{
public:
    ForwardCommandMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
};

class ReverseDrivingMotorState : public MotorState 
{
public:
    ReverseDrivingMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
};

class ReverseBrakingMotorState : public MotorState 
{
public:
    ReverseBrakingMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
}

class ReverseCommandMotorState : public MotorState 
{
public:
    ReverseCommandMotorState() : MotorState()
    {}

    MotorStatePtr targetVelocityEvent(double targetVelocity);
    MotorStatePtr isDrivingEvent(bool isDriving);
    MotorStatePtr updateEvent();

    double getMotorCommand() const;
    VelocityEstimate getVelocityEstimate() const;
}

class MotorStateMachine
{
public:
    MotorStateMachine();

private:
    ros::Subscriber targetVelocitySubscriber;
    ros::Subscriber isDrivingSubscriber;

    ros::Publisher estimatedVelocity;

    ros::Timer stateUpdateTimer;

    std::MotorState state;

    void targetVelocityCallback(const std_msgs::Float64::ConstPtr& msg);
    void isDrivingCallback(const std_msgs::Float64::ConstPtr& msg);
    void updateCallback();
};

}

#endif  // YOSHICAR_BASE_CONTROLLER_MOTORSTATEMACHINE_H