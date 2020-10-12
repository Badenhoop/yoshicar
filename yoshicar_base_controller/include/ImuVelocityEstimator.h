#include <ros/ros.h>

class ImuVelocityEstimator
{
public:
    ImuVelocityEstimator();

private:
    ros::Subscriber imuSubscriber;
    ros::Subscriber velocityResetSubscriber;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void resetCallback(const std_msgs::Float64::ConstPtr& msg);
}