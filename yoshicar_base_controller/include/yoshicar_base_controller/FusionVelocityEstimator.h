#include <ros/ros.h>
#include <yoshicar_msgs/VelocityEstimate.h>

namespace yoshicar
{
class FusionVelocityEstimator
{
public:
  FusionVelocityEstimator();

private:
  ros::Subscriber msmEstimateSubscriber;
  ros::Subscriber imuEstimateSubscriber;

  ros::Publisher fusionEstimatePublisher;

  void msmEstimateCallback(const yoshicar_msgs::VelocityEstimateConstPtr& velocityEstimate);
  void imuEstimateCallback(const yoshicar_msgs::VelocityEstimateConstPtr& velocityEstimate);

  double velocity;
  double variance;
};

}