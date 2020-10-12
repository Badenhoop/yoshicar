#include "yoshicar_base_controller/FusionVelocityEstimator.h"

namespace yoshicar
{
FusionVelocityEstimator::FusionVelocityEstimator() : velocity(0), variance(0.01)
{
  ros::NodeHandle nh;
  msmEstimateSubscriber = nh.subscribe<yoshicar_msgs::VelocityEstimate>(
      "msm_velocity_estimate", 1, &FusionVelocityEstimator::msmEstimateCallback, this);
  imuEstimateSubscriber = nh.subscribe<yoshicar_msgs::VelocityEstimate>(
      "imu_velocity_estimate", 1, &FusionVelocityEstimator::imuEstimateCallback, this);
  fusionEstimatePublisher = nh.advertise<yoshicar_msgs::VelocityEstimate>("fusion_velocity_estimate", 1);
}

void FusionVelocityEstimator::msmEstimateCallback(const yoshicar_msgs::VelocityEstimateConstPtr& velocityEstimate)
{
}

void FusionVelocityEstimator::imuEstimateCallback(const yoshicar_msgs::VelocityEstimateConstPtr& velocityEstimate)
{
}

}  // namespace yoshicar