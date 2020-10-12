#include <std_msgs/Bool.h>
#include <yoshicar_msgs/VelocityEstimate.h>
#include "yoshicar_base_controller/ImuVelocityEstimator.h"

namespace yoshicar
{
ImuVelocityEstimator::ImuVelocityEstimator()
  : acceleration(0)
  , velocity(0.)
  , variance(0.01)
  , lastStopTime(ros::Time::now())
  , stopCounter(0.)
  , prevAcceleration(0.)
  , prevMeasurementTime(ros::Time::now())
{
  ros::NodeHandle nh;
  ros::NodeHandle nhPriv{ "~" };

  imuSubscriber = nh.subscribe<sensor_msgs::Imu>("imu/data", 1, &ImuVelocityEstimator::imuCallback, this);
  isDrivingPublisher = nh.advertise<std_msgs::Bool>("is_driving", 1);
  velocityEstimatePublisher = nh.advertise<yoshicar_msgs::VelocityEstimate>("imu_velocity_estimate", 1);

  auto averageWindowSize = nhPriv.param<int>("acceleration_average_window_size", 4);
  acceleration = MovingAverage(averageWindowSize);

  noiseThreshold = nhPriv.param<double>("acceleration_noise_threshold", 0.01);
  stopWindowSize = nhPriv.param<int>("stop_window_size", 5);
  varianceGrowthFactor = nhPriv.param<double>("variance_growth_factor", 1.2);
}

void ImuVelocityEstimator::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  acceleration.update(msg->linear_acceleration.y);
  double currAcceleration = acceleration.get();
  auto currMeasurementTime = msg->header.stamp;
  double deltaTime = (currMeasurementTime - prevMeasurementTime).toSec();
  // Trapezoidal integration
  velocity += deltaTime * (currAcceleration + prevAcceleration) / 2.;

  double timeSinceLastStop = (currMeasurementTime - lastStopTime).toSec();
  variance = timeSinceLastStop * varianceGrowthFactor;

  bool isDriving = true;
  if (currAcceleration < noiseThreshold)
  {
    stopCounter++;
    if (stopCounter >= stopWindowSize)
    {
      velocity = 0.;
      variance = 0.01;
      isDriving = false;
      lastStopTime = currMeasurementTime;
    }
  }
  else
  {
    stopCounter = 0;
  }

  std_msgs::Bool isDrivingMsg;
  isDrivingMsg.data = isDriving;
  isDrivingPublisher.publish(isDriving);

  yoshicar_msgs::VelocityEstimate velocityEstimate;
  velocityEstimate.velocity = velocity;
  velocityEstimate.variance = variance;
  velocityEstimatePublisher.publish(velocityEstimate);

  prevAcceleration = currAcceleration;
  prevMeasurementTime = currMeasurementTime;
}

ImuVelocityEstimator::MovingAverage::MovingAverage(std::size_t size)
{
  measurements.resize(size, 0.);
}

void ImuVelocityEstimator::MovingAverage::update(double measurement)
{
  measurements[end] = measurement;
  end = (end + 1) % measurements.size();
}

double ImuVelocityEstimator::MovingAverage::get()
{
  double average = 0.;
  for (auto measurement : measurements)
    average += measurement;
  return average / measurements.size();
}

}  // namespace yoshicar