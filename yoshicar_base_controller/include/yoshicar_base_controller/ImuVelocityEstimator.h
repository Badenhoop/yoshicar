#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

namespace yoshicar
{

class ImuVelocityEstimator
{
public:
  ImuVelocityEstimator();

private:
  class MovingAverage
  {
  public:
    explicit MovingAverage(std::size_t size);

    void update(double measurement);

    double get();

  private:
    std::vector<double> measurements;
    std::size_t end{0};
  };

  ros::Subscriber imuSubscriber;

  ros::Publisher isDrivingPublisher;
  ros::Publisher velocityEstimatePublisher;

  MovingAverage acceleration;
  double velocity;
  double variance;
  ros::Time lastStopTime;
  double noiseThreshold;
  std::size_t stopCounter;
  std::size_t stopWindowSize;
  double prevAcceleration;
  ros::Time prevMeasurementTime;
  double varianceGrowthFactor;

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
};

}  // namespace yoshicar