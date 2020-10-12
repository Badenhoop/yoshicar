//
// Created by philipp on 13.08.19.
//

#include "../include/yoshicar_base_controller/YoshicarHardwareInterface.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hardware_interface_node");

  double updateRateHz;
  ros::param::get("~update_rate", updateRateHz);
  ros::Rate rate{ updateRateHz };
  ros::Duration period{ 1.0 / updateRateHz };

  ros::NodeHandle nh;
  yoshicar::HardwareInterface hardwareInterface;
  controller_manager::ControllerManager cm(&hardwareInterface, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    hardwareInterface.read();
    cm.update(ros::Time::now(), period);
    hardwareInterface.write();
    rate.sleep();
  }

  spinner.stop();
  return 0;
}