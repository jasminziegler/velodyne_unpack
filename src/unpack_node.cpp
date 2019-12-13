/*
 * unpack_node.cpp
 *
 *  Created on: Dec 9, 2019
 *      Author: jasmin
 */

#include "ros/ros.h"
#include "VelodyneRectifier.h"


boost::shared_ptr<fd_scanner::VelodyneRectifier> velodyne_rectifier;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_unpack_node");
  ros::NodeHandle private_nh("~");
  double loop_rate;
  private_nh.param("loop_rate", loop_rate, 30.0);
  //ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1/loop_rate), loopCallback);
  velodyne_rectifier.reset(new fd_scanner::VelodyneRectifier());
  velodyne_rectifier->init();
  ros::spin();
  velodyne_rectifier.reset();
  return 0;
}
