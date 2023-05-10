/**
 * @file main.cpp
 * @author SeongChang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-10 18:18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pose_fusion.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion_node");
  ros::NodeHandle nh;

  Fusion fusion(nh, 6378137, 1/298.257223563, 52, true);
  fusion.init();
  fusion.run();

  return 0;
}
