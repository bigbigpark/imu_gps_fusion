/**
 * @file fusion.hpp
 * @author SeongChang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-10 18:17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "types.hpp"
#include "tic_toc.hpp"
#include "gps_converter.hpp"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class Fusion
{
public:
  Fusion(ros::NodeHandle& nh,
         double a,
         double f,
         int utm_zone,
         bool hemi): nh_(nh), gps_converter_(a, f, utm_zone, hemi)
  {

  }

  ~Fusion()
  {

  }

  void init();
  void run();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber lio_sub_;
  ros::Subscriber leader_utm_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher utm_path_pub_;
  ros::Publisher utm_rotated_path_pub_;
  ros::Publisher lidar_pub_;
  ros::Publisher leader_utm_pub_;

  // Msg
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::PointCloud2 lidar_msg_;
  nav_msgs::Odometry lio_msg_;
  nav_msgs::Odometry pose_msg_;
  geometry_msgs::Point leader_utm_msg_;

  // Queue
  std::queue<sensor_msgs::Imu> imu_queue_;
  std::queue<sensor_msgs::NavSatFix> gps_queue_;
  std::queue<sensor_msgs::PointCloud2> lidar_queue_;
  std::queue<nav_msgs::Odometry> lio_queue_;
  std::queue<geometry_msgs::Point> leader_utm_queue_;

  // Bool
  bool is_imu_ready_;
  bool is_gps_ready_;
  bool is_gps_first_received_;
  bool is_lidar_received_;
  bool is_lio_received_;
  bool is_leader_utm_received_;

  // Callback
  void imuCallback(const sensor_msgs::Imu::Ptr& msg);
  void gpsCallback(const sensor_msgs::NavSatFix::Ptr& msg);
  void lidarCallback(const sensor_msgs::PointCloud2::Ptr& msg);
  void lioCallback(const nav_msgs::Odometry::Ptr& msg);
  void leaderUTMCallback(const geometry_msgs::Point::Ptr& msg);

  // Variable
  std::mutex buf_;
  Eigen::Vector3d utm_pose_;
  Eigen::Vector3d prev_utm_pose_;
  Eigen::Vector3d utm_rotated_pose_;
  double ori_x_, ori_y_;
  int loop_cnt_;
  std::vector<double> utm_heading_list;
  int robot_id_;

  // Publish the path
  nav_msgs::Path utm_path_;
  nav_msgs::Path utm_rotated_path_;
  
  // Func
  void updateIMU();
  void updateGPS();
  void updateLIDAR();
  void updateLIO();
  void updateLeaderUTM();
  void convertNMEA2UTM();
  void tranformUTM(double angle);
  void transformIMU(double angle);
  void transformLIO();
  void publish();
  void reset();

  // 3rd Library
  GpsConverter gps_converter_;

  // File
};