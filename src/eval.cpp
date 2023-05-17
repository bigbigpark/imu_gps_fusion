/**
 * @file eval.cpp
 * @author SeongChang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-17 14:32
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <vector>
#include <fstream>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Subscriber xpose_sub;
ros::Subscriber ref_path_sub;
std::mutex buf;

std::queue<nav_msgs::Path> ref_path_queue;
std::queue<nav_msgs::Odometry> xpose_queue;

Eigen::Vector3d xpose;
Eigen::Vector3d ref_path;

std::fstream file;

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
    angle -= 2.*M_PI;

  while(angle < -M_PI)
    angle += 2.*M_PI;

  return angle;
}

void xposeCallback(const nav_msgs::Odometry::Ptr& msg)
{
  buf.lock();
  xpose_queue.push(*msg);
  buf.unlock();
}

void refPathCallback(const nav_msgs::Path::Ptr& msg)
{
  buf.lock();
  ref_path_queue.push(*msg);
  buf.unlock();
}

void calculateFormationError()
{
  if (!xpose_queue.empty() && !ref_path_queue.empty())
  {
    buf.lock();
    auto xpose_msg = xpose_queue.front();
    xpose_queue.pop();

    auto ref_path_msg = ref_path_queue.front();
    ref_path_queue.pop();
    buf.unlock();

    // xpose
    tf2::Quaternion quat(xpose_msg.pose.pose.orientation.x,
                         xpose_msg.pose.pose.orientation.y,
                         xpose_msg.pose.pose.orientation.z,
                         xpose_msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw = pi_to_pi(yaw);
    xpose << xpose_msg.pose.pose.position.x, xpose_msg.pose.pose.position.y, yaw;

    // ref_path
    ref_path << ref_path_msg.poses[0].pose.position.x, ref_path_msg.poses[0].pose.position.y, 0.0;

    // --------------------------------
    std::cout << "xpose: " << xpose << std::endl;
    std::cout << "ref_path: " << ref_path << std::endl;
    auto norm = (xpose - ref_path).norm();

    Eigen::Vector2d p1 (xpose(0,0), xpose(1,0));
    Eigen::Vector2d p2 (ref_path(0,0), ref_path(1,0));
    std::cout << "norm: " << norm << std::endl;

    // --------------------------------

    file << std::to_string( ros::Time::now().toSec() ) << "," 
         << xpose(0,0) << "," << xpose(1,0) << "," << xpose(2,0) << ","
         << ref_path(0,0) << "," << ref_path(1,0) << "," << ref_path(2,0) << "," << (p1-p2).norm() << "\n";
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eval_node");
  ros::NodeHandle nh;

  xpose_sub     = nh.subscribe("/rbt1/gps_local_pose", 1, xposeCallback);
  ref_path_sub  = nh.subscribe("/rbt1/ref_path", 1, refPathCallback);
  file.open("/home/scpark/data/2023-05-17/form_err.txt", std::ios::out);

  ros::Rate r(10);
  while(ros::ok())
  {
    ros::spinOnce();

    calculateFormationError();

    r.sleep();
  }

  file.close();

  return 0;
}