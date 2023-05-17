
#include "pose_fusion.hpp"

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
    angle -= 2.*M_PI;

  while(angle < -M_PI)
    angle += 2.*M_PI;

  return angle;
}

// ------------------------------------------------------------------

void Fusion::lioCallback(const nav_msgs::Odometry::Ptr& msg)
{
  buf_.lock();
  lio_queue_.push(*msg);
  buf_.unlock();
}

void Fusion::lidarCallback(const sensor_msgs::PointCloud2::Ptr& msg)
{
  buf_.lock();
  lidar_queue_.push(*msg);
  buf_.unlock();
}

void Fusion::imuCallback(const sensor_msgs::Imu::Ptr& msg)
{
  buf_.lock();
  imu_queue_.push(*msg);
  buf_.unlock();
}

void Fusion::gpsCallback(const sensor_msgs::NavSatFix::Ptr& msg)
{
  buf_.lock();
  gps_queue_.push(*msg);
  buf_.unlock();
}

void Fusion::leaderUTMCallback(const geometry_msgs::Point::Ptr& msg)
{
  buf_.lock();
  leader_utm_queue_.push(*msg);
  buf_.unlock();
}

// ------------------------------------------------------------------

void Fusion::reset()
{
  is_gps_ready_ = false;
  is_imu_ready_ = false;
  is_lidar_received_ = false;
  is_lio_received_ = false;
}

void Fusion::publish()
{
  if (is_gps_ready_ && is_lio_received_ && is_lidar_received_ && is_leader_utm_received_)
  {
    // Publish utm path
    utm_path_.header.frame_id = "map";
    utm_path_.header.stamp = ros::Time::now();

    auto pose = geometry_msgs::PoseStamped();
    pose.pose.position.x = utm_pose_(0,0);
    pose.pose.position.y = utm_pose_(1,0);
    pose.pose.orientation.x = imu_msg_.orientation.x;
    pose.pose.orientation.y = imu_msg_.orientation.y;
    pose.pose.orientation.z = imu_msg_.orientation.z;
    pose.pose.orientation.w = imu_msg_.orientation.w;
    utm_path_.poses.push_back(pose);
    utm_path_pub_.publish(utm_path_);

    // Publish utm rotated path
    utm_rotated_path_.header.frame_id = "map";
    utm_rotated_path_.header.stamp = ros::Time::now();

    auto pose2 = geometry_msgs::PoseStamped();
    pose2.pose.position.x = utm_rotated_pose_(0,0);
    pose2.pose.position.y = utm_rotated_pose_(1,0);
    pose2.pose.orientation.x = lio_msg_.pose.pose.orientation.x;
    pose2.pose.orientation.y = lio_msg_.pose.pose.orientation.y;
    pose2.pose.orientation.z = lio_msg_.pose.pose.orientation.z;
    pose2.pose.orientation.w = lio_msg_.pose.pose.orientation.w;
    utm_rotated_path_.poses.push_back(pose2);
    utm_rotated_path_pub_.publish(utm_rotated_path_);

    // Publish raw lidar data
    lidar_msg_.header.frame_id = "velodyne";
    lidar_msg_.header.stamp = ros::Time::now();
    lidar_pub_.publish(lidar_msg_);

    // Publish UGV's pose
    pose_msg_.header.frame_id = "map";
    pose_msg_.header.stamp = ros::Time::now();
    pose_msg_.pose.pose.position.x = utm_rotated_pose_(0,0);
    pose_msg_.pose.pose.position.y = utm_rotated_pose_(1,0);
    pose_msg_.pose.pose.orientation.x = lio_msg_.pose.pose.orientation.x;
    pose_msg_.pose.pose.orientation.y = lio_msg_.pose.pose.orientation.y;
    pose_msg_.pose.pose.orientation.z = lio_msg_.pose.pose.orientation.z;
    pose_msg_.pose.pose.orientation.w = lio_msg_.pose.pose.orientation.w;
    pose_pub_.publish(pose_msg_);

    // Publish Leader UTM
    if (robot_id_ == 1) leader_utm_pub_.publish(leader_utm_msg_);

    // TF broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "velodyne";
    transformStamped.transform.translation.x = utm_rotated_pose_(0,0);
    transformStamped.transform.translation.y = utm_rotated_pose_(1,0);
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = lio_msg_.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = lio_msg_.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = lio_msg_.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = lio_msg_.pose.pose.orientation.w;
  
    br.sendTransform(transformStamped);
  }
}

void Fusion::transformLIO()
{
  if (is_lio_received_)
  {
    tf2::Quaternion q(lio_msg_.pose.pose.orientation.x,
                      lio_msg_.pose.pose.orientation.y,
                      lio_msg_.pose.pose.orientation.z,
                      lio_msg_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw *= -1;
    yaw = pi_to_pi(yaw);

    std::cout << "yaw [rad]: " << yaw << std::endl;
    std::cout << "yaw [deg]: " << yaw*180.0/M_PI << std::endl;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    quat.normalize();

    lio_msg_.pose.pose.orientation.w = quat.getW();
    lio_msg_.pose.pose.orientation.x = quat.getX();
    lio_msg_.pose.pose.orientation.y = quat.getY();
    lio_msg_.pose.pose.orientation.z = quat.getZ();
  }
}

void Fusion::transformIMU(double angle)
{
  if (is_imu_ready_)
  {
    angle *= M_PI/180.0;

    tf2::Quaternion quat(imu_msg_.orientation.x,
                         imu_msg_.orientation.y,
                         imu_msg_.orientation.z,
                         imu_msg_.orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaw -= angle;
    yaw *= -1;

    std::cout << "yaw [rad]: " << yaw << std::endl;
    std::cout << "yaw [deg]: " << yaw*180.0/M_PI << std::endl;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();

    imu_msg_.orientation.w = q.getW();
    imu_msg_.orientation.x = q.getX();
    imu_msg_.orientation.y = q.getY();
    imu_msg_.orientation.z = q.getZ();
  }
}

void Fusion::tranformUTM(double angle)
{
  if (is_gps_ready_)
  {
    angle *= M_PI/180.0;

    Eigen::Matrix3d T;
    T << cos(angle), -sin(angle), 0,
         sin(angle),  cos(angle), 0,
                  0,           0, 1;

    utm_rotated_pose_ = T*utm_pose_;

    // Calculate utm heading angle
    double heading = std::atan2(utm_pose_(1,0)-prev_utm_pose_(1,0), utm_pose_(0,0)-prev_utm_pose_(0,0));
    heading = pi_to_pi(heading);
    std::cout << "utm heading [rad]: " << heading << std::endl;
    std::cout << "utm heading [deg]: " << heading*180.0/M_PI << std::endl;

    if (loop_cnt_ < 70)
    {
      loop_cnt_++;

      Eigen::Vector2d p1 (utm_pose_(0,0), utm_pose_(1,0));
      Eigen::Vector2d p2 (prev_utm_pose_(0,0), prev_utm_pose_(1,0));

      auto norm = (p1-p2).norm();
      std::cout << "norm: " << norm << std::endl;

      if (norm > 0.15)  utm_heading_list.push_back(heading);
    }
    else
    {
      double avg = std::accumulate(utm_heading_list.begin(), utm_heading_list.end(), 0.0) / utm_heading_list.size();
      std::cout << "avg utm heading [rad]: "<< GREEN << avg << END << std::endl;
      std::cout << "avg utm heading [deg]: "<< GREEN << avg*180.0/M_PI << END << std::endl;
    }

    prev_utm_pose_ = utm_pose_;
  }
}

void Fusion::convertNMEA2UTM()
{
  if (is_gps_ready_ && is_leader_utm_received_)
  {
    double e, n;
    gps_converter_.forward(gps_msg_.latitude, gps_msg_.longitude, e, n);

    if (is_gps_first_received_)
    {
      is_gps_first_received_ = false;
      ori_x_ = e;
      ori_y_ = n;

      if (robot_id_ == 1)
      {
        leader_utm_msg_.x = ori_x_;
        leader_utm_msg_.y = ori_y_;
      }
      else
      {
        ori_x_ = leader_utm_msg_.x;
        ori_y_ = leader_utm_msg_.y;
      }
    }
    utm_pose_ << e - ori_x_, n - ori_y_, 1;

    std::cout << std::setprecision(3) << "utm_pose: " << utm_pose_(0,0) << ", " << utm_pose_(1,0) << std::endl;
  } 
}

void Fusion::updateLeaderUTM()
{
  if (!leader_utm_queue_.empty() && robot_id_ != 1)
  {
    buf_.lock();
    leader_utm_msg_ = leader_utm_queue_.front();
    leader_utm_queue_.pop();
    is_leader_utm_received_ = true;
    buf_.unlock();
  }
  if (robot_id_ == 1) is_leader_utm_received_ = true;
}

void Fusion::updateLIO()
{
  if (!lio_queue_.empty())
  {
    buf_.lock();
    lio_msg_ = lio_queue_.front();
    lio_queue_.pop();
    is_lio_received_ = true;
    buf_.unlock();
  }
}

void Fusion::updateLIDAR()
{
  if (!lidar_queue_.empty())
  {
    buf_.lock();
    lidar_msg_ = lidar_queue_.front();
    lidar_queue_.pop();
    is_lidar_received_ = true;
    buf_.unlock();
  }
}

void Fusion::updateGPS()
{
  if (!gps_queue_.empty())
  {
    buf_.lock();
    gps_msg_ = gps_queue_.front();
    gps_queue_.pop();
    is_gps_ready_ = true;
    buf_.unlock();
  }
}

void Fusion::updateIMU()
{
  if (!imu_queue_.empty())
  {
    buf_.lock();
    imu_msg_ = imu_queue_.front();
    imu_queue_.pop();
    is_imu_ready_ = true;
    buf_.unlock();
  }
}

void Fusion::run()
{
  ros::Rate r(5); // dt: 0.2

  while(ros::ok())
  {
    ros::spinOnce();
    std::cout << "\n\n ----- \n";

    updateIMU();
    updateGPS();
    updateLIDAR();
    updateLIO();
    updateLeaderUTM();
    convertNMEA2UTM();
    tranformUTM(126); // input: angle [deg]
    // transformIMU(85); // input: angle [deg]
    transformLIO();
    publish();
    reset();

    r.sleep();
  }
}

void Fusion::init()
{
  // Robot ID
  robot_id_ = 1;
  std::string ns = "/rbt" + std::to_string(robot_id_);

  // ROS
  imu_sub_  = nh_.subscribe(ns + "/gx5/imu/data", 1, &Fusion::imuCallback, this);
  gps_sub_  = nh_.subscribe(ns + "/ublox_gps/fix", 1, &Fusion::gpsCallback, this);
  lidar_sub_  = nh_.subscribe(ns + "/velodyne_points", 1, &Fusion::lidarCallback, this);
  lio_sub_ = nh_.subscribe(ns + "/Odometry", 1, &Fusion::lioCallback, this);
  
  // Leader UTM origin pub
  if (robot_id_ == 1)
  {
    leader_utm_pub_ = nh_.advertise<geometry_msgs::Point>("/rbt1/utm_origin", 1);
  }
  else
  {
    leader_utm_sub_ = nh_.subscribe("/rbt1/utm_origin", 1, &Fusion::leaderUTMCallback, this);
  }

  pose_pub_ = nh_.advertise<nav_msgs::Odometry>(ns + "/pose", 1);
  utm_path_pub_ = nh_.advertise<nav_msgs::Path>(ns + "/utm_path", 1);
  utm_rotated_path_pub_ = nh_.advertise<nav_msgs::Path>(ns + "/utm_rotated_path", 1);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ns + "/lidar", 1);

  // Variable
  ori_x_ = 0.0;
  ori_y_ = 0.0;

  is_imu_ready_ = false;
  is_gps_ready_ = false;
  is_gps_first_received_ = true;
  is_lidar_received_ = false;
  is_lio_received_ = false;

  utm_pose_ << 0.0, 0.0, 1.0;
  prev_utm_pose_ << 0.0, 0.0, 1.0;

  loop_cnt_ = 0;
}