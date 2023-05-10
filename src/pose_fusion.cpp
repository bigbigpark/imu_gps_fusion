
#include "pose_fusion.hpp"

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

// ------------------------------------------------------------------

void Fusion::reset()
{
  is_gps_ready_ = false;
  is_imu_ready_ = false;
  is_lidar_received_ = false;
}

void Fusion::publish()
{
  if (is_gps_ready_ && is_imu_ready_ && is_lidar_received_)
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
    pose2.pose.orientation.x = imu_msg_.orientation.x;
    pose2.pose.orientation.y = imu_msg_.orientation.y;
    pose2.pose.orientation.z = imu_msg_.orientation.z;
    pose2.pose.orientation.w = imu_msg_.orientation.w;
    utm_rotated_path_.poses.push_back(pose2);
    utm_rotated_path_pub_.publish(utm_rotated_path_);

    // Publish raw lidar data
    lidar_msg_.header.frame_id = "velodyne";
    lidar_msg_.header.stamp = ros::Time::now();
    lidar_pub_.publish(lidar_msg_);

    // TF broadcaster
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "velodyne";
    transformStamped.transform.translation.x = utm_rotated_pose_(0,0);
    transformStamped.transform.translation.y = utm_rotated_pose_(1,0);
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = imu_msg_.orientation.x;
    transformStamped.transform.rotation.y = imu_msg_.orientation.y;
    transformStamped.transform.rotation.z = imu_msg_.orientation.z;
    transformStamped.transform.rotation.w = imu_msg_.orientation.w;
  
    br.sendTransform(transformStamped);
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
  }
}

void Fusion::convertNMEA2UTM()
{
  if (is_gps_ready_)
  {
    double e, n;
    gps_converter_.forward(gps_msg_.latitude, gps_msg_.longitude, e, n);

    if (is_gps_first_received_)
    {
      is_gps_first_received_ = false;
      ori_x_ = e;
      ori_y_ = n;
    }

    utm_pose_ << e - ori_x_, n - ori_y_, 1;
    std::cout << std::setprecision(3) << "utm_pose: " << utm_pose_(0,0) << ", " << utm_pose_(1,0) << std::endl;
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
    convertNMEA2UTM();
    tranformUTM(125); // input: angle [deg]
    transformIMU(85); // input: angle [deg]
    publish();
    reset();

    r.sleep();
  }

}

void Fusion::init()
{
  // ROS
  imu_sub_  = nh_.subscribe("/rbt1/gx5/imu/data", 1, &Fusion::imuCallback, this);
  gps_sub_  = nh_.subscribe("/rbt1/ublox_gps/fix", 1, &Fusion::gpsCallback, this);
  lidar_sub_  = nh_.subscribe("/rbt1/velodyne_points", 1, &Fusion::lidarCallback, this);

  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("/rbt1/pose", 1);
  utm_path_pub_ = nh_.advertise<nav_msgs::Path>("/rbt1/utm_path", 1);
  utm_rotated_path_pub_ = nh_.advertise<nav_msgs::Path>("/rbt1/utm_rotated_path", 1);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/rbt1/lidar", 1);

  // Variable
  ori_x_ = 0.0;
  ori_y_ = 0.0;

  is_imu_ready_ = false;
  is_gps_ready_ = false;
  is_gps_first_received_ = true;
  is_lidar_received_ = false;
}