#pragma once

#include "types.hpp"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <GeographicLib/TransverseMercator.hpp>

using namespace std;
using namespace GeographicLib;

// Define a UTM projection for an arbitrary ellipsoid
  /**
   * @param a       equatorial radius
   * @param f       flattening
   * @param zone    the UTM zone (52N <- Korea)
   * @param northp  hemisphere (북반구: N, 남반구: S)
   */
class GpsConverter
{
public:
  GpsConverter(double a, double f, int zone, bool northp);
  ~GpsConverter();

  // Init
  void init(ros::NodeHandle& nh);

  // WGS84 to UTM
  void forward(double lat, double lon, double& x, double& y);

  // UTM to WGS84
  void reverse(double x, double y, double& lat, double& lon);

private:
  bool isFirstReceived;

  TransverseMercator tm_;       // The projection
  double lon0_;                 // Central longitude
  double falseeasting_, falsenorthing_;

  double a_;
  double f_;
  int zone_;
  bool northp_;
};