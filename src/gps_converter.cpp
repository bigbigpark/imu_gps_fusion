
#include "gps_converter.hpp"

using namespace GeographicLib;

GpsConverter::GpsConverter(double a,    
               double f,    
               int    zone,    
               bool   northp)  
    : tm_(a, f, Constants::UTM_k0())
    , lon0_(6 * zone - 183)
    , falseeasting_(5e5)
    , falsenorthing_(northp ? 0 : 100e5)
{
  if (!(zone >= 1 && zone <= 60))
    throw GeographicErr("input utm_zone is not in [1,60]");
}

GpsConverter::~GpsConverter()
{

}

// Init
void GpsConverter::init(ros::NodeHandle& nh)
{
  isFirstReceived = false;
}

// WGS84 to UTM
void GpsConverter::forward(double lat, double lon, double& x, double& y)
{
  // lon0_은 경도의 중심
  tm_.Forward(lon0_, lat, lon, x, y);
  x += falseeasting_;
  y += falsenorthing_;
}
 // UTM to WGS84
void GpsConverter::reverse(double x, double y, double& lat, double& lon)
{
  x -= falseeasting_;
  y -= falsenorthing_;
  tm_.Reverse(lon0_, x, y, lat, lon);
}