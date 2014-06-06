#include <goal_list/gps.h>
// Radius of earth in m
#define R 6371000

segment gpsDist(const sensor_msgs::NavSatFix &start,
                const sensor_msgs::NavSatFix &end) {
  segment result;

  double start_lat, start_lon, end_lat, end_lon;
  start_lat = start.latitude / 180.0 * M_PI;
  start_lon = start.longitude / 180.0 * M_PI;
  end_lat = end.latitude / 180.0 * M_PI;
  end_lon = end.longitude / 180.0 * M_PI;

  // Haversine formula (http://www.movable-type.co.uk/scripts/latlong.html)
  double delta_lat = end_lat - start_lat;
  double delta_lon = end_lon - start_lon;
  double a = sin(delta_lat/2.0)*sin(delta_lat/2.0) + 
    cos(start_lat)*cos(end_lat)*sin(delta_lon/2.0)*sin(delta_lon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = R * c; // distance to goal
  result.distance = d;

  // Bearing formula (from above)
  //  expressed as radians east of North
  double theta = atan2( sin(delta_lon)*cos(end_lat),
      cos(start_lat)*sin(end_lat) - 
      sin(start_lat)*cos(end_lat)*cos(delta_lon));
  result.heading = theta;

  return result;
}
