
#ifndef DAGNY_GPS_H
#define DAGNY_GPS_H

#include <sensor_msgs/NavSatFix.h>

struct segment {
  double distance;
  double heading;
};

segment gpsDist(const sensor_msgs::NavSatFix &start,
                const sensor_msgs::NavSatFix &end);

#endif
