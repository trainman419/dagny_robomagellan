/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2014 Austin Hendrix
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// notes: we should publish a utm local frame that is within 1km of our
// current position in an attempt to make rviz happy.
// See: https://github.com/ros-visualization/rviz/issues/502

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

class NavSatTfPub {
  public:
    NavSatTfPub();

  private:
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr & msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber fix_sub_;
    ros::Publisher odom_pub_;

    geodesy::UTMPoint initial_point_;
    bool initial_point_valid_;
    bool relative_;

    std::string frame_id_;
    bool override_frame_id_;

    std::string child_frame_id_;
    bool override_child_frame_id_;
};

NavSatTfPub::NavSatTfPub() : pnh_("~"), initial_point_valid_(false) {
  fix_sub_ = nh_.subscribe("fix", 10, &NavSatTfPub::fixCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  pnh_.param<bool>("relative", relative_, false);

  override_frame_id_ = pnh_.getParam("frame_id", frame_id_);
  pnh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");
}

void NavSatTfPub::fixCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
  geographic_msgs::GeoPoint geo_point = geodesy::toMsg(*msg);
  geodesy::UTMPoint utm_point(geo_point);

  if(!initial_point_valid_) {
    initial_point_ = utm_point;
    initial_point_valid_ = true;
  }

  if(relative_) {
    if(utm_point.zone == initial_point_.zone
        && utm_point.band == initial_point_.band) {
      utm_point.altitude -= initial_point_.altitude;
      utm_point.northing -= initial_point_.northing;
      utm_point.easting  -= initial_point_.easting;
    } else {
      ROS_ERROR("Initial UTM zone and current zone are not the same!");
      // TODO: do something more clever here...
    }
  }

  nav_msgs::Odometry odom;
  odom.header = msg->header;
  if(override_frame_id_) {
    odom.header.frame_id = frame_id_;
  }
  odom.child_frame_id = child_frame_id_;
  odom.pose.pose.position.x = utm_point.easting;
  odom.pose.pose.position.y = utm_point.northing;
  odom.pose.pose.position.z = utm_point.altitude;

  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  for( int i=0; i<3; i++ ) {
    for(int j=0; j<3; j++ ) {
      odom.pose.covariance[i*6+j] = msg->position_covariance[i*3+j];
    }
  }
  odom_pub_.publish(odom);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "utm_tf_pub");
  NavSatTfPub pub;
  ros::spin();
}
