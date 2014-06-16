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
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

class NavSatTfPub {
  public:
    NavSatTfPub();

  private:
    void compassCallback(const std_msgs::Float32::ConstPtr & msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr & msg);
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr & msg);
    void publish(const std_msgs::Header & header);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber fix_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber compass_sub_;

    ros::Publisher odom_pub_;

    geodesy::UTMPoint initial_point_;
    bool initial_point_valid_;
    bool relative_;

    std::string frame_id_;
    bool override_frame_id_;

    std::string child_frame_id_;
    bool override_child_frame_id_;

    geometry_msgs::Quaternion heading_;
    double heading_cov_[9];

    std::string gps_frame_id_;
    geodesy::UTMPoint current_point_;
    double gps_cov_[9];

    bool fix_hdop_;
};

NavSatTfPub::NavSatTfPub() 
  : pnh_("~"),
    initial_point_valid_(false)
{
  heading_.w = 1.0;

  fix_sub_ = nh_.subscribe("fix", 10, &NavSatTfPub::fixCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  pnh_.param<bool>("relative", relative_, false);
  pnh_.param<bool>("fix_hdop", fix_hdop_, false);

  override_frame_id_ = pnh_.getParam("frame_id", frame_id_);
  pnh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

  bool have_compass = false;
  std::string compass_topic;
  if( pnh_.getParam("compass_topic", compass_topic) ) {
    ROS_INFO("Subscribing to %s for heading data", compass_topic.c_str());
    compass_sub_ = nh_.subscribe(compass_topic, 10,
        &NavSatTfPub::compassCallback, this);

    double compass_var;
    pnh_.param<double>("compass_variance", compass_var, 0.1);
    for( int i=0; i<8; i++ ) {
      heading_cov_[i] = 0.0;
    }
    heading_cov_[9] = compass_var;
    have_compass = true;
  }

  bool have_imu = false;
  std::string imu_topic;
  if( pnh_.getParam("imu_topic", imu_topic) ) {
    ROS_ASSERT(!have_compass);
    ROS_INFO("Subscribing to %s for imu orientation data", imu_topic.c_str());
    imu_sub_ = nh_.subscribe(imu_topic, 10,
        &NavSatTfPub::imuCallback, this);
    have_imu = true;
  }

  if( !have_compass && !have_imu ) {
    ROS_WARN("No compass or IMU topic provided for orientation. "
        "Orientation will be set to 0. "
        "This probably will not work well with sensor fusion.");
  }
}

void NavSatTfPub::compassCallback(const std_msgs::Float32::ConstPtr & msg) {
  heading_ = tf::createQuaternionMsgFromYaw(msg->data);
  std_msgs::Header fake_header;
  fake_header.stamp = ros::Time::now();
  fake_header.frame_id = frame_id_;
  publish(fake_header);
}

void NavSatTfPub::imuCallback(const sensor_msgs::Imu::ConstPtr & msg) {
  heading_ = msg->orientation;
  for( int i=0; i<9; i++ ) {
    heading_cov_[i] = msg->orientation_covariance[i];
  }
  std_msgs::Header fake_header;
  fake_header.stamp = ros::Time::now();
  fake_header.frame_id = frame_id_;
  publish(fake_header);
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
      // this is always bad data when I see it, so just ignore it
      return;
    }
  }
  current_point_ = utm_point;

  if( fix_hdop_ ) {
    for( int i=0; i<3; i++ ) {
      double v = msg->position_covariance[i*4];
      v = sqrt(v);
      const double A = 3.0;
      const double B = 4.0;
      v = A * v;
      v = sqrt( v*v + B*B );
      gps_cov_[i*4] = v;
    }
    for( int i=0; i<9; i++ ) {
      if( i%4 != 0 ) {
        gps_cov_[i] = 0.0;
      }
    }
  } else {
    for( int i=0; i<9; i++ ) {
      gps_cov_[i] = msg->position_covariance[i];
    }
  }

  gps_frame_id_ = msg->header.frame_id;
  publish(msg->header);
}

void NavSatTfPub::publish(const std_msgs::Header & header) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = header.frame_id;
  odom.header.stamp    = header.stamp;
  if(override_frame_id_) {
    odom.header.frame_id = frame_id_;
  }
  odom.child_frame_id = child_frame_id_;
  odom.pose.pose.position.x = current_point_.easting;
  odom.pose.pose.position.y = current_point_.northing;
  odom.pose.pose.position.z = current_point_.altitude;

  odom.pose.pose.orientation = heading_;
  for( int i=0; i<3; i++ ) {
    for(int j=0; j<3; j++ ) {
      // copy in gps covariance
      odom.pose.covariance[i*6+j] = gps_cov_[i*3+j];

      // copy in heading covariance
      odom.pose.covariance[(i+3)*6 + (j+3)] = heading_cov_[i*3+j];
    }
  }
  odom_pub_.publish(odom);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "utm_tf_pub");
  NavSatTfPub pub;
  ros::spin();
}
