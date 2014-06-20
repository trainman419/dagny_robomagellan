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
//
// Implementation:
//  publish a utm -> local_utm frame that captures the initial position
//  publish a local_utm -> gps frame that captures the difference between the
//  initial position and the current position
//
// TODO list:
//  - now that I'm publishing the utm_local frame, consider removing the
//    relative_ flag and parameter
//  - consider removing all of the compass code. it can be handled by a separate
//    tf publisher or by the Kalman filter
//  - consider adding a flag to disable publishing the gps frame. when using a
//    Kalman filter, the robot's position will be estimated by the filter

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

class NavSatTfPub {
  public:
    NavSatTfPub();

  private:
    void compassCallback(const std_msgs::Float32::ConstPtr & msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr & msg);
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr & msg);
    void publish(const ros::Time & header);

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

    bool heading_valid_;
    geometry_msgs::Quaternion heading_;
    double heading_cov_[9];

    bool gps_valid_;
    std::string gps_frame_id_;
    geodesy::UTMPoint current_point_;
    double gps_cov_[9];

    bool fix_hdop_;

    // tf2 publishers
    tf2_ros::StaticTransformBroadcaster utm_local_tf;
    tf2_ros::TransformBroadcaster gps_tf;
};

NavSatTfPub::NavSatTfPub() 
  : pnh_("~"),
    initial_point_valid_(false),
    heading_valid_(false),
    gps_valid_(false)
{
  heading_.w = 1.0;
  current_point_.easting = 0.0;
  current_point_.northing = 0.0;
  current_point_.altitude = 0.0;

  fix_sub_ = nh_.subscribe("fix", 10, &NavSatTfPub::fixCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  pnh_.param<bool>("relative", relative_, false);
  pnh_.param<bool>("fix_hdop", fix_hdop_, false);

  override_frame_id_ = pnh_.getParam("frame_id", frame_id_);
  pnh_.param<std::string>("child_frame_id", child_frame_id_, "gps");

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
  heading_valid_ = true;
  heading_ = tf::createQuaternionMsgFromYaw(msg->data);
  publish(ros::Time::now());
}

void NavSatTfPub::imuCallback(const sensor_msgs::Imu::ConstPtr & msg) {
  heading_valid_ = true;
  heading_ = msg->orientation;
  for( int i=0; i<9; i++ ) {
    heading_cov_[i] = msg->orientation_covariance[i];
  }
  publish(ros::Time::now());
}

void NavSatTfPub::fixCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
  geographic_msgs::GeoPoint geo_point = geodesy::toMsg(*msg);
  geodesy::UTMPoint utm_point(geo_point);

  if(!initial_point_valid_) {
    initial_point_ = utm_point;
    initial_point_valid_ = true;

    geometry_msgs::TransformStamped utm_local;
    utm_local.header.stamp = ros::Time::now();
    utm_local.header.frame_id = "utm";
    utm_local.child_frame_id = "utm_local";
    utm_local.transform.translation.x = initial_point_.easting;
    utm_local.transform.translation.y = initial_point_.northing;
    utm_local.transform.translation.z = initial_point_.altitude;

    // zero rotation
    // we get rotation from the compass, or not at all
    utm_local.transform.rotation.w = 1.0;

    utm_local_tf.sendTransform(utm_local);
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
      v = v*v + B*B;
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

  gps_valid_ = true;
  gps_frame_id_ = msg->header.frame_id;
  publish(msg->header.stamp);
}

void NavSatTfPub::publish(const ros::Time & stamp) {
  nav_msgs::Odometry odom;
  std::string frame;
  if( relative_ ) {
    frame = "utm_local";
  } else {
    frame = "utm";
  }
  odom.header.frame_id = frame;
  odom.header.stamp    = stamp;
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
      if( gps_valid_ ) {
        odom.pose.covariance[i*6+j] = gps_cov_[i*3+j];
      } else {
        /*
        if( i == j ) {
          odom.pose.covariance[i*6+j] = 9999.9;
        } else {
        */
          odom.pose.covariance[i*6+j] = 0;
        //}
      }

      // copy in heading covariance
      if( heading_valid_ ) {
        odom.pose.covariance[(i+3)*6 + (j+3)] = heading_cov_[i*3+j];
      } else {
        /*
        if( i == j ) {
          odom.pose.covariance[(i+3)*6 + (j+3)] = 9999.9;
        } else {
        */
          odom.pose.covariance[(i+3)*6 + (j+3)] = 0;
        //}
      }
    }
  }
  odom_pub_.publish(odom);

  // publish utm_local to gps transform
  geometry_msgs::TransformStamped gps;
  gps.header.frame_id = frame;
  gps.header.stamp = stamp;
  gps.child_frame_id = child_frame_id_;
  gps.transform.translation.x = odom.pose.pose.position.x;
  gps.transform.translation.y = odom.pose.pose.position.y;
  gps.transform.translation.z = odom.pose.pose.position.z;
  gps.transform.rotation = odom.pose.pose.orientation;

  gps_tf.sendTransform(gps);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "utm_tf_pub");
  NavSatTfPub pub;
  ros::spin();
}
