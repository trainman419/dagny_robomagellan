/*
 * goal_list.cpp
 * A list of GPS goals for the robot, maintained as a ROS node.
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>
#include <math.h>

#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <dagny_driver/Goal.h>

#include <nav_msgs/Odometry.h>

using namespace std;

vector<sensor_msgs::NavSatFix> * goals;
unsigned int current_goal;

bool loop = false;

// publisher for current goal
ros::Publisher goal_pub;
ros::Publisher goal_update_pub;

geometry_msgs::Point last_odom;
   
bool active = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   //ROS_INFO("Got position update");
   last_odom = msg->pose.pose.position;
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr & msg) {
   // we've reached the goal. switch to the next goal.
   ROS_INFO("Goal Reached");
   ++current_goal;
   if( current_goal >= goals->size() ) {
      if( loop ) {
         current_goal = 0;
         ROS_INFO("Last goal. Looping around");
      } else {
         active = false;
         ROS_INFO("Last goal. Deactivating");
      }
   }
}

void sendCurrentGoalUpdate() {
   dagny_driver::Goal g;
   g.operation = dagny_driver::Goal::SET_CURRENT;
   g.id = current_goal;
   goal_update_pub.publish(g);
}

void goalInputCallback(const dagny_driver::Goal::ConstPtr & goal) {
   switch(goal->operation) {
      case dagny_driver::Goal::APPEND:
         goals->push_back(goal->goal);
         // re-activate if we're inactive
         active = true;
         sendCurrentGoalUpdate();
         break;
      case dagny_driver::Goal::DELETE:
         {
            ROS_INFO("Removing goal at %d", goal->id);

            vector<sensor_msgs::NavSatFix>::iterator itr = goals->begin();
            int id = goal->id;
            while( id ) {
               ++itr;
               id--;
            }
            goals->erase(itr);
            if( current_goal > goal->id )
               current_goal--;
            if( goals->size() > 0 ) {
               sendCurrentGoalUpdate();
            } else {
               ROS_INFO("No goals; deactivating");
               active = false;
            }
         }
         break;
      default:
         ROS_ERROR("Unimplemented goal list operation: %d", goal->operation);
         break;
   }
}

// Radius of earth in m
#define R 6371000

// Goal Tolerance
//  TODO: convert to parameter shared with path planner
#define GOAL_TOLERANCE 5.0

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
   geometry_msgs::Point goal = last_odom; // goal, in odom frame
   if( active ) {
      sensor_msgs::NavSatFix gps_goal = goals->at(current_goal);

      // use last_odom as the position in the odom frame that corresponds to
      // this GPS location
      double start_lat, start_lon, end_lat, end_lon;
      start_lat = msg->latitude / 180.0 * M_PI;
      start_lon = msg->longitude / 180.0 * M_PI;
      end_lat = gps_goal.latitude / 180.0 * M_PI;
      end_lon = gps_goal.longitude / 180.0 * M_PI;

      // Haversine formula (http://www.movable-type.co.uk/scripts/latlong.html)
      double delta_lat = end_lat - start_lat;
      double delta_lon = end_lon - start_lon;
      double a = sin(delta_lat/2.0)*sin(delta_lat/2.0) + 
         cos(start_lat)*cos(end_lat)*sin(delta_lon/2.0)*sin(delta_lon/2);
      double c = 2 * atan2(sqrt(a), sqrt(1 - a));
      double d = R * c; // distance to goal


      // Bearing formula (from above)
      //  expressed as radians east of North
      double theta = atan2( sin(delta_lon)*cos(end_lat),
            cos(start_lat)*sin(end_lat) - 
            sin(start_lat)*cos(end_lat)*cos(delta_lon));

      // convert to radians North of East
      double heading = (M_PI / 2.0) - theta;

      ROS_INFO("Goal %d: distance %lf, heading %lf", current_goal, d, heading);

      // no need to normalize heading
      goal.x = last_odom.x + d * cos(heading);
      goal.y = last_odom.y + d * sin(heading);

      goal_pub.publish(goal);
   }
}

int main(int argc, char ** argv) {
   goals = new vector<sensor_msgs::NavSatFix>();
   current_goal = 0;

   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   // load goal list from parameter server
   XmlRpc::XmlRpcValue xml_goals;
   n.getParam("goals", xml_goals);
   if( xml_goals.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
      ROS_ERROR("param 'goals' is not a list");
   } else {
      for( int i=0; i<xml_goals.size(); ++i ) {
         if( xml_goals[i].getType() != XmlRpc::XmlRpcValue::TypeArray ) {
            ROS_ERROR("goals[%d] is not a list", i);
         } else {
            if( xml_goals[i].size() != 2 ) {
               ROS_ERROR("goals[%d] is not a pair", i);
            } else if( 
             xml_goals[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
             xml_goals[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble ) {
               ROS_ERROR("goals[%d] is not a pair of doubles", i);
            } else {
               sensor_msgs::NavSatFix g;
               g.latitude = xml_goals[i][0];
               g.longitude = xml_goals[i][1];
               goals->push_back(g);
            }
         }
      }
   }
   ROS_INFO("Loaded %d goals", goals->size());

   active = goals->size() > 0;
   n.getParam("loop", loop);


   ros::Subscriber odom = n.subscribe("odom", 2, odomCallback);
   ros::Subscriber gps = n.subscribe("gps", 2, gpsCallback);
   ros::Subscriber goal_input = n.subscribe("goal_input", 10,
         goalInputCallback);
   ros::Subscriber goal_reached = n.subscribe("goal_reached", 1, 
         goalReachedCallback);

   goal_pub = n.advertise<geometry_msgs::Point>("current_goal", 10);
   goal_update_pub = n.advertise<dagny_driver::Goal>("goal_updates", 10);

   ROS_INFO("Goal List ready");

   ros::spin();
}
