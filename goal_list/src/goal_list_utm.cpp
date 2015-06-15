/*
 * goal_list.cpp
 * A list of GPS goals for the robot, maintained as a ROS node.
 *
 * Author: Austin Hendrix
 */

/*
 * Rearchitecting:
 *  - UKF takes care of incorporating GPS position now. This no longer needs to
 *    incorporate GPS feedback
 *  - UKF/utm_tf_pub publishes position relative to our starting position.
 *    What do we do with that here? Do we assume that the current position
 *    is the starting line? Or do we switch to publishing absolute UTM
 *    coordinates, and then publish goals in the absolute UTM frame?
 *  - Should we try to deal with altitude? Using the geodesy C++ API lets
 *    me handle altitude without too much trouble
 *    Don't bother dealing with altitude. The change in distance between
 *    sea level and Boulder is about 0.25% - less than my ability to measure
 *    anyway
 */

#include <stdint.h>
#include <math.h>

#include <vector>

#include <ros/ros.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <dagny_driver/Goal.h>

#include <nav_msgs/Odometry.h>

#include <goal_list/gps.h>

using namespace std;

vector<sensor_msgs::NavSatFix> * goals;
unsigned int current_goal;

bool loop = false;

// publisher for current goal
ros::Publisher goal_pub;
ros::Publisher goal_update_pub;

bool active = false;

void publishGoal() {
  // this is a tad inefficient, but it's the least-intrusive change to the
  // current architecture
  geographic_msgs::GeoPoint geo_goal = geodesy::toMsg(goals->at(current_goal));
  geodesy::UTMPoint utm_goal(geo_goal);
  geometry_msgs::PointStamped goal;
  goal.header.frame_id = "utm";
  goal.header.stamp = ros::Time::now();
  goal.point.x = utm_goal.easting;
  goal.point.y = utm_goal.northing;
  goal.point.z = utm_goal.altitude;

  goal_pub.publish(goal);
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr & msg) {
   // we've reached the goal. switch to the next goal.
   ROS_INFO("Goal %d reached", current_goal);
   ++current_goal;
   if( current_goal >= goals->size() ) {
      if( loop && goals->size() > 0 ) {
         current_goal = 0;
         ROS_INFO("Last goal. Looping around");
         publishGoal();
      } else {
         active = false;
         ROS_INFO("Last goal. Deactivating");
      }
   } else {
     publishGoal();
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
         if( current_goal >= goals->size() ) {
            current_goal = goals->size() - 1;
         }
         // re-activate if we're inactive
         if(!active) {
           active = true;
           publishGoal();
         }
         sendCurrentGoalUpdate();
         break;
      case dagny_driver::Goal::DELETE:
         {
            const int id = goal->id;
            if( id < 0 || id >= goals->size() ) {
              ROS_ERROR("Invalid goal id %d", id);
              return;
            }

            ROS_INFO("Removing goal at %d", goal->id);

            vector<sensor_msgs::NavSatFix>::iterator itr = goals->begin();
            goals->erase(itr + id);
            if( current_goal == id ) {
               publishGoal();
            } else if( current_goal > id ) {
               current_goal--;
            }

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
   ROS_INFO("Loaded %zd goals", goals->size());

   active = goals->size() > 0;
   n.getParam("loop", loop);

   ros::Subscriber goal_input = n.subscribe("goal_input", 10,
         goalInputCallback);
   ros::Subscriber goal_reached = n.subscribe("goal_reached", 1, 
         goalReachedCallback);

   // latched - we publish one message, and only send updates when we get a
   // message on goal_reached
   goal_pub = n.advertise<geometry_msgs::PointStamped>("current_goal", 10,
       true);
   goal_update_pub = n.advertise<dagny_driver::Goal>("goal_updates", 10);

   if( active ) publishGoal();

   ROS_INFO("Goal List ready");

   ros::spin();
}
