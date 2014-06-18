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
 *  - Publishing the initial position to UTM transform from the utm publisher
 *    would be a good use of the tf2 static publisher, and would probably apply
 *    to Junior as well
 */

#include <stdint.h>
#include <math.h>

#include <vector>

#include <ros/ros.h>

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
      if( loop && goals->size() > 0 ) {
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
         if( current_goal >= goals->size() ) {
            current_goal = goals->size() - 1;
         }
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


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
   geometry_msgs::PointStamped goal; // goal, in odom frame
   if( active ) {
      sensor_msgs::NavSatFix gps_goal = goals->at(current_goal);

      segment diff = gpsDist(*msg, gps_goal);

      // use last_odom as the position in the odom frame that corresponds to
      // this GPS location

      // convert to radians North of East
      double heading = (M_PI / 2.0) - diff.heading;

      ROS_INFO("Goal %d: distance %lf, heading %lf", current_goal, diff.distance,
               heading);

      // no need to normalize heading
      goal.point.x = last_odom.x + diff.distance * cos(heading);
      goal.point.y = last_odom.y + diff.distance * sin(heading);
      goal.point.z = 0.0;

      goal.header.frame_id = "odom";
      goal.header.stamp = ros::Time::now();

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
   ROS_INFO("Loaded %zd goals", goals->size());

   active = goals->size() > 0;
   n.getParam("loop", loop);


   ros::Subscriber odom = n.subscribe("odom", 2, odomCallback);
   ros::Subscriber gps = n.subscribe("gps", 2, gpsCallback);
   ros::Subscriber goal_input = n.subscribe("goal_input", 10,
         goalInputCallback);
   ros::Subscriber goal_reached = n.subscribe("goal_reached", 1, 
         goalReachedCallback);

   goal_pub = n.advertise<geometry_msgs::PointStamped>("current_goal", 10);
   goal_update_pub = n.advertise<dagny_driver::Goal>("goal_updates", 10);

   ROS_INFO("Goal List ready");

   ros::spin();
}
