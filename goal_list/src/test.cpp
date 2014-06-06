#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <goal_list/gps.h>

using namespace std;

int main(int argc, char ** argv) {
   vector<sensor_msgs::NavSatFix> * goals = new vector<sensor_msgs::NavSatFix>();

   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   // load goal list from parameter server
   XmlRpc::XmlRpcValue xml_goals;
   n.getParam("goals_rel", xml_goals);
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

   // load start GPS position
   sensor_msgs::NavSatFix start;

   XmlRpc::XmlRpcValue xml_start;
   n.getParam("start", xml_start);
   if( xml_goals.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
      ROS_ERROR("param 'start' is not a list");
   } else {
      for( int i=0; i<xml_start.size(); ++i ) {
        if( xml_start.size() != 2 ) {
          ROS_ERROR("start is not a pair", i);
        } else if(xml_start[0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
                  xml_start[1].getType() != XmlRpc::XmlRpcValue::TypeDouble ) {
          ROS_ERROR("start is not a pair of doubles", i);
        } else {
          start.latitude = xml_start[0];
          start.longitude = xml_start[1];
        }
      }
   }
   ROS_INFO("Loaded start location");
   goals->insert(goals->begin(), start);

   for( int i=1; i<goals->size(); i++ ) {
     segment dist = gpsDist(goals->at(i-1), goals->at(i));
     ROS_INFO("Distance from %d to %d: %f", i-1, i, dist.distance);
   }
}
