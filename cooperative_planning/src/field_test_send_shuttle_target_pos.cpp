#include "ros/ros.h"

#include <sstream>

#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>



  struct TargetPosition {
    int x;
    int y;
    int z;
  };


  int pos1_x = 0;
  int pos1_y = 0;
  int pos1_z = -5;
  
  int pos2_x = 68;
  int pos2_y = -27;
  int pos2_z = -5;

  int pos3_x = -63;
  int pos3_y = 23;
  int pos3_z = -5;
  
  int pos4_x = 0;
  int pos4_y = 0;
  int pos4_z = -5;
  
  /*int pos5_x = 0;
  int pos5_y = 0;
  int pos5_z = -3;
  
  int pos6_x = 6;
  int pos6_y = 0;
  int pos6_z = -3;
  
  int pos7_x = 0;
  int pos7_y = 0;
  int pos7_z = -3;*/

  //TargetPosition posArray[7] = { {pos1_x, pos1_y, pos1_z}, {pos2_x, pos2_y, pos2_z} ,{pos3_x, pos3_y, pos3_z} ,{pos4_x, pos4_y, pos4_z}, {pos5_x, pos5_y, pos5_z}, {pos6_x, pos6_y, pos6_z}, {pos7_x, pos7_y, pos7_z} };
  TargetPosition posArray[4] = { {pos1_x, pos1_y, pos1_z}, {pos2_x, pos2_y, pos2_z} ,{pos3_x, pos3_y, pos3_z}, {pos4_x, pos4_y, pos4_z} };

  int reached = 0;
  int currentPos = 0;
  int finished_trajectory = 0;

void posReachedCb(const std_msgs::Int64::ConstPtr& msg){
    reached = msg->data;
    ROS_WARN_STREAM("Last Desired Position Reached");

    if(reached){
      if(((sizeof(posArray) / sizeof(TargetPosition)) - 1) != currentPos)
        currentPos++;
      else{
         finished_trajectory = 1;

      }
    }
        
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "send_shuttle_target_pos");
  ros::NodeHandle n;
  ros::Rate rate(10);
 
  ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Point>("cooperative_planning/shuttleController/desired_local_position", 10);
  ros::Subscriber reached_pos_sub = n.subscribe("cooperative_planning/shuttleController/reached_target_pos", 10, posReachedCb);
  ros::Publisher trajectory_finished_pub = n.advertise<std_msgs::Empty>("cooperative_planning/shuttleController/finished_trajectory", 10);
  


  int count = 0;
  while (ros::ok()){
   
    geometry_msgs::Point pos_to_send;
    pos_to_send.x = posArray[currentPos].x;
    pos_to_send.y = posArray[currentPos].y;
    pos_to_send.z = posArray[currentPos].z;
    target_pos_pub.publish(pos_to_send);


    if(finished_trajectory){
        std_msgs::Empty msg;
        trajectory_finished_pub.publish(msg);
        ros::Duration(2).sleep(); 
        break;
    }
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}