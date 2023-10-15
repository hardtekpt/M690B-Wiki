#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/Utils.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>



#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <ros/package.h>




ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * shuttle = nullptr;
DroneLib::UAV * target = nullptr;

DroneLib::DroneInfo shuttle_drone_info;
DroneLib::DroneInfo target_drone_info;


//Controller Variables
geometry_msgs::Point desiredPosition;
geometry_msgs::Point desiredVelocityLinear;
double desiredHeading;

double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 1;
int proximity_radius_shuttle = 1;
int proximity_radius_target = 10;
int target_shuttle_vertical_closeness = 6;
int target_shuttle_horizontal_closeness = 4;
int verticalDistance = 2;

int targetReached = 0;
int reachedFlag = 0;

int informPointReached = 0;
int reachedFlagTarget = 0;

int targetReachedShuttle = 0;
int targetCloseFlag = 0;

int captureMoment = 0;
int captureStatus = 0;

double position_error[3][1];
double previous_error[3][1];

double Tprev;

int finished_trajectory = 0;


//hardcoded trajectory points
double shuttle_waiting_point[3][1];
double target_inform_point[3][1];
double shuttle_stop_area[3][1];

double referential_relative_to_shuttle_x;
double referential_relative_to_shuttle_y;
double referential_relative_to_shuttle_z;
























int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_target_logger");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

   /* Get the namespace of the drones and other parameters */
    shuttle_drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespaceShuttle");
    shuttle_drone_info.ID = DroneGimmicks::getParameters<double>(*nh, "IDShuttle");
    shuttle_drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    shuttle_drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    shuttle_drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    shuttle_drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    shuttle_drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    shuttle_drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 

    target_drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespaceTarget");
    shuttle_drone_info.ID = DroneGimmicks::getParameters<double>(*nh, "IDTarget");
    target_drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    target_drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    target_drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    target_drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    target_drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    target_drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 


    /* Create the drone objects */
    shuttle = new DroneLib::UAV(shuttle_drone_info.drone_ns, 
        shuttle_drone_info.mass, 
        shuttle_drone_info.radius, 
        shuttle_drone_info.height, 
        shuttle_drone_info.num_rotors, 
        shuttle_drone_info.thrust_curve,
        nh, nh_p); 

    target = new DroneLib::UAV(target_drone_info.drone_ns, 
        target_drone_info.mass, 
        target_drone_info.radius, 
        target_drone_info.height, 
        target_drone_info.num_rotors, 
        target_drone_info.thrust_curve,
        nh, nh_p); 


    
    
    double aux_ned[3][1], aux_enu[3][1];
    aux_enu[0][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceX"); 
    aux_enu[1][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceY"); 
    aux_enu[2][0] =  DroneGimmicks::getParameters<double>(*nh, "initialRelativeDistanceZ");
    
    DroneLib::enu_to_ned(aux_enu, aux_ned);
    referential_relative_to_shuttle_x = aux_ned[0][0];
    referential_relative_to_shuttle_y = aux_ned[1][0];
    referential_relative_to_shuttle_z = aux_ned[2][0];


   

    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/shuttle_reached_desired_position", 10);
    ros::Publisher target_reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_reached_inform_point", 10);
    ros::Publisher target_is_close_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_is_close", 10);
    ros::Publisher capture_status_pub = nh->advertise<std_msgs::Int32>("cooperative_planning/state_machine/capture_success", 10);

    ros::Publisher  accel_shuttle_pub = nh->advertise<geometry_msgs::Vector3Stamped>("/"+shuttle_drone_info.drone_ns+"/mavros/setpoint_accel/accel", 1);
    
    
    
    
    
    
    ros::Rate rate(10.0);
    ROS_WARN("STARTING OFFBOARD TARGET STATE LOGGER");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

   


  // file name
    std::string file_name = "target_states_logsE";
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";
    std::string lib_name = prefix_lib + file_name + ".csv";



    //int lolo =1; int lele =23; int lulu = 423;
    std::ofstream myfile;
     /* myfile.open (lib_name, std::ios::app);
      myfile << "a,b,c,\n";
      myfile <<lolo << "," <<lele << "," <<lulu <<  "\n";
      
      myfile.close();*/

    while(ros::ok() && !finished_trajectory){
      
       

        myfile.open (lib_name, std::ios::app);
      myfile  << target->ekf.pos[0][0] + referential_relative_to_shuttle_x << "," << target->ekf.pos[1][0] + referential_relative_to_shuttle_y << "," << target->ekf.pos[2][0] + referential_relative_to_shuttle_z << "," <<  target->ekf.vel[0][0] << "," << target->ekf.vel[1][0] << "," << target->ekf.vel[2][0] << "," <<  target->ekf.att_euler[3][0] <<  "\n";
    ROS_WARN("logging...");

      
   

      myfile.close();


        ros::spinOnce();
        rate.sleep();
    }




    return 0;


























}
