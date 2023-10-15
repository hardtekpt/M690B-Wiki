#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/Utils.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>

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

//double kp_velocity = 5, ki_velocity = kp_velocity/4, kp_heading = 1;
double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 1;

int proximity_radius = 1;
int verticalDistance = 2;

int targetReached = 0;
int reachedFlag = 0;

double position_error[3][1];
double previous_error[3][1];

double Tprev;


double referential_relative_to_shuttle_x;
double referential_relative_to_shuttle_y;
double referential_relative_to_shuttle_z;


void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){
      double e_pX = desiredPosition.x  - shuttle->ekf.pos[0][0];
    double e_pY = desiredPosition.y  - shuttle->ekf.pos[1][0];
    double e_pZ = desiredPosition.z  - shuttle->ekf.pos[2][0];

    double Tatual = ros::Time::now().toSec();
    position_error[0][0] += (Tatual - Tprev)*e_pX;
    position_error[1][0] += (Tatual - Tprev)*e_pY;
    position_error[2][0] += (Tatual - Tprev)*e_pZ;
    
    double e_dX = (e_pX  - previous_error[0][0])/(Tatual - Tprev);
    double e_dY = (e_pY  - previous_error[1][0])/(Tatual - Tprev);
    double e_dZ = (e_pZ  - previous_error[2][0])/(Tatual - Tprev);

    Tprev = Tatual;
    previous_error[0][0]= e_pX; previous_error[1][0]= e_pY; previous_error[2][0]=e_pZ;

    desiredVelocityLinear.x = kp_velocity*e_pX + ki_velocity*position_error[0][0] + kd_velocity*e_dX;
    desiredVelocityLinear.y = kp_velocity*e_pY + ki_velocity*position_error[1][0] + kd_velocity*e_dY;
    desiredVelocityLinear.z = kp_velocity*e_pZ + ki_velocity*position_error[2][0] + kd_velocity*e_dZ;
    
 

            
    calculateDesiredHeading();

}
 

void updateDesiredPosition(double x, double y, double z){
       
    desiredPosition.x = target->ekf.pos[0][0] + x;
    desiredPosition.y = target->ekf.pos[1][0] + y;
    desiredPosition.z = target->ekf.pos[2][0] - verticalDistance + z;
    
}     





int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_follower");
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
    double referential_relative_to_shuttle_x = aux_ned[0][0];
    double referential_relative_to_shuttle_y = aux_ned[1][0];
    double referential_relative_to_shuttle_z = aux_ned[2][0];




    ros::Rate rate(10.0);

    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE FOLLOWING A QUADROTOR");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -10.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();

    shuttle->set_pos_yaw(pos, yaw, 10);

    
    Tprev = ros::Time::now().toSec();
    position_error[0][0]= 0; position_error[1][0]= 0; position_error[2][0]=0;
    while(ros::ok()){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        updateDesiredPosition(referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);


        calculateDesiredVelocityLinear();

        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        shuttle->set_vel_yaw(vel, desiredHeading, 0.001);

      
        







        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
