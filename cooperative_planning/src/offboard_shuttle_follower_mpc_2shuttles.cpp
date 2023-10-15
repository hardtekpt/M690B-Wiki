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


#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>
#include <ros/package.h>

using namespace casadi;


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

double kp_velocity = 5, ki_velocity = kp_velocity/4, kp_heading = 1;
int proximity_radius = 1;
int verticalDistance = 3;
int targetReached = 0;
int reachedFlag = 0;

DM xx = DM::zeros(14,26);
DM uu = DM::zeros(4,25);






void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - target->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){
    /*desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - target->ekf.pos[0][0]);
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - target->ekf.pos[1][0]);
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - target->ekf.pos[2][0]);*/


    


    desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - target->ekf.pos[0][0]) + ki_velocity*((desiredPosition.x  - target->ekf.pos[0][0]) - target->ekf.vel[0][0]) ;
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - target->ekf.pos[1][0]) + ki_velocity*((desiredPosition.y  - target->ekf.pos[1][0]) - target->ekf.vel[1][0]) ;
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - target->ekf.pos[2][0]) + ki_velocity*((desiredPosition.z  - target->ekf.pos[2][0]) - target->ekf.vel[2][0]) ;
    calculateDesiredHeading();
}
 

void updateDesiredPosition(double x, double y, double z){
       
    desiredPosition.x = shuttle->ekf.pos[0][0] - x;
    desiredPosition.y = shuttle->ekf.pos[1][0] - y;
    desiredPosition.z = shuttle->ekf.pos[2][0] - verticalDistance - z;
    
}     


void mpcController( Function mpc, int relative_x,int relative_y,int relative_z){
       

    std::vector<double> xx0 = {  target->ekf.pos[0][0] ,target->ekf.pos[1][0],target->ekf.pos[2][0],  target->ekf.vel[0][0],target->ekf.vel[1][0],target->ekf.vel[2][0],  target->ekf.att_euler[3][0],     shuttle->ekf.pos[0][0] - relative_x,shuttle->ekf.pos[1][0] - relative_y,shuttle->ekf.pos[2][0] - relative_z - 2,  shuttle->ekf.vel[0][0],shuttle->ekf.vel[1][0],shuttle->ekf.vel[2][0],  shuttle->ekf.att_euler[3][0] }; //shuttle and target states for mpc
    
    ros::Time start_time = ros::Time::now();


        std::vector<DM> arg1 ={DM(xx0), xx, uu};

        std::vector<DM> res = mpc(arg1);
    casadi::Matrix<double> result_xx = res.at(1);
    casadi::Matrix<double> result_uu = res.at(0);

        xx = result_xx;
        uu = result_uu;
        

//    std::vector<DM> res = mpc(DM(xx0));

    
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    std::cout << "MPC computation time:  " << delta_t_sec << std::endl;

//std::cout << "lalala      " << res.at(0) << std::endl;
        std::cout << "lalala      " << res.at(1) << std::endl;
       



  /*  std::cout << "result uu: " << res.at(0) << std::endl;
    std::cout << "result xx: " << res.at(1) << std::endl;
    std::cout << "input xx0: " << xx0 << std::endl;
    casadi::Matrix<double> result_matrix = res.at(1);
    //std::cout << "result first element: " << result_matrix(0,0) << std::endl;
*/

    //predicted states for velocity and psi in next instance
   /* double vel_x = (double)result_matrix(3,0);
    double vel_y = (double)result_matrix(4,0);
    double vel_z = (double)result_matrix(5,0);
    double psi = (double)result_matrix(6,0);*/
 
    double vel_x = (double)result_xx(3,1);
    double vel_y = (double)result_xx(4,1);
    double vel_z = (double)result_xx(5,1);
    double psi = (double)result_xx(6,1);


    std::cout << "vel set values:  x: " << vel_x << " y: "  << vel_y << " z: "  << vel_z << std::endl;
    double vel_ned[3][1];
    vel_ned[0][0] = vel_x;
    vel_ned[1][0] = vel_y;
    vel_ned[2][0] = vel_z;

    target->set_vel_yaw(vel_ned, psi, 0.001);

    
/* quando o mpc retornava so uu, controlo em aceleracao
    //std::vector<double> xx0 = {  20, 10, -20, 0,0,0, 0,     0,0,-15, 11,0,0,0 };
    std::vector<double> xx0 = {  shuttle->ekf.pos[0][0] ,shuttle->ekf.pos[1][0],shuttle->ekf.pos[2][0],  shuttle->ekf.vel[0][0],shuttle->ekf.vel[0][0],shuttle->ekf.vel[0][0],  shuttle->ekf.att_euler[3][0],     target->ekf.pos[0][0] ,target->ekf.pos[1][0],target->ekf.pos[2][0],  target->ekf.vel[0][0],target->ekf.vel[0][0],target->ekf.vel[0][0],  target->ekf.att_euler[3][0] }; //shuttle and target states for mpc
    
    ros::Time start_time = ros::Time::now();
    std::vector<DM> res = mpc(DM(xx0));

    
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    std::cout << "MPC computation time:  " << delta_t_sec << std::endl;
    std::cout << "result: " << res.at(0) << std::endl;
    
    casadi::Matrix<double> result_matrix = res.at(0);
    //std::cout << "result first element: " << result_matrix(0,0) << std::endl;


    //valores vindos do mpc, confirmar o modelo para saber o que representam
    double accel_x = (double)result_matrix(0,0);
    double accel_y = (double)result_matrix(1,0);
    double accel_z = (double)result_matrix(2,0);
    double omegaz = (double)result_matrix(3,0);



    double accel_ned[3][1];
    accel_ned[0][0] = accel_x*shuttle_drone_info.mass;
    accel_ned[1][0] = accel_y*shuttle_drone_info.mass;
    accel_ned[2][0] = accel_z*shuttle_drone_info.mass - 9.81*shuttle_drone_info.mass ;

    //FAZER AINDA O SET DO YAW COM O SETPOINT CORRESPONDENTE CERTo




    std_msgs::Header h;
	geometry_msgs::Vector3Stamped accel_msg;

	h.stamp = ros::Time::now();
	accel_msg.header = h;

	double accel_enu[3][1];
	DroneLib::ned_to_enu(accel_ned, accel_enu);
	
	
   accel_msg.vector.x = accel_enu[0][0];
   accel_msg.vector.y = accel_enu[1][0];
   accel_msg.vector.z = accel_enu[2][0];

	pub.publish(accel_msg);
    */
         
  
         
    
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




     // file name
    std::string file_name = "gen_follower_only";
     // code predix
    std::string prefix_code = ros::package::getPath("cooperative_planning") + "/include/";
    // shared library prefix
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";

    // Create a new NLP solver instance from the compiled code
    std::string lib_name = prefix_lib + file_name + ".so";
    // Use CasADi's "external" to load the compiled function

    Function mpc_control = external("F",lib_name);


    ros::Rate rate(10.0);

    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE FOLLOWING A QUADROTOR");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 5.0;
    desiredPosition.z = -8.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    target->start_offboard_mission();

    target->set_pos_yaw(pos, yaw, 10);

    
    

    while(ros::ok()){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Current Position: " << target->ekf.pos[0][0]  << "  " << target->ekf.pos[1][0]  << "  " << target->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        updateDesiredPosition(referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);


        /*calculateDesiredVelocityLinear();

        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        target->set_vel_yaw(vel, desiredHeading, 0.001);

      */
        
        mpcController( mpc_control,referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);







        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
