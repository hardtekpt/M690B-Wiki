#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <drone_utils_cpp/Utils.h>
#include <mavros_cpp/UAV.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

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

#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <ros/package.h>
#include <string>
#include <fstream>

using namespace casadi;



ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * shuttle = nullptr;

DroneLib::DroneInfo shuttle_drone_info;


//Controller Variables
geometry_msgs::Point desiredPosition;
geometry_msgs::Point desiredVelocityLinear;
double desiredHeading;

double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 1;
int proximity_radius_shuttle = 1;
int proximity_radius_target = 15;
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



int lala = 0;
DM xx = DM::zeros(14,26);
DM uu = DM::zeros(4,25);




double minimum_distance = 10000;



void mpcController( Function mpc,double relative_x, double relative_y, double relative_z,double state1,double state2,double state3,double state4,double state5,double state6,double state7){
       


    std::vector<double> xx0 = {  shuttle->ekf.pos[0][0] ,shuttle->ekf.pos[1][0],shuttle->ekf.pos[2][0],  shuttle->ekf.vel[0][0],shuttle->ekf.vel[1][0],shuttle->ekf.vel[2][0],  shuttle->ekf.att_euler[3][0],     state1 + relative_x ,state2  + relative_y,state3 + relative_z,  state4,state5,state6,  state7}; //shuttle and target states for mpc
    
    ros::Time start_time = ros::Time::now();
    
    
    std::vector<DM> arg1 ={DM(xx0), xx, uu};

        std::vector<DM> res = mpc(arg1);
    casadi::Matrix<double> result_xx = res.at(1);
    casadi::Matrix<double> result_uu = res.at(0);

        xx = result_xx;
        uu = result_uu;
        


    
    ros::Duration delta_t = ros::Time::now() - start_time;
    double delta_t_sec = delta_t.toSec();
    std::cout << "MPC computation time:  " << delta_t_sec << std::endl;
    //std::cout << "result uu: " << res.at(0) << std::endl;
    //std::cout << "result xx: " << res.at(1) << std::endl;
    
    casadi::Matrix<double> result_matrix = res.at(1);
    //std::cout << "result first element: " << result_matrix(0,0) << std::endl;


    //predicted states for velocity and psi in next instance
    double vel_x = (double)result_matrix(3,1);
    double vel_y = (double)result_matrix(4,1);
    double vel_z = (double)result_matrix(5,1);
    double psi = (double)result_matrix(6,2);

    double vel_ned[3][1];
    vel_ned[0][0] = vel_x;
    vel_ned[1][0] = vel_y;
    vel_ned[2][0] = vel_z;

    shuttle->set_vel_yaw(vel_ned, psi, 0.001);

    
    
}     



int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_follower_mpc_controller");
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

    

    ros::Publisher target_states_pub = nh->advertise<std_msgs::String>("cooperative_planning/virtual_point_states", 10);
    ros::Publisher target_pose_pub = nh->advertise<visualization_msgs::Marker>("cooperative_planning/target_pose", 10);


    /* Create the drone objects */
    shuttle = new DroneLib::UAV(shuttle_drone_info.drone_ns, 
        shuttle_drone_info.mass, 
        shuttle_drone_info.radius, 
        shuttle_drone_info.height, 
        shuttle_drone_info.num_rotors, 
        shuttle_drone_info.thrust_curve,
        nh, nh_p); 

   

    

    
    referential_relative_to_shuttle_x = 0;
    referential_relative_to_shuttle_y = 0;
    referential_relative_to_shuttle_z = 0;
    target_inform_point[0][0]=40; target_inform_point[1][0]=-15; target_inform_point[2][0]=-25;    


     // file name
    std::string file_name = "gen";
     // code predix
    std::string prefix_code = ros::package::getPath("cooperative_planning") + "/include/";
    // shared library prefix
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";

    // Create a new NLP solver instance from the compiled code
    std::string lib_name = prefix_lib + file_name + ".so";
    // Use CasADi's "external" to load the compiled function

    Function mpc_control = external("F",lib_name);

    
    ros::Rate rate(20.0);
    ROS_WARN("STARTING OFFBOARD MPC CONTROLLER FOR SHUTTLE DRONE FOLLOWING VIRTUAL POINT");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -20.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();
    shuttle->set_pos_yaw(pos, yaw, 3);

    
    
    Tprev = ros::Time::now().toSec();
    position_error[0][0]= 0; position_error[1][0]= 0; position_error[2][0]=0;
    previous_error[0][0]= 0; previous_error[1][0]= 0; previous_error[2][0]=0;





    desiredPosition.x = 0;
    desiredPosition.y = 0;
    desiredPosition.z = -25;   
    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
	yaw = 0.0;
    
    shuttle->set_pos_yaw(pos, yaw, 3);


  // file name
    std::string log_file_name = "target_states_logs";
    std::string prefix_log = ros::package::getPath("cooperative_planning") + "/include/";
    std::string log_name = prefix_log + log_file_name + ".csv";

    std::ifstream myfile;
    myfile.open (log_name);



int target_reached_inform = 0;







    while(ros::ok() && !finished_trajectory){
      
        //ros::Time start_time_main = ros::Time::now();

        std::string line;
        
        if(std::getline(myfile, line)){


            std::stringstream ss(line);
             double result[7];
            int i = 0;    
            double aux;
            while(ss >> aux){
                result[i] = aux ;
                if(ss.peek() == ',') ss.ignore();
                i++; 
                }



            double state1 = result[0];
            double state2 = result[1];
            double state3 = result[2];
            double state4 = result[3];
            double state5 = result[4];
            double state6 = result[5];
            double state7 = result[6];

            std_msgs::String msg;
                msg.data = line;

            target_states_pub.publish(msg);


         
            if ( (abs(state1+ referential_relative_to_shuttle_x- target_inform_point[0][0]) < proximity_radius_target) && (abs(state2 + referential_relative_to_shuttle_y - target_inform_point[1][0]) < proximity_radius_target) )
                target_reached_inform = 1;


            if(target_reached_inform)   mpcController( mpc_control, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z, state1,state2,state3,state4,state5,state6,state7);
                
            else shuttle->set_pos_yaw(pos, yaw, 0.001);



            double minimum_distance_aux = sqrt( pow(2,(shuttle->ekf.pos[0][0] - (state1 + referential_relative_to_shuttle_x) ) )  + pow(2,(shuttle->ekf.pos[1][0] -(state2 + referential_relative_to_shuttle_y )))  + pow(2,(shuttle->ekf.pos[2][0]-( state3 + referential_relative_to_shuttle_z ))) );
            if( minimum_distance_aux < minimum_distance)
                    minimum_distance = minimum_distance_aux;
                ROS_WARN_STREAM("minimum distance between drones: " << minimum_distance);
                ROS_WARN_STREAM("current distance between drones: " << minimum_distance_aux);

        }
        else{
            myfile.close();

            break;
        }





        //ros::Duration delta_t_main = ros::Time::now() - start_time_main;
        //double delta_t_sec_main = delta_t_main.toSec();
        //std::cout << "Main loop computation time:  " << delta_t_sec_main << std::endl;
        

        ros::spinOnce();
        rate.sleep();
    }
    
    
    shuttle->set_pos_yaw(pos, yaw, 10);

    shuttle->auto_land();


    return 0;


























}
