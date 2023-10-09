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

ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * shuttle = nullptr;

DroneLib::DroneInfo shuttle_drone_info;


//Controller Variables
geometry_msgs::Point desiredPosition;
geometry_msgs::Point desiredVelocityLinear;
double desiredHeading;

double kp_velocity = 1, ki_velocity = 0, kd_velocity = 0, kp_heading = 0;

double proximity_radius = 0.7;
int targetReached = 0;
int reachedFlag = 0;


double position_error[3][1];
double previous_error[3][1];

double Tprev;

int finished_trajectory = 0;

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

       /*if( desiredVelocityLinear.x  > 1 )
        desiredVelocityLinear.x = 1;
    else {if( desiredVelocityLinear.x  < -1 )
            desiredVelocityLinear.x = -1;}


    if( desiredVelocityLinear.y  > 1 )
        desiredVelocityLinear.y = 1;
    else {if( desiredVelocityLinear.y  < -1 )
            desiredVelocityLinear.y = -1;}

    if( desiredVelocityLinear.y  > 1 )
        desiredVelocityLinear.y = 1;
    else {if( desiredVelocityLinear.y  < -1 )
            desiredVelocityLinear.y = -1;}*/
    calculateDesiredHeading();
}


void updateDesiredPosCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova posicao
        //rostopic pub /cooperative_planning/shuttleController/desired_local_position geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

    desiredPosition.x = msg->x;
    desiredPosition.y = msg->y;
    desiredPosition.z = msg->z;
}
void finishedTrajectoryCb(const std_msgs::Empty::ConstPtr& msg){
  
    finished_trajectory = 1;
}


void desiredPosReached(ros::Publisher pub){
       
    if ( (abs(shuttle->ekf.pos[0][0]- desiredPosition.x) < proximity_radius) && (abs(shuttle->ekf.pos[1][0]- desiredPosition.y) < proximity_radius) && (abs(shuttle->ekf.pos[2][0]- desiredPosition.z) < proximity_radius) )
        targetReached = 1;
    else{
        targetReached = 0;
        reachedFlag = 0;
    }  
        

    if (targetReached && reachedFlag == 0){
        reachedFlag = 1;
        std_msgs::Int64 msg;
        msg.data = 1;
        pub.publish(msg);

    }
         
    
}     






int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_vel_control");
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

    
    /* Create the drone objects */
    shuttle = new DroneLib::UAV(shuttle_drone_info.drone_ns, 
        shuttle_drone_info.mass, 
        shuttle_drone_info.radius, 
        shuttle_drone_info.height, 
        shuttle_drone_info.num_rotors, 
        shuttle_drone_info.thrust_curve,
        nh, nh_p); 


    //---------------------------------------------------------------

    ros::Subscriber desired_pos_sub = nh->subscribe("cooperative_planning/shuttleController/desired_local_position", 10, updateDesiredPosCb);
    ros::Subscriber trajectory_finished_sub = nh->subscribe("cooperative_planning/shuttleController/finished_trajectory", 10, finishedTrajectoryCb);
    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Int64>("cooperative_planning/shuttleController/reached_target_pos", 10);


    ros::Rate rate(20.0);

    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE ");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -3.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();

    shuttle->set_pos_yaw(pos, yaw, 5);

    
    Tprev = ros::Time::now().toSec();
    position_error[0][0]= 0; position_error[1][0]= 0; position_error[2][0]=0;
    previous_error[0][0]= 0; previous_error[1][0]= 0; previous_error[2][0]=0;


    while(ros::ok() && !finished_trajectory){
      

        ROS_WARN_STREAM("Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        desiredPosReached(reached_pos_pub);

        calculateDesiredVelocityLinear();

        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        shuttle->set_vel_yaw(vel, desiredHeading, 0.01);

        ros::spinOnce();
        rate.sleep();
    }

    shuttle->auto_land();

    return 0;


}