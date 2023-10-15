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





void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){//REVER ISTO POSSIVELMENTE ESTA MAL; CONFIRMAR DESIRED POSTION DEPOIS DE O COMANDAR COM GANHOS DIFERENTES
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


void captureStatusResult(ros::Publisher pub){
    //Develop here capture sensing mechanism, as of now, all captures fail (msg.data = 0;)
    std_msgs::Int32 msg;
    msg.data = 0;
    pub.publish(msg);
}

void executeCaptureManeuverCb(const std_msgs::Empty::ConstPtr& msg){

    captureMoment = 1;
    //Review capture maneuver, as of now, shuttle's desired position just changes to target's current position
   /* desiredVelocityLinear.x = shuttle->ekf.pos[0][0];
    desiredVelocityLinear.y = shuttle->ekf.pos[1][0];
    desiredVelocityLinear.z = 5;
*/

   // desiredPosition.x = target->ekf.pos[0][0] +  referential_relative_to_shuttle_x;
   // desiredPosition.y = target->ekf.pos[1][0] + referential_relative_to_shuttle_y;
   // desiredPosition.z = target->ekf.pos[2][0] + referential_relative_to_shuttle_z;
    
    
    captureStatus = 1;

}


void updateDesiredPosCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova posicao
        //rostopic pub /cooperative_planning/state_machine/desired_local_position geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

    desiredPosition.x = msg->x;
    desiredPosition.y = msg->y;
    desiredPosition.z = msg->z;
}

void updateDesiredVelCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova velocidade
        //rostopic pub /cooperative_planning/state_machine/desired_local_velocity geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

   
    desiredVelocityLinear.x = msg->x;
    desiredVelocityLinear.y = msg->y;
    desiredVelocityLinear.z = msg->z;
}


void updateDesiredPositionFollower(double x, double y, double z){
       
    desiredPosition.x = target->ekf.pos[0][0] + x;
    desiredPosition.y = target->ekf.pos[1][0] + y;
    //desiredPosition.z = target->ekf.pos[2][0] - verticalDistance + z;
    desiredPosition.z = target->ekf.pos[2][0]  + z;
    
}     



void desiredPosReached(ros::Publisher pub){
       
    if ( (abs(shuttle->ekf.pos[0][0]- desiredPosition.x) < proximity_radius_shuttle) && (abs(shuttle->ekf.pos[1][0]- desiredPosition.y) < proximity_radius_shuttle) && (abs(shuttle->ekf.pos[2][0]- desiredPosition.z) < proximity_radius_shuttle) )
        targetReached = 1;
    else{
        targetReached = 0;
        reachedFlag = 0;
    }  
        

    if (targetReached && reachedFlag == 0){
        reachedFlag = 1;
        std_msgs::Empty msg;
        pub.publish(msg);

    }
         
    
}     



void targetDesiredPosReached(ros::Publisher pub, double x, double y, double z){
       
    if ( (abs(target->ekf.pos[0][0] + x- target_inform_point[0][0]) < proximity_radius_target) && (abs(target->ekf.pos[1][0] + y - target_inform_point[1][0]) < proximity_radius_target) )
        informPointReached = 1;
    else{
        informPointReached = 0;
        reachedFlagTarget = 0;
    }  
        

    if (informPointReached && reachedFlagTarget == 0){
        reachedFlagTarget = 1;
        std_msgs::Empty msg;
        pub.publish(msg);

    }
         
    
}     


void targetIsCloseWarning(ros::Publisher pub, double x, double y, double z){
       
    if ( (abs(target->ekf.pos[0][0] + x- shuttle->ekf.pos[0][0]) < target_shuttle_horizontal_closeness) && (abs(target->ekf.pos[1][0] + y- shuttle->ekf.pos[1][0]) < target_shuttle_horizontal_closeness) && (abs(target->ekf.pos[2][0] + z - shuttle->ekf.pos[2][0]) < target_shuttle_vertical_closeness) )
        targetReachedShuttle = 1;
    else{
        targetReachedShuttle = 0;
        targetCloseFlag = 0;
    }  
        

    if (targetReachedShuttle && targetCloseFlag == 0){
        targetCloseFlag = 1;
        std_msgs::Empty msg;
        pub.publish(msg);

    }
         
    
}     


void finishedTrajectoryCb(const std_msgs::Empty::ConstPtr& msg){
  
    finished_trajectory = 1;
}


int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "offboard_shuttle_follower_controller");
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


    //hardcoded trajectory points
    shuttle_waiting_point[0][0]=5; shuttle_waiting_point[1][0]=-100; shuttle_waiting_point[2][0]=-22;    
    target_inform_point[0][0]=50; target_inform_point[1][0]=-100; target_inform_point[2][0]=-22;    
    //target_inform_point[0][0]=15; target_inform_point[1][0]=-100; target_inform_point[2][0]=-22;    
    shuttle_stop_area[0][0]=5; shuttle_stop_area[1][0]=100; shuttle_stop_area[2][0]=-22;    




    ros::Subscriber desired_pos_sub = nh->subscribe("cooperative_planning/state_machine/desired_local_position", 10, updateDesiredPosCb);
    ros::Subscriber desired_vel_sub = nh->subscribe("cooperative_planning/state_machine/desired_local_velocity", 10, updateDesiredVelCb);
    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/shuttle_reached_desired_position", 10);
    ros::Publisher target_reached_pos_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_reached_inform_point", 10);
    ros::Publisher target_is_close_pub = nh->advertise<std_msgs::Empty>("cooperative_planning/state_machine/target_is_close", 10);
    ros::Subscriber execute_capture_maneuver_sub = nh->subscribe("cooperative_planning/state_machine/execute_capture_maneuver", 10, executeCaptureManeuverCb);
    ros::Publisher capture_status_pub = nh->advertise<std_msgs::Int32>("cooperative_planning/state_machine/capture_success", 10);
    ros::Subscriber trajectory_finished_sub = nh->subscribe("/cooperative_planning/state_machine/finished_trajectory", 10, finishedTrajectoryCb);

    ros::Rate rate(20.0);

    ROS_WARN("STARTING OFFBOARD VELOCITY CONTROLLER FOR SHUTTLE DRONE BASED ON STATE MACHINE");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 5.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -3.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    shuttle->start_offboard_mission();

    //shuttle->set_pos_yaw(pos, yaw, 10);

    
    
    Tprev = ros::Time::now().toSec();
    position_error[0][0]= 0; position_error[1][0]= 0; position_error[2][0]=0;
    previous_error[0][0]= 0; previous_error[1][0]= 0; previous_error[2][0]=0;

    while(ros::ok() && !finished_trajectory){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Shuttle Current Position: " << shuttle->ekf.pos[0][0]  << "  " << shuttle->ekf.pos[1][0]  << "  " << shuttle->ekf.pos[2][0]);
        ROS_WARN_STREAM("Shuttle Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);
        ROS_WARN_STREAM("Shuttle Current Velocity: " << shuttle->ekf.vel[0][0]  << "  " << shuttle->ekf.vel[1][0]  << "  " << shuttle->ekf.vel[2][0]);
        ROS_WARN_STREAM("Shuttle Desired Velocity: " << desiredVelocityLinear.x  << "  " << desiredVelocityLinear.y  << "  " << desiredVelocityLinear.z);

        ROS_WARN_STREAM("Target Current Position: " << target->ekf.pos[0][0] + referential_relative_to_shuttle_x  << "  " << target->ekf.pos[1][0] + referential_relative_to_shuttle_y << "  " << target->ekf.pos[2][0] + referential_relative_to_shuttle_z );


        desiredPosReached(reached_pos_pub);
        targetDesiredPosReached(target_reached_pos_pub, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);
        targetIsCloseWarning(target_is_close_pub, referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);
        

        //if(!captureMoment)
            calculateDesiredVelocityLinear();
        
        /*if(captureStatus){
            captureStatusResult(capture_status_pub);
            captureStatus = 0;
            captureMoment = 0;
            
        }*/

        if(captureMoment) updateDesiredPositionFollower(referential_relative_to_shuttle_x,referential_relative_to_shuttle_y,referential_relative_to_shuttle_z);


        vel[0][0]=desiredVelocityLinear.x; vel[1][0]=desiredVelocityLinear.y; vel[2][0]=desiredVelocityLinear.z;    
        shuttle->set_vel_yaw(vel, desiredHeading, 0.001);
        //REVEER O CONTROLADOR LA DE CIMA PRA CORRIGIr os ganhos


        //pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
        //shuttle->set_pos_yaw(pos, yaw, 0.01);
        
       



        ros::spinOnce();
        rate.sleep();
    }


    shuttle->auto_land();


    return 0;


























}
