#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>
#include <drone_utils_cpp/Utils.h>

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
geometry_msgs::Point desiredAttitude;
geometry_msgs::Point desiredVelocityLinear;
double desiredHeading;

double kp_velocity = 5, ki_velocity = kp_velocity/4, kp_heading = 1;

int proximity_radius = 1;
int targetReached = 0;
int reachedFlag = 0;


void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){
    /*desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - shuttle->ekf.pos[0][0]);
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - shuttle->ekf.pos[1][0]);
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - shuttle->ekf.pos[2][0]);*/

    desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - target->ekf.pos[0][0]) + ki_velocity*((desiredPosition.x  - target->ekf.pos[0][0]) - target->ekf.vel[0][0]) ;
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - target->ekf.pos[1][0]) + ki_velocity*((desiredPosition.y  - target->ekf.pos[1][0]) - target->ekf.vel[1][0]) ;
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - target->ekf.pos[2][0]) + ki_velocity*((desiredPosition.z  - target->ekf.pos[2][0]) - target->ekf.vel[2][0]) ;
    calculateDesiredHeading();
}


void updateDesiredPosCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova posicao
        //rostopic pub /cooperative_planning/targetController/desired_local_position geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

    desiredPosition.x = msg->x;
    desiredPosition.y = msg->y;
    desiredPosition.z = msg->z;
}

void updateDesiredAttCb(const geometry_msgs::Point::ConstPtr& msg){
        //Enviar mensagem configuracao nova attitude
        //rostopic pub /cooperative_planning/targetController/desired_local_attitude geometry_msgs/Point  '{x: 10.0, y: 10.0, z: 10.0}'

    desiredAttitude.x = msg->x;
    desiredAttitude.y = msg->y;
    desiredAttitude.z = msg->z;
}


void desiredPosReached(ros::Publisher pub){
       
    if ( (abs(target->ekf.pos[0][0]- desiredPosition.x) < proximity_radius) && (abs(target->ekf.pos[1][0]- desiredPosition.y) < proximity_radius) && (abs(target->ekf.pos[2][0]- desiredPosition.z) < proximity_radius) )
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


void sendAttitudeSetPoint(ros::Publisher pub){
	std_msgs::Header h;
	mavros_msgs::AttitudeTarget msg;
	geometry_msgs::Quaternion q_msg;

	h.stamp = ros::Time::now();
	msg.header = h;

	double att_euler[3][1];
    att_euler[0][0]=desiredAttitude.x; att_euler[1][0]=desiredAttitude.y; att_euler[2][0]=desiredAttitude.z;

	double att_enu[3][1];
	DroneLib::ned_to_enu(att_euler, att_enu);
	tf2::Quaternion q_tf; q_tf.setRPY(att_enu[0][0],att_enu[1][0],att_enu[2][0]); q_msg = tf2::toMsg(q_tf);
	
	
	msg.orientation = q_msg;

	pub.publish(msg);
    
         
    
}     




void sendTakeOffCommand(ros::Publisher pub){
	std_msgs::Header h;
	double pos_enu[3][1];
	mavros_msgs::PositionTarget msg;

    //takeoff 15 meters in front of initial position and at 20 meters of altitude
	double pos_ned[3][1];
    pos_ned[0][0]= 15; pos_ned[1][0]=0; pos_ned[2][0]=-20;    


	h.stamp = ros::Time::now();
	msg.header = h;
	msg.coordinate_frame = 1; msg.type_mask = 4096;
	DroneLib::ned_to_enu(pos_ned, pos_enu);
    msg.position.x = pos_enu[0][0]; msg.position.y = pos_enu[1][0]; msg.position.z = pos_enu[2][0]; 
	pub.publish(msg);

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





    //---------------------------------------------------------------

    ros::Subscriber desired_pos_sub = nh->subscribe("cooperative_planning/targetController/desired_local_position", 10, updateDesiredPosCb);
    ros::Subscriber desired_att_sub = nh->subscribe("cooperative_planning/targetController/desired_local_attitude", 10, updateDesiredAttCb);
    ros::Publisher reached_pos_pub = nh->advertise<std_msgs::Int64>("cooperative_planning/targetController/reached_target_pos", 10);
    ros::Publisher set_att_pub = nh->advertise<mavros_msgs::AttitudeTarget>("/"+target_drone_info.drone_ns+"/mavros/setpoint_raw/attitude", 1);

	ros::Publisher  pos_target_pub = nh->advertise<mavros_msgs::PositionTarget>("/"+target_drone_info.drone_ns+"/mavros/setpoint_raw/local", 1);


    ros::Rate rate(20.0);

    ROS_WARN("STARTING OFFBOARD CONTROLLER FOR TARGET DRONE ");
    double pos[3][1], vel[3][1];
	double yaw, t0, t;

    desiredPosition.x = 0.0;
    desiredPosition.y = 0.0;
    desiredPosition.z = -20.0;

    desiredAttitude.x = 0.0;
    desiredAttitude.y = 0.0;
    desiredAttitude.z = 0.0;

    pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
    //pos[0][0]=0; pos[1][0]=0; pos[2][0]=-3;
	yaw = 0.0;
    
    target->start_offboard_mission();

    sendTakeOffCommand(pos_target_pub); //nao esta a funcioanr
 
    target->set_pos_yaw(pos, yaw, 10);

    
    

    while(ros::ok()){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Current Position: " << target->ekf.pos[0][0]  << "  " << target->ekf.pos[1][0]  << "  " << target->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        desiredPosReached(reached_pos_pub);


        //calculateDesiredVelocityLinear();

        pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
        target->set_pos_yaw(pos, yaw, 0.01);


        //sendAttitudeSetPoint(set_att_pub);





        ros::spinOnce();
        rate.sleep();
    }

    return 0;





















}
