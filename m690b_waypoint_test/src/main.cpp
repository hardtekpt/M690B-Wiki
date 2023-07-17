#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>

ros::NodeHandle * nh;
ros::NodeHandle * nh_p;

DroneLib::UAV * uav = nullptr;
DroneLib::DroneInfo drone_info;

int main (int argc, char ** argv){
    /* Initiate the node */
    ros::init(argc, argv, "m690b_waypoint_test_node");
    nh = new ros::NodeHandle();
    nh_p = new ros::NodeHandle("~");

    /* Get the namespace of the drone and other parameters */
    drone_info.drone_ns = DroneGimmicks::getParameters<std::string>(*nh, "namespace");
    drone_info.mass = DroneGimmicks::getParameters<double>(*nh, "drone_params/mass");
    drone_info.radius = DroneGimmicks::getParameters<double>(*nh, "drone_params/radius");
    drone_info.height = DroneGimmicks::getParameters<double>(*nh, "drone_params/height");
    drone_info.num_rotors = DroneGimmicks::getParameters<int>(*nh, "drone_params/num_motors");
    drone_info.g = DroneGimmicks::getParameters<double>(*nh, "drone_params/g");
    drone_info.thrust_curve = DroneGimmicks::getParameters<std::string>(*nh, "drone_params/thrust_curve"); 

    /* Create the drone object */
    uav = new DroneLib::UAV(drone_info.drone_ns, 
        drone_info.mass, 
        drone_info.radius, 
        drone_info.height, 
        drone_info.num_rotors, 
        drone_info.thrust_curve,
        nh, nh_p); 

    double pos[3][1];
	double yaw = 0.0;

    ros::Duration(5).sleep();
    ROS_WARN_STREAM("Program will start");
    
    // Set to offboard mode and arm
    uav->start_offboard_mission();

    // Takeoff
    pos[0][0]=0.0; pos[1][0]=0.0; pos[2][0]=-4.0;
    uav->set_pos_yaw(pos, yaw, 8);

    // Visit waypoint 1
    pos[0][0]=3.0; pos[1][0]=0.0; pos[2][0]=-4.0;
    uav->set_pos_yaw(pos, yaw, 6);

    // Visit waypoint 2
    pos[0][0]=3.0; pos[1][0]=3.0; pos[2][0]=-4.0;
    uav->set_pos_yaw(pos, yaw, 6);

    // Visit waypoint 3
    pos[0][0]=0.0; pos[1][0]=3.0; pos[2][0]=-4.0;
    uav->set_pos_yaw(pos, yaw, 6);

    // Return to takeoff location
    pos[0][0]=0.0; pos[1][0]=0.0; pos[2][0]=-4.0;
	uav->set_pos_yaw(pos, yaw, 6);

    // Land
    uav->auto_land();

    return 0;
}