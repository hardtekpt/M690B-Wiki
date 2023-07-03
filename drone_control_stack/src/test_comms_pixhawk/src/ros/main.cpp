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
    ros::init(argc, argv, "lissajous_node");
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

    ROS_INFO("STARTING");
    ros::Duration(5).sleep();

    // Telemetry test
    int t = 0;
    while (t < 10){
	ROS_WARN_STREAM(uav->sen.imu.ang_vel[0][0] << uav->sen.imu.ang_vel[1][0] << uav->sen.imu.ang_vel[2][0]);
        ROS_WARN_STREAM(uav->sen.gps.pos[0][0] << uav->sen.gps.pos[1][0] << uav->sen.gps.pos[2][0]);
	ros::Duration(1).sleep();
	t += 1;
    }


    // Arming test
    uav->arm_drone();
    ros::Duration(5).sleep();
    uav->disarm_drone();

    return 0;
}
