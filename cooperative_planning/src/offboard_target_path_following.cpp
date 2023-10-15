#include <drone_gimmicks_library/DroneGimmicks.h>
#include <ros/ros.h>
#include <drone_utils_cpp/DroneInfo.h>
#include <mavros_cpp/UAV.h>
#include <drone_utils_cpp/Utils.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int64.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Dense>

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





//Path following controller variables
double gamma_limit = M_PI/6;
double phi_limit = M_PI/6;
double kphi= 1;

double k1_line = 1;
double k2_line = 10;
double k1_orb = 1;
double k2_orb = 10;

//dubins path

//segment line 1
double c0_line1[3] = {5, -100, -20};
double psi_l_line1 = 0;
double gamma_l_line1 = 0;

//segment orb 2
double rh_orb2 = 22.5;
double lambda_orb2 = 1;
double gamma_h_orb2 = 0;
double psi_h_orb2 = 3*M_PI/2;
double ch_orb2[3] = {27.5, 100, -20};

//segment line 3
double c0_line3[3] = {50, 100, -20};
double psi_l_line3 = M_PI;
double gamma_l_line3 = 0;

//segment orb 4
double rh_orb4 = 22.5;
double lambda_orb4 = 1;
double gamma_h_orb4 = 0;
double psi_h_orb4 = 3*M_PI/2;
double ch_orb4[3] = {27.5, -100, -20};


int pathsType[4] = {0,1,0,1};
double paths_c0[4][3] = { {5, -100, -20}/*c0_line1*/,
                          {27.5, 100, -20}/*ch_orb2*/,
                          {50, 100, -20}/*c0_line3*/,
                          {27.5, -100, -20}/*ch_orb4*/};


double _psi_l[4] = {psi_l_line1,0,psi_l_line3,0};
double _gamma_l[4] = {gamma_l_line1,0,gamma_l_line3,0};

double _rh_h[4] = {0,rh_orb2,0,rh_orb4};
double _lambda_h[4] = {0,lambda_orb2,0,lambda_orb4};
double _gamma_h[4] = {0,gamma_h_orb2,0,gamma_h_orb4};
double _psi_h[4] = {0,psi_h_orb2,0,psi_h_orb4};


double u[3] = {0,0,0};
double psidPrevious = 0;


//Target Path Parameters
//---------------------------------------------------
double proximity_to_next_path = 10;
int _path_segment = 0;
double p0[3];
double psi0;


//---------------------------------------------------




void calculateDesiredHeading(){
   desiredHeading = kp_heading*((atan2(desiredVelocityLinear.y , desiredVelocityLinear.x ) - M_PI/2) - shuttle->ekf.ang_vel[2][0]);

}

void calculateDesiredVelocityLinear(){
    /*desiredVelocityLinear.x = kp_velocity*(desiredPosition.x  - shuttle->ekf.pos[0][0]);
    desiredVelocityLinear.y = kp_velocity*(desiredPosition.y  - shuttle->ekf.pos[1][0]);
    desiredVelocityLinear.z = kp_velocity*(desiredPosition.z  - shuttle->ekf.pos[2][0]);*/


    //REFAZER ESte COntrolador, etsa mal
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


void sendAttitudeSetPoint(ros::Publisher pub, double att_euler[3][1]){
	std_msgs::Header h;
	mavros_msgs::AttitudeTarget msg;
	geometry_msgs::Quaternion q_msg;

	h.stamp = ros::Time::now();
	msg.header = h;

    //att_euler[0][0]=desiredAttitude.x; att_euler[1][0]=desiredAttitude.y; att_euler[2][0]=desiredAttitude.z;

	double att_enu[3][1];
	DroneLib::ned_to_enu(att_euler, att_enu);
	tf2::Quaternion q_tf; q_tf.setRPY(att_enu[0][0],att_enu[1][0],att_enu[2][0]); q_msg = tf2::toMsg(q_tf);
	
	
    msg.thrust = 9;
	msg.orientation = q_msg;


    msg.type_mask = 1 | 2 | 4 ;
	pub.publish(msg);
    
         
    
}     


double saturation(double angle, double angle_limit){
    if(angle >= angle_limit)
        return angle_limit;
    else{
        if(angle <= -angle_limit)
            return -angle_limit;
        else return angle;
    } 
}


void pathFollowingController(ros::Publisher pub){
	// gammac, phic, psid - v, psi, psiD1

    double v = sqrt( pow(target->ekf.vel[0][0], 2) + pow(target->ekf.vel[1][0], 2) + pow(target->ekf.vel[2][0], 2) );
   
    double gammac = -saturation(asin(u[2]/v), gamma_limit);
   
   
    double psid = atan2(u[1],u[0]);
    while( abs(psid - psidPrevious) > (3*M_PI/2)){
        if( (psid - psidPrevious) > (3*M_PI/2) )
            psid = psid - 2*M_PI;
        else if( (psid - psidPrevious) > -(3*M_PI/2) )  
                psid = psid + 2*M_PI;        
    }

    double phic = -saturation(kphi*( psid - target->ekf.att_euler[2][0]), phi_limit);
                
        
    
	double att_enu[3][1];
    att_enu[0][0]=phic; att_enu[1][0]=gammac; 

    sendAttitudeSetPoint(pub, att_enu);
         
    
}     



void complexPathGenerator(int path_type, int path_segment){
    double v = sqrt( pow(target->ekf.vel[0][0], 2) + pow(target->ekf.vel[1][0], 2) + pow(target->ekf.vel[2][0], 2) );
	
    if(path_type == 0){
        double psi_l = _psi_l[path_segment];
        double gamma_l = _gamma_l[path_segment];

        /*double cl[3];
        cl[0] =  paths_c0[path_segment][0];
        cl[1] =  paths_c0[path_segment][1];
        cl[2] =  paths_c0[path_segment][2];

        double r[3] = {target->ekf.pos[0][0], target->ekf.pos[1][0], target->ekf.pos[2][0]};

        double n_lon[3] = {-sin(psi_l), cos(psi_l), 0};

        double n_lat[3] = {-cos(psi_l)*sin(gamma_l) , -sin(psi_l)*sin(gamma_l) , -cos(gamma_l)};*/


        /*Eigen::MatrixXd cl(3,1);
		 cl(0,0) = paths_c0[path_segment][0];	
		 cl(1,0) = paths_c0[path_segment][1];
		 cl(2,0) = paths_c0[path_segment][2];

        Eigen::MatrixXd r(3,1);
		 r(0,0) = target->ekf.pos[0][0];	
		 r(1,0) = target->ekf.pos[1][0];	
		 r(2,0) = target->ekf.pos[2][0];	

        Eigen::MatrixXd n_lon(3,1);
		 n_lon(0,0) = -sin(psi_l);	
		 n_lon(1,0) = cos(psi_l);	
		 n_lon(2,0) = 0;

        Eigen::MatrixXd n_lat(3,1);
		 n_lat(0,0) = -cos(psi_l)*sin(gamma_l) ;	
		 n_lat(1,0) = -sin(psi_l)*sin(gamma_l);	
		 n_lat(2,0) = -cos(gamma_l);



        Eigen::MatrixXd u_line(3,1);
        u_line = -k1_line*(n_lon*(n_lon.transpose()) + n_lat*(n_lat.transpose()))*(r - cl) - k2_line*n_lon.cross(n_lat);

        Eigen::MatrixXd u_normalized(3,1);
        u_normalized = v*(u_line/(u_line.norm()));

        u[0] = u_normalized(0,0);
        u[1] = u_normalized(1,0);
        u[2] = u_normalized(2,0);*/


        Eigen::Vector3d cl;
		 cl(0) = paths_c0[path_segment][0];	
		 cl(1) = paths_c0[path_segment][1];
		 cl(2) = paths_c0[path_segment][2];
        cl = cl.transpose();

        Eigen::Vector3d r;
		 r(0) = target->ekf.pos[0][0];	
		 r(1) = target->ekf.pos[1][0];	
		 r(2) = target->ekf.pos[2][0];	
        r = r.transpose();

        Eigen::Vector3d n_lon;
		 n_lon(0) = -sin(psi_l);	
		 n_lon(1) = cos(psi_l);	
		 n_lon(2) = 0;
        n_lon = n_lon.transpose();

        Eigen::Vector3d n_lat;
		 n_lat(0) = -cos(psi_l)*sin(gamma_l) ;	
		 n_lat(1) = -sin(psi_l)*sin(gamma_l);	
		 n_lat(2) = -cos(gamma_l);
        n_lat = n_lat.transpose();


        Eigen::Vector3d u_line;
        u_line = -k1_line*(n_lon*(n_lon.transpose()) + n_lat*(n_lat.transpose()))*(r - cl) - k2_line*n_lon.cross(n_lat);

        Eigen::Vector3d u_normalized;
        u_normalized = v*(u_line/(u_line.norm()));
        
        u[0] = u_normalized(0);
        u[1] = u_normalized(1);
        u[2] = u_normalized(2);
    }   

    else{
        double rh_h = _rh_h[path_segment];
        double lambda_h = _lambda_h[path_segment];
        double gamma_h = _gamma_h[path_segment];
        double psi_h = _psi_h[path_segment];


        /*Eigen::MatrixXd ch(3,1);
		 ch(0,0) = paths_c0[path_segment][0];	
		 ch(1,0) = paths_c0[path_segment][1];
		 ch(2,0) = paths_c0[path_segment][2];

        Eigen::MatrixXd r(3,1);
		 r(0,0) = target->ekf.pos[0][0];	
		 r(1,0) = target->ekf.pos[1][0];	
		 r(2,0) = target->ekf.pos[2][0];

        Eigen::MatrixXd dalpha_cyl(3,1);
		 dalpha_cyl(0,0) = 2*(r(0,0) - ch(0,0))/rh_h;	
		 dalpha_cyl(1,0) = 2*(r(1,0) - ch(1,0))/rh_h;	
		 dalpha_cyl(2,0) = 0;

        Eigen::MatrixXd dalpha_pl(3,1);
		 dalpha_pl(0,0) = (tan(gamma_h)/lambda_h)*(-(r(1,0) - ch(1,0)))/( pow((r(0,0) - ch(0,0)),2) + pow((r(1,0) - ch(1,0)),2));	
		 dalpha_pl(1,0) = (tan(gamma_h)/lambda_h)*(-(r(0,0) - ch(0,0)))/( pow((r(0,0) - ch(0,0)),2) + pow((r(1,0) - ch(1,0)),2));		
		 dalpha_pl(2,0) = 1/rh_h;

        double alpha_cyl =  pow(((r(0,0) - ch(0,0))/rh_h),2) + pow(((r(1,0) - ch(1,0))/rh_h),2) - 1;
        double alpha_pl =  pow(((r(2,0) - ch(2,0))/rh_h),2) + (tan(gamma_h)/lambda_h)*((atan((r(1,0) - ch(1,0))/(r(0,0) - ch(0,0)))) - psi_h) ;


        Eigen::MatrixXd u_line(3,1);
        u_line = -k1_orb*(-alpha_cyl*dalpha_cyl + alpha_pl*dalpha_pl) - lambda_h*k2_orb*( dalpha_cyl.cross(dalpha_pl) );//rever isto com o matalab

        Eigen::MatrixXd u_normalized(3,1);
        u_normalized = v*(u_line/(u_line.norm()));

        u[0] = u_normalized(0,0);
        u[1] = u_normalized(1,0);
        u[2] = u_normalized(2,0);*/

        Eigen::Vector3d ch;
		 ch(0) = paths_c0[path_segment][0];	
		 ch(1) = paths_c0[path_segment][1];
		 ch(2) = paths_c0[path_segment][2];
        ch = ch.transpose();

        Eigen::Vector3d r;
		 r(0) = target->ekf.pos[0][0];	
		 r(1) = target->ekf.pos[1][0];	
		 r(2) = target->ekf.pos[2][0];
        r = r.transpose();

        Eigen::Vector3d dalpha_cyl;
		 dalpha_cyl(0) = 2*(r(0,0) - ch(0,0))/rh_h;	
		 dalpha_cyl(1) = 2*(r(1,0) - ch(1,0))/rh_h;	
		 dalpha_cyl(2) = 0;
        dalpha_cyl = dalpha_cyl.transpose();

        Eigen::Vector3d dalpha_pl;
		 dalpha_pl(0) = (tan(gamma_h)/lambda_h)*(-(r(1,0) - ch(1,0)))/( pow((r(0,0) - ch(0,0)),2) + pow((r(1,0) - ch(1,0)),2));	
		 dalpha_pl(1) = (tan(gamma_h)/lambda_h)*(-(r(0,0) - ch(0,0)))/( pow((r(0,0) - ch(0,0)),2) + pow((r(1,0) - ch(1,0)),2));		
		 dalpha_pl(2) = 1/rh_h;
        dalpha_pl = dalpha_pl.transpose();

        double alpha_cyl =  pow(((r(0,0) - ch(0,0))/rh_h),2) + pow(((r(1,0) - ch(1,0))/rh_h),2) - 1;
        double alpha_pl =  pow(((r(2,0) - ch(2,0))/rh_h),2) + (tan(gamma_h)/lambda_h)*((atan((r(1,0) - ch(1,0))/(r(0,0) - ch(0,0)))) - psi_h) ;


        Eigen::Vector3d u_line;
        u_line = -k1_orb*(-alpha_cyl*dalpha_cyl + alpha_pl*dalpha_pl) - lambda_h*k2_orb*( dalpha_cyl.cross(dalpha_pl) );//rever isto com o matalab

        Eigen::Vector3d u_normalized;
        u_normalized = v*(u_line/(u_line.norm()));

        u[0] = u_normalized(0);
        u[1] = u_normalized(1);
        u[2] = u_normalized(2);
    }
    
}     



void complexPathManager(){
   int number_of_paths =  sizeof(pathsType);

   if(_path_segment != number_of_paths){
 
        double c_n = paths_c0[_path_segment + 1][0];	
	    double c_e = paths_c0[_path_segment + 1][1];
	    double c_d = paths_c0[_path_segment + 1][2];

        int path_type_next = pathsType[_path_segment + 1];
        double distance_to_next_path = sqrt( pow( (c_n -target->ekf.pos[0][0]) , 2) + pow( (c_e -target->ekf.pos[1][0]) , 2) + pow( (c_d -target->ekf.pos[2][0]) , 2)  );

        if(path_type_next == 0){
            if(distance_to_next_path < proximity_to_next_path)
                _path_segment++;
        }
        else{
            if(distance_to_next_path < (proximity_to_next_path + _rh_h[_path_segment + 1]))
                _path_segment++;
        }
   }

   int path_type = pathsType[_path_segment];
   complexPathGenerator(path_type, _path_segment);
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

    //sendTakeOffCommand(pos_target_pub); //nao esta a funcioanr
 
    //target->set_pos_yaw(pos, yaw, 10);

    
    //VERIFICAR REFEENRTIAL RELATIVO DOS DOIS DRONES


    //target initial conditions
    p0[0] = target->ekf.pos[0][0]; p0[1] = target->ekf.pos[1][0]; p0[2] = target->ekf.pos[2][0];
    psi0 = target->ekf.att_euler[2][0];
   
    while(ros::ok()){
      


        //ROS_WARN_STREAM("Current Position" << shuttle->sen.gps.pos[0][0]  << "  " << shuttle->sen.gps.pos[1][0]  << "  " << shuttle->sen.gps.pos[2][0]);
        ROS_WARN_STREAM("Current Position: " << target->ekf.pos[0][0]  << "  " << target->ekf.pos[1][0]  << "  " << target->ekf.pos[2][0]);
        ROS_WARN_STREAM("Desired Position: " << desiredPosition.x  << "  " << desiredPosition.y  << "  " << desiredPosition.z);



        //desiredPosReached(reached_pos_pub);


        //calculateDesiredVelocityLinear();

        //pos[0][0]=desiredPosition.x; pos[1][0]=desiredPosition.y; pos[2][0]=desiredPosition.z;
        //target->set_pos_yaw(pos, yaw, 0.01);


        //sendAttitudeSetPoint(set_att_pub);
        complexPathManager();
        pathFollowingController(set_att_pub);




        ros::spinOnce();
        rate.sleep();
    }

    return 0;





















}
