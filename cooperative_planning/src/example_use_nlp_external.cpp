#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <ros/package.h>

using namespace casadi;








int main(int argc, char **argv){

     ros::init(argc, argv, "example_use_nlp_external");
    ros::NodeHandle nh;

     // file name
    std::string file_name = "gen";
     // code predix
    std::string prefix_code = ros::package::getPath("cooperative_planning") + "/include/";
    // shared library prefix
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";


    // Create a new NLP solver instance from the compiled code
    std::string lib_name = prefix_lib + file_name + ".so";

      std::cout << "---" << std::endl;
        std::cout << "Usage from CasADi C++:" << std::endl;
        std::cout << std::endl;
        
       // std::vector<double> x0 = {  20, 10, -20, 0,0,0, 0,     0,0,-15, 11,0,0,0 };
        std::vector<double> x0 = {  777, 20, -10, 5,0,0, 5.5,     1055,20,-10, 11,0,0,5.5 };
        // Use CasADi's "external" to load the compiled function

        //casadi::Importer C = Importer('gen.c','shell');
        Function f = external("F",lib_name);

        // Use like any other CasADi function
        std::map<std::string, casadi::MX> arg;//, res;
        //std::vector<MX> arg = {reshape(DM(x), 2, 2), 5};
        arg["xx0"] = 0;

        ros::Time start_time = ros::Time::now();

        
        DM xx = DM::zeros(14,26);
        DM uu = DM::zeros(4,25);

     
        std::vector<DM> arg1 ={DM(x0), xx, uu};


        //std::vector<DM> res = f(DM(x0));

     
        std::vector<DM> res = f(arg1);

        std::cout << "lalala      " << res.at(0).size() << std::endl;
        std::cout << "lalala      " << res.at(1).size() << std::endl;
       
        casadi::Matrix<double> result_vector = res.at(0);
      std::cout << "leleleel      " << (double)result_vector(0,0) << std::endl;


        ros::Duration delta_t = ros::Time::now() - start_time;
        double delta_t_sec = delta_t.toSec();
        /*std::cout << "MPC computation time:  " << delta_t_sec << std::endl;


        std::cout << "result uu: " << res.at(0) << std::endl;

        casadi::Matrix<double> result_vector = res.at(0);
        std::cout << "result first element: " << result_vector(0,0) << std::endl;

        std::cout << "result xx: " << res.at(1) << std::endl;

        casadi::Matrix<double> result_vector_xx = res.at(1);
        std::cout << "result first element: " << result_vector(0,0) << std::endl;*/

    return 0;
}