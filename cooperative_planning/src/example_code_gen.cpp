#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <ros/package.h>



int main(){
   
    // file name
    std::string file_name = "gen";
     // code predix
    std::string prefix_code = ros::package::getPath("cooperative_planning") + "/include/";
    // shared library prefix
    std::string prefix_lib = ros::package::getPath("cooperative_planning") + "/include/";


   std::string path = ros::package::getPath("cooperative_planning");


    // compile c code to a shared library
    std::string compile_command = "gcc -fPIC -shared -O3 " + 
        prefix_code + file_name + ".cpp -o " +
        prefix_lib + file_name + ".so";

    std::cout << compile_command << std::endl;
    int compile_flag = std::system(compile_command.c_str());
    casadi_assert(compile_flag==0, "Compilation failed");
    std::cout << "Compilation successed!" << std::endl;

    return 0;
}