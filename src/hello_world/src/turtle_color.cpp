#include "ros/ros.h"


// rosparam list
// /turtlesim/background_b
// /turtlesim/background_g
// /turtlesim/background_r


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "change_color");
    // First Method
    // ros::param::set("/turtlesim/background_b", 0);
    // ros::param::set("/turtlesim/background_r", 0);
    // ros::param::set("/turtlesim/background_g", 0);

    // Second Method
    // The handler will put "turtlesim" and "background_r" together to form a full name of the param  
    ros::NodeHandle nh("turtlesim");
    nh.setParam("background_r",255);
    nh.setParam("background_g",255);
    nh.setParam("background_b",255);



    return 0;
}
