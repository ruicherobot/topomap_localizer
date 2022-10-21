/*******************************************************************************************
 * File:        maplite.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: Create maplite ROS node
*******************************************************************************************/
#include <maplite.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "maplite");
    ros::NodeHandle n;
    maplite::MapliteClass maplite(n);
    std::cout << "maplite localization working " << std::endl;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
