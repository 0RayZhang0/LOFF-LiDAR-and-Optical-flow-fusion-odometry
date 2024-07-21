/***********************************************************
 * LOFF: LiDAR and Optical flow fusion odometry            *
 *                                                         *
 * Our algorithm fuses LiDAR SLAM and optical flow odometry* 
 * to provide position estimation for VAVs                 *
 * in LiDAR degeneracy environments.                       *
 *                                                         *
 * The details are shown in our paper                      *
 * (写这个其实没什么用，就是觉得很帅就放在这里而已)                *
 *                                                         *
 *                                                         *
 * Authors: Ray.Zhang                                      *
 * Contact: @tongji.edu.cn                                 *
 ***********************************************************/

#include <loff.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "loff_node");
    ros::NodeHandle nh("~");

    loff node(nh);
    node.sayhello();
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}