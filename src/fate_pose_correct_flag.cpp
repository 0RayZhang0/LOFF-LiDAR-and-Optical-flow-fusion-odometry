// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <std_msgs/Int64.h>
#include <mavros_msgs/RCIn.h>

std_msgs::Bool pose_correct_flag;
std_msgs::Int64 front_point;
bool msgOut;
bool count_or_rc;                   // Decide who is going to control the publisher of "pose_correct_flag". True for count, false for rc.
int count=0,count_vec=0;
std::vector<int> true_false;

// "fate" the healthy signal of LiDAR SLAM flag_Li
ros::Publisher  flag_pub;
//  "fate" the recovery signal flag_re
ros::Publisher  front_point_pub;
ros::Subscriber lidar_sub;
ros::Subscriber rc_sub;

void callback_lidar(const geometry_msgs::PoseStamped::ConstPtr& msg){
    count++;
    if(msgOut){ std::cout<<"count: "<<count<<std::endl;}
    front_point_pub.publish(front_point);

    if(count==true_false[2*count_vec]){
        pose_correct_flag.data = true;
        flag_pub.publish(pose_correct_flag);
        return;
    }
    if (count==true_false[2*count_vec+1])   
    {
        pose_correct_flag.data = false;
        flag_pub.publish(pose_correct_flag);
        if(count_vec<(true_false.size()/2)){count_vec++;}
        return;
    }
}

void callback_rc(const mavros_msgs::RCIn::ConstPtr& rc){
    // Channel number is determined by the RC
    if(rc->channels[7] > 1600){
        pose_correct_flag.data = false;
        flag_pub.publish(pose_correct_flag);
    }else{
        pose_correct_flag.data = true;
        flag_pub.publish(pose_correct_flag);
    }

    if(rc->channels[5]>1400){
        front_point_pub.publish(front_point);
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fate_pose");
    ros::NodeHandle nh;
    
    ros::param::param("~fate/true_false",true_false,std::vector<int>(5,100));
    ros::param::param("~fate/cout",msgOut,false);
    ros::param::param("~fate/count_rc",count_or_rc,false);
    std::cout<<" Parameter for ROS_bag, true_false [pose_correct_flag true / false]";
    for(auto i:true_false){std::cout<<" "<<i;}std::cout<<std::endl;
    std::cout<< true_false.size()<<std::endl;

    front_point.data = 300;

    flag_pub = nh.advertise<std_msgs::Bool>("/loff/rc_pose_correct", 1,true);
    front_point_pub = nh.advertise<std_msgs::Int64>("/path_plan/recover_size",1,true);     
    // lidar_sub = nh.subscribe("/robot/dlio/odom_node/pose",1,&callback_lidar,ros::TransportHints().tcpNoDelay());
 
    if(count_or_rc){
        ////////////////  using ROS bag 
        lidar_sub = nh.subscribe("/mavros/vision_pose/pose",1,&callback_lidar,ros::TransportHints().tcpNoDelay());
    }else{
        ////////////////  using RC (Remote control) button
        rc_sub = nh.subscribe("/mavros/rc/in",1,&callback_rc,ros::TransportHints().tcpNoDelay());
    }
    
    ros::spin();    
    return 0;
}