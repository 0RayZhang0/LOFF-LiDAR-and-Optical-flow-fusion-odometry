// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
//  mavros
#include <mavros_msgs/State.h>

#include <memory>
#include <thread>
#include <mutex>


// Eigen
#include <Eigen/Eigen>


class loff {
public:
    loff(ros::NodeHandle node_handle);

    void sayhello();
    struct PoseStamped{
        Eigen::Vector3f p;
        Eigen::Quaternionf q;
        std_msgs::Header header;
    };
private:
    int num_threads_;
    ros::NodeHandle nh;

    // subscribers
    ros::Subscriber lidarPose_sub;
    ros::Subscriber opticalFlow_sub;        //subscriber for optical flow odometry
    ros::Subscriber correct_flag_sub;
    ros::Subscriber rc_correct_flag_sub;
    ros::Subscriber pointNum_sub;
    ros::Subscriber mavros_state_sub;
    // publishers
    ros::Publisher visionPose_pub;
    ros::Publisher crash_pub;
    ros::Publisher path_pub;

    bool lidar_correct_=true;
    bool lidar_correct_rc_=true;
    std_msgs::Bool lidar_recovered_;         // switching back to LiDAR SLAM
    int num_correctTest_=999;               //  Num of recovery test
    int num_frontpoint_=0;                 //   Num of point clouds in front of LiDAR
    int param_correctTest_set;
    int param_frontpointNum_threshold;

    PoseStamped pose_cur, pose_last_correct;
    // PoseStamped pose_prv;
    float yaw_=0;                                     // UAV's yaw
    bool firs_OF_ = true;                       //  First optical flow data
    float opticalflow_integration;
    Eigen::Vector3f pose_corr{0.,0.,0.};        //  The offset between LiDAR SLAM and optical flow odometry
    Eigen::Vector3f pose_opticalFlow_yawFrame_first{0.,0.,0.};
    Eigen::Vector3f pose_opticalFlow_yawFrame_now{0.,0.,0.};
    nav_msgs::Path path_;

    mavros_msgs::State uav_state_;


    enum Fusion_Mode_options{
        using_Lidar,
        using_OpticalFlow,
        using_OpticalFlow_reconnecting
    };Fusion_Mode_options fusion_mode_;

    //thread
    std::thread debug_thread;
    std::mutex mtx_print;

    void callback_opticalFlow(const geometry_msgs::Vector3::ConstPtr& pose);
    void callback_lidar(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void callback_correct_flag(const std_msgs::Bool::ConstPtr& lidar_correct);
    void callback_rc_correct_flag(const std_msgs::Bool::ConstPtr& rc_lidar_correct);
    void callback_pointNum(const std_msgs::Int64::ConstPtr& pointNum);
    void callback_mavros_state(const mavros_msgs::State::ConstPtr& state);

    geometry_msgs::PoseStamped poseAddOpticalFlow(const Eigen::Quaternionf& yaw);
    float extractYaw(const geometry_msgs::PoseStamped::ConstPtr& pose,const float& yaw_reference);
    float extractYaw(const Eigen::Quaternionf& orientation,const float&yaw_reference);
    bool yawDifferenceDetect(const float& yaw_cur);
    PoseStamped transformPose(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs);
    geometry_msgs::PoseStamped transformPose(const loff::PoseStamped& loff_msgs);
    std::string q2str(const Eigen::Quaternionf& q);
    void debug();
};