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

void loff::sayhello(){
  printf("\033[2J\033[1;1H");
  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|               lidar_opticalflow_fusion Odometry                   |"
            << std::endl;
  std::cout << "| Input parameter :                                                        | "<<std::endl;
  std::cout << "| The threshold of recovery test (Num) : "
            <<std::left <<std::setfill(' ')<<std::setw(30)<<this->param_correctTest_set << "         |"
            <<std::endl;
  std::cout << "| The threshold of recovery signal flag (Num of points): "
            <<std::left <<std::setfill(' ')<<std::setw(30)<<this->param_frontpointNum_threshold << "         |"
            <<std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;
}

loff::loff(ros::NodeHandle node_handle) : nh(node_handle){

    this->opticalFlow_sub = this->nh.subscribe("/usb_opticalflow",1,&loff::callback_opticalFlow,this,ros::TransportHints().tcpNoDelay());

    this->mavros_state_sub = this->nh.subscribe("/mavros/state",1,&loff::callback_mavros_state,this,ros::TransportHints().tcpNoDelay());

    this->lidarPose_sub=this->nh.subscribe("/robot/dlio/odom_node/pose",1,&loff::callback_lidar,this,ros::TransportHints().tcpNoDelay());
    // this->lidarPose_sub=this->nh.subscribe("/mavros/vision_pose/pose",1,&loff::callback_lidar,this,ros::TransportHints().tcpNoDelay());
    this->correct_flag_sub = this->nh.subscribe("/robot/dlio_odom/pose_correct",1,&loff::callback_correct_flag,this,ros::TransportHints().tcpNoDelay());
    this->rc_correct_flag_sub = this->nh.subscribe("/loff/rc_pose_correct",1,&loff::callback_rc_correct_flag,this,ros::TransportHints().tcpNoDelay());
    this->pointNum_sub = this->nh.subscribe("/path_plan/recover_size",1,&loff::callback_pointNum,this,ros::TransportHints().tcpNoDelay());
    
    
    this->visionPose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1,true);
    this->crash_pub = this->nh.advertise<std_msgs::Bool>("Lidar_good",1,true);
    this->path_pub = this->nh.advertise<nav_msgs::Path>("/loff/path",1,true);

    ros::param::param("~loff/num_lidarReconnectTest",this->param_correctTest_set,2);
    ros::param::param("~loff/num_lidarFrontPointsThresh",this->param_frontpointNum_threshold,10);
}

void loff::callback_lidar(const geometry_msgs::PoseStamped::ConstPtr& lidarPose){ 
    this->yaw_ = extractYaw(lidarPose,this->yaw_);
    Eigen::Quaternionf yaw_q{cos(this->yaw_/2),0,0,sin(this->yaw_/2)};      // UAV's yaw (Quaternion)
    yaw_q.normalize();
    this->pose_cur=transformPose(lidarPose);
    this->pose_cur.p = this->pose_cur.p +  this->pose_corr;
    // Eigen::Quaternionf orientation
    if(this->lidar_correct_rc_ && this->lidar_correct_ ){
        if(this->num_correctTest_>=this->param_correctTest_set){
            // Only use lidar pose
            this->visionPose_pub.publish(transformPose(this->pose_cur));
            this->fusion_mode_ = using_Lidar;
            // this->pose_prv = this->pose_cur;
            this->pose_last_correct = this->pose_cur;

            this->lidar_recovered_.data = true;
            this->crash_pub.publish(this->lidar_recovered_);
            this->firs_OF_ = true;
        }else{
            this->visionPose_pub.publish( poseAddOpticalFlow(yaw_q));
            this->fusion_mode_ = using_OpticalFlow_reconnecting;
            // this->pose_prv = this->pose_cur;

            if(this->num_frontpoint_>=this->param_frontpointNum_threshold){
                this->num_correctTest_++;                                            ///////////////////////
            }
            if(num_correctTest_==param_correctTest_set){
                this->pose_corr = this->pose_cur.p - transformPose(lidarPose).p ;
            }
        }
        
    }else{
        this->num_correctTest_=0;
        this->visionPose_pub.publish( poseAddOpticalFlow(yaw_q));
        this->fusion_mode_ = using_OpticalFlow;
        // this->pose_prv = this->pose_cur;
    }
    this->debug_thread = std::thread(&loff::debug,this);
    debug_thread.detach();
}

/// @brief Extract yaw from the pose, without roll and pitch
/// @param poseStamped::ConstPtr and reference yaw（because Euler's angle is not unique）
/// @return yaw
float loff::extractYaw(const geometry_msgs::PoseStamped::ConstPtr& pose,const float& yaw_reference){
    Eigen::Quaternionf orientation(pose->pose.orientation.w,pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z);
    float yaw=extractYaw(orientation,yaw_reference);

    return yaw;
}
/// @brief Extract yaw from the pose, without roll and pitch
/// @param orientation (Quaternionf ) 和 reference yaw（because Euler's angle is not unique）
/// @return yaw
float loff::extractYaw(const Eigen::Quaternionf& orientation,const float&yaw_reference){
    float yaw=orientation.toRotationMatrix().eulerAngles(0,1,2)[2];
    if( cos(  yaw-yaw_reference )< 0.5){ 
        yaw=yaw+ M_PI;
    }
    return yaw;
}

void loff::callback_correct_flag(const std_msgs::Bool::ConstPtr& lidar_correct){
    this->lidar_correct_=lidar_correct->data;
    if(!this->lidar_correct_){
        this->lidar_recovered_.data = false;
        this->crash_pub.publish(this->lidar_recovered_);

    }
}

void loff::callback_rc_correct_flag(const std_msgs::Bool::ConstPtr& rc_lidar_correct){
    this->lidar_correct_rc_=rc_lidar_correct->data;
    if(!this->lidar_correct_rc_){
        this->lidar_recovered_.data = false;
        this->crash_pub.publish(this->lidar_recovered_);

    }
}

void loff::callback_mavros_state(const mavros_msgs::State::ConstPtr& state){
    this->uav_state_ = *state;
}


void loff::callback_opticalFlow(const geometry_msgs::Vector3::ConstPtr& pose){
    if(!this->lidar_recovered_.data ){
        if(this->firs_OF_){
            this->pose_opticalFlow_yawFrame_first<<pose->x,pose->y,pose->z;
            this->pose_opticalFlow_yawFrame_now = this->pose_opticalFlow_yawFrame_first;
            this->opticalflow_integration = 0;
            this->firs_OF_ = false;
        }else{
            this->pose_opticalFlow_yawFrame_now<<pose->x,pose->y,pose->z;
        }
    }
}

void loff::callback_pointNum(const std_msgs::Int64::ConstPtr& pointNum){
    this->num_frontpoint_ = pointNum->data;
}

/// @brief LiDAR SLAM + OF integration（in yaw frame）
/// @param yaw(Quaternionf) 
/// @return Pose (geometry_msgs)
geometry_msgs::PoseStamped loff::poseAddOpticalFlow(const Eigen::Quaternionf& yaw){
    yawDifferenceDetect(this->yaw_);
    Eigen::Vector3f pose_yawFrame = pose_last_correct.q.inverse() * this->pose_cur.p;
    // using opticalFlow_device
    Eigen::Vector3f delta_opticalFlow (this->pose_opticalFlow_yawFrame_now.x()-this->pose_opticalFlow_yawFrame_first.x(),0,0);
    this->opticalflow_integration += (pose_last_correct.q.inverse()*yaw*delta_opticalFlow)[0];
    pose_yawFrame[0] = this->pose_last_correct.p[0]+this->opticalflow_integration;
    this->pose_opticalFlow_yawFrame_first = this->pose_opticalFlow_yawFrame_now;
    this->pose_cur.p = pose_last_correct.q * pose_yawFrame;
    return transformPose(this->pose_cur);
}                                                     

// Detect angle difference 
bool loff::yawDifferenceDetect(const float& yaw_cur){
    float yaw_last_correct = extractYaw(this->pose_last_correct.q,yaw_cur);
    if( cos(yaw_last_correct-yaw_cur) > cos(20 * M_PI / 180) ){
        return true;
    }else{ ROS_ERROR("Yaw difference is tooooooooooooo large! (%f degree)",(yaw_last_correct-yaw_cur)*180/M_PI );return false;}
}

// geo_msgs -> loff
loff::PoseStamped loff::transformPose(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs){
    loff::PoseStamped loff_msgs;
    loff_msgs.p<<geo_msgs->pose.position.x,geo_msgs->pose.position.y,geo_msgs->pose.position.z;
    loff_msgs.q=Eigen::Quaternionf(geo_msgs->pose.orientation.w,geo_msgs->pose.orientation.x
                ,geo_msgs->pose.orientation.y,geo_msgs->pose.orientation.z);
    loff_msgs.header = geo_msgs->header;
    return loff_msgs;
}

// loff -> geo_msgs
geometry_msgs::PoseStamped loff::transformPose(const loff::PoseStamped& loff_msgs){
    geometry_msgs::PoseStamped geo_msgs;
    geo_msgs.header = loff_msgs.header;
    geo_msgs.pose.position.x=loff_msgs.p.x();
    geo_msgs.pose.position.y=loff_msgs.p.y();
    geo_msgs.pose.position.z=loff_msgs.p.z();
    geo_msgs.pose.orientation.w=loff_msgs.q.w();    
    geo_msgs.pose.orientation.x=loff_msgs.q.x();
    geo_msgs.pose.orientation.y=loff_msgs.q.y();
    geo_msgs.pose.orientation.z=loff_msgs.q.z();
    return geo_msgs;
}
std::string loff::q2str(const Eigen::Quaternionf& q){
    int width = 8;
    std::ostringstream oss;
    oss.precision(5);
    oss <<" ["<<std::setw(width) <<std::fixed << q.w() << std::setw(width) << q.x() << std::setw(width) << q.y() << std::setw(width) << q.z()<<" ] ";
    return oss.str();
}

// Status message output
void loff::debug(){
    if(this->mtx_print.try_lock()){
        std::cout << std::endl
                << "+-------------------------------------------------------------------+" << std::endl;
        std::cout << "|               lidar_opticalflow_fusion Odometry                   |"
                << std::endl;
        std::cout <<"| LOFF's position: " <<std::endl;
            std::cout<<"|"  <<std::setw(2)<< this->pose_cur.p.transpose() <<q2str(pose_cur.q)<<std::endl;
        std::cout <<"| LOFF's yaw: " <<std::setprecision(2) <<std::fixed
                <<this->yaw_*180/M_PI<<"  degree"<<std::endl;
        std::cout <<"| Last LiDAR SLAM position: " <<std::endl;
            std::cout<<"|"  <<std::setw(2)<< this->pose_last_correct.p.transpose()  <<q2str(pose_last_correct.q)<<std::endl;
        std::cout <<"| optical flow odometry input: " << std::setw(2)
                <<this->pose_opticalFlow_yawFrame_now.transpose()
                <<" (Integration of OF: "<<this->opticalflow_integration<<" )"<<std::endl;
        std::cout <<"| Offset of position: "<<std::setw(2)<< this->pose_corr.transpose()<<std::endl;
        
        std::cout <<"| RC mode: connected-"<<std::boolalpha<<bool(uav_state_.connected)<<"  mode:   "<<uav_state_.mode<<std::endl;
        std::cout <<"| LOFF status: ";
        switch(this->fusion_mode_){
            case using_Lidar:
                std::cout << " using_Lidar ";
                break;
            case using_OpticalFlow:
                std::cout << " using OpticalFlow";
                break;
            case using_OpticalFlow_reconnecting:
                std::cout << " using OpticalFlow and reconnecting. The "<<this->num_correctTest_<<" th test(s) /"<<this->param_correctTest_set<<" tests";
                break;
            default:
                std::cout << "Unknown";
                break;
        }std::cout<<std::endl;
        std::cout << "|                                                                   |" << std::endl;
        std::cout << "+-------------------------------------------------------------------+" << std::endl;

        for(int i=0;i<5;i++){printf("\n");}
        this->mtx_print.unlock();
    }

    this->path_.header = this->pose_cur.header;
    this->path_.poses.push_back(transformPose(this->pose_cur));
    this->path_pub.publish(this->path_);
}  