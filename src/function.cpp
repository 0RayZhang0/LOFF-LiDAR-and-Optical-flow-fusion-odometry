#include <ros/ros.h>
#include <pcl/common/centroid.h>

////////////////////////////////////////////////////////////////////////////
//  1
//  This function calculates the yaw angle based on the given point cloud.

float caluYaw(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_yaw, bool left_pointcloud) {
    if(point_cloud_yaw->size() <30){ROS_ERROR("Nothing here!");return 0;}
    pcl::PointCloud<pcl::Normal>::Ptr point_cloud_normal (boost::make_shared<pcl::PointCloud<pcl::Normal>>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nor;
    nor.setInputCloud(point_cloud_yaw);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    nor.setSearchMethod(tree);
    nor.setKSearch(15);
    // nor.setRadiusSearch(0.3);     //Doesn't work well in here.
    nor.compute(*point_cloud_normal);
    auto compareByCurvature = [](const pcl::Normal& a, const pcl::Normal& b) {
        return a.curvature < b.curvature;
    };
    std::sort(point_cloud_normal->begin(), point_cloud_normal->end(), compareByCurvature);

    Eigen::Vector3f vec{0, 0, 0};
    for(int i = 0; i< 30;i++) {
        vec.x() +=point_cloud_normal->points[i].normal_x;
        vec.y() +=point_cloud_normal->points[i].normal_y;
        vec.z() +=point_cloud_normal->points[i].normal_z;
    }
    vec.normalize();
    float yaw_modify_cos = vec.dot(Eigen::Vector3f(1,0,0));
    float yaw_modify = left_pointcloud ? ( -M_PI_2-(-acos(yaw_modify_cos) )) : ( M_PI_2-(acos(yaw_modify_cos)) );
    return yaw_modify;
}

////////////////////////////////////////////////////////////////////////////
//  2
//  The failure detector fuction is written with reference to the functions (dlio::OdomNode::integrateImu and dlio::OdomNode::propagateState) in DLIO.
//  See the code in DLIO for more details.
//  By combining with (7) in my paper, it should be easy to realize this function.






////////////////////////////////////////////////////////////////////////////
//  3
//  A simple example of the recover_detector
//  The LOFF will not switch back to using LiDAR until the recover_detector is triggered.

int recover_size = 100;  // Set the threshold according to the actual situation.
int recover_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){
    if(point_cloud->size() <10){ROS_ERROR("Nothing here!");return -1;}
    pcl::PointCloud<pcl::Normal>::Ptr point_cloud_normal (boost::make_shared<pcl::PointCloud<pcl::Normal>>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nor;
    nor.setInputCloud(point_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    nor.setSearchMethod(tree);
    nor.setKSearch(5);
    nor.compute(*point_cloud_normal);

    Eigen::Vector3f normal_eigen{0.,0.,0.};
    int count_nan = 0;
    int count_parallel = 0;
    for(auto normal_pcl : point_cloud_normal->points){
        if(std::isnan(normal_pcl.normal_x)){
            count_nan++;
            continue;
        }
        normal_eigen = normal_pcl.getNormalVector3fMap();

        // Counting the number of those vectors perpendicular to the degeneracy direction.
        if(abs(  Eigen::Vector3f(1.0 ,0. ,0.).dot(normal_pcl.getNormalVector3fMap())  ) > cos(30. * M_PI / 180.)){
            count_parallel++;
        }
    }

    return count_parallel;
}







////////////////////////////////////////////////////////////////////////////
//  4
//  A simple example of an optical flow odometer.
//  It is recommended to find a better optical flow odometer device 
//  or rewrite this code. 

cv::Mat pre_image;
void image_receive_callback(cv::Mat frame, Eigen::Vector3f& imu_acc, float time_dif, bool imu_flag){
    cv::Mat cur_image;
    cv::Point2f average = Point2f(0.0f, 0.0f);
    cv::Point2f pixel_sum(0.0f, 0.0f);
    cv::Point2f distance_dif(0.0f, 0.0f);
    cv::Point2f normalization_space(0.0f, 0.0f);
    cv::cvtColor(frame, cur_image, cv::COLOR_RGB2GRAY);
   // The key point of previous frame.
    cv::goodFeaturesToTrack(pre_image,p0,100,0.1,10,mask,7);     
    if (p0.size() == 0) {
        pre_image = cur_image;
        return;
    }
    std::vector<cv::Point2f>p1;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria criteria=cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS),10,0.03);
    cv::calcOpticalFlowPyrLK(pre_image,cur_image,p0,p1,status,err,cv::Size(15,15),2,criteria);

    std::vector<cv::Point2f> good_point;

    pre_image=cur_image;

    int count = 0;
    int bad_point_count = 0;
    for(int i=0;i<p0.size();i++){
        if(status[i]==1){
            good_point.push_back(p1[i]);
            float delt_x = (p1[i].x - p0[i].x);
            float delt_y =  (p1[i].y - p0[i].y);
            if (cv::norm(p1[i] - p0[i]) > (max_pixel_per_frame / lambda)) {
                continue;
            }
            if (abs(delt_x) < 1 && abs(delt_y) < 1) continue; //filter
            if (abs(delt_x > (max_pixel_per_frame / lambda)) || abs(delt_y > (max_pixel_per_frame / lambda))) {
                bad_point_count++;
                continue;
            }
            // std::cout << "delt_x =  " << delt_x << std::endl;
            pixel_sum.x +=  delt_x;
            pixel_sum.y += delt_y;
            count ++;
        }
    }
    float pixelx_lastframe = ((velocity.x * time_dif) * micrometer_to_meter) / lambda / f_x;
    // std::cout << "lambda = " << lambda << std::endl;
    switch (count)
    {
    case 0:
        ROS_ERROR("0 point ------------------");
        stop_img_count++;
        if (stop_img_count > stop_threshold) {
            stop_flag = true;
            velocity = Point2f(0.0f, 0.0f);
            last_velocity = velocity;
            average = Point2f(0.0f, 0.0f);
            std::cout << "!!!!!!!!!!!! UAV stop !!!!!!!!!!!!!!!!!!!!!!\n";
            std::cout << "stop_img_count: " << stop_img_count << std::endl;
        } else {
            average = last_average * 0.7 + Point2f(pixelx_lastframe, 0) * 0.3;
        }
        // std::cout << "average x: " << average.x << std::endl;
        last_average = average;
        break;
    case 1:
        ROS_ERROR("1 point ------------------");
        // average = Point2f(0.0f, 0.0f);
        // last_average = average;
        // // break;
        if(last_average.x == 0.0f) {
            average = cv::Point2f(0.0f, 0.0f);
        } else {
            average = last_average * 0.7 + Point2f(pixelx_lastframe, 0) * 0.3;
        }
        // std::cout << "average x: " << average.x << std::endl;
        last_average = average;
        stop_img_count = 0;
        stop_flag = false;
        break;
    default:
        average = pixel_sum / count;
        // std::cout << "average x: " << average.x << std::endl;
        last_average = average;
        stop_img_count = 0;
        stop_flag =false;
        break;
    }
    ////
    //   An average filter is needed to write by yourself.
    ////
    average.x = AverageFilter(average.x, filter_buffer_x, window_size_x);
    average.y = AverageFilter(average.y, filter_buffer_vel, window_size_x);
    //um,10e-6m
    normalization_space.x = average.x * f_x;
    normalization_space.y = average.y * f_y;
    //m
    distance_dif = (normalization_space * lambda) / micrometer_to_meter;
    if(abs(pitch) < 20) {

    }
    velocity.x = distance_dif.x / time_dif;
    velocity.x = AverageFilter(velocity.x, filter_buffer_vel, window_size_vel);
    pose += distance_dif;

    ////////////////
    //  kalman filter
    ////////////////
    if (imu_flag == true) { //kalman filter
        switch (count)
        {
        case 0 :
            if(stop_flag == true) { 
                std::cout << "-----------------uav stop---------------------\n";
                kf_x.X(1) = 0.0f; // Can not find any keypoint in current frame.
            } else { 
                kf_x.process(time_dif, imu_acc.x(), pose.x, velocity.x);
            }
            break;
        case 1 :
            kf_x.process(time_dif, imu_acc.x(), pose.x, velocity.x);
            break;
        default:
            kf_x.process(time_dif, imu_acc.x(), pose.x, velocity.x);
            break;
        }
    } else {
        ROS_ERROR("imu_flag is false");
    }


    printf("\033[2J\033[1;1H");
    std::cout << "opitcal_pose: " <<  pose.x << std::endl;
    std::cout << "velocity: " << velocity.x << std::endl;

    geometry_msgs::Vector3 optical_pose;
    optical_pose.x = pose.x;
    optical_pose_pub.publish(optical_pose);
}