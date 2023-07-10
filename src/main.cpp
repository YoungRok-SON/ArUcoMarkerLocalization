/* 
author      : YoungRok Son
<<<<<<< HEAD
Date        : 2023-07-06
=======
Date        : 2023-07-01
>>>>>>> ab00bfc0fa180fabb556fc74233d83c2183691c4
Email       : dudfhr3349@gmail.com
Description : This pakcage is for testing indoor UAV localization using ArUco marker of opencv. 
*/


<<<<<<< HEAD
#include "utilities.hpp"
#include "imageProcessing.hpp"

#include "ros/ros.h"
#include <image_transport/image_transport.h> // includes everything we need to publish and subscribe to images.

#include <cv_bridge/cv_bridge.h>

// Do Image Processing whatever you want
imageProcessing IP(g_mat_cam_intrinsic, g_mat_cam_distortion_coeff);
cv::Mat g_mat_input_img;

void imgCallbackNode(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::imshow("Input View", cv_bridge::toCvShare(msg, "bgr8")->image);
        g_mat_input_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cv::waitKey(30);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
    // Get Pose of ArUco marker.
    std::vector< cv::Vec3d > vvec3d_rot_vecs, vvec3d_trans_vecs;
    IP.getMarkerPose(g_mat_input_img, vvec3d_rot_vecs, vvec3d_trans_vecs);

    // Calculate Pose of camera.


    // Publish Pose data to MAVROS.
}

int main(int argc, char **argv)
{
    std::cout << "main is running now..." << std::endl;
    
    // ROS Init
    ros::init(argc,argv, "ArucoMarkerLocalization");
    ros::NodeHandle nh;

    cv::namedWindow("Input View");
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("/usb_cam/image_raw",1,imgCallbackNode);
    ros::spin();
    cv::destroyAllWindows();


    std::cout << "main finished..." << std::endl;

=======
#include "../include/utilities.hpp"


int main()
{
    std::cout << "main is running..." << std::endl;
>>>>>>> ab00bfc0fa180fabb556fc74233d83c2183691c4
    return 1;
}