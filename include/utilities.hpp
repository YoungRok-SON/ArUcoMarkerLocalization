/* 
author      : YoungRok Son
Date        : 2023-07-04
Email       : dudfhr3349@gmail.com
Description : This header file is set of utility functions.
*/
#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>


#include "ros/ros.h"
#include "image_transport/image_transport.h" // includes everything we need to publish and subscribe to images.
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

#include <cmath>

#ifndef __UTILITIES_HPP__
#define __UTILITIES_HPP__

/* Global variables */
inline std::string g_str_image_path = "/home/youngmoney/git_ws/bag_to_image/images/frame000012.png";

inline 	cv::Mat g_mat_cam_intrinsic        = (cv::Mat1d(3, 3) << 568.93159, 0.0 , 338.17156, 0.0, 568.75656, 212.69893, 0.0, 0.0, 1.0);
inline 	cv::Mat g_mat_cam_distortion_coeff = (cv::Mat1d(1, 5) << -0.402742, 0.163489, 0.001390, -0.004012, 0.000000);



/* Global Functions */
inline bool importImages(std::string in_img_path, cv::Mat &out_imoprted_img)
{
    out_imoprted_img = cv::imread(in_img_path);

    if(out_imoprted_img.empty())
    {
        std::cout << "[Utilites][importImages] Imported imag is empty!" << std::endl;
        return false;
    }
    
    return true;
}

inline double rad2deg(double in_value)
{
    return in_value * 180.0 / M_PI;
}

inline double euler2quaternion(std::vector<double> in_pose_vector,
                               tf2::Quaternion &out_rot_quaternion)
{
    tf2::Quaternion q_rotation_quaternion;
    q_rotation_quaternion.setRPY(in_pose_vector[3],in_pose_vector[4],in_pose_vector[5]);
    q_rotation_quaternion.normalize();
    out_rot_quaternion = q_rotation_quaternion;
}


#endif
