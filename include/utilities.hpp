<<<<<<< HEAD
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


#endif
=======
#include <iostream>
>>>>>>> ab00bfc0fa180fabb556fc74233d83c2183691c4
