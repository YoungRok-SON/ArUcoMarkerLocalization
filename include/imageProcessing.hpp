/* 
author      : YoungRok Son
Date        : 2023-07-04
Email       : dudfhr3349@gmail.com
Description : This header file is generated for image pre processing such 
              as applying intrinsic parms and detect an arUco marker.
*/

#include "utilities.hpp"
#include "opencv4/opencv2/aruco.hpp"

#ifndef __IMAGE_PROCESSING_HPP__
#define __IMAGE_PROCESSING_HPP__


class imageProcessing
{
private:
    /* data */
public:
    imageProcessing(cv::Mat in_camera_intrinsic, cv::Mat in_camera_distortion_coeff);
    ~imageProcessing();
    bool init();
public:
/* Functions */
    bool stretchImage(cv::Mat in_mat_distorted_img,
                      cv::Mat &out_mat_flated_img);
    bool getMarkerPose(cv::Mat in_mat_cam_img,
                       std::vector< cv::Vec3d > &out_vecvec3d_rot_vecs,
                       std::vector< cv::Vec3d > &out_vecvec3d_trans_vecs);

                        


public:
/* Member Variables */
    cv::Size m_mat_image_size;  // ToDo 나중에 로스로 만들면 이부분 자동으로 바뀌게 설정 필요
    cv::Mat m_mat_camera_intrinsic, m_mat_camera_distortion_coeff;
    cv::Mat m_mat_remap_1, m_mat_remap_2;
    int m_i_marker_type;

};

#endif