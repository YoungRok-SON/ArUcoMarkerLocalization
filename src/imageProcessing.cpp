
#include "imageProcessing.hpp"


imageProcessing::imageProcessing( cv::Mat in_camera_intrinsic, cv::Mat in_camera_distortion_coeff)
{
    m_mat_camera_intrinsic = in_camera_intrinsic.clone();
    m_mat_camera_distortion_coeff = in_camera_distortion_coeff.clone();
    m_i_marker_type =  cv::aruco::DICT_6X6_250;
    m_mat_image_size = cv::Size(640,480);

    if(init() == false)
    {
        std::cout << "[imageProcessing] Class has failed to initialize the node." << std::endl;
        return;
    }

}


imageProcessing::~imageProcessing()
{

}

bool imageProcessing::init()
{
    /* Member variables initialization*/
    if(m_mat_camera_intrinsic.empty() || m_mat_camera_distortion_coeff.empty())
    {
        std::cout << "[imageProcessing][init] Cameara intrinsic or distortion coeff doesn't exist." << std::endl;
        return false;
    }

    cv::initUndistortRectifyMap(m_mat_camera_intrinsic, m_mat_camera_distortion_coeff, cv::Mat(),
                                m_mat_camera_intrinsic, m_mat_image_size, CV_32FC1,
                                m_mat_remap_1, m_mat_remap_2);

    return true;
}


// 이 함수를 남겨놓는 이유가 있나? --> ToDo 나중에 처리할게 많아지면 다른 함수로 이름을 바꿔야 할듯?
bool imageProcessing::stretchImage(cv::Mat in_mat_distorted_img,
                                   cv::Mat &out_mat_flated_img)
{
    if(in_mat_distorted_img.empty())
    {
        std::cout << "[imageProcessing][stretchImage] Input image is empty!" << std::endl; 
        return false;
    }

    // initUndistortRectifyMap를 매번 불러울 이유가 없음
    cv::remap(in_mat_distorted_img, out_mat_flated_img, m_mat_remap_1,m_mat_remap_2, cv::INTER_LINEAR);

    return true;
}


bool imageProcessing::getCameraPose(cv::Mat in_mat_cam_img,
                                    std::vector< double > &out_vecd_pose)
{
    // Aruco marker detection
    std::vector<int> ids;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(m_i_marker_type);
    std::vector<std::vector<cv::Point2f> > corners;

    cv::Mat mat_original_img;
    in_mat_cam_img.copyTo(mat_original_img);
    cv::aruco::detectMarkers(mat_original_img, dictionary, corners, ids);

    // Draw Detected marker.
    // if(ids.size() > 0)
    // {
    //     cv::aruco::drawDetectedMarkers(mat_original_img,corners,ids);
    // }


    // Marker Pose estimation
    std::vector<cv::Vec3d> out_vvec3d_rot_vecs, out_vvec3d_trans_vecs;
    cv::aruco::estimatePoseSingleMarkers (corners, 0.19, m_mat_camera_intrinsic, g_mat_cam_distortion_coeff, out_vvec3d_rot_vecs, out_vvec3d_trans_vecs);

    in_mat_cam_img.copyTo(m_mat_detected_marker_img);
    for (int idx = 0; idx < out_vvec3d_rot_vecs.size(); ++idx)
    {
        auto rvec = out_vvec3d_rot_vecs[idx];
        auto tvec = out_vvec3d_trans_vecs[idx];
        cv::aruco::drawAxis (m_mat_detected_marker_img, m_mat_camera_intrinsic, g_mat_cam_distortion_coeff, rvec, tvec, 0.1);
        
        double d_pose_roll = 0;
        double d_pose_pitch = 0;
        double d_pose_yaw = 0;
        double d_pose_x = 0; // Front(x)
        double d_pose_y = 0; // Right(y)
        double d_pose_z = 0; // Down(z)
        
        // Change coordinate from detection to quad body coordinate.
        d_pose_roll = -rad2deg(rvec[1]);

        if(rad2deg(rvec[0]) > 0)
            d_pose_pitch = 180 - rad2deg(rvec[0]) ;
        else if(rad2deg(rvec[0]) < 0)
            d_pose_pitch = -180 - rad2deg(rvec[0]);
        else 
            d_pose_pitch = 0.;
            
        d_pose_yaw = rad2deg(rvec[2]);
        
        d_pose_x = - tvec[2];
        d_pose_y = - tvec[0];
        d_pose_z = - tvec[1];
        out_vecd_pose.push_back(d_pose_x);
        out_vecd_pose.push_back(d_pose_y);
        out_vecd_pose.push_back(d_pose_z);
        out_vecd_pose.push_back(rvec[1]);
        out_vecd_pose.push_back(rvec[0]);
        out_vecd_pose.push_back(rvec[2]);

        std::cout << "Marker id: " << idx << std::endl;
        std::cout << "Marker rotation vector_ R: " << d_pose_roll << " P: " << d_pose_pitch << " Y: " << d_pose_yaw << std::endl;
        std::cout << "Marker translation vector_ F: " << d_pose_x << " R: " << d_pose_y << " D: "<< d_pose_z << std::endl;
    }

}
