/* 
author      : YoungRok Son
Date        : 2023-07-06
Email       : dudfhr3349@gmail.com
Description : This pakcage is for testing indoor UAV localization using ArUco marker of opencv. 
*/


#include "utilities.hpp"
#include "imageProcessing.hpp"


// Do Image Processing whatever you want
imageProcessing IP(g_mat_cam_intrinsic, g_mat_cam_distortion_coeff);
cv::Mat g_mat_input_img;

image_transport::Subscriber sub_img;

ros::Publisher odom_pub;
image_transport::Publisher imgpub_detected_marker;

tf::StampedTransform transform;


void imgCallbackNode(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        g_mat_input_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
    // Get Pose of ArUco marker.
    std::vector< cv::Vec3d > vvec3d_rot_vecs, vvec3d_trans_vecs;
    std::vector<double> vecd_camera_pose;
    IP.getCameraPose(g_mat_input_img, vecd_camera_pose);

    // Transform Pose to quaternion and nav_msgs type.
    nav_msgs::Odometry navodom_camera_pose_msg;
    // navodom_camera_pose_msg.header.frame_id 나중에 px4에서 인식 안되면 이거 바꿔줘야할수도 있음.

    
    navodom_camera_pose_msg.header.stamp = msg->header.stamp;
    navodom_camera_pose_msg.header.seq = msg->header.seq;
    // navodom_camera_pose_msg.header.frame_id =  "base_link_frd";
    navodom_camera_pose_msg.header.frame_id =  "odom";
    // navodom_camera_pose_msg.child_frame_id = "odom";
    // navodom_camera_pose_msg.header.frame_id =  "camera_link";
    navodom_camera_pose_msg.child_frame_id = "base_link";

    if(vecd_camera_pose.size() == 6)
    {
        navodom_camera_pose_msg.pose.pose.position.x = vecd_camera_pose[0];
        navodom_camera_pose_msg.pose.pose.position.y = vecd_camera_pose[1];
        navodom_camera_pose_msg.pose.pose.position.z = vecd_camera_pose[2];
        tf2::Quaternion tfquart_orientation;
        euler2quaternion(vecd_camera_pose, tfquart_orientation);

        navodom_camera_pose_msg.pose.pose.orientation.x = tfquart_orientation.x();
        navodom_camera_pose_msg.pose.pose.orientation.y = tfquart_orientation.y();
        navodom_camera_pose_msg.pose.pose.orientation.z = tfquart_orientation.z();
        navodom_camera_pose_msg.pose.pose.orientation.w = tfquart_orientation.w();
        




        // Publish Pose data to MAVROS.
        odom_pub.publish(navodom_camera_pose_msg);
        // Transform mat to sensor_msgs format and publish.
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", IP.m_mat_detected_marker_img).toImageMsg();
        imgpub_detected_marker.publish(msg);
    }
    
}

int main(int argc, char **argv)
{
    std::cout << "main is running now..." << std::endl;
    
    // ROS Init
    ros::init(argc,argv, "ArucoMarkerLocalization");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("/usb_cam/image_raw",1,imgCallbackNode);
    imgpub_detected_marker= it.advertise("/detected_marker", 1);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 1);

    ros::spin();

    std::cout << "main finished..." << std::endl;

    return 1;
}

