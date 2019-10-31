/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

//
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

// OPEN CV
#include <opencv2/core/core.hpp>

// ORB_SLAM2
#include "../../../include/System.h"
#include"../../../include/Converter.h"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>


// Tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>



using namespace std;


std::string parent_name;
std::string child_name;

tf2_ros::TransformBroadcaster* tf_transform_broadcaster;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

  
    // Parameters
    //
    ros::param::param<std::string>("~camera_base_link", parent_name,"camera_orb_slam2");
    std::cout<<"camera_base_link="<<parent_name<<std::endl;
    //
    ros::param::param<std::string>("~world_base_link", child_name,"world_orb_slam2");
    std::cout<<"world_base_link="<<child_name<<std::endl;


    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);


    //
    tf_transform_broadcaster=new tf2_ros::TransformBroadcaster();

    //
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    //
    if(tf_transform_broadcaster)
        delete tf_transform_broadcaster;
    
    //
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //
    cv::Mat camera_pose_mat=mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()).clone();
    
    
    // Check
    if(camera_pose_mat.empty())
    {
      ROS_DEBUG("Localization failed");
      return;
    }
    
    // Current Time Stamp
    ros::Time curr_stamp = msg->header.stamp;
    
    
    // Tf transform
    tf2::Matrix3x3 tf_rot(camera_pose_mat.at<float>(0,0), camera_pose_mat.at<float>(0,1), camera_pose_mat.at<float>(0,2),
                       camera_pose_mat.at<float>(1,0), camera_pose_mat.at<float>(1,1), camera_pose_mat.at<float>(1,2),
                       camera_pose_mat.at<float>(2,0), camera_pose_mat.at<float>(2,1), camera_pose_mat.at<float>(2,2));

    tf2::Vector3 tf_orig(camera_pose_mat.at<float>(0,3), camera_pose_mat.at<float>(1,3), camera_pose_mat.at<float>(2,3));

    tf2::Transform tf2_transform=tf2::Transform(tf_rot, tf_orig);

    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);
    
    
    // TF
    geometry_msgs::TransformStamped transform_stamped;

    // Header
    transform_stamped.header.stamp=curr_stamp;
    transform_stamped.header.frame_id=parent_name;
    // Child frame id
    transform_stamped.child_frame_id=child_name;
    // Transform
    transform_stamped.transform = transform;
    // Publish
    tf_transform_broadcaster->sendTransform(transform_stamped);
    
    
    //
    return;
}


