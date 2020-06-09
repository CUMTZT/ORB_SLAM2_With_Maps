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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>


#include"System.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <Converter.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>

#include<opencv2/core/core.hpp>
#include "MapPoint.h"
#include "pointCloudPublisher.h"
#include <stdio.h>
#include <unistd.h>


using namespace std;


class ImageGrabber{
public:
    ImageGrabber(ORB_SLAM2::System &_SLAM) :
            SLAM(_SLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System &SLAM;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    //Check settings file
    cout << "配置文件路径："<<argv[2] << endl;
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << argv[2] << endl;
        exit(-1);
    }

    // for point cloud resolution
    string rgbTopic = fsSettings["Image.RGB"];
    string depthTopic = fsSettings["Image.Depth"];

    //获取地图保存路径
    string mapsavepath = fsSettings["Map.SavePath"];


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);



    if(access(mapsavepath.c_str(),0) == 0)
    {
        SLAM.LoadMap(mapsavepath);
    }
    ros::NodeHandle nh;

    PointCloudPublisher publisher(&nh,&SLAM);
    ImageGrabber igb(SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgbTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depthTopic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2));
    ros::spin();


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    cout<<"开始进入保存地图"<<endl;
    SLAM.SaveMap(mapsavepath);
    SLAM.SaveKeyFrameTrajectoryTUM("/home/inspur/KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    SLAM.TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

}
