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


#include"../../../include/System.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <Converter.h>
#include"../../../include/System.h"
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

using namespace std;

//! parameters
bool read_from_topic = false, read_from_camera = false;
std::string image_topic = "/camera/image_raw";
int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);
void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
             ros::Publisher &pub_all_kf_and_pts, int frame_id);


class ImageGrabber{
public:
    ImageGrabber(ORB_SLAM2::System &_SLAM, ros::Publisher &_pub_pts_and_pose,
                 ros::Publisher &_pub_all_kf_and_pts) :
            SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose),
            pub_all_kf_and_pts(_pub_all_kf_and_pts), frame_id(0){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System &SLAM;
    ros::Publisher &pub_pts_and_pose;
    ros::Publisher &pub_all_kf_and_pts;
    int frame_id;
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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ros::NodeHandle nh;

    PointCloudPublisher publisher(&nh,SLAM.GetPointCloudMapping());
    ros::Publisher pub_pts_and_pose = nh.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
    ros::Publisher pub_all_kf_and_pts = nh.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
    ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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
    publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, frame_id);
    ++frame_id;
}

void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
             ros::Publisher &pub_all_kf_and_pts, int frame_id) {
    if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
        pub_all_pts = true;
        pub_count = 0;
    }
    if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {
        pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
        geometry_msgs::PoseArray kf_pt_array;
        vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
        //! placeholder for number of keyframes
        kf_pt_array.poses.push_back(geometry_msgs::Pose());
        sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
        unsigned int n_kf = 0;
        for (auto key_frame : key_frames) {
            // pKF->SetPose(pKF->GetPose()*Two);

            if (key_frame->isBad())
                continue;

            cv::Mat R = key_frame->GetRotation().t();
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
            cv::Mat twc = key_frame->GetCameraCenter();
            geometry_msgs::Pose kf_pose;

            kf_pose.position.x = twc.at<float>(0);
            kf_pose.position.y = twc.at<float>(1);
            kf_pose.position.z = twc.at<float>(2);
            kf_pose.orientation.x = q[0];
            kf_pose.orientation.y = q[1];
            kf_pose.orientation.z = q[2];
            kf_pose.orientation.w = q[3];
            kf_pt_array.poses.push_back(kf_pose);

            unsigned int n_pts_id = kf_pt_array.poses.size();
            //! placeholder for number of points
            kf_pt_array.poses.push_back(geometry_msgs::Pose());
            std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
            unsigned int n_pts = 0;
            for (auto map_pt : map_points) {
                if (!map_pt || map_pt->isBad()) {
                    //printf("Point %d is bad\n", pt_id);
                    continue;
                }
                cv::Mat pt_pose = map_pt->GetWorldPos();
                if (pt_pose.empty()) {
                    //printf("World position for point %d is empty\n", pt_id);
                    continue;
                }
                geometry_msgs::Pose curr_pt;
                //printf("wp size: %d, %d\n", wp.rows, wp.cols);
                //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
                curr_pt.position.x = pt_pose.at<float>(0);
                curr_pt.position.y = pt_pose.at<float>(1);
                curr_pt.position.z = pt_pose.at<float>(2);
                kf_pt_array.poses.push_back(curr_pt);
                ++n_pts;
            }
            geometry_msgs::Pose n_pts_msg;
            n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
            kf_pt_array.poses[n_pts_id] = n_pts_msg;
            ++n_kf;
        }
        geometry_msgs::Pose n_kf_msg;
        n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
        kf_pt_array.poses[0] = n_kf_msg;
        kf_pt_array.header.frame_id = "1";
        kf_pt_array.header.seq = frame_id + 1;
        printf("Publishing data for %u keyfranmes\n", n_kf);
        pub_all_kf_and_pts.publish(kf_pt_array);
    }
    else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
        ++pub_count;
        SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
        ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        //while (pKF->isBad())
        //{
        //	Trw = Trw*pKF->mTcp;
        //	pKF = pKF->GetParent();
        //}

        vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        Trw = Trw*pKF->GetPose()*Two;
        cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
        cv::Mat Tcw = lit*Trw;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        //geometry_msgs::Pose camera_pose;
        //std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
        std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
        int n_map_pts = map_points.size();

        //printf("n_map_pts: %d\n", n_map_pts);

        //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        geometry_msgs::PoseArray pt_array;
        //pt_array.poses.resize(n_map_pts + 1);

        geometry_msgs::Pose camera_pose;

        camera_pose.position.x = twc.at<float>(0);
        camera_pose.position.y = twc.at<float>(1);
        camera_pose.position.z = twc.at<float>(2);

        camera_pose.orientation.x = q[0];
        camera_pose.orientation.y = q[1];
        camera_pose.orientation.z = q[2];
        camera_pose.orientation.w = q[3];

        pt_array.poses.push_back(camera_pose);

        //printf("Done getting camera pose\n");

        for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

            if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
                //printf("Point %d is bad\n", pt_id);
                continue;
            }
            cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

            if (wp.empty()) {
                //printf("World position for point %d is empty\n", pt_id);
                continue;
            }
            geometry_msgs::Pose curr_pt;
            //printf("wp size: %d, %d\n", wp.rows, wp.cols);
            //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
            curr_pt.position.x = wp.at<float>(0);
            curr_pt.position.y = wp.at<float>(1);
            curr_pt.position.z = wp.at<float>(2);
            pt_array.poses.push_back(curr_pt);
            //printf("Done getting map point %d\n", pt_id);
        }
        //sensor_msgs::PointCloud2 ros_cloud;
        //pcl::toROSMsg(*pcl_cloud, ros_cloud);
        //ros_cloud.header.frame_id = "1";
        //ros_cloud.header.seq = ni;

        //printf("valid map pts: %lu\n", pt_array.poses.size()-1);

        //printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
        //pub_cloud.publish(ros_cloud);
        pt_array.header.frame_id = "1";
        pt_array.header.seq = frame_id + 1;
        pub_pts_and_pose.publish(pt_array);
        //pub_kf.publish(camera_pose);
    }
}

inline bool isInteger(const std::string & s){
    if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

    char * p;
    strtol(s.c_str(), &p, 10);

    return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps){
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()){
        string s;
        getline(fTimes, s);
        if (!s.empty()){
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}