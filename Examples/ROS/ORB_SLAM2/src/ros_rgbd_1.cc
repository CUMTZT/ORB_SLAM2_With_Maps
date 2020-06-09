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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include "include/System.h"
#include "include/Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) { get_base_to_camera = false; }

    ORB_SLAM2::System *mpSLAM;
    ros::Publisher odom_pub;

    bool publish_tf;
    std::string base_frame_id;
    std::string camera_frame_id;
    std::string world_frame_id;

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
    void Vo_pub();
    void getBaseToCameraTf();

private:
    std::mutex MutexPose;

    ros::Time pose_sync_time;
    cv::Mat Tcw;

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    bool get_base_to_camera;
    tf::Transform base_to_camera; // static, cached
    tf::Transform camera_to_base; // static, cached, calculated from base_to_laser_

    int getPose(float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw);
    void printTf(tf::Transform tf);
    void printStampedTf(tf::StampedTransform sTf);
    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
};

int main(int argc, char **argv)
{
    std::string voc_path;
    std::string cam_conf_path;
    bool use_gui;

    ros::init(argc, argv, "RGBD");
    // ros::start();

    // if (argc != 3)
    // {
    //     cerr << endl
    //          << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
    //     // ros::shutdown();
    //     return 1;
    // }

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    if (!nh_private.getParam("use_gui", use_gui))
    {
        use_gui = true;
    }

    if (!nh_private.getParam("voc_path", voc_path))
    {
        voc_path = "";
    }

    if (!nh_private.getParam("cam_conf_path", cam_conf_path))
    {
        cam_conf_path = "";
    }

    if (voc_path == "" || cam_conf_path == "")
    {
        ROS_ERROR("Please check VOCABULARY PATH and CAMERA CONFIG PATH.");
        // ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    ORB_SLAM2::System SLAM(voc_path.data(), cam_conf_path.data(), ORB_SLAM2::System::RGBD, use_gui);
    ImageGrabber igb(&SLAM);

    nh_private.param<bool>("publish_tf", igb.publish_tf, true);
    nh_private.param<std::string>("world_frame_id", igb.world_frame_id, "odom");
    nh_private.param<std::string>("base_frame_id", igb.base_frame_id, "base_link");
    nh_private.param<std::string>("camera_frame_id", igb.camera_frame_id, "camera_link");
    igb.odom_pub = nh.advertise<nav_msgs::Odometry>("v_odom", 50);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    boost::function0<void> f_get_tf = boost::bind(&ImageGrabber::getBaseToCameraTf, &igb);
    boost::thread get_tf_thread(f_get_tf);
    boost::function0<void> f_pub = boost::bind(&ImageGrabber::Vo_pub, &igb);
    boost::thread pub_thread(f_pub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    unique_lock<mutex> lock(MutexPose);
    pose_sync_time = ros::Time::now();
    // ROS_INFO_STREAM(__FILE__ << "(" << __LINE__ << ") Sync time checkpoint:"
    //                          << "[" << pose_sync_time << "]");
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
}

void ImageGrabber::Vo_pub()
{
    ros::Rate rate(20);
    bool warn_flag = false;

    float x, y, z, qx, qy, qz, qw;
    int ret_val;
    tf::Transform sensordata_tf, output_tf;
    tf::StampedTransform odom_trans;
    nav_msgs::Odometry odom;

    //usleep(500000);

    while (ros::ok())
    {
        if (mpSLAM->GetTrackingState() != 2)
        {
            continue;
        }

        // ret_val = mpSLAM->GetlastKeyFrame(x, y, z, qx, qy, qz, qw);

        // if (ret_val != 0)
        //     continue;
        // odom.header.stamp = ros::Time::now();

        unique_lock<mutex> lock(MutexPose);
        ret_val = getPose(x, y, z, qx, qy, qz, qw);
        lock.unlock();

        if (ret_val != 0)
            continue;

        sensordata_tf.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setX(qx);
        q.setY(qy);
        q.setZ(qz);
        q.setW(qw);
        sensordata_tf.setRotation(q);

        // ROS_INFO_STREAM(__FILE__ << "(" << __LINE__ << ") Sync time checkpoint:"
        //                          << "[" << pose_sync_time << "]");

        if (get_base_to_camera)
        {
            if (warn_flag)
            {
                ROS_INFO("Get TF form '%s' to '%s', Publish TF '%s' to '%s'.", base_frame_id.data(), camera_frame_id.data(), world_frame_id.data(), base_frame_id.data());
                warn_flag = false;
            }

            output_tf = base_to_camera * sensordata_tf * camera_to_base;

            // tf::Quaternion temp = output_tf.getRotation();
            // temp.normalize();
            // output_tf.setRotation(temp);

            odom_trans = tf::StampedTransform(output_tf, pose_sync_time, world_frame_id, base_frame_id);
        }
        else
        {
            if (!warn_flag)
            {
                ROS_WARN("Can`t get TF form '%s' to '%s',Publish TF '%s' to '%s' instead.", base_frame_id.data(), camera_frame_id.data(), world_frame_id.data(), camera_frame_id.data());
                warn_flag = true;
            }
            output_tf = sensordata_tf;
            odom_trans = tf::StampedTransform(output_tf, pose_sync_time, world_frame_id, camera_frame_id);
        }

        if (publish_tf)
        {
            tf_broadcaster.sendTransform(odom_trans);
        }

        odom.header.stamp = pose_sync_time;
        odom.header.frame_id = world_frame_id;
        odom.child_frame_id = odom_trans.child_frame_id_;

        odom.pose.pose.position.x = output_tf.getOrigin().getX();
        odom.pose.pose.position.y = output_tf.getOrigin().getY();
        odom.pose.pose.position.z = output_tf.getOrigin().getZ();

        odom.pose.pose.orientation.x = output_tf.getRotation().x();
        odom.pose.pose.orientation.y = output_tf.getRotation().y();
        odom.pose.pose.orientation.z = output_tf.getRotation().z();
        odom.pose.pose.orientation.w = output_tf.getRotation().w();
        odom.pose.covariance[0] = 1e-3;
        odom.pose.covariance[7] = 1e-3;
        odom.pose.covariance[14] = 1e-6;
        odom.pose.covariance[21] = 1e-6;
        odom.pose.covariance[28] = 1e-6;
        odom.pose.covariance[35] = 1e-3;

        //  [ 1e-3, 0, 0, 0, 0, 0,
        //    0, 1e-3, 0, 0, 0, 0,
        //    0, 0, 1e6, 0, 0, 0,
        //    0, 0, 0, 1e6, 0, 0,
        //    0, 0, 0, 0, 1e6, 0,
        //    0, 0, 0, 0, 0, 1e3 ];
        odom_pub.publish(odom);

        rate.sleep();
    }
}

int ImageGrabber::getPose(float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw)
{
    if (Tcw.empty())
        return -1;

    // pose_sync_time = ros::Time::now();
    cv::Mat Ow;
    cv::Mat R = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = R.t();
    Ow = -Rwc * tcw;
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);

    x = Ow.at<float>(2);
    y = -Ow.at<float>(0);
    z = -Ow.at<float>(1);
    qx = -q[2];
    qy = q[0];
    qz = q[1];
    qw = q[3];
    // cout << setprecision(7) << " x=" << Ow.at<float>(2) << " y=" << -Ow.at<float>(0) << " z=" << -Ow.at<float>(1)
    //      << " qx=" << q[2] << " qy=" << q[0] << " qz=" << q[1] << " qw=" << q[3] << endl;
    Tcw.release();
    return 0;
}

void ImageGrabber::getBaseToCameraTf()
{
    bool warn_flag = false;
    tf::StampedTransform base_to_camera_tf;
    while (ros::ok())
    {
        // ros::Time t = ros::Time::now();
        ros::Time t = pose_sync_time;
        // ROS_INFO_STREAM(__FILE__ << "(" << __LINE__ << ") Sync time checkpoint:"
        //                          << "[" << t << "]");
        // ROS_INFO_STREAM(__FILE__ << "(" << __LINE__ << ") Sync time checkpoint:"
        //                          << "[" << pose_sync_time << "]");
        try
        {
            bool ret = tf_listener.waitForTransform(base_frame_id, camera_frame_id, t, ros::Duration(1.0));
            //ROS_INFO("%s", ret ? "TRUE" : "FALSE");
            tf_listener.lookupTransform(base_frame_id, camera_frame_id, t, base_to_camera_tf);
        }
        catch (tf::TransformException ex)
        {
            if (!warn_flag)
            {
                ROS_WARN("Could not get initial transform from '%s' to '%s' frame, %s", base_frame_id.data(), camera_frame_id.data(), ex.what());
                warn_flag = true;
            }
            get_base_to_camera = false;
            continue;
        }
        base_to_camera = base_to_camera_tf;
        camera_to_base = base_to_camera.inverse();

        warn_flag = false;
        get_base_to_camera = true;
    }
}

void ImageGrabber::printTf(tf::Transform tf)
{
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tf.getOrigin();
    cout << "vector from reference frame to to child frame: " << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfR = tf.getBasis();
    cout << "orientation of child frame w/rt reference frame: " << endl;
    tfVec = tfR.getRow(0);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfVec = tfR.getRow(1);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    tfVec = tfR.getRow(2);
    cout << tfVec.getX() << "," << tfVec.getY() << "," << tfVec.getZ() << endl;
    quat = tf.getRotation();
    cout << "quaternion: " << quat.x() << ", " << quat.y() << ", "
         << quat.z() << ", " << quat.w() << endl;
}

void ImageGrabber::printStampedTf(tf::StampedTransform sTf)
{
    tf::Transform tf;
    cout << "frame_id: " << sTf.frame_id_ << endl;
    cout << "child_frame_id: " << sTf.child_frame_id_ << endl;
    tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf
    printTf(tf);                      //and print its components
}

tf::Transform ImageGrabber::get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}