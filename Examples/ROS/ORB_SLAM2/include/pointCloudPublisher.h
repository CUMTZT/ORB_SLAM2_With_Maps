//
// Created by inspur on 2020/5/13.
//

#ifndef ORB_SLAM2_POINTCLOUDPUBLISHER_H
#define ORB_SLAM2_POINTCLOUDPUBLISHER_H

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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "System.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PointCloudPublisher
{
public:
    PointCloudPublisher(ros::NodeHandle* nh,ORB_SLAM2::System* SLAM);
private:
    ros::Publisher mPointCloudPub;//点云发布
    ros::Publisher mOctomapPub;//八叉树发布
    ros::Publisher mOccupancyMapPub;//2D占据栅格地图发布
    ros::Publisher mOdomPub;//odom数据发布

    bool publish_tf;
    std::string base_frame_id;
    std::string camera_frame_id;
    std::string world_frame_id;

    ros::NodeHandle* nh;
    ORB_SLAM2::System* SLAM;
    shared_ptr<thread> PointCloudPublishThread;
    shared_ptr<thread> OdomPublishThread;
    shared_ptr<PointCloudMapping> mpPointCloudMapping;
    DataCache<System::PointCloud>*mpPointCloudDataCache;
    int32_t miPointCloudDataCacheIndex;
    DataCache<cv::Mat>*mpTcwDataCache;
    int32_t miTcwDataCacheIndex;
    void PublishMap();
    void PublishOdom();

    std::mutex MutexPose;

    ros::Time pose_sync_time;
    cv::Mat Tcw;

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    bool get_base_to_camera;
    tf::Transform base_to_camera; // static, cached
    tf::Transform camera_to_base; // static, cached, calculated from base_to_laser_

    void getBaseToCameraTf();
    int getPose(float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw);
    void printTf(tf::Transform tf);
    void printStampedTf(tf::StampedTransform sTf);
    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
};



#endif //ORB_SLAM2_POINTCLOUDPUBLISHER_H
