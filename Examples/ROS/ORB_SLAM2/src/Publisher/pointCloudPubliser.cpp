//
// Created by inspur on 2020/5/13.
//

#include "pointCloudPublisher.h"
#include "unistd.h"
#include "DataCache/DataCache.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include "Converter.h"

using namespace NS_DataCache;
PointCloudPublisher::PointCloudPublisher(ros::NodeHandle* nh,ORB_SLAM2::System* SLAM )
{
    this->nh = nh;
    this->SLAM = SLAM;
    mpPointCloudMapping = SLAM->GetPointCloudMapping();
    mPointCloudPub = this->nh->advertise<sensor_msgs::PointCloud2>("/orbslam2/pointcloud", 100);
    mOctomapPub = this->nh->advertise<octomap_msgs::Octomap>("/orbslam2/octomap", 5);
    mOccupancyMapPub = this->nh->advertise<octomap_msgs::Octomap>("/orbslam2/occupancymap", 5);
    mOdomPub = this->nh->advertise<nav_msgs::Odometry>("/orbslam2/odometry", 5);

    mpPointCloudDataCache = mpPointCloudMapping->GetPointCloudDataCache();
    miPointCloudDataCacheIndex = mpPointCloudMapping->GetPointCloudDataCacheIndex();
    mpTcwDataCache = this->SLAM->GetTcwDataCache();
    miTcwDataCacheIndex = this->SLAM->GetTcwDataCacheIndex();

    PointCloudPublishThread = make_shared<thread>( bind( &PointCloudPublisher::PublishMap, this ) );
    //OdomPublishThread = make_shared<thread>( bind( &PointCloudPublisher::PublishOdom, this ) );

}
void PointCloudPublisher::PublishMap()
{
    int count = 0;
    //地图类数据发布
    sensor_msgs::PointCloud2 pointCloud;
    octomap::Pointcloud octo_cloud;
    PointCloudMapping::PointCloud tempPointCloud;
    octomap_msgs::Octomap octomap_msg;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );
    octomap_msgs::Octomap occupancymap_msg;
    while(ros::ok())
    {
        if (RET_SUCCESS == mpPointCloudDataCache->GetDataByIndex(miPointCloudDataCacheIndex,tempPointCloud))
        {
            //发布点云
            toROSMsg(tempPointCloud, pointCloud);
            pointCloud.header.stamp = ros::Time::now();
            pointCloud.header.frame_id = "camera_link";
            cout<<"Publish PointCloud: "<<count<<" 点云个数云："<<tempPointCloud.points.size()<<endl;
            mPointCloudPub.publish(pointCloud);

            //发布octomap
            for (auto p:tempPointCloud.points)
            {


                //去除天空地面
                if(p.z>1 || p.z<-0.5)
                {
                    continue;
                }
                // 将点云里的点插入到octomap中
                tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
            }
            // 更新octomap
            tree.updateInnerOccupancy();
            //fullMapToMsg负责转换成message
            octomap_msgs::fullMapToMsg(tree, octomap_msg);
            octomap_msg.header.frame_id = "camera_link";
            octomap_msg.header.stamp = ros::Time::now();
            mOctomapPub.publish(octomap_msg);

            //发布occupancymap
            octomap_msgs::binaryMapToMsg(tree, occupancymap_msg);
            occupancymap_msg.header.frame_id = "camera_link";
            occupancymap_msg.header.stamp = ros::Time::now();
            occupancymap_msg.id = 1;
            occupancymap_msg.binary = 1;
            occupancymap_msg.resolution = tree.getResolution();
            mOccupancyMapPub.publish(octomap_msg);
        }

        count++;
        usleep(10);
    }
}

void PointCloudPublisher::PublishOdom()
{
    publish_tf = true;
    world_frame_id = "odom";
    base_frame_id = "base_link";
    camera_frame_id = "camera_link";
    ros::Rate rate(20);
    bool warn_flag = false;
    float x, y, z, qx, qy, qz, qw;
    int ret_val;
    tf::Transform sensordata_tf, output_tf;
    tf::StampedTransform odom_trans;
    nav_msgs::Odometry odom;

    while (ros::ok())
    {
        if (SLAM->GetTrackingState() != 2)
        {
            continue;
        }

        // ret_val = SLAM->GetlastKeyFrame(x, y, z, qx, qy, qz, qw);

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
        mOdomPub.publish(odom);

        rate.sleep();
    }
}

int PointCloudPublisher::getPose(float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw)
{
    cv::Mat Tcw;
    if (RET_SUCCESS == mpTcwDataCache->GetDataByIndex(miTcwDataCacheIndex,Tcw))
    {
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
    }
    return 0;
}

void PointCloudPublisher::getBaseToCameraTf()
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

void PointCloudPublisher::printTf(tf::Transform tf)
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

void PointCloudPublisher::printStampedTf(tf::StampedTransform sTf)
{
    tf::Transform tf;
    cout << "frame_id: " << sTf.frame_id_ << endl;
    cout << "child_frame_id: " << sTf.child_frame_id_ << endl;
    tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf
    printTf(tf);                      //and print its components
}

tf::Transform PointCloudPublisher::get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}