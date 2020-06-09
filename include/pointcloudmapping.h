/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include "DataCache/DataCache.h"

using namespace ORB_SLAM2;
using namespace NS_DataCache;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping( double resolution_ );

    DataCache<PointCloud>* GetPointCloudDataCache();
    int32_t GetPointCloudDataCacheIndex();
    bool GetPointCloudStatus();
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();
    std::mutex* GetGlobalMapLock();
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    PointCloud::Ptr globalMap;
    PointCloud::Ptr globalMapBack;
    shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false;
    bool bIsPointCloudGenerated = false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    double resolution = 0.05;
    pcl::VoxelGrid<PointT>  voxel;
private:
    DataCache<PointCloud>*mpPointCloudDataCache;
    int32_t miDataCacheIndex;
};

#endif // POINTCLOUDMAPPING_H
