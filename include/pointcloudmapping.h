#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <condition_variable>

namespace ORB_SLAM2
{
    class PointCloudMapping
    {
        public:

        PointCloudMapping();

        void InsertKeyFrame(KeyFrame* kf, cv::Mat& color_img, cv::Mat& depth_img);

        void GeneratePointCloud(KeyFrame* kf, cv::Mat& color_img, cv::Mat& depth_img);

        void UpdateCloud();

        void SaveCloud();

        void Viewer();

        void ShutDown();

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        
        PointCloud::Ptr mpGlobalCloud;
        shared_ptr<thread> mptViewerThread;
        vector<KeyFrame*> mvKeyFrames;
        size_t mnLastKeyFrameId = 0;
        mutex mmKeyFrameMutex;
        mutex mmCloudeUpdateMutex;
        mutex mmShutDownMutex;
        bool mbLoopBusy = false;
        bool mbShutDownFlag = false;

        double resolution = 0.04;
        pcl::VoxelGrid<PointT> voxel;
    };
}

#endif