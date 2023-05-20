#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

namespace ORB_SLAM2
{
    class PointCloudMapping
    {
        public:

        PointCloudMapping();

        void InsertKeyFrame(KeyFrame* kf, cv::Mat& color_img, cv::Mat& depth_img);

        void GeneratePointCloud(KeyFrame* kf, cv::Mat& color_img, cv::Mat& depth_img);

        void UpdateCloud();

        void Viewer();

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        
        PointCloud::Ptr mpGlobalCloud;
        shared_ptr<thread> mptViewerThread;
        vector<KeyFrame*> mvKeyFrames;
        size_t mnLastKeyFrameId = 0;
        mutex mmKeyFrameMutex;
    };
}

#endif