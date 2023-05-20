
#include "pointcloudmapping.h"
#include <iostream>

namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping()
    {
        std::cout << "Point cloud mapping has structed. " << std::endl;
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame* kf)
    {
        std::cout << "Insert key frame pose: " << std::endl << kf->GetPose() << std::endl;
    }

    void UpdateCloud()
    {
        
    }
}