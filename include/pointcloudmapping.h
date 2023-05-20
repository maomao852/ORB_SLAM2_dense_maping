#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

namespace ORB_SLAM2
{
    class PointCloudMapping
    {
        public:

        PointCloudMapping();

        void InsertKeyFrame(KeyFrame* kf);

        void UpdateCloud();
    };
}

#endif