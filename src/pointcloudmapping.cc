
#include "pointcloudmapping.h"
#include "Converter.h"


namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping()
    {
        std::cout << "Point cloud mapping has structed. " << std::endl;
        mpGlobalCloud = boost::make_shared< PointCloud >( );
        voxel.setLeafSize(resolution, resolution, resolution);
        mptViewerThread = make_shared<thread>(bind(&PointCloudMapping::Viewer, this));
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame* kf, cv::Mat &color_img, cv::Mat &depth_img)
    {
        cout << "recieve a keyframe, id = " << kf->mnId << endl;

        unique_lock<mutex> lck(mmKeyFrameMutex);
        mvKeyFrames.push_back(kf);
        GeneratePointCloud(kf, color_img, depth_img);  

    }

    void PointCloudMapping::GeneratePointCloud(KeyFrame* kf, cv::Mat &color_img, cv::Mat &depth_img)
    {

        // cv::imshow("color img", color_img);
        // cv::imshow("depth img", depth_img);
        PointCloud::Ptr tmp (new PointCloud());
        for ( int m=0; m<depth_img.rows; m+=3 )
        {
            for ( int n=0; n<depth_img.cols; n+=3 )
            {
                float d = depth_img.ptr<float>(m)[n];
                if (d < 0.01 || d>5)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;
                
                p.b = color_img.ptr<uchar>(m)[n*3];
                p.g = color_img.ptr<uchar>(m)[n*3+1];
                p.r = color_img.ptr<uchar>(m)[n*3+2];

                // cout << p.x << " " << p.y << " " << p.z << endl;
                    
                tmp->points.push_back(p);
            }
        }
        cout << "The keyframe has point clouds: " << tmp->points.size() << endl;
        kf->mptrPointCloud = tmp;
    }

    void PointCloudMapping::UpdateCloud()
    {
        unique_lock<mutex> lck(mmCloudeUpdateMutex);
        mbLoopBusy = true;
        cout << endl << endl << "start loop map point." << endl << endl;
        PointCloud::Ptr tmp(new PointCloud);
        for(int i = 0; i < mvKeyFrames.size(); i++)
        {
            if(!mvKeyFrames[i]->isBad())
            {
                PointCloud::Ptr cloud(new PointCloud);
                pcl::transformPointCloud( *(mvKeyFrames[i]->mptrPointCloud), *cloud, Converter::toMatrix4d(mvKeyFrames[i]->GetPoseInverse()));
                *tmp += *cloud;
            }
        }

        cout << "finsh loop map" << endl;
        voxel.setInputCloud(tmp);
        voxel.filter(*mpGlobalCloud);
        mbLoopBusy = false;
    }

    void PointCloudMapping::Viewer()
    {
        pcl::visualization::CloudViewer viewer("Dense map viewer");
        while(1)
        {
            // cout << "wait << " << endl;
            size_t N = 0;
            {
                unique_lock<mutex> lck(mmKeyFrameMutex);

                if(mbLoopBusy)
                {
                    continue;
                }

                {
                    unique_lock<mutex> lck_shut_down(mmShutDownMutex);
                    if(mbShutDownFlag)
                    {
                        cout << "Viewer has break " << endl;
                        break;
                    }
                }

                N = mvKeyFrames.size();
                if(N == mnLastKeyFrameId)
                {
                    continue;
                }
                else
                {
                    unique_lock<mutex> lck_(mmCloudeUpdateMutex);
                    for(size_t i = mnLastKeyFrameId; i < N; i++)
                    {
                        PointCloud::Ptr p (new PointCloud);
                        pcl::transformPointCloud( *(mvKeyFrames[i]->mptrPointCloud), *p, Converter::toMatrix4d(mvKeyFrames[i]->GetPoseInverse()));
                        //cout<<"处理好第i个点云"<<i<<endl;
                        *mpGlobalCloud += *p;
                    }

                    PointCloud::Ptr tmp(new PointCloud);
                    voxel.setInputCloud(mpGlobalCloud);
                    voxel.filter(*tmp);
                    mpGlobalCloud->swap(*tmp);
                    mnLastKeyFrameId = N;
                    cout << "Total has point clouds: " << mpGlobalCloud->points.size() << endl;
                }
            }
            
            viewer.showCloud(mpGlobalCloud);
            
        }
    }

    void PointCloudMapping::SaveCloud()
    {
        pcl::io::savePCDFile("result.pcd", *mpGlobalCloud);
        cout << "global cloud save finished !" << endl;
    }

    void PointCloudMapping::ShutDown()
    {
        {
            unique_lock<mutex> lck(mmShutDownMutex);
            mbShutDownFlag = true;
        }
        mptViewerThread->join();
        cout << "Point cloud mapping has shut down! " << endl;
    }
}