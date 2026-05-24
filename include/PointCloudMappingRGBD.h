#ifndef POINTCLOUDMAPPING_RGBD_H
#define POINTCLOUDMAPPING_RGBD_H

#include "System.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>

namespace ORB_SLAM3
{

class PointCloudMappingRGBD
{
public:
    using PointT = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT>;

    PointCloudMappingRGBD(double resolution_ = 0.04,
                          double meank_ = 50,
                          double stdthresh_ = 1.0,
                          double unit_ = 1000.0);

    ~PointCloudMappingRGBD();

    void shutdown();
    void save();

    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    PointCloud::Ptr getGlobalPointCloud(); //for ros2 rviz visualization

private:
    struct KeyFrameData {
        unsigned long keyframe_id = 0;
        float fx = 0.0f;
        float fy = 0.0f;
        float cx = 0.0f;
        float cy = 0.0f;
        Eigen::Matrix4f Twc = Eigen::Matrix4f::Identity();
        cv::Mat color;
        cv::Mat depth;
        std::chrono::steady_clock::time_point enqueue_time;
    };

    // 严格统一返回类型
    PointCloudMappingRGBD::PointCloud::Ptr GetPointCloud(const KeyFrameData& data);

    void Run();
    void printTimingSummary() const;
    
    PointCloudMappingRGBD::PointCloud::Ptr globalMap;
    std::mutex globalMapMutex;

    std::queue<KeyFrameData> mqNewKeyFrames;
    std::mutex mqMutex;
    std::condition_variable mcvNewKF;

    std::thread mThread;
    std::atomic<bool> mbStop{false};
    std::atomic<bool> mbHasShutdown{false};

    double resolution;
    double meank;
    double stdthresh;
    double unit;

    pcl::VoxelGrid<PointT> voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor;

    size_t mProcessedKFCount = 0;

    std::chrono::steady_clock::time_point mThreadStartTime;
    std::chrono::steady_clock::time_point mThreadEndTime;
    std::chrono::steady_clock::time_point mFirstKFProcessTime;
    std::chrono::steady_clock::time_point mLastKFProcessTime;
    bool mHasProcessedAnyKF = false;
    double mTotalQueueWaitMs = 0.0;
    double mTotalPointCloudBuildMs = 0.0;
    double mTotalVoxelFilterMs = 0.0;
    double mTotalMergeMs = 0.0;
    double mTotalMappingMs = 0.0;
};

} // namespace ORB_SLAM3

#endif // POINTCLOUDMAPPING_RGBD_H
