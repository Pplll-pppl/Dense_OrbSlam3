#ifndef POINTCLOUDMAPPING_RGBD_H
#define POINTCLOUDMAPPING_RGBD_H

#include "System.h"

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
    // 新增：回环检测回调
    void onLoopClosureDetected();

private:
    // 严格统一返回类型
    PointCloudMappingRGBD::PointCloud::Ptr GetPointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    void Run();

    // 新增：全局优化方法
    void optimizeGlobalPointCloud();
    bool needGlobalOptimization();
    
    PointCloudMappingRGBD::PointCloud::Ptr globalMap;
    std::mutex globalMapMutex;

    struct KeyFrameData {
        KeyFrame* kf = nullptr;
        cv::Mat color;
        cv::Mat depth;
    };

    std::queue<KeyFrameData> mqNewKeyFrames;
    std::mutex mqMutex;
    std::condition_variable mcvNewKF;

    std::thread mThread;
    std::atomic<bool> mbStop{false};

    double resolution;
    double meank;
    double stdthresh;
    double unit;

    pcl::VoxelGrid<PointT> voxel;
    pcl::StatisticalOutlierRemoval<PointT> sor;

    size_t mProcessedKFCount = 0;
    static constexpr size_t GLOBAL_VOXEL_INTERVAL = 10;

    // 新增：全局优化相关参数
    int mGlobalOptimizationInterval;
    bool mEnableGlobalOptimization;
    bool mLastLoopClosure;
};

} // namespace ORB_SLAM3

#endif // POINTCLOUDMAPPING_RGBD_H