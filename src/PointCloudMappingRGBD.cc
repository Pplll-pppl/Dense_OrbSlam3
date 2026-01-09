#include "PointCloudMappingRGBD.h"

namespace ORB_SLAM3
{

PointCloudMappingRGBD::PointCloudMappingRGBD(double resolution_, double meank_, double stdthresh_, double unit_)
    : resolution(resolution_), meank(meank_), stdthresh(stdthresh_), unit(unit_)
{
    std::cout << "[PointCloudMappingRGBD] Initializing async RGB-D dense mapping..." << std::endl;

    globalMap.reset(new PointCloudMappingRGBD::PointCloud());

    voxel.setLeafSize(resolution, resolution, resolution);
    sor.setMeanK(static_cast<int>(meank));
    sor.setStddevMulThresh(stdthresh);
    // 全局优化参数
    mGlobalOptimizationInterval = 50; // 每25个关键帧全局优化一次
    mEnableGlobalOptimization = true;
    mLastLoopClosure = false;

    mThread = std::thread(&PointCloudMappingRGBD::Run, this);  // 正确类名
}

PointCloudMappingRGBD::~PointCloudMappingRGBD()
{
    shutdown();
}

void PointCloudMappingRGBD::shutdown()
{
    mbStop = true;
    mcvNewKF.notify_all();

    if (mThread.joinable())
        mThread.join();

    save();
}

void PointCloudMappingRGBD::save()
{
    std::unique_lock<std::mutex> lock(globalMapMutex);
    if (globalMap && globalMap->size() > 0)
    {
        pcl::io::savePCDFileASCII("./PointCloudMapping_RGBD.pcd", *globalMap);
        std::cout << "[PointCloudMappingRGBD] Saved to ./PointCloudMapping_RGBD.pcd | Points: "
                  << globalMap->size() << std::endl;
    }
}

void PointCloudMappingRGBD::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    KeyFrameData data;
    data.kf = kf;
    data.color = color.clone();
    data.depth = depth.clone();

    {
        std::unique_lock<std::mutex> lock(mqMutex);
        mqNewKeyFrames.push(data);
    }
    mcvNewKF.notify_one();
}

PointCloudMappingRGBD::PointCloud::Ptr PointCloudMappingRGBD::GetPointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloudMappingRGBD::PointCloud::Ptr tmp(new PointCloudMappingRGBD::PointCloud());

    if (depth.type() != CV_32F)
        depth.convertTo(depth, CV_32F);

    for (int v = 0; v < depth.rows; ++v)
    {
        const float* depth_ptr = depth.ptr<float>(v);
        const uchar* color_ptr = color.ptr<uchar>(v);

        for (int u = 0; u < depth.cols; ++u)
        {
            float d = depth_ptr[u];
            if (d <= 0.0f) continue;

            float z = d / unit;
            if (z < 0.3f || z > 10.0f) continue;

            PointT p;
            p.z = z;
            p.x = (u - kf->cx) * p.z / kf->fx;
            p.y = (v - kf->cy) * p.z / kf->fy;

            int idx = u * 3;
            p.r = color_ptr[idx];
            p.g = color_ptr[idx + 1];
            p.b = color_ptr[idx + 2];

            tmp->points.push_back(p);
        }
    }

    PointCloudMappingRGBD::PointCloud::Ptr cloud(new PointCloudMappingRGBD::PointCloud());
    pcl::transformPointCloud(*tmp, *cloud, kf->GetPoseInverse().matrix());
    cloud->is_dense = false;

    return cloud;
}

// 判断是否需要全局优化
bool PointCloudMappingRGBD::needGlobalOptimization()
{
    // 定期全局优化
    if (mProcessedKFCount % mGlobalOptimizationInterval == 0)
    {
        return true;
    }
    
    return false;
}

// 全局点云优化
void PointCloudMappingRGBD::optimizeGlobalPointCloud()
{
    if (!globalMap || globalMap->empty()) return;
    
    std::cout << "[PointCloudMappingRGBD_Optimized] 执行全局点云优化..." << std::endl;
    
    size_t before_size = globalMap->size();
    
    // 强力体素滤波
    pcl::VoxelGrid<PointT> aggressive_voxel;
    aggressive_voxel.setLeafSize(resolution * 0.95, resolution * 0.95, resolution * 0.95);
    aggressive_voxel.setInputCloud(globalMap);
    aggressive_voxel.filter(*globalMap);
    
    // 强力离群点去除
    pcl::StatisticalOutlierRemoval<PointT> aggressive_sor;
    aggressive_sor.setMeanK(10); 
    aggressive_sor.setStddevMulThresh(3);  
    aggressive_sor.setInputCloud(globalMap);
    aggressive_sor.filter(*globalMap);
    
    size_t after_size = globalMap->size();
    std::cout << "[PointCloudMappingRGBD_Optimized] 全局优化完成 | 优化前: " << before_size 
              << " | 优化后: " << after_size << " | 移除点数: " << (before_size - after_size) << std::endl;
}

// 回环检测回调
void PointCloudMappingRGBD::onLoopClosureDetected()
{
    std::unique_lock<std::mutex> lock(globalMapMutex);
    
    if (!mLastLoopClosure)
    {
        std::cout << "[PointCloudMappingRGBD] 检测到回环，执行强力优化..." << std::endl;
        
        size_t before_size = globalMap->size();
        
        // 回环检测后的强力优化
        pcl::VoxelGrid<PointT> loop_voxel;
        loop_voxel.setLeafSize(resolution * 1.1, resolution * 1.1, resolution * 1.1);
        loop_voxel.setInputCloud(globalMap);
        loop_voxel.filter(*globalMap);
        
        pcl::StatisticalOutlierRemoval<PointT> loop_sor;
        loop_sor.setMeanK(50);
        loop_sor.setStddevMulThresh(1.5);  
        loop_sor.setInputCloud(globalMap);
        loop_sor.filter(*globalMap);
        
        size_t after_size = globalMap->size();
        mLastLoopClosure = true;
        
        std::cout << "[PointCloudMappingRGBD_Optimized] 回环优化完成 | 优化前: " << before_size 
                  << " | 优化后: " << after_size << " | 移除点数: " << (before_size - after_size) << std::endl;
    }
}

void PointCloudMappingRGBD::Run()
{
    std::cout << "[PointCloudMappingRGBD] Mapping thread started." << std::endl;

    while (!mbStop)
    {
        KeyFrameData data;
        {
            std::unique_lock<std::mutex> lock(mqMutex);
            if (mqNewKeyFrames.empty())
            {
                mcvNewKF.wait(lock);
                continue;
            }
            data = mqNewKeyFrames.front();
            mqNewKeyFrames.pop();
        }

        PointCloudMappingRGBD::PointCloud::Ptr currentCloud = GetPointCloud(data.kf, data.color, data.depth);
        if (!currentCloud || currentCloud->empty()) continue;

        PointCloudMappingRGBD::PointCloud::Ptr filtered(new PointCloudMappingRGBD::PointCloud());
        voxel.setInputCloud(currentCloud);
        voxel.filter(*filtered);
        sor.setInputCloud(filtered);
        sor.filter(*filtered);

        {
            std::unique_lock<std::mutex> lock(globalMapMutex);

            *globalMap += *filtered;

            // 检查是否需要全局优化
            if (mEnableGlobalOptimization && needGlobalOptimization())
            {
                optimizeGlobalPointCloud();
            }

        }

        mProcessedKFCount++;

        if (mProcessedKFCount % 500 == 0)
        {
            // std::unique_lock<std::mutex> lock(globalMapMutex);
            // voxel.setInputCloud(globalMap);
            // voxel.filter(*globalMap);
            std::unique_lock<std::mutex> lock(globalMapMutex);
            
            // 轻量级体素滤波
            voxel.setInputCloud(globalMap);
            voxel.filter(*globalMap);
            
            // 轻量级离群点去除
            pcl::StatisticalOutlierRemoval<PointT> periodic_sor;
            periodic_sor.setMeanK(30);
            periodic_sor.setStddevMulThresh(2);
            periodic_sor.setInputCloud(globalMap);
            periodic_sor.filter(*globalMap);
        }

        if (mProcessedKFCount % 10 == 0)
        {
            std::cout << "[PointCloudMappingRGBD_Optimized] 关键帧 " << data.kf->mnId
                      << " | 新增: " << filtered->size()
                      << " | 总计: " << globalMap->size() << std::endl;
        }
    }
}

} // namespace ORB_SLAM3