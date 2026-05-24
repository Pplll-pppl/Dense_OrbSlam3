#include "PointCloudMappingRGBD.h"

#include <iomanip>

namespace ORB_SLAM3
{

PointCloudMappingRGBD::PointCloudMappingRGBD(double resolution_, double meank_, double stdthresh_, double unit_)
    : resolution(resolution_),
      meank(meank_),
      stdthresh(stdthresh_),
      unit(unit_)
{
    std::cout << "[PointCloudMappingRGBD] Initializing async RGB-D dense mapping..." << std::endl;

    globalMap.reset(new PointCloudMappingRGBD::PointCloud());
    globalMap->is_dense = false;
    globalMap->width = 0;
    globalMap->height = 1;

    voxel.setLeafSize(resolution, resolution, resolution);
    sor.setMeanK(static_cast<int>(meank));
    sor.setStddevMulThresh(stdthresh);
    mThreadStartTime = std::chrono::steady_clock::now();
    mThreadEndTime = mThreadStartTime;

    mThread = std::thread(&PointCloudMappingRGBD::Run, this);  // 正确类名
}

PointCloudMappingRGBD::~PointCloudMappingRGBD()
{
    shutdown();
}

void PointCloudMappingRGBD::shutdown()
{
    bool expected = false;
    if (!mbHasShutdown.compare_exchange_strong(expected, true))
        return;

    mbStop.store(true);
    mcvNewKF.notify_all();

    if (mThread.joinable())
        mThread.join();

    printTimingSummary();
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
    if (!kf || color.empty() || depth.empty())
        return;

    if (kf->isBad())
        return;

    KeyFrameData data;
    data.keyframe_id = kf->mnId;
    data.fx = kf->fx;
    data.fy = kf->fy;
    data.cx = kf->cx;
    data.cy = kf->cy;
    data.Twc = kf->GetPoseInverse().matrix();
    data.color = color.clone();
    data.depth = depth.clone();
    data.enqueue_time = std::chrono::steady_clock::now();

    {
        std::unique_lock<std::mutex> lock(mqMutex);
        if (mbStop.load())
        {
            std::cerr << "[PointCloudMappingRGBD] Ignore keyframe " << data.keyframe_id
                      << " because shutdown has already been requested." << std::endl;
            return;
        }
        mqNewKeyFrames.push(data);
    }
    mcvNewKF.notify_one();
}

PointCloudMappingRGBD::PointCloud::Ptr PointCloudMappingRGBD::GetPointCloud(const KeyFrameData& data)
{
    PointCloudMappingRGBD::PointCloud::Ptr tmp(new PointCloudMappingRGBD::PointCloud());

    const float fx = data.fx;
    const float fy = data.fy;
    const float cx = data.cx;
    const float cy = data.cy;
    const cv::Mat& color = data.color;

    if (fx <= 0.0f || fy <= 0.0f || color.empty() || data.depth.empty())
        return tmp;

    cv::Mat depth;
    if (data.depth.type() != CV_32F)
        data.depth.convertTo(depth, CV_32F);
    else
        depth = data.depth;

    if (color.rows != depth.rows || color.cols != depth.cols || color.channels() < 3)
        return tmp;

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
            p.x = (u - cx) * p.z / fx;
            p.y = (v - cy) * p.z / fy;

            int idx = u * 3;
            p.r = color_ptr[idx];
            p.g = color_ptr[idx + 1];
            p.b = color_ptr[idx + 2];

            tmp->points.push_back(p);
        }
    }

    PointCloudMappingRGBD::PointCloud::Ptr cloud(new PointCloudMappingRGBD::PointCloud());
    pcl::transformPointCloud(*tmp, *cloud, data.Twc);
    cloud->is_dense = false;
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;

    return cloud;
}


void PointCloudMappingRGBD::Run()
{
    std::cout << "[PointCloudMappingRGBD] Mapping thread started." << std::endl;
    mThreadStartTime = std::chrono::steady_clock::now();

    while (true)
    {
        KeyFrameData data;
        {
            std::unique_lock<std::mutex> lock(mqMutex);
            mcvNewKF.wait(lock, [this]() {
                return mbStop.load() || !mqNewKeyFrames.empty();
            });

            if (mqNewKeyFrames.empty())
                break;

            data = mqNewKeyFrames.front();
            mqNewKeyFrames.pop();
        }

        const auto process_start = std::chrono::steady_clock::now();
        const double queue_wait_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                process_start - data.enqueue_time)
                .count();

        const auto build_start = std::chrono::steady_clock::now();
        PointCloudMappingRGBD::PointCloud::Ptr currentCloud = GetPointCloud(data);
        const auto build_end = std::chrono::steady_clock::now();
        const double build_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                build_end - build_start)
                .count();
        if (!currentCloud || currentCloud->empty()) continue;

        const auto voxel_start = std::chrono::steady_clock::now();
        PointCloudMappingRGBD::PointCloud::Ptr voxel_filtered(new PointCloudMappingRGBD::PointCloud());
        voxel.setInputCloud(currentCloud);
        voxel.filter(*voxel_filtered);
        voxel_filtered->is_dense = false;
        voxel_filtered->width = static_cast<uint32_t>(voxel_filtered->points.size());
        voxel_filtered->height = 1;
        const auto voxel_end = std::chrono::steady_clock::now();
        const double voxel_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                voxel_end - voxel_start)
                .count();

        if (voxel_filtered->empty())
            continue;

        PointCloudMappingRGBD::PointCloud::Ptr filtered(new PointCloudMappingRGBD::PointCloud());
        if (voxel_filtered->size() > static_cast<std::size_t>(meank))
        {
            sor.setInputCloud(voxel_filtered);
            sor.filter(*filtered);
        }
        else
        {
            *filtered = *voxel_filtered;
        }

        filtered->is_dense = false;
        filtered->width = static_cast<uint32_t>(filtered->points.size());
        filtered->height = 1;

        if (filtered->empty())
            continue;

        double merge_ms = 0.0;
        {
            const auto merge_start = std::chrono::steady_clock::now();
            std::unique_lock<std::mutex> lock(globalMapMutex);

            *globalMap += *filtered;
            
            globalMap->width = static_cast<uint32_t>(globalMap->points.size());
            globalMap->height = 1;

            const auto merge_end = std::chrono::steady_clock::now();
            merge_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    merge_end - merge_start)
                    .count();
        }

        const auto process_end = std::chrono::steady_clock::now();
        const double mapping_ms =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                process_end - process_start)
                .count();

        if (!mHasProcessedAnyKF)
        {
            mFirstKFProcessTime = process_start;
            mHasProcessedAnyKF = true;
        }
        mLastKFProcessTime = process_end;
        mTotalQueueWaitMs += queue_wait_ms;
        mTotalPointCloudBuildMs += build_ms;
        mTotalVoxelFilterMs += voxel_ms;
        mTotalMergeMs += merge_ms;
        mTotalMappingMs += mapping_ms;
        mProcessedKFCount++;

        if (mProcessedKFCount % 10 == 0)
        {
            std::cout << "[PointCloudMappingRGBD_Optimized] 关键帧 " << data.keyframe_id
                      << " | 新增: " << filtered->size()
                      << " | 总计: " << globalMap->size()
                      << " | 本帧映射: " << std::fixed << std::setprecision(2) << mapping_ms << " ms"
                      << " | 队列等待: " << queue_wait_ms << " ms" << std::endl;
        }
    }

    mThreadEndTime = std::chrono::steady_clock::now();
}

void PointCloudMappingRGBD::printTimingSummary() const
{
    const auto summary_end =
        (mThreadEndTime >= mThreadStartTime) ? mThreadEndTime : std::chrono::steady_clock::now();
    const double thread_lifetime_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(summary_end - mThreadStartTime)
            .count();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[PointCloudMappingRGBD] Timing summary" << std::endl;
    std::cout << "  Mapping thread lifetime: " << thread_lifetime_s << " s" << std::endl;
    std::cout << "  Processed keyframes: " << mProcessedKFCount << std::endl;

    if (!mHasProcessedAnyKF || mProcessedKFCount == 0)
    {
        std::cout << "  No keyframes were processed by dense mapping." << std::endl;
        return;
    }

    const double active_mapping_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(mLastKFProcessTime -
                                                                  mFirstKFProcessTime)
            .count();
    const double denom = static_cast<double>(mProcessedKFCount);

    std::cout << "  Active mapping span: " << active_mapping_s << " s" << std::endl;
    std::cout << "  Total mapping compute: " << (mTotalMappingMs / 1000.0) << " s" << std::endl;
    std::cout << "  Avg per-keyframe mapping: " << (mTotalMappingMs / denom) << " ms" << std::endl;
    std::cout << "  Avg queue wait: " << (mTotalQueueWaitMs / denom) << " ms" << std::endl;
    std::cout << "  Avg point-cloud build: " << (mTotalPointCloudBuildMs / denom) << " ms"
              << std::endl;
    std::cout << "  Avg voxel filter: " << (mTotalVoxelFilterMs / denom) << " ms" << std::endl;
    std::cout << "  Avg merge/update: " << (mTotalMergeMs / denom) << " ms" << std::endl;
}
PointCloudMappingRGBD::PointCloud::Ptr PointCloudMappingRGBD::getGlobalPointCloud()
{
    std::unique_lock<std::mutex> lock(globalMapMutex);
    if (!globalMap || globalMap->empty())
        return nullptr;
    
    PointCloud::Ptr result(new PointCloud(*globalMap));
    
    result->width = static_cast<uint32_t>(result->points.size());
    result->height = 1;
    
    return result;
}

} // namespace ORB_SLAM3
