#include "SlamDataProcess.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace ORB_SLAM3
{

namespace
{
double NowSeconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MakeCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->is_dense = false;
    cloud->height = 1;
    return cloud;
}

void FinalizeCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    if (!cloud)
        return;
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
}

Eigen::Matrix4f CvMatToEigen4f(const cv::Mat& mat)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    if (mat.empty() || mat.rows < 4 || mat.cols < 4)
        return result;

    cv::Mat mat32;
    if (mat.type() == CV_32F)
        mat32 = mat;
    else
        mat.convertTo(mat32, CV_32F);

    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
            result(r, c) = mat32.at<float>(r, c);
    }

    return result;
}
} // namespace

SlamDataProcess::SlamDataProcess(System* pSystem,
                                 FrameDrawer* pFrameDrawer,
                                 MapDrawer* pMapDrawer,
                                 Tracking* pTracking,
                                 const string& strSettingPath,
                                 Atlas* pAtlas,
                                 bool useDense)
    : mpSystem(pSystem),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpTracker(pTracking),
      mpAtlas(pAtlas),
      mpPointClouds(nullptr),
      mbUseDense(useDense),
      mpDenseMapper(nullptr),
      mT(100.0),
      mImageWidth(0.0f),
      mImageHeight(0.0f),
      mbFinishRequested(false),
      mbFinished(false),
      mbStopped(false),
      mbStopRequested(false),
      mbGetNewCamPose(false),
      mbGetPointCloud(false),
      mbGetDrawFrame(false),
      mbGetTrajectory(false),
      mbGetDensePointCloud(false)
{
    mInitCam2Ground_R.setIdentity();
    mInitCam2Ground_t.setZero();
    mTrans_cam2ground.setIdentity();
    mCam2Vehicle_R.setIdentity();
    mCam2Vehicle_t.setZero();
    mTrans_cam2vehicle.setIdentity();
    mCam2GroundNow_T.setIdentity();
    mVehicle2GroundNow_T.setIdentity();

    mCurrentPoseData.cam_pose_to_ground = Eigen::Matrix4f::Identity();
    mCurrentPoseData.vehicle_pose_to_ground = Eigen::Matrix4f::Identity();
    mCurrentPoseData.has_new_pose = false;
    mCurrentPoseData.timestamp = 0.0;

    mCurrentPointCloudData.all_points = MakeCloud();
    mCurrentPointCloudData.ref_points = MakeCloud();
    mCurrentPointCloudData.has_new_cloud = false;
    mCurrentPointCloudData.timestamp = 0.0;

    mCurrentPointCloudMappingData.map_points = MakeCloud();
    mCurrentPointCloudMappingData.has_new_map = false;
    mCurrentPointCloudMappingData.timestamp = 0.0;

    mCurrentDensePointCloudData.dense_points = MakeCloud();
    mCurrentDensePointCloudData.has_new_dense_cloud = false;
    mCurrentDensePointCloudData.timestamp = 0.0;

    mCurrentFrameData.has_new_frame = false;
    mCurrentFrameData.timestamp = 0.0;

    mCurrentTrajectoryData.has_new_trajectory = false;
    mCurrentTrajectoryData.timestamp = 0.0;

    cv::FileStorage settings(strSettingPath, cv::FileStorage::READ);
    if (settings.isOpened())
    {
        const cv::FileNode fps_node = settings["Camera.fps"];
        if (!fps_node.empty())
        {
            const double fps = static_cast<double>(fps_node);
            if (fps > 0.0)
                mT = 1000.0 / fps;
        }

        const cv::FileNode width_node = settings["Camera.width"];
        if (!width_node.empty())
            mImageWidth = static_cast<float>(width_node);

        const cv::FileNode height_node = settings["Camera.height"];
        if (!height_node.empty())
            mImageHeight = static_cast<float>(height_node);
    }

    if (mbUseDense)
        mpDenseMapper = new PointCloudMappingRGBD();
}

SlamDataProcess::~SlamDataProcess()
{
    RequestFinish();

    if (mpDenseMapper)
    {
        mpDenseMapper->shutdown();
        delete mpDenseMapper;
        mpDenseMapper = nullptr;
    }
}

void SlamDataProcess::Run()
{
    while (true)
    {
        if (CheckFinish())
            break;

        if (Stop())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            continue;
        }

        UpdateCurrentPose();
        UpdatePointCloud();
        UpdateDensePointCloud();
        UpdateCurrentFrame();
        UpdateTrajectory();

        const auto sleep_ms = std::max(1, static_cast<int>(mT));
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    SetFinish();
}

void SlamDataProcess::RequestFinish()
{
    std::lock_guard<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

void SlamDataProcess::RequestStop()
{
    std::lock_guard<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool SlamDataProcess::isFinished()
{
    std::lock_guard<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

bool SlamDataProcess::isStopped()
{
    std::lock_guard<std::mutex> lock(mMutexStop);
    return mbStopped;
}

void SlamDataProcess::Release()
{
    std::lock_guard<std::mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}

void SlamDataProcess::SetCurrentCameraPose(const cv::Mat& Tcw)
{
    if (Tcw.empty())
        return;

    std::lock_guard<std::mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbGetNewCamPose = true;
}

void SlamDataProcess::InsertKeyFrameForDense(KeyFrame* pKF, cv::Mat& color, cv::Mat& depth)
{
    if (!mbUseDense || !mpDenseMapper || !pKF || color.empty() || depth.empty())
        return;

    {
        std::lock_guard<std::mutex> lock(mMutexKeyFrameCache);
        if (mKeyFrameCache.find(pKF->mnId) != mKeyFrameCache.end())
            return;

        CachedKeyFrame cached;
        cached.color = color.clone();
        cached.depth = depth.clone();
        cached.timestamp = pKF->mTimeStamp;
        mKeyFrameCache[pKF->mnId] = cached;

        constexpr std::size_t kMaxCachedKeyFrames = 200;
        while (mKeyFrameCache.size() > kMaxCachedKeyFrames)
            mKeyFrameCache.erase(mKeyFrameCache.begin());
    }

    mpDenseMapper->insertKeyFrame(pKF, color, depth);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SlamDataProcess::GetGlobalDenseMap()
{
    if (!mpDenseMapper)
        return nullptr;

    return mpDenseMapper->getGlobalPointCloud();
}

void SlamDataProcess::UpdateCurrentPose()
{
    cv::Mat camera_pose;
    {
        std::lock_guard<std::mutex> lock(mMutexCamera);
        if (!mbGetNewCamPose || mCameraPose.empty())
            return;

        camera_pose = mCameraPose.clone();
        mbGetNewCamPose = false;
    }

    const Eigen::Matrix4f Tcw = CvMatToEigen4f(camera_pose);
    const Eigen::Matrix4f Twc = Tcw.inverse();

    mCam2GroundNow_T = mTrans_cam2ground * Twc;
    mVehicle2GroundNow_T = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();

    const double timestamp = NowSeconds();
    {
        std::lock_guard<std::mutex> lock(mMutexCamera);
        mCurrentPoseData.cam_pose_to_ground = mCam2GroundNow_T;
        mCurrentPoseData.vehicle_pose_to_ground = mVehicle2GroundNow_T;
        mCurrentPoseData.has_new_pose = true;
        mCurrentPoseData.timestamp = timestamp;
    }

    {
        std::lock_guard<std::mutex> lock(mMutexTrajectory);
        mCurrentTrajectoryData.camera_trajectory.push_back(mCam2GroundNow_T);
        mCurrentTrajectoryData.vehicle_trajectory.push_back(mVehicle2GroundNow_T);
        mCurrentTrajectoryData.has_new_trajectory = true;
        mCurrentTrajectoryData.timestamp = timestamp;
    }
}

void SlamDataProcess::UpdatePointCloud()
{
    if (!mpAtlas)
        return;

    std::vector<MapPoint*> all_map_points = mpAtlas->GetAllMapPoints();
    std::vector<MapPoint*> ref_map_points = mpAtlas->GetReferenceMapPoints();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud = MakeCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref_cloud = MakeCloud();

    for (MapPoint* map_point : all_map_points)
    {
        if (!map_point || map_point->isBad())
            continue;

        const Eigen::Vector3f pos = map_point->GetWorldPos();

        pcl::PointXYZRGB point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        point.r = 230;
        point.g = 230;
        point.b = 230;
        all_cloud->points.push_back(point);
    }

    for (MapPoint* map_point : ref_map_points)
    {
        if (!map_point || map_point->isBad())
            continue;

        const Eigen::Vector3f pos = map_point->GetWorldPos();

        pcl::PointXYZRGB point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        point.r = 255;
        point.g = 70;
        point.b = 70;
        ref_cloud->points.push_back(point);
    }

    FinalizeCloud(all_cloud);
    FinalizeCloud(ref_cloud);

    if (all_cloud->empty() && ref_cloud->empty())
        return;

    const double timestamp = NowSeconds();
    {
        std::lock_guard<std::mutex> lock(mMutexPointCloud);
        mCurrentPointCloudData.all_points = all_cloud;
        mCurrentPointCloudData.ref_points = ref_cloud;
        mCurrentPointCloudData.has_new_cloud = true;
        mCurrentPointCloudData.timestamp = timestamp;
    }

    {
        std::lock_guard<std::mutex> lock(mMutexPointCloudMapping);
        mCurrentPointCloudMappingData.map_points = all_cloud;
        mCurrentPointCloudMappingData.has_new_map = true;
        mCurrentPointCloudMappingData.timestamp = timestamp;
    }
}

void SlamDataProcess::UpdatePointCloudMapping()
{
    UpdatePointCloud();
}

void SlamDataProcess::UpdateDensePointCloud()
{
    if (!mbUseDense || !mpDenseMapper)
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud = mpDenseMapper->getGlobalPointCloud();
    if (!dense_cloud || dense_cloud->empty())
        return;

    std::lock_guard<std::mutex> lock(mMutexDenseMap);
    mCurrentDensePointCloudData.dense_points = dense_cloud;
    mCurrentDensePointCloudData.has_new_dense_cloud = true;
    mCurrentDensePointCloudData.timestamp = NowSeconds();
}

void SlamDataProcess::UpdateCurrentFrame()
{
    if (!mpFrameDrawer)
        return;

    const float image_scale = mpSystem ? mpSystem->GetImageScale() : 1.0f;
    cv::Mat frame = mpFrameDrawer->DrawFrame(image_scale);
    if (frame.empty())
        return;

    if (frame.channels() == 1)
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

    std::lock_guard<std::mutex> lock(mMutexFrame);
    mCurrentFrameData.current_frame = frame.clone();
    mCurrentFrameData.has_new_frame = true;
    mCurrentFrameData.timestamp = NowSeconds();
}

void SlamDataProcess::UpdateTrajectory()
{
}

SlamDataProcess::PoseData SlamDataProcess::GetCurrentPoseData()
{
    std::lock_guard<std::mutex> lock(mMutexCamera);
    PoseData data = mCurrentPoseData;
    mCurrentPoseData.has_new_pose = false;
    return data;
}

SlamDataProcess::PointCloudData SlamDataProcess::GetCurrentPointCloudData()
{
    std::lock_guard<std::mutex> lock(mMutexPointCloud);
    PointCloudData data = mCurrentPointCloudData;
    mCurrentPointCloudData.has_new_cloud = false;
    return data;
}

SlamDataProcess::PointCloudMappingData SlamDataProcess::GetCurrentPointCloudMappingData()
{
    std::lock_guard<std::mutex> lock(mMutexPointCloudMapping);
    PointCloudMappingData data = mCurrentPointCloudMappingData;
    mCurrentPointCloudMappingData.has_new_map = false;
    return data;
}

SlamDataProcess::DensePointCloudData SlamDataProcess::GetCurrentDensePointCloudData()
{
    std::lock_guard<std::mutex> lock(mMutexDenseMap);
    DensePointCloudData data = mCurrentDensePointCloudData;
    mCurrentDensePointCloudData.has_new_dense_cloud = false;
    return data;
}

SlamDataProcess::FrameData SlamDataProcess::GetCurrentFrameData()
{
    std::lock_guard<std::mutex> lock(mMutexFrame);
    FrameData data = mCurrentFrameData;
    data.current_frame = mCurrentFrameData.current_frame.clone();
    mCurrentFrameData.has_new_frame = false;
    return data;
}

SlamDataProcess::TrajectoryData SlamDataProcess::GetCurrentTrajectoryData()
{
    std::lock_guard<std::mutex> lock(mMutexTrajectory);
    TrajectoryData data = mCurrentTrajectoryData;
    mCurrentTrajectoryData.has_new_trajectory = false;
    return data;
}

PointClouds* SlamDataProcess::getPointClouds()
{
    return mpPointClouds;
}

Eigen::Matrix4f SlamDataProcess::getTransCam2Ground()
{
    return mTrans_cam2ground;
}

Eigen::Matrix4f SlamDataProcess::getTransCam2Vehicle()
{
    return mTrans_cam2vehicle;
}

bool SlamDataProcess::Stop()
{
    std::lock_guard<std::mutex> lock(mMutexStop);
    if (mbStopRequested && !mbStopped)
        mbStopped = true;
    return mbStopped;
}

bool SlamDataProcess::CheckFinish()
{
    std::lock_guard<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SlamDataProcess::SetFinish()
{
    std::lock_guard<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

} // namespace ORB_SLAM3
