#ifndef DATAPUB_H
#define DATAPUB_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "KeyFrame.h"
#include "PointCloudMappingRGBD.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <map>

using namespace std;

namespace ORB_SLAM3
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;
    class KeyFrame;
    class PointClouds;
    class Atlas;
    class PointCloudMappingRGBD;


    class SlamDataProcess
{
public:
    SlamDataProcess(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, 
                Tracking *pTracking, const string &strSettingPath, Atlas* pAtlas, 
                bool useDense = false);
    
    ~SlamDataProcess();

    // Data processing functions (no ROS2 dependencies)
    void Run();
    void RequestFinish();
    void RequestStop();
    bool isFinished();
    bool isStopped();
    void Release();
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    
    // Dense mapping functions
    void EnableDenseMapping(bool enable) { mbUseDense = enable; }
    bool IsDenseMappingEnabled() const { return mbUseDense; }
    void InsertKeyFrameForDense(KeyFrame* pKF, cv::Mat& color, cv::Mat& depth);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetGlobalDenseMap();

    void UpdateCurrentPose();
    void UpdatePointCloud();
    void UpdatePointCloudMapping();
    void UpdateDensePointCloud();
    void UpdateCurrentFrame();
    void UpdateTrajectory();

    // Data access interfaces for external publishers
    struct PoseData {
        Eigen::Matrix4f cam_pose_to_ground;
        Eigen::Matrix4f vehicle_pose_to_ground;
        bool has_new_pose;
        double timestamp;
    };

    struct DensePointCloudData {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points;
        bool has_new_dense_cloud;
        double timestamp;
    };

    struct PointCloudMappingData {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_points;
        bool has_new_map;
        double timestamp;
    };

    struct PointCloudData {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref_points;
        bool has_new_cloud;
        double timestamp;
    };
    
    struct FrameData {
        cv::Mat current_frame;
        bool has_new_frame;
        double timestamp;
    };
    
    struct TrajectoryData {
        std::vector<Eigen::Matrix4f> camera_trajectory;
        std::vector<Eigen::Matrix4f> vehicle_trajectory;
        bool has_new_trajectory;
        double timestamp;
    };

    // Public interface methods for data access
    PoseData GetCurrentPoseData();
    PointCloudData GetCurrentPointCloudData();
    PointCloudMappingData GetCurrentPointCloudMappingData();
    DensePointCloudData GetCurrentDensePointCloudData();
    FrameData GetCurrentFrameData();
    TrajectoryData GetCurrentTrajectoryData();
    
    PointClouds* getPointClouds();
    Eigen::Matrix4f getTransCam2Ground();
    Eigen::Matrix4f getTransCam2Vehicle();
    
private:
    bool Stop();
    bool CheckFinish();
    void SetFinish();

    
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    Atlas* mpAtlas;
    PointClouds* mpPointClouds;
    
    // Dense mapping
    bool mbUseDense;
    PointCloudMappingRGBD* mpDenseMapper;
    std::mutex mMutexDenseMap;
    DensePointCloudData mCurrentDensePointCloudData;
    
    // Keyframe cache for dense mapping
    struct CachedKeyFrame {
        cv::Mat color;
        cv::Mat depth;
        double timestamp;
    };
    std::map<unsigned long, CachedKeyFrame> mKeyFrameCache;
    std::mutex mMutexKeyFrameCache;

    // Configuration
    double mT;  // 1/fps in ms
    float mImageWidth, mImageHeight;

    // Thread control
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
    
    // Data synchronization
    std::mutex mMutexCamera;
    std::mutex mMutexPointCloud;
    std::mutex mMutexPointCloudMapping;
    std::mutex mMutexFrame;
    std::mutex mMutexTrajectory;
    
    // Current data
    cv::Mat mCameraPose;
    PoseData mCurrentPoseData;
    PointCloudData mCurrentPointCloudData;
    PointCloudMappingData mCurrentPointCloudMappingData;
    FrameData mCurrentFrameData;
    TrajectoryData mCurrentTrajectoryData;
    
    // Status flags
    bool mbGetNewCamPose;
    bool mbGetPointCloud;
    bool mbGetDrawFrame;
    bool mbGetTrajectory;
    bool mbGetDensePointCloud;
    
    // Transformation matrices
    Eigen::Matrix3f mInitCam2Ground_R;
    Eigen::Vector3f mInitCam2Ground_t;
    Eigen::Matrix4f mTrans_cam2ground;
    
    Eigen::Matrix3f mCam2Vehicle_R;
    Eigen::Vector3f mCam2Vehicle_t;
    Eigen::Matrix4f mTrans_cam2vehicle;

    Eigen::Matrix4f mCam2GroundNow_T;   
    Eigen::Matrix4f mVehicle2GroundNow_T;
};

}

#endif // DATAPUB_H