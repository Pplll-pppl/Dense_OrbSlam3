# SlamDataProcess 与 ROS2 RGB-D 数据流说明

日期：2026-05-24

本文说明 `SlamDataProcess` 到底做了什么，以及整条 RGB-D 数据链路如何从相机 ROS2 消息进入 `orbslam3_dense_ros2`，再被转换成 OpenCV/PCL 数据，送入 ORB-SLAM3 内部类进行跟踪、建图、稠密点云生成，最后再发布回 ROS2。

相关源码：

```text
src/orbslam3_dense_ros2/src/orb_slam3_main.cpp
src/orbslam3_dense_ros2/src/ros2_slam_publisher.cpp
src/orbslam3_dense_ros2/include/orbslam3_dense_ros2/ros2_slam_publisher.h
dense_orbslam3/include/SlamDataProcess.h
dense_orbslam3/src/SlamDataProcess.cc
dense_orbslam3/include/System.h
dense_orbslam3/src/System.cc
dense_orbslam3/include/Tracking.h
dense_orbslam3/src/Tracking.cc
dense_orbslam3/include/PointCloudMappingRGBD.h
dense_orbslam3/src/PointCloudMappingRGBD.cc
```

## 1. 总体架构

整个系统可以分成四层：

```text
RealSense / ROS2 camera driver
  -> sensor_msgs::msg::Image RGB
  -> sensor_msgs::msg::Image Depth

orbslam3_dense_ros2
  -> Ros2SlamPublisher 节点
  -> message_filters 同步 RGB + Depth
  -> 手动转换 sensor_msgs::Image 到 cv::Mat

dense_orbslam3 / ORB-SLAM3
  -> System::TrackRGBD()
  -> Tracking::GrabImageRGBD()
  -> Frame 构造、ORB 特征、深度关联、Tracking::Track()
  -> Atlas / MapPoint / KeyFrame / LocalMapping / LoopClosing / Viewer

SlamDataProcess + ROS2 publishers
  -> 从 ORB-SLAM3 内部状态整理 pose、path、稀疏点云、tracking frame、稠密点云
  -> 发布 PoseStamped / Path / PointCloud2 / Image / TF
```

一句话概括：

`Ros2SlamPublisher` 负责 ROS2 接入和发布，`System/Tracking` 负责真正的 SLAM，`SlamDataProcess` 负责把 ORB-SLAM3 内部结果整理成 ROS2 publisher 线程能安全读取的数据缓存。

## 2. SlamDataProcess 到底做了什么

`SlamDataProcess` 不是 ORB-SLAM3 的跟踪核心，也不负责提取 ORB 特征。它更像一个“SLAM 内部数据到 ROS2 输出数据的中间整理层”。

它主要做六件事：

1. 保存 ORB-SLAM3 内部对象指针。
2. 接收当前相机位姿，并转换成 ROS2 要发布的坐标系位姿。
3. 从 Atlas 读取稀疏地图点，生成 PCL 点云。
4. 从 FrameDrawer 读取当前跟踪图像。
5. 维护相机轨迹和 vehicle 轨迹缓存。
6. 如果启用 dense mapping，把关键帧 RGB-D 数据送进 `PointCloudMappingRGBD`，并周期性取出全局稠密点云。

### 2.1 保存 ORB-SLAM3 内部对象

构造函数接收这些对象：

```cpp
SlamDataProcess(System* pSystem,
                FrameDrawer* pFrameDrawer,
                MapDrawer* pMapDrawer,
                Tracking* pTracking,
                const string& strSettingPath,
                Atlas* pAtlas,
                bool useDense);
```

这些对象来自 `Ros2SlamPublisher` 初始化好的 `System`：

```cpp
FrameDrawer* frame_drawer = slam_system_->GetFrameDrawer();
MapDrawer* map_drawer = slam_system_->GetMapDrawer();
Tracking* tracker = slam_system_->GetTracker();
Atlas* atlas = slam_system_->GetAtlas();

slam_data_processor_ = new SlamDataProcess(
    slam_system_, frame_drawer, map_drawer, tracker, settings_file, atlas, use_dense);
```

各对象的用途：

```text
System        系统总入口，提供 image scale 等全局信息
FrameDrawer   生成当前 tracking frame 可视化图
MapDrawer     当前代码中保存了指针，但主要发布流程里使用较少
Tracking      获取 last keyframe，用于稠密建图
Atlas         获取所有 MapPoint / Reference MapPoint，用于稀疏点云
```

### 2.2 读取配置

`SlamDataProcess` 会读取 settings yaml：

```cpp
Camera.fps
Camera.width
Camera.height
```

其中 `Camera.fps` 用来计算自己的循环周期：

```cpp
mT = 1000.0 / fps;
```

`Run()` 线程每次更新数据后睡眠约 `mT` 毫秒。

### 2.3 自己维护一个数据整理线程

`Ros2SlamPublisher::Run()` 会启动：

```cpp
std::thread slam_data_thread(&SlamDataProcess::Run, slam_data_processor_);
```

`SlamDataProcess::Run()` 循环做：

```cpp
UpdateCurrentPose();
UpdatePointCloud();
UpdateDensePointCloud();
UpdateCurrentFrame();
UpdateTrajectory();
```

也就是说，它不是每收到一帧就同步发布，而是用自己的周期不断从 ORB-SLAM3 内部对象中取最新状态，整理成缓存。

### 2.4 接收当前相机位姿

RGB-D 回调里，ORB-SLAM3 跟踪后返回 `Sophus::SE3f pose`：

```cpp
Sophus::SE3f pose = slam_system_->TrackRGBD(rgb_image, depth_image, timestamp);
```

随后 wrapper 把 `Sophus::SE3f` 转成 `cv::Mat`，交给 `SlamDataProcess`：

```cpp
slam_data_processor_->SetCurrentCameraPose(cv_pose);
```

`SlamDataProcess::SetCurrentCameraPose()` 只是加锁保存：

```cpp
mCameraPose = Tcw.clone();
mbGetNewCamPose = true;
```

这里保存的是 ORB-SLAM3 返回的 `Tcw`，即 world 到 camera 的变换。

### 2.5 把 Tcw 转成要发布的 Twc

`UpdateCurrentPose()` 中：

```cpp
const Eigen::Matrix4f Tcw = CvMatToEigen4f(camera_pose);
const Eigen::Matrix4f Twc = Tcw.inverse();
```

然后转换到发布用的 ground/map 坐标：

```cpp
mCam2GroundNow_T = mTrans_cam2ground * Twc;
mVehicle2GroundNow_T = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
```

当前默认情况下：

```text
mTrans_cam2ground   = Identity
mTrans_cam2vehicle  = Identity
```

所以现在基本等价于：

```text
camera pose in map = inverse(Tcw)
vehicle pose in map = camera pose in map
```

然后缓存到：

```cpp
mCurrentPoseData.cam_pose_to_ground
mCurrentPoseData.vehicle_pose_to_ground
```

同时追加轨迹：

```cpp
mCurrentTrajectoryData.camera_trajectory.push_back(mCam2GroundNow_T);
mCurrentTrajectoryData.vehicle_trajectory.push_back(mVehicle2GroundNow_T);
```

### 2.6 从 Atlas 生成稀疏点云

`UpdatePointCloud()` 从 ORB-SLAM3 的 `Atlas` 取地图点：

```cpp
std::vector<MapPoint*> all_map_points = mpAtlas->GetAllMapPoints();
std::vector<MapPoint*> ref_map_points = mpAtlas->GetReferenceMapPoints();
```

然后把每个 `MapPoint` 的世界坐标转成 PCL 点：

```cpp
const Eigen::Vector3f pos = map_point->GetWorldPos();

pcl::PointXYZRGB point;
point.x = pos.x();
point.y = pos.y();
point.z = pos.z();
```

颜色约定：

```text
all_points: 灰白色 r=230, g=230, b=230
ref_points: 红色   r=255, g=70,  b=70
```

输出缓存：

```cpp
mCurrentPointCloudData.all_points
mCurrentPointCloudData.ref_points
```

这些最终会被 ROS2 publisher 线程发布为：

```text
/orb_slam3/all_points
/orb_slam3/ref_points
```

类型：

```text
sensor_msgs::msg::PointCloud2
```

### 2.7 获取当前 tracking frame

`UpdateCurrentFrame()` 调用：

```cpp
cv::Mat frame = mpFrameDrawer->DrawFrame(image_scale);
```

如果是灰度图，会转成 BGR：

```cpp
if (frame.channels() == 1)
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
```

然后缓存到：

```cpp
mCurrentFrameData.current_frame
```

最终发布到：

```text
/orb_slam3/tracking_frame
```

类型：

```text
sensor_msgs::msg::Image
```

### 2.8 管理稠密建图

如果 `use_dense=true`，构造函数里会创建：

```cpp
mpDenseMapper = new PointCloudMappingRGBD();
```

RGB-D 回调每次跟踪后，会取最后一个关键帧：

```cpp
KeyFrame* kf = slam_system_->GetTracker()->GetLastKeyFrame();
```

然后调用：

```cpp
slam_data_processor_->InsertKeyFrameForDense(kf, rgb_image, depth_image);
```

`SlamDataProcess::InsertKeyFrameForDense()` 做两件事：

1. 用 keyframe id 去重，避免同一个关键帧重复插入。
2. 调用 `mpDenseMapper->insertKeyFrame(pKF, color, depth)` 把关键帧和 RGB-D 图像交给稠密建图线程。

`PointCloudMappingRGBD` 内部有自己的线程和队列：

```text
insertKeyFrame()
  -> 把 KeyFrameData 放入 mqNewKeyFrames
  -> 通知 mapping thread

PointCloudMappingRGBD::Run()
  -> 等待新关键帧
  -> 根据 RGB-D + fx/fy/cx/cy 反投影生成相机坐标点云
  -> 用 KeyFrame 的 Twc 变换到世界坐标
  -> VoxelGrid 降采样
  -> StatisticalOutlierRemoval 去离群点
  -> 合并到 globalMap
```

`SlamDataProcess::UpdateDensePointCloud()` 周期性取：

```cpp
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud =
    mpDenseMapper->getGlobalPointCloud();
```

然后缓存到：

```cpp
mCurrentDensePointCloudData.dense_points
```

最终由 ROS2 publisher 线程发布。

## 3. 从相机数据到 ORB-SLAM3 的完整输入流程

### 3.1 main 函数解析参数

入口：

```text
src/orbslam3_dense_ros2/src/orb_slam3_main.cpp
```

命令行参数顺序：

```text
1. vocabulary path
2. settings yaml path
3. use_dense
4. rgb_topic
5. depth_topic
6. use_viewer
```

默认值：

```cpp
voc_file = "/home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt";
settings_file = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml";
use_dense = true;
use_viewer = true;
rgb_topic = "/camera/camera/color/image_raw";
depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";
```

然后创建节点：

```cpp
g_node = std::make_shared<ORB_SLAM3::Ros2SlamPublisher>(
    voc_file, settings_file, use_dense, use_viewer, rgb_topic, depth_topic);
```

### 3.2 Ros2SlamPublisher 构造 ORB-SLAM3 System

构造函数里创建：

```cpp
slam_system_ = new System(voc_file, settings_file, System::RGBD, use_viewer);
```

这一步发生在 `dense_orbslam3/src/System.cc` 内部，主要做：

```text
加载 settings yaml
加载 ORB vocabulary
创建 KeyFrameDatabase
创建 Atlas
创建 Tracking
创建 LocalMapping
创建 LoopClosing
按 use_viewer 创建 Viewer / Pangolin 线程
```

从这一步开始，ORB-SLAM3 内部的核心模块已经准备好。

### 3.3 创建 ROS2 publishers

`Ros2SlamPublisher` 创建这些发布者：

```text
/orb_slam3/camera_pose         geometry_msgs::msg::PoseStamped
/orb_slam3/vehicle_pose        geometry_msgs::msg::PoseStamped
/orb_slam3/camera_trajectory   nav_msgs::msg::Path
/orb_slam3/vehicle_trajectory  nav_msgs::msg::Path
/orb_slam3/all_points          sensor_msgs::msg::PointCloud2
/orb_slam3/ref_points          sensor_msgs::msg::PointCloud2
/orb_slam3/dense_points        sensor_msgs::msg::PointCloud2
/orb_slam3/tracking_frame      sensor_msgs::msg::Image
```

还创建两个 TF broadcaster：

```text
tf_broadcaster_
map_to_slam_map_broadcaster_
```

### 3.4 订阅并同步 RGB-D 图像

订阅器：

```cpp
rgb_sub_.subscribe(this, rgb_topic, image_qos.get_rmw_qos_profile());
depth_sub_.subscribe(this, depth_topic, image_qos.get_rmw_qos_profile());
```

同步器：

```cpp
sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), rgb_sub_, depth_sub_);
```

回调：

```cpp
sync_->registerCallback(
    std::bind(&Ros2SlamPublisher::rgbd_callback, this,
              std::placeholders::_1, std::placeholders::_2));
```

这里用的是 `message_filters::sync_policies::ApproximateTime`，也就是近似时间同步。RGB 图像和 depth 图像时间戳足够接近时，才会一起进入 `rgbd_callback()`。

## 4. rgbd_callback 内部发生了什么

回调入口：

```cpp
void Ros2SlamPublisher::rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
```

### 4.1 ROS2 Image 转 cv::Mat

RGB：

```cpp
cv::Mat rgb_image = ImageMsgToBgrMat(*rgb_msg);
```

支持：

```text
bgr8  -> CV_8UC3
rgb8  -> CV_8UC3, RGB 转 BGR
bgra8 -> CV_8UC3
rgba8 -> CV_8UC3
mono8 -> CV_8UC3
```

Depth：

```cpp
cv::Mat depth_image = ImageMsgToDepthMat(*depth_msg);
```

支持：

```text
16UC1  -> CV_16UC1
mono16 -> CV_16UC1
32FC1  -> 乘 1000 转成 CV_16UC1 毫米深度
```

为什么这里不用 `cv_bridge`：

```text
避免 ROS2 系统 cv_bridge 把系统 OpenCV 链回进程。
当前方式直接使用 sensor_msgs::msg::Image 的 data/step/encoding 字段构造 cv::Mat。
```

### 4.2 生成时间戳

```cpp
double timestamp = rgb_msg->header.stamp.sec +
                   rgb_msg->header.stamp.nanosec * 1e-9;
```

这个时间戳会传入 ORB-SLAM3，用作当前帧时间。

### 4.3 调用 ORB-SLAM3 System::TrackRGBD

```cpp
Sophus::SE3f pose = slam_system_->TrackRGBD(
    rgb_image, depth_image, timestamp);
```

这是从 ROS2 wrapper 进入 ORB-SLAM3 的关键调用。

## 5. ORB-SLAM3 内部处理链路

### 5.1 System::TrackRGBD

文件：

```text
dense_orbslam3/src/System.cc
```

函数：

```cpp
Sophus::SE3f System::TrackRGBD(
    const cv::Mat& im,
    const cv::Mat& depthmap,
    const double& timestamp,
    const vector<IMU::Point>& vImuMeas,
    string filename)
```

主要步骤：

1. 检查传感器类型必须是 `RGBD` 或 `IMU_RGBD`。
2. clone RGB 和 depth，必要时按 settings resize。
3. 检查 localization mode 是否切换。
4. 检查 reset 请求。
5. 如果是 `IMU_RGBD`，先喂 IMU 数据。
6. 调用：

```cpp
Sophus::SE3f Tcw =
    mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);
```

7. 更新 `System` 内部状态：

```cpp
mTrackingState = mpTracker->mState;
mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
```

8. 返回 `Tcw`。

### 5.2 Tracking::GrabImageRGBD

文件：

```text
dense_orbslam3/src/Tracking.cc
```

函数：

```cpp
Sophus::SE3f Tracking::GrabImageRGBD(
    const cv::Mat& imRGB,
    const cv::Mat& imD,
    const double& timestamp,
    string filename)
```

主要步骤：

1. 保存 RGB 图：

```cpp
mImGray = imRGB;
cv::Mat imDepth = imD;
```

2. 如果是三通道或四通道图，转灰度：

```cpp
if (mImGray.channels() == 3)
{
    if (mbRGB)
        cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
        cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
}
```

当前 wrapper 传入的是 BGR，所以是否用 `COLOR_BGR2GRAY` 取决于 settings 里的 `mbRGB` 配置。

3. 深度图转 `CV_32F`：

```cpp
if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);
```

`mDepthMapFactor` 来自 RGB-D settings。它负责把输入深度值转换成 ORB-SLAM3 内部使用的尺度。对 RealSense 常见 `16UC1` 毫米深度，settings 通常会配置对应 factor，使内部深度变成以米为单位或 ORB-SLAM3 期望的浮点深度。

4. 构造当前帧：

```cpp
mCurrentFrame = Frame(
    mImGray,
    imDepth,
    timestamp,
    mpORBextractorLeft,
    mpORBVocabulary,
    mK,
    mDistCoef,
    mbf,
    mThDepth,
    mpCamera);
```

这个 `Frame` 构造过程中会做 ORB 特征提取、深度关联、坐标准备等工作。

5. 调用真正的跟踪：

```cpp
Track();
```

`Track()` 是 ORB-SLAM3 的核心流程，会根据当前状态执行：

```text
初始化
运动模型跟踪
参考关键帧跟踪
局部地图跟踪
决定是否插入新关键帧
把关键帧交给 LocalMapping
更新当前帧 pose
```

6. 返回当前帧 pose：

```cpp
return mCurrentFrame.GetPose();
```

返回的是 `Tcw`，也就是 world 到 camera 的位姿。

## 6. ORB-SLAM3 结果如何回到 ROS2

ORB-SLAM3 跟踪结束后，`rgbd_callback()` 继续执行。

### 6.1 位姿进入 SlamDataProcess

```cpp
if (!pose.matrix().isZero()) {
    cv::Mat cv_pose = cv::Mat::eye(4, 4, CV_32F);
    Eigen::Matrix4f eigen_pose = pose.matrix();
    ...
    slam_data_processor_->SetCurrentCameraPose(cv_pose);
}
```

然后 `SlamDataProcess::UpdateCurrentPose()` 在自己的线程里把 `Tcw` 转成 `Twc`，生成：

```text
PoseData.cam_pose_to_ground
PoseData.vehicle_pose_to_ground
TrajectoryData.camera_trajectory
TrajectoryData.vehicle_trajectory
```

### 6.2 关键帧进入稠密建图

如果启用 dense：

```cpp
KeyFrame* kf = slam_system_->GetTracker()->GetLastKeyFrame();
if (kf) {
    slam_data_processor_->InsertKeyFrameForDense(kf, rgb_image, depth_image);
}
```

`InsertKeyFrameForDense()` 会去重并把 RGB-D 和关键帧交给 `PointCloudMappingRGBD`。

需要注意：

```text
这里不是每一帧都生成稠密点云，而是拿 Tracking 当前 last keyframe 来做关键帧级稠密建图。
```

## 7. ROS2 发布线程如何工作

`Ros2SlamPublisher::Run()` 启动多个线程：

```cpp
pose_thread_ = std::thread(&Ros2SlamPublisher::PublishPoseData, this);
pointcloud_thread_ = std::thread(&Ros2SlamPublisher::PublishPointCloudData, this);
dense_pointcloud_thread_ = std::thread(&Ros2SlamPublisher::PublishDensePointCloudData, this);
frame_thread_ = std::thread(&Ros2SlamPublisher::PublishFrameData, this);
trajectory_thread_ = std::thread(&Ros2SlamPublisher::PublishTrajectoryData, this);
```

这些线程不直接访问 ORB-SLAM3 内部对象，而是从 `SlamDataProcess` 拿整理好的数据：

```text
GetCurrentPoseData()
GetCurrentPointCloudData()
GetCurrentDensePointCloudData()
GetCurrentFrameData()
GetCurrentTrajectoryData()
```

这种设计的好处：

```text
ORB-SLAM3 跟踪线程、SlamDataProcess 整理线程、ROS2 发布线程彼此解耦。
发布慢了不会直接阻塞 TrackRGBD。
读取 ORB-SLAM3 内部状态集中在 SlamDataProcess，便于加锁和维护。
```

### 7.1 发布 pose 和 TF

线程：

```cpp
PublishPoseData()
```

数据来源：

```cpp
SlamDataProcess::PoseData pose_data =
    slam_data_processor_->GetCurrentPoseData();
```

发布：

```text
/orb_slam3/camera_pose   geometry_msgs::msg::PoseStamped
/orb_slam3/vehicle_pose  geometry_msgs::msg::PoseStamped
```

同时发布 TF：

```text
map -> camera
map -> base_link
```

默认 frame：

```text
ground_frame_  = "map"
camera_frame_  = "camera"
vehicle_frame_ = "base_link"
```

### 7.2 发布稀疏点云

线程：

```cpp
PublishPointCloudData()
```

数据来源：

```cpp
SlamDataProcess::PointCloudData cloud_data =
    slam_data_processor_->GetCurrentPointCloudData();
```

发布：

```text
/orb_slam3/all_points   sensor_msgs::msg::PointCloud2
/orb_slam3/ref_points   sensor_msgs::msg::PointCloud2
```

转换函数：

```cpp
PCLToROS(cloud_data.all_points, ground_frame_)
PCLToROS(cloud_data.ref_points, ground_frame_)
```

### 7.3 发布稠密点云

线程：

```cpp
PublishDensePointCloudData()
```

发布频率固定为：

```cpp
rclcpp::Rate rate(2.0);
```

也就是 2 Hz，比位姿/稀疏点云低，避免大点云发布太频繁。

发布：

```text
/orb_slam3/dense_points   sensor_msgs::msg::PointCloud2
```

注意：当前 `octomap_converter.cpp` 里订阅的是：

```text
/orb_slam3/dense_pointcloud
```

而当前 publisher 实际发布的是：

```text
/orb_slam3/dense_points
```

如果要让 `octomap_converter` 直接吃实时稠密点云，需要统一这两个话题名，或者用 ROS2 remap。

### 7.4 发布 tracking frame

线程：

```cpp
PublishFrameData()
```

发布：

```text
/orb_slam3/tracking_frame   sensor_msgs::msg::Image
```

转换：

```cpp
MatToImageMsg(frame_data.current_frame, camera_frame_)
```

输出 encoding：

```text
bgr8
```

### 7.5 发布轨迹

线程：

```cpp
PublishTrajectoryData()
```

发布：

```text
/orb_slam3/camera_trajectory    nav_msgs::msg::Path
/orb_slam3/vehicle_trajectory   nav_msgs::msg::Path
```

## 8. PCL 点云到 ROS2 PointCloud2 的转换

函数：

```cpp
sensor_msgs::msg::PointCloud2 Ros2SlamPublisher::PCLToROS(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
    const std::string& frame_id)
```

输出字段：

```text
field  offset  datatype
x      0       FLOAT32
y      4       FLOAT32
z      8       FLOAT32
rgb    12      FLOAT32
```

点大小：

```cpp
cloud_msg.point_step = 16;
cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
```

颜色打包：

```cpp
const uint32_t rgb_packed =
    (static_cast<uint32_t>(point.r) << 16) |
    (static_cast<uint32_t>(point.g) << 8) |
    static_cast<uint32_t>(point.b);

float rgb_float = 0.0f;
std::memcpy(&rgb_float, &rgb_packed, sizeof(rgb_float));
```

这里沿用了 PCL/ROS 中常见的 `rgb` float packed 布局。

## 9. 线程关系图

当前运行后主要有这些线程：

```text
ROS2 executor thread
  -> rclcpp::spin()
  -> 触发 rgbd_callback()
  -> 调用 System::TrackRGBD()

ORB-SLAM3 internal threads
  -> LocalMapping
  -> LoopClosing
  -> Viewer/Pangolin, if use_viewer=true

SlamDataProcess thread
  -> 周期性整理 pose / sparse cloud / dense cloud / frame / trajectory

ROS2 publisher threads
  -> PublishPoseData()
  -> PublishPointCloudData()
  -> PublishDensePointCloudData()
  -> PublishFrameData()
  -> PublishTrajectoryData()

PointCloudMappingRGBD thread, if use_dense=true
  -> 等待关键帧 RGB-D
  -> 反投影生成点云
  -> 滤波
  -> 合并 global dense map
```

这个结构里，`SlamDataProcess` 是 ORB-SLAM3 内部状态和 ROS2 发布线程之间的缓冲层。

## 10. 数据方向总结

输入方向：

```text
RealSense
  -> /camera/camera/color/image_raw
  -> /camera/camera/aligned_depth_to_color/image_raw
  -> message_filters ApproximateTime
  -> Ros2SlamPublisher::rgbd_callback()
  -> ImageMsgToBgrMat(), ImageMsgToDepthMat()
  -> System::TrackRGBD()
  -> Tracking::GrabImageRGBD()
  -> Frame()
  -> Tracking::Track()
  -> Atlas / MapPoint / KeyFrame 更新
```

输出方向：

```text
Tracking 返回 Tcw
  -> Ros2SlamPublisher::rgbd_callback()
  -> SlamDataProcess::SetCurrentCameraPose()
  -> SlamDataProcess::UpdateCurrentPose()
  -> /orb_slam3/camera_pose
  -> /orb_slam3/vehicle_pose
  -> TF map->camera, map->base_link

Atlas MapPoints
  -> SlamDataProcess::UpdatePointCloud()
  -> PCLToROS()
  -> /orb_slam3/all_points
  -> /orb_slam3/ref_points

FrameDrawer
  -> SlamDataProcess::UpdateCurrentFrame()
  -> MatToImageMsg()
  -> /orb_slam3/tracking_frame

Last KeyFrame + RGB-D
  -> SlamDataProcess::InsertKeyFrameForDense()
  -> PointCloudMappingRGBD::insertKeyFrame()
  -> PointCloudMappingRGBD::Run()
  -> SlamDataProcess::UpdateDensePointCloud()
  -> PCLToROS()
  -> /orb_slam3/dense_points

Pose history
  -> SlamDataProcess::UpdateCurrentPose()
  -> TrajectoryToPath()
  -> /orb_slam3/camera_trajectory
  -> /orb_slam3/vehicle_trajectory
```

## 11. SlamDataProcess 的一句话定义

`SlamDataProcess` 是一个面向 ROS2 输出的 SLAM 数据聚合器：它不做核心 SLAM 跟踪，而是从 ORB-SLAM3 的 `System / Tracking / Atlas / FrameDrawer / PointCloudMappingRGBD` 中取结果，统一转换成 pose、path、sparse cloud、dense cloud、tracking frame 等缓存，再让 ROS2 publisher 线程安全地发布出去。

