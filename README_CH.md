<p align="center">
  <img src="docs/assets/demo_dense_mapping_09s_12s.gif" width="48%" alt="稠密建图预览" />
  <img src="docs/assets/demo_mapping_result_19s_24s.gif" width="48%" alt="建图结果预览" />
</p>

<h1 align="center">Dense-ORB-SLAM3</h1>

<p align="center">
  <a href="README_EN.md">English</a> ·
  <a href="README_CH.md">中文</a>
</p>

本仓库是 [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) 的修改版本，包含 ORB-SLAM3 核心库、RGB-D 稠密建图示例、相机配置、标定工具和点云处理工具。

如果需要通过 ROS 2 使用本仓库，请把同级 ROS 2 wrapper [`orbslam3_dense_ros2`](../src/orbslam3_dense_ros2/README.md) 放在同一个工作空间中。ROS 2 wrapper 会链接本仓库生成的库，并使用这里的词袋、相机配置、数据集和点云文件。远端仓库：[Pplll-pppl/dense_orbslam3_ros2](https://github.com/Pplll-pppl/dense_orbslam3_ros2)。

## 推荐目录

```text
<your_ws>/
  dense_orbslam3/              # 本仓库
  src/orbslam3_dense_ros2/     # 可选 ROS 2 wrapper
```

下面所有命令中的 `<your_ws>` 都需要替换成你自己的工作空间路径。

## 文件位置与作用

| 路径 | 作用 |
|------|------|
| `Vocabulary/ORBvoc.txt` | ORB-SLAM3 可执行文件和 ROS 2 wrapper 都需要的 ORB 词袋 |
| `Examples/RGB-D/TUM*.yaml` | TUM RGB-D 相机参数 |
| `Examples/RGB-D/RealSense_D435i.yaml` | RealSense D435i RGB-D 相机参数 |
| `Examples/RGB-D/associations/*.txt` | TUM RGB-D 时间戳关联文件 |
| `dataset/TUM-RGBD/<sequence>` | 可选的本地 TUM RGB-D 数据集 |
| `lib/libORB_SLAM3.so` | `./build.sh` 生成的核心库，也是 ROS 2 wrapper 链接的库 |
| `Examples/RGB-D/rgbd_tum_dense.cc` | 离线 RGB-D 稠密建图示例 |
| `Examples/RGB-D/*realsense_D435i*.cc` | RealSense RGB-D 示例 |
| `Examples/RGB-D/*.pcd` | 稠密点云输入或输出，可继续做 OctoMap 或尺寸计算 |
| `Examples/RGB-D/room_size_calc*.cc` | 房间尺寸和结构点云分析工具 |
| `Examples/Calibration/*` | RealSense 录制和 Kalibr 标定辅助文件 |
| `3rdParty/` | 如果使用本地构建依赖，这里存放 OpenCV、PCL、Pangolin 等 |

ROS 2 wrapper 通常会用到 `lib/libORB_SLAM3.so`、`Vocabulary/ORBvoc.txt`、对应的相机 `.yaml`、可选的数据集/association 文件，以及需要转 OctoMap 的 `.pcd` 文件。

---

## 修改内容

- 已在 **Ubuntu 24.04** 和 **ROS2 Jazzy** 上测试（OpenCV 4.9.0）
- C++ 标准从 C++11 更新到 C++17
- RGB-D ROS2 稠密建图依赖和崩溃修复说明：[docs/rgbd_dense_ros2_dependency_and_crash_fix.md](docs/rgbd_dense_ros2_dependency_and_crash_fix.md)

**Rectified** camera type

## 编译

克隆仓库：

```bash
git clone https://github.com/Pplll-pppl/Dense_OrbSlam3.git dense_orbslam3
```

安装原版 [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) / [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git) 所需依赖。此外，生成稠密点云需要 PCL 1.10 或兼容版本。PCL 可以和 OpenCV、Pangolin 放在同一类本地依赖目录中。

然后执行：

```bash
cd <your_ws>/dense_orbslam3
chmod +x build.sh
./build.sh
```

编译后会在 `lib/` 中生成 `libORB_SLAM3.so`，并在 `Examples/` 中生成可执行文件。

## 运行示例

本项目集成了稠密建图功能。离线 RGB-D 稠密建图主要使用 `rgbd_tum_dense.cc`。

### TUM RGB-D 稠密建图

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D

./rgbd_tum_dense \
  --tum <your_ws>/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:<your_ws>/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
  --voc <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param <your_ws>/dense_orbslam3/Examples/RGB-D/TUM1.yaml
```

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D

./rgbd_tum_dense \
  --tum <your_ws>/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg3_long_office_household:<your_ws>/dense_orbslam3/Examples/RGB-D/associations/fr3_room.txt \
  --voc <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param <your_ws>/dense_orbslam3/Examples/RGB-D/TUM3.yaml
```

### RealSense D435i

这些命令需要连接 RealSense D435i。

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D

./rgbd_realsense_D435i \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  <your_ws>/dense_orbslam3/trajectory_d435i.txt
```

RealSense D435i 稠密建图：

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D

./rgbd_dense_realsense_D435i \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  <your_ws>/dense_orbslam3/trajectory_d435i.txt \
  --dense
```

RealSense D435i RGB-D-Inertial 稠密建图：

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D-Inertial

./rgbd_dense_inertial_realsense_D435i \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D-Inertial/RealSense_D435i.yaml \
  <your_ws>/dense_orbslam3/trajectory_d435i.txt \
  --dense
```

## 标定

Kalibr 和 ROS1 是常见的相机标定工具。但在 Ubuntu 24.04 中，Kalibr 与 ROS1 的本地集成不够方便，所以可以使用 Docker 运行 Kalibr，并先用 ORB-SLAM3 提供的 `recorder_realsense_D435i.cc` 录制图像和 IMU 数据。

### Docker 中部署 Kalibr

安装 Docker 相关依赖：

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
```

添加 Docker GPG key：

```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/docker.gpg
```

添加 Docker 软件源：

```bash
echo "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

安装 Docker Engine：

```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
```

验证 Docker：

```bash
docker --version
```

拉取 Kalibr 镜像：

```bash
docker pull stereolabs/kalibr:latest
docker images
```

进入容器：

```bash
docker run -it --rm -v <your_ws>/dense_orbslam3:/data stereolabs/kalibr /bin/bash
```

### 相机内参

先录制相机数据：

```bash
./Examples/Calibration/recorder_realsense_D435i ./Examples/Calibration/recorder_visual
```

处理 IMU 数据：

```bash
python3 ./Examples/Calibration/python_scripts/process_imu.py ./Examples/Calibration/recorder_visual/
```

进入 Kalibr 容器：

```bash
docker run -it --rm -v <your_ws>/dense_orbslam3:/data stereolabs/kalibr /bin/bash
```

转换为 ROS bag：

```bash
kalibr_bagcreater --folder /data/Examples/Calibration/recorder_visual/ --output-bag /data/Examples/Calibration/recorder_visual.bag
```

开始标定：

```bash
kalibr_calibrate_cameras --bag /data/Examples/Calibration/recorder_visual.bag --topics /cam0/image_raw --models pinhole-radtan --target /data/Examples/Calibration/recorder_empty/april_6x6_80x80cm_larues.yaml
```

标定成功后即可得到相机内参。

注意：每次重新录制 bag 前，请删除 `recorder_visual/` 中的旧数据。

### 常见 Kalibr 问题和解决方案

错误记录：

```text
Cameras are not connected through mutual observations, please check the dataset. Maybe adjust the approx. sync. tolerance.
Traceback (most recent call last):
  File "/kalibr_workspace/devel/bin/kalibr_calibrate_cameras", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 447, in <module>
    main()
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 204, in main
    graph.plotGraph()
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py", line 311, in plotGraph
    edge_label=self.G.es["weight"],
KeyError: 'Attribute does not exist'
```

原因：`stereolabs/kalibr` 镜像中的

```text
/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py
```

对单相机标定不兼容。相关函数原本是：

```python
def isGraphConnected(self):
    #check if all vertices are connected
    return self.G.adhesion()
```

临时方案是在容器内把 `isGraphConnected` 改成：

```python
def isGraphConnected(self):

    if self.numCams == 1:
        # Since igaph 0.8, adhesion correctly returns 0 for the non-connected one cam case.
        # which evaluates to false later on. So we skip the check and return true in the one camera case.
        return True
    else:
        #check if all vertices are connected
        return self.G.adhesion()
    #returns the list of cam_ids that share common view with the specified cam_id

def getCamOverlaps(self, cam_id):
```

这个修改是在容器内完成的，所以只在当前容器生命周期内有效，容器停止后原代码会恢复。如果需要长期使用，建议基于修改后的代码构建自己的 Kalibr 镜像。

## RGB-D 可执行命令

下面命令从该目录运行：

```bash
cd <your_ws>/dense_orbslam3/Examples/RGB-D
```

### TUM 示例

```bash
./rgbd_tum \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
  <your_ws>/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room \
  <your_ws>/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt
```

```bash
./rgbd_tum_dense \
  --tum <your_ws>/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:<your_ws>/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
  --voc <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param <your_ws>/dense_orbslam3/Examples/RGB-D/TUM1.yaml
```

### RealSense 示例

这些命令需要连接 RealSense D435i。

```bash
./rgbd_realsense_D435i \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  <your_ws>/dense_orbslam3/trajectory_d435i.txt
```

```bash
./rgbd_dense_realsense_D435i \
  <your_ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <your_ws>/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  <your_ws>/dense_orbslam3/trajectory_d435i.txt \
  --dense
```

### 点云工具

```bash
./rotate_pcd
```

```bash
./light_pca_shear_project \
  <your_ws>/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd \
  <your_ws>/dense_orbslam3/Examples/RGB-D/light_pca_shear_project_corrected.pcd \
  <your_ws>/dense_orbslam3/Examples/RGB-D/light_pca_shear_project_projection.csv
```

### 房间尺寸计算

```bash
./room_size_calc_non_destructive_planes
```

也可以传入自己的点云路径：

```bash
./room_size_calc_non_destructive_planes /absolute/path/to/your_cloud.pcd
```
