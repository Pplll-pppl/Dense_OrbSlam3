# Dense-ORB-SLAM3

语言：中文 | [English](README_EN.md)

本仓库是基于 [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) 修改的版本。

如果你希望将本项目封装到 ROS2 中使用，请参考 [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)。

---

## 修改内容

- 已在 **Ubuntu 24.04** 和 **ROS2 Jazzy**（OpenCV 4.9.0）环境下成功测试。
- 从 C++11 更新到 C++17。
- RGB-D ROS2 稠密建图依赖和崩溃修复说明：[docs/rgbd_dense_ros2_dependency_and_crash_fix.md](docs/rgbd_dense_ros2_dependency_and_crash_fix.md)。

**Rectified** 相机类型。

## 编译方法

克隆仓库：

```bash
git clone https://github.com/Pplll-pppl/Dense_OrbSlam3.git dense_orbslam3
```

安装与原版项目相同的依赖（[ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) / [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)）。
此外，生成稠密点云需要 PCL 1.10。你可以将它安装在与 OpenCV 和 Pangolin 相同的路径下。

然后执行：

```bash
cd path_to_your_ws/dense_orbslam3
chmod +x build.sh
./build.sh
```

编译完成后，会在 *lib* 文件夹下生成 **libORB_SLAM3.so**，并在 *Examples* 文件夹下生成可执行文件。

## 运行示例

- 本项目集成了稠密建图功能。目前可用的稠密建图示例是 **rgbd_tum_dense.cc**。

运行稠密建图示例：

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D # 请修改为你自己的工作空间路径

./rgbd_tum_dense --tum your_own_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:your_own_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt --voc your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt --param your_own_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml
# 请修改为你自己的工作空间路径
```

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D # 请修改为你自己的工作空间路径

./rgbd_tum_dense --tum your_own_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg3_long_office_household:your_own_ws/dense_orbslam3/Examples/RGB-D/associations/fr3_room.txt --voc your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt --param your_own_ws/dense_orbslam3/Examples/RGB-D/TUM3.yaml
# 请修改为你自己的工作空间路径
```

- 运行 RealSense D435i 示例：

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D # 请修改为你自己的工作空间路径
```

```bash
./rgbd_realsense_D435i your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt your_own_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml your_own_ws/dense_orbslam3/trajectory_d435i.txt
```

- 运行 RealSense D435i 稠密建图示例：

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D # 请修改为你自己的工作空间路径
```

```bash
./rgbd_dense_realsense_D435i your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt your_own_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml your_own_ws/dense_orbslam3/trajectory_d435i.txt --dense
```

- 运行 RealSense D435i 惯性稠密建图示例：

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D-Inertial # 请修改为你自己的工作空间路径
```

```bash
./rgbd_dense_inertial_realsense_D435i your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt your_own_ws/dense_orbslam3/Examples/RGB-D-Inertial/RealSense_D435i.yaml your_own_ws/dense_orbslam3/trajectory_d435i.txt --dense
```

### 标定

目前，Kalibr 和 ROS1 是两种常用的相机标定工具。
但在 Ubuntu 24.04 上，Kalibr 与 ROS1 的集成并不完善。
因此，本地编译 Kalibr 并在 ROS1 中使用可能会比较困难。
为了解决这个问题，可以在 Docker 中使用 Kalibr，并先使用 ORB-SLAM3 提供的脚本（`recorder_realsense_D435i.cc`）录制数据（图像和 IMU 数据）。

#### 在 Docker 中部署 Kalibr

首先，在宿主机系统中安装相关软件包：

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
```

然后添加 Docker 官方 GPG 密钥：

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

验证 Docker 是否安装成功：

```bash
docker --version
# 输出示例：Docker version 26.1.3, build b72abbb
```

从 Docker Hub 拉取 Kalibr 镜像：

```bash
docker pull stereolabs/kalibr:latest
```

查看 Docker 镜像：

```bash
docker images
```

创建并运行容器：

```bash
docker run -it --rm -v your_own_ws/dense_orbslam3:/data stereolabs/kalibr /bin/bash
```

#### 相机内参

首先录制相机数据：

```bash
./Examples/Calibration/recorder_realsense_D435i ./Examples/Calibration/recorder_visual
```

然后处理 IMU 数据：

```bash
python3 ./Examples/Calibration/python_scripts/process_imu.py ./Examples/Calibration/recorder_visual/
```

进入 Docker 容器：

```bash
docker run -it --rm -v your_own_ws/dense_orbslam3:/data stereolabs/kalibr /bin/bash
```

将数据转换为 ROS bag：

```bash
kalibr_bagcreater --folder /data/Examples/Calibration/recorder_visual/ --output-bag /data/Examples/Calibration/recorder_visual.bag
```

开始标定：

```bash
kalibr_calibrate_cameras --bag /data/Examples/Calibration/recorder_visual.bag --topics /cam0/image_raw --models pinhole-radtan --target /data/Examples/Calibration/recorder_empty/april_6x6_80x80cm_larues.yaml
```

如果标定成功，就可以获得相机内参。

**注意**：每次录制新的 bag 前，请记得删除 `recorder_visual` 文件夹中的原始数据 **!!!**

##### 可能出现的错误及解决方法

- 错误信息：

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

  - 原因：stereolabs 的 Kalibr 镜像源码中，文件 **"/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py"** 的部分代码不兼容单目相机标定。具体位置在 **isCameraConnected** 函数中，原始代码如下：

    ```python
    def isGraphConnected(self):
        #check if all vertices are connected
        return self.G.adhesion()
    ```

  - 解决方法：将 **"/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py"** 文件中的 **isGraphConnected** 函数修改为：

    ```python
    def isGraphConnected(self):

        if self.numCams == 1:
            # Since igraph 0.8, adhesion correctly returns 0 for the non-connected one cam case.
            #   which evaluates to false later on. So we skip the check and return true in the one camera case.
            return True
        else:
            #check if all vertices are connected
            return self.G.adhesion()
        #returns the list of cam_ids that share common view with the specified cam_id

    def getCamOverlaps(self, cam_id):
    ```

    **说明**：该操作是在容器内完成的，因此只是临时修改；容器停止后，原始代码会恢复。如果需要永久修复，建议基于修改后的代码构建自己的 Kalibr 镜像。

## RGB-D 可执行命令

请在以下目录中运行下面的命令：

```bash
cd your_own_ws/dense_orbslam3/Examples/RGB-D
```

### TUM 示例

```bash
./rgbd_tum \
  your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  your_own_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
  your_own_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room \
  your_own_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt
```

```bash
./rgbd_tum_dense \
  --tum your_own_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:your_own_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
  --voc your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param your_own_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml
```

### RealSense 示例

这些命令需要连接 RealSense D435i 设备。

```bash
./rgbd_realsense_D435i \
  your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  your_own_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  your_own_ws/dense_orbslam3/trajectory_d435i.txt
```

```bash
./rgbd_dense_realsense_D435i \
  your_own_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  your_own_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  your_own_ws/dense_orbslam3/trajectory_d435i.txt \
  --dense
```