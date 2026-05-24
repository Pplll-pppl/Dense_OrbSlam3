# RGB-D 稠密建图 ROS2 崩溃与依赖统一修改记录

日期：2026-05-17

本文记录这次排查和修复的内容：`orbslam3_dense_ros2` 在接收 RealSense RGB-D 数据后，Pangolin 短暂弹出、能看到部分稀疏点云，随后发生 segmentation fault。最终处理方向是两个：

- 统一 OpenCV/PCL 动态库版本，避免同一进程里混用系统库和 `dense_orbslam3/3rdParty` 下的本地库。
- 修复 RGB-D 稠密建图线程里 PCL 点云过滤与点云元数据维护的风险点。

现在核心运行链路已经统一为：

```text
orb_slam3_main
  -> libORB_SLAM3.so
      -> OpenCV 4.9: dense_orbslam3/3rdParty/Opencv/build-full/lib
      -> PCL 1.15:   dense_orbslam3/3rdParty/pcl/pcl_install/lib
      -> Pangolin:   dense_orbslam3/3rdParty/Pangolin/build/src
      -> DBoW2:      使用同一套 OpenCV 4.9 编译
```

## 1. 原始现象

运行命令类似：

```bash
ros2 run orbslam3_dense_ros2 orb_slam3_main \
  /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  true \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  true
```

程序能完成初始化，并且已经收到第一帧 RGB-D：

```text
Received synchronized RGB-D frame #1: rgb=rgb8 depth=16UC1
Starting the Viewer
First KF:0; Map init KF:0
New Map created with 948 points
Assignment with new_width equal to 0,setting width to size of the cloud and height to 1
[ros2run]: Segmentation fault
```

这个现象说明：

- 不是 vocabulary 加载阶段崩溃。
- 不是 ORB-SLAM3 `System` 创建阶段崩溃。
- Pangolin 能短暂显示，说明 Viewer 本身可以启动。
- 崩溃发生在 RGB-D 数据进入后，SLAM 创建地图、关键帧和点云处理开始联动的阶段。

因此排查重点放在运行时动态库混用、ROS2 图像/点云转换、PCL 过滤和点云发布这几处。

## 2. 根因分析

### 2.1 OpenCV 4.6 和 OpenCV 4.9 混用

修复前通过 `ldd` 和 `objdump` 看到过同一个进程里同时加载：

```text
libopencv_core.so.406 => /lib/x86_64-linux-gnu/libopencv_core.so.406
libopencv_imgproc.so.406 => /lib/x86_64-linux-gnu/libopencv_imgproc.so.406
libopencv_core.so.409 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib/libopencv_core.so.409
```

这很危险。`cv::Mat`、特征描述子、图像 buffer、OpenCV 模块内部符号和 allocator 都可能跨库边界传递。即使源码 API 看起来兼容，ABI、依赖图和符号解析也不一定兼容。

### 2.2 PCL 1.14 和 PCL 1.15 混用

`libORB_SLAM3.so` 一部分链路使用本地 PCL 1.15：

```text
/home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/lib
```

但 ROS2 包的旧 CMake cache 和旧 link.txt 中出现过系统 PCL 1.14：

```text
/usr/include/pcl-1.14
/usr/lib/x86_64-linux-gnu/libpcl_*.so
```

PCL 类型大量依赖模板、Eigen 对齐、编译选项和模块 ABI。`pcl::PointCloud<pcl::PointXYZRGB>` 这类对象如果在一个 PCL 版本里创建，又被另一个 PCL 版本相关的代码处理，可能出现非常隐蔽的崩溃。

### 2.3 ROS2 便利包带回系统库

原 ROS2 wrapper 依赖过这些包：

```xml
cv_bridge
image_transport
pcl_conversions
pcl_ros
octomap_ros
```

这些包来自 ROS2 Jazzy 的系统安装，通常是按系统 OpenCV/PCL 编译的。只要它们进入 CMake link interface，就可能把系统 OpenCV 4.6 或系统 PCL 1.14 带回进程。

本项目的主链路其实不必须依赖它们。图像可以直接从 `sensor_msgs::msg::Image` 转 `cv::Mat`，点云也可以直接读写 `sensor_msgs::msg::PointCloud2` 的字段布局。

### 2.4 PCL 过滤存在输入输出别名风险

稠密建图里原来存在类似这种模式：

```cpp
sor.setInputCloud(filtered);
sor.filter(*filtered);
```

也就是输入和输出是同一个点云对象。对 PCL 过滤器来说，这种 in-place 写法不总是安全。过滤器内部可能一边读取 input cloud 的索引和字段，一边重写 output cloud 的内容和尺寸。点数、width、height 或 allocator 状态稍微变化，就可能触发异常行为。

同时日志里出现：

```text
Assignment with new_width equal to 0,setting width to size of the cloud and height to 1
```

也提示点云的 `width` / `height` 元数据不够稳定。PCL 点云不是只有 `points` 数组，还依赖：

```text
cloud->width
cloud->height
cloud->is_dense
```

对非组织点云，安全约定一般是：

```text
height = 1
width = points.size()
```

## 3. 具体修改

### 3.1 ORB-SLAM3 主库固定使用本地 PCL 1.15

文件：

```text
dense_orbslam3/CMakeLists.txt
```

设置本地 PCL 根目录：

```cmake
set(PCL_INSTALL_DIR /home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install)
```

头文件固定到本地 PCL 1.15：

```cmake
set(PCL_INCLUDE_DIRS
    ${PCL_INSTALL_DIR}/include/pcl-1.15
    ${PCL_INSTALL_DIR}/include
    /usr/include/vtk-9.1
    /usr/include/eigen3
    /usr/include/freetype2
)
```

库目录只保留本地 PCL lib：

```cmake
set(PCL_LIBRARY_DIRS
        ${PCL_INSTALL_DIR}/lib
)
```

最关键的是 PCL 库改成绝对路径：

```cmake
set(PCL_LIBRARIES
    ${PCL_INSTALL_DIR}/lib/libpcl_common.so
    ${PCL_INSTALL_DIR}/lib/libpcl_filters.so
    ${PCL_INSTALL_DIR}/lib/libpcl_visualization.so
    ${PCL_INSTALL_DIR}/lib/libpcl_io.so
    ${PCL_INSTALL_DIR}/lib/libpcl_kdtree.so
    ${PCL_INSTALL_DIR}/lib/libpcl_search.so
    ${PCL_INSTALL_DIR}/lib/libpcl_segmentation.so
    ${PCL_INSTALL_DIR}/lib/libpcl_sample_consensus.so
    ${PCL_INSTALL_DIR}/lib/libpcl_surface.so
    pthread
    atomic
    vtkCommonCore-9.1
    vtkCommonDataModel-9.1
    vtkRenderingCore-9.1
    vtkRenderingOpenGL2-9.1
    vtkInteractionStyle-9.1
    vtkRenderingFreeType-9.1
    vtkFiltersCore-9.1
)
```

原理：

- `-lpcl_common` 会让链接器自己按搜索路径找库，有可能找到 `/usr/lib/x86_64-linux-gnu/libpcl_common.so`。
- `${PCL_INSTALL_DIR}/lib/libpcl_common.so` 是绝对路径，链接目标不会走偏。

同时给 `libORB_SLAM3.so` 设置 rpath：

```cmake
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${PROJECT_SOURCE_DIR}/3rdParty/Opencv/build-full/lib:${PCL_INSTALL_DIR}/lib:${PROJECT_SOURCE_DIR}/3rdParty/Pangolin/build/src:${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib:${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib"
    INSTALL_RPATH "${PROJECT_SOURCE_DIR}/3rdParty/Opencv/build-full/lib:${PCL_INSTALL_DIR}/lib:${PROJECT_SOURCE_DIR}/3rdParty/Pangolin/build/src:${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib:${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib"
)
```

rpath 的作用是把运行时动态库查找路径写进目标文件，避免运行时因为 `LD_LIBRARY_PATH` 或系统默认路径而加载错版本。

### 3.2 ORB-SLAM3 主库固定使用本地 OpenCV 4.9

文件：

```text
dense_orbslam3/CMakeLists.txt
```

固定 OpenCV CMake config：

```cmake
set(OpenCV_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full" CACHE PATH "OpenCV config path" FORCE)
find_package(OpenCV 4.2)
```

这样 CMake 不会再自动找到系统路径：

```text
/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

### 3.3 DBoW2 也固定使用同一套 OpenCV 4.9

文件：

```text
dense_orbslam3/Thirdparty/DBoW2/CMakeLists.txt
```

设置：

```cmake
set(OpenCV_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full" CACHE PATH "OpenCV config path" FORCE)
find_package(OpenCV 4 QUIET)
target_link_libraries(DBoW2 ${OpenCV_LIBS})
set_target_properties(DBoW2 PROPERTIES
  BUILD_RPATH "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib"
  INSTALL_RPATH "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib"
)
```

原因：

`libORB_SLAM3.so` 会链接 `libDBoW2.so`。如果 ORB-SLAM3 用 OpenCV 4.9，而 DBoW2 用系统 OpenCV 4.6，最终进程仍然会混入两个 OpenCV 版本。

### 3.4 ROS2 wrapper 固定使用本地 OpenCV/PCL

文件：

```text
src/orbslam3_dense_ros2/CMakeLists.txt
```

固定 CMake config：

```cmake
set(OpenCV_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full" CACHE PATH "OpenCV config path" FORCE)
set(PCL_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/share/pcl-1.15" CACHE PATH "PCL config path" FORCE)
```

设置本地根路径：

```cmake
set(DENSE_ORBSLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../dense_orbslam3")
set(DENSE_PCL_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/pcl/pcl_install")
set(DENSE_PCL_INCLUDE_DIR "${DENSE_PCL_ROOT}/include/pcl-1.15")
set(DENSE_OPENCV_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/Opencv/build-full")
set(DENSE_PANGOLIN_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/Pangolin/build/src")
```

ROS2 wrapper 的 PCL 库同样改为绝对路径：

```cmake
set(PCL_LIBRARIES
    ${DENSE_PCL_ROOT}/lib/libpcl_common.so
    ${DENSE_PCL_ROOT}/lib/libpcl_filters.so
    ${DENSE_PCL_ROOT}/lib/libpcl_io.so
    ${DENSE_PCL_ROOT}/lib/libpcl_kdtree.so
    ${DENSE_PCL_ROOT}/lib/libpcl_search.so
    pthread
    atomic
    vtkCommonCore-9.1
    vtkCommonDataModel-9.1
    vtkRenderingCore-9.1
    vtkRenderingOpenGL2-9.1
    vtkInteractionStyle-9.1
    vtkRenderingFreeType-9.1
    vtkFiltersCore-9.1
)
```

主程序运行时路径：

```cmake
set_target_properties(orb_slam3_main PROPERTIES
  BUILD_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
  INSTALL_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
)
```

OctoMap 工具节点也增加 PCL rpath：

```cmake
set_target_properties(octomap_converter pcd_to_octomap_node PROPERTIES
  BUILD_RPATH "${DENSE_PCL_ROOT}/lib"
  INSTALL_RPATH "${DENSE_PCL_ROOT}/lib"
)
```

这个小改动解决了安装后单独检查 `pcd_to_octomap_node` 时出现的：

```text
libpcl_common.so.1.15 => not found
libpcl_io.so.1.15 => not found
```

### 3.5 移除会带回系统库的 ROS 依赖

文件：

```text
src/orbslam3_dense_ros2/package.xml
```

移除：

```xml
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>pcl_conversions</depend>
<depend>pcl_ros</depend>
<depend>octomap_ros</depend>
<depend>pcl_msgs</depend>
```

保留：

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>message_filters</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>visualization_msgs</depend>
<depend>std_msgs</depend>
<depend>octomap_msgs</depend>
```

原理：

- `cv_bridge` / `image_transport` 容易把 ROS 自带 OpenCV 链进来。
- `pcl_conversions` / `pcl_ros` 容易把 ROS 自带 PCL 链进来。
- `octomap_ros` 对当前手写转换流程不是必须的，还会增加不必要的传递依赖。

### 3.6 用手动转换替代 cv_bridge 和 pcl_conversions

文件：

```text
src/orbslam3_dense_ros2/src/ros2_slam_publisher.cpp
src/orbslam3_dense_ros2/include/orbslam3_dense_ros2/ros2_slam_publisher.h
src/orbslam3_dense_ros2/src/octomap_converter.cpp
```

图像转换改为直接处理 `sensor_msgs::msg::Image`：

```cpp
cv::Mat ImageMsgToBgrMat(const sensor_msgs::msg::Image& msg);
cv::Mat ImageMsgToDepthMat(const sensor_msgs::msg::Image& msg);
sensor_msgs::msg::Image MatToImageMsg(const cv::Mat& image, const std::string& frame_id);
```

深度图处理策略：

- `16UC1` / `mono16` 直接按毫米深度使用。
- `32FC1` 按米转毫米，转换成 `16UC1`。

点云发布改为手写 `PointCloud2`：

```cpp
sensor_msgs::msg::PointCloud2 PCLToROS(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
    const std::string& frame_id);
```

字段布局明确写死为：

```text
x   float32 offset 0
y   float32 offset 4
z   float32 offset 8
rgb float32 offset 12
point_step = 16
height = 1
width = points.size()
```

OctoMap 转换节点也改成直接按 `PointCloud2.fields` 的 offset 读取：

```cpp
fieldOffset(msg, "x")
fieldOffset(msg, "y")
fieldOffset(msg, "z")
fieldOffset(msg, "rgb")
```

这样避免通过 `pcl_conversions` 进入系统 PCL 链路。

### 3.7 修复 RGB-D 稠密建图线程的点云处理风险

文件：

```text
dense_orbslam3/src/PointCloudMappingRGBD.cc
```

全局点云初始化时补齐元数据：

```cpp
globalMap.reset(new PointCloudMappingRGBD::PointCloud());
globalMap->is_dense = false;
globalMap->width = 0;
globalMap->height = 1;
```

每帧点云构建完成后补齐元数据：

```cpp
pcl::transformPointCloud(*tmp, *cloud, data.Twc);
cloud->is_dense = false;
cloud->width = static_cast<uint32_t>(cloud->points.size());
cloud->height = 1;
```

VoxelGrid 使用单独输出点云：

```cpp
PointCloudMappingRGBD::PointCloud::Ptr voxel_filtered(new PointCloudMappingRGBD::PointCloud());
voxel.setInputCloud(currentCloud);
voxel.filter(*voxel_filtered);
voxel_filtered->is_dense = false;
voxel_filtered->width = static_cast<uint32_t>(voxel_filtered->points.size());
voxel_filtered->height = 1;
```

SOR 也使用单独输出点云：

```cpp
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
```

增加空点云跳过：

```cpp
if (!currentCloud || currentCloud->empty()) continue;
if (voxel_filtered->empty()) continue;
if (filtered->empty()) continue;
```

合并后刷新全局点云尺寸：

```cpp
*globalMap += *filtered;
globalMap->width = static_cast<uint32_t>(globalMap->points.size());
globalMap->height = 1;
```

`getGlobalPointCloud()` 返回副本时也刷新尺寸：

```cpp
PointCloud::Ptr result(new PointCloud(*globalMap));
result->width = static_cast<uint32_t>(result->points.size());
result->height = 1;
```

原理总结：

- 不让 PCL filter 输入和输出指向同一个 cloud，避免别名写入。
- 每次改变点云内容后维护 `width = points.size()` 和 `height = 1`。
- 空点云不进入后续过滤、合并、发布流程。

## 4. 构建方法

重新构建 ORB-SLAM3：

```bash
cmake -S dense_orbslam3 -B dense_orbslam3/build -DCMAKE_BUILD_TYPE=Release
cmake --build dense_orbslam3/build --target ORB_SLAM3 -j8
```

清理 ROS2 包 CMake cache 并重新构建：

```bash
colcon build --packages-select orbslam3_dense_ros2 --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

之后如果只是普通增量编译：

```bash
colcon build --packages-select orbslam3_dense_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

运行前 source：

```bash
source /home/ricky/WCR_ws/install/setup.bash
```

## 5. 验证方法

### 5.1 验证 libORB_SLAM3.so

```bash
objdump -p dense_orbslam3/lib/libORB_SLAM3.so | rg "NEEDED|RUNPATH|opencv|pcl|DBoW|pangolin"
ldd dense_orbslam3/lib/libORB_SLAM3.so | rg "opencv|pcl|DBoW|pangolin|not found"
```

期望看到：

```text
libopencv_*.so.409 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib/...
libpcl_*.so.1.15 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/lib/...
libpangolin.so => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/Pangolin/build/src/libpangolin.so
libDBoW2.so => /home/ricky/WCR_ws/dense_orbslam3/Thirdparty/DBoW2/lib/libDBoW2.so
```

不应该再出现：

```text
libopencv_*.so.406
/usr/lib/x86_64-linux-gnu/libpcl_*.so
/usr/include/pcl-1.14
```

### 5.2 验证 orb_slam3_main

```bash
objdump -p install/orbslam3_dense_ros2/lib/orbslam3_dense_ros2/orb_slam3_main | rg "NEEDED|RUNPATH|opencv|pcl|ORB|pangolin"
ldd install/orbslam3_dense_ros2/lib/orbslam3_dense_ros2/orb_slam3_main | rg "opencv|pcl|ORB_SLAM3|DBoW|pangolin|not found"
```

期望看到：

```text
libopencv_imgproc.so.409 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib/...
libopencv_core.so.409 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full/lib/...
libpcl_common.so.1.15 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/lib/...
libpcl_filters.so.1.15 => /home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/lib/...
libORB_SLAM3.so => /home/ricky/WCR_ws/dense_orbslam3/lib/libORB_SLAM3.so
```

### 5.3 验证三个 ROS2 可执行文件

```bash
for exe in orb_slam3_main octomap_converter pcd_to_octomap_node; do
  echo "### $exe"
  objdump -p install/orbslam3_dense_ros2/lib/orbslam3_dense_ros2/$exe | rg "RUNPATH|NEEDED.*(opencv|pcl|ORB|pangolin)" || true
  ldd install/orbslam3_dense_ros2/lib/orbslam3_dense_ros2/$exe | rg "opencv|pcl|ORB_SLAM3|pangolin|not found" || true
done
```

修复后：

- `orb_slam3_main` 的 OpenCV 指向本地 OpenCV 4.9。
- `orb_slam3_main` 的 PCL 指向本地 PCL 1.15。
- `pcd_to_octomap_node` 可以找到本地 `libpcl_common.so.1.15` 和 `libpcl_io.so.1.15`。
- 不应出现 `not found`。

### 5.4 搜索旧依赖残留

```bash
rg -n "(/usr/lib/x86_64-linux-gnu/libpcl_|/usr/include/pcl-1\\.14|libopencv_.*406|opencv.*4\\.6|pcl_conversions|octomap_ros|cv_bridge|image_transport|pcl_ros)" \
  build/orbslam3_dense_ros2/CMakeCache.txt \
  build/orbslam3_dense_ros2/CMakeFiles/*/link.txt \
  src/orbslam3_dense_ros2/package.xml \
  src/orbslam3_dense_ros2/CMakeLists.txt \
  dense_orbslam3/build/CMakeFiles/ORB_SLAM3.dir/link.txt \
  dense_orbslam3/build/CMakeCache.txt
```

期望没有输出。

## 6. 后续维护规则

### 6.1 不要轻易重新引入 cv_bridge

如果重新加入 ROS2 系统自带的 `cv_bridge`，很可能再次把系统 OpenCV 带进来。除非把 `cv_bridge` 也用本地 OpenCV 4.9 重新编译，否则当前手写转换方式更稳。

### 6.2 不要轻易重新引入 pcl_conversions / pcl_ros

如果重新加入 `pcl_conversions` 或 `pcl_ros`，很可能再次把系统 PCL 1.14 带进来。当前手写 `PointCloud2` 转换虽然朴素，但依赖边界清楚。

### 6.3 改依赖路径后必须清 CMake cache

如果 OpenCV/PCL 路径变化，旧 cache 会继续污染构建。ROS2 包建议使用：

```bash
colcon build --packages-select orbslam3_dense_ros2 --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 6.4 PCL 点云处理后必须维护元数据

每次创建、过滤、变换、合并或复制非组织点云后，建议保证：

```cpp
cloud->is_dense = false;
cloud->width = static_cast<uint32_t>(cloud->points.size());
cloud->height = 1;
```

避免这种输入输出同对象的过滤方式：

```cpp
filter.setInputCloud(cloud);
filter.filter(*cloud);
```

推荐：

```cpp
PointCloud::Ptr output(new PointCloud());
filter.setInputCloud(input);
filter.filter(*output);
```

## 7. 最终结果

这次修改后，运行时动态库关系收敛到单一版本：

```text
OpenCV: dense_orbslam3/3rdParty/Opencv/build-full/lib, version 4.9
PCL:    dense_orbslam3/3rdParty/pcl/pcl_install/lib, version 1.15
```

同时，RGB-D 稠密建图线程避免了 PCL in-place filtering，并持续维护点云 `width` / `height` 元数据。这样同时降低了 ABI 混用风险和点云处理过程中的内存/元数据风险，是这次 Pangolin 弹出后首帧附近崩溃问题的主要修复点。
