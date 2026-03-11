# developing notes

**项目背景**：使用 ORB-SLAM3 / dense_orbslam3 中的 `Examples/Calibration/recorder_realsense_D435i` 录制 RealSense D435i 的 IR + IMU 数据，用于后续相机/IMU calibration。

**硬件环境**：Mini PC（Beelink SER8 或类似 AMD Ryzen 系列），Linux 系统（Ubuntu），RealSense D435i 相机。

**最终结果**：程序成功运行，能正常录制 cam0/IR 图像 + IMU 数据，cam0 视窗显示正常，文件正确写入。

## 遇到的问题列表

### 1. 程序启动后卡死（Hang），无 cam0 视窗、无文件生成
- **现象**：列出所有 sensor options 后停止响应；`xwininfo -tree -root | grep cam0` 无输出；需 Ctrl+C 强制退出。
- **根本原因**：`pipeline.start()` 成功，但 IR stream 未产生任何 frame → callback 不触发 → `image_ready` 永远 false → 主循环卡在 `cond_image_rec.wait()`。
- **相关表现**：IMU 文件被创建但为空或极少数据；图像完全无输出。
- **解决方向**：调整 stream 配置、降低 FPS、确保 IR stream 稳定产生 frame。

### 2. 命令行参数错误
- **现象**：加 `-v` 或多参数时报 Usage 提示。
- **原因**：程序只接受单一参数（保存文件夹路径），不支持其他 flag。
- **解决**：正确用法：`./recorder_realsense_D435i /path/to/folder`

### 3. 硬编码 IR 分辨率不稳定
- **现象**：原代码使用 640×480@30fps IR stream，常导致无 frame 或启动失败。
- **原因**：D435i IR/stereo 原生推荐分辨率为 848×480，640×480 在某些 USB 条件或 librealsense 版本下不稳定。
- **解决**：修改为 `848×480@30`（或降至 15fps 测试）。

### 4. 程序崩溃：xioctl(VIDIOC_S_FMT) failed, errno=5 (Input/output error)
- **现象**：启动 pipeline 时直接 terminate，抛出 `rs2::backend_error`，核心 dump。
- **dmesg 关键日志**：
  - `uvcvideo ... Non-zero status (-71) in video completion handler`（EPROTO）
  - `Failed to set UVC probe control : -32`（EPIPE）
  - 偶见 USB disconnect/reconnect
- **原因**：kernel uvcvideo 驱动在设置视频格式时底层 USB I/O 失败。
- **常见触发因素**：USB 供电/带宽不足、缆线质量差、Mini PC USB 控制器特性。

### 5. dmesg 权限问题，无法读取内核日志
- **现象**：`dmesg | grep ...` 报 `Operation not permitted`。
- **原因**：`kernel.dmesg_restrict = 1`（现代 Linux 默认安全设置）。
- **解决**：`sudo sysctl kernel.dmesg_restrict=0`（临时）或永久配置 `/etc/sysctl.d/`。

### 6. USB 层面的整体不稳定（Mini PC 常见）
- **表现**：realsense-viewer 正常，但自定义程序易触发 I/O 错误。
- **原因汇总**：
  - USB 3.0 供电/带宽边缘（D435i + IMU 功耗高）
  - 缆线/端口问题（前置埠、hub、长线、转接头）
  - Mini PC（AMD 平台）USB 控制器供电或相容性
  - kernel uvcvideo 驱动对高负载 stream 敏感
- **关键验证**：realsense-viewer 中 USB 显示 SuperSpeed、IR 848×480 + IMU 流畅无掉帧 → 证明硬件基本 OK。

## 最终解决路径

1. **验证基础**  
   使用 `realsense-viewer` 确认：  
   - IR (左/右) 848×480@30 + IMU 正常流畅  
   - USB 速度为 SuperSpeed  
   - 无错误/掉帧

2. **代码调整**  
   - IR stream 改为 `848, 480, RS2_FORMAT_Y8, 15`（make sure the alignment of the resolution between realsense viewer and hard coding, what's more `30fps 降至 15fps` 可以避免 USB 带宽压力）  
   - 可选：先注释掉 IMU stream 测试单 IR 稳定性  
   - 加 emitter 控制：`RS2_OPTION_EMITTER_ENABLED = 1`, `LASER_POWER = 150`

3. **硬件优化**  
   - 使用原厂短 USB 3.0/3.1 线，直插后置 USB 埠  
   - 拔插相机重置状态，重启主机清除 uvcvideo 残留

4. **未采用但备选方案**  
   - 强制使用 libusb/rsusb backend（`-DFORCE_RSUSB_BACKEND=true` 重建 librealsense）  
   - 降级/升级 librealsense 版本（2.53.1 或 2.55.x 测试）

## 结论

所有问题本质上是 **D435i 在 Linux + Mini PC 环境下启动 IR + IMU stream 的稳定性** 问题，涉及：
- 软件层：硬编码参数不合适、callback 死锁
- 驱动层：uvcvideo 对 USB I/O 错误敏感（-71/-32/5）
- 硬件层：Mini PC USB 供电/控制器特性