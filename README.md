# Dense-ORB-SLAM3

This repository is a modified version of [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  

If you want to wrap this project with ROS2, please check [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)

--- 

## Modification
- Succesfully tested in **Ubuntu 24.04** and **ROS2 Jazzy**(with OpenCV 4.9.0)
- Update from C++11 to C++17
- RGB-D ROS2 dense mapping dependency and crash fix notes: [docs/rgbd_dense_ros2_dependency_and_crash_fix.md](docs/rgbd_dense_ros2_dependency_and_crash_fix.md)

**Rectified** camera type  

## How to build
Clone the repository:
```
git clone https://github.com/Pplll-pppl/Dense_OrbSlam3.git dense_orbslam3
```

Install same required dependencies as original version ([ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) / [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)). 
Additionally, PCL 1.10 is required to generate dense point cloud. You can install it in the same path of OpenCV and Pangolin.
Then,  Execute:
```
cd path_to_your_ws/dense_orbslam3
chmod +x build.sh
./build.sh
```
This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

## Run examples
- This project integrate dense mapping function. Currently, only **rgbd_tum_dense.cc** is available.
To run the dense mapping example, execute:
```
cd ~/dense_orbslam3/Examples/RGB-D ## change to your ws path

./rgbd_tum_dense --tum /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt --voc /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt --param /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml
 ## change to your ws path

```

```
cd ~/dense_orbslam3/Examples/RGB-D ## change to your ws path

./rgbd_tum_dense --tum /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg3_long_office_household:/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr3_room.txt --voc /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt --param /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM3.yaml
 ## change to your ws path

```

- To run the example with realsense D435i, execute:
  
```
cd ~/WCR_ws/dense_orbslam3/Examples/RGB-D ## change to your ws path
```

```
./rgbd_realsense_D435i ~/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt ~/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml ~/WCR_ws/dense_orbslam3/trajectory_d435i.txt
```
- To run the example with realsense D435i dense mapping, execute:
  
```
cd ~/dense_orbslam3/Examples/RGB-D ## change to your ws path
```

```
./rgbd_dense_realsense_D435i ~/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt ~/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml ~/WCR_ws/dense_orbslam3/trajectory_d435i.txt --dense
```
- To run the example with realsense D435i inertial dense mapping, execute:
  
```
cd ~/dense_orbslam3/Examples/RGB-D-Inertial ## change to your ws path
```

```
./rgbd_dense_inertial_realsense_D435i ~/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt ~/WCR_ws/dense_orbslam3/Examples/RGB-D-Inertial/RealSense_D435i.yaml ~/WCR_ws/dense_orbslam3/trajectory_d435i.txt --dense
```


### Calibration

Nowadays Kalibr and ROS1 are two popular tools for camera calibration. 
However, for ubuntu 24.04, Kalibr is not well integrated with ROS1. 
So it could be difficult to build Kalibr locally and use it in ROS1. To figure this dilemma, we can use kalibr in docker and first record data (pictures and IMU data) with the script (recorder_realsense_D435i.cc) provided by orbslam3.

#### Deploy Kalibr in Docker

First, install relevant packages in your host system:
```
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
```
Then add official GPG key to docker:
```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/docker.gpg

```
Add software source of docker:
```
eecho "deb [arch=$(dpkg --print-architecture)] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

```
install docker Engine:
```
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
```
validate docker installation:
```
docker --version
# output like : Docker version 26.1.3, build b72abbb
```
Pull kalibr image from docker hub:
```
docker pull stereolabs/kalibr:latest
```
Check images in docker:
```
docker images
```
Create and run a container:
```
docker run -it --rm -v /home/ricky/WCR_ws/dense_orbslam3:/data stereolabs/kalibr /bin/bash 
```

#### Camera intrinsics

First record the camera data:

```
./Examples/Calibration/recorder_realsense_D435i ./Examples/Calibration/recorder_visual
```
Then process IMU data:
```
python3 ./Examples/Calibration/python_scripts/process_imu.py ./Examples/Calibration/recorder_visual/
```
To get into docker container:
```
docker run -it --rm -v /home/ricky/WCR_ws/dense_orbslam3:/data stereolabs/kalibr /bin/bash
```

convert the data into ROS bag:
```
kalibr_bagcreater --folder /data/Examples/Calibration/recorder_visual/ --output-bag /data/Examples/Calibration/recorder_visual.bag
```
Start calibration:
```
kalibr_calibrate_cameras --bag /data/Examples/Calibration/recorder_visual.bag --topics /cam0/image_raw --models pinhole-radtan --target /data/Examples/Calibration/recorder_empty/april_6x6_80x80cm_larues.yaml
```

If the calibration is successful, you can get the camera intrinsics.

**attention** Everytime recording a new bag please remember to delete the original data in recorder_visual folder **!!!**

##### potential error and solution
- **error: Cameras are not connected through mutual observations, please check the dataset. Maybe adjust the approx. sync. tolerance.
Traceback (most recent call last):
  File "/kalibr_workspace/devel/bin/kalibr_calibrate_cameras", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 447, in <module>
    main()
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras", line 204, in main
    graph.plotGraph()
  File "/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py", line 311, in plotGraph
    edge_label=self.G.es["weight"],
KeyError: 'Attribute does not exist'**
  
  - reason: in the source code of the steroelab's Kalibr image, part of the code in file **"/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py"** is not compatible for single camera calibration.Specifically, it is in the function **isCameraConnected**, which is originally:
    ```
    def isGraphConnected(self):
        #check if all vertices are connected
        return self.G.adhesion()
    ```



  - solution: modify the function **isGraphConnected** in file **"/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py"** as follows:
    ```
    def isGraphConnected(self):

        if self.numCams == 1:
            # Since igaph 0.8, adhesion correctly returns 0 for the non-connected one cam case.
            #   which evaluates to false later on. So we skip the check and return true in the one camera case.
            return True
        else:
            #check if all vertices are connected
            return self.G.adhesion()
        #returns the list of cam_ids that share common view with the specified cam_id

    def getCamOverlaps(self, cam_id):
    ```
    **notice**: We conduct this operation in container which means it's only a temporary adjustment, and the original code will be restored after the container is stopped. So to have a permanent fix, it is recommended that create your own Kalibr image with the modified code.

## Min Rect Debug Clouds

For `Examples/RGB-D/room_size_calc_min_rect_demo.cc`, the intermediate debug point clouds are saved to:

```
/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/min_rect_debug_clouds
```

The files are named with `generation order + name`, so it is easy to inspect the pipeline step by step in CloudCompare or PCL viewer.

### Debug cloud meanings

`01_cloud_raw.pcd`  
Original input point cloud before any processing. Use it to inspect the raw data quality, sparsity, and noise level.

`02_cloud_preprocess.pcd`  
Point cloud after voxel downsampling and SOR filtering. Use it to check whether preprocessing removes noise while preserving the main structure.

`03_cloud_hull.pcd`  
Hull point cloud extracted from the preprocessed cloud. This is the main contour source used by the later contour analysis stages.

`04_contour_ordered.pcd`  
Hull points ordered along the principal plane. The geometry is similar to the hull cloud, but the point order is now arranged along the contour.

`05_contour_after_light_bar.pcd`  
Contour after the light-bar style simplification. This stage removes many nearly collinear redundant contour points.

`06_contour_after_angle_prune.pcd`  
Contour after angle-based pruning. This is usually cleaner than the previous stage and closer to a compact structural contour.

`07_contour_dense.pcd`  
Dense contour generated by resampling the simplified contour. This file is especially useful for projection-based and support-based size estimation.

`08_contour_simplified_for_pca.pcd`  
The final simplified contour converted into a point cloud for PCA and axis rectification.

`09_plane_00_inliers.pcd`, `10_plane_01_inliers.pcd`, ...  
Inlier point clouds of each fitted plane. These files are useful for checking whether wall, floor, ceiling, or other structural surfaces are segmented correctly.

### Recommended viewing order

1. First inspect `01 -> 02 -> 03` to verify raw input, preprocessing quality, and hull extraction.
2. Then inspect `04 -> 05 -> 06 -> 07` to verify contour ordering, simplification, pruning, and dense resampling.
3. Finally inspect `plane_xx_inliers` to verify structural plane fitting and segmentation quality.

## RGB-D Executable Commands

Run the commands below from:

```bash
cd /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D
```

### TUM examples

```bash
./rgbd_tum \
  /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
  /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt
```

```bash
./rgbd_tum_dense \
  --tum /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
  --voc /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml
```

### RealSense examples

These commands require a connected RealSense D435i device.

```bash
./rgbd_realsense_D435i \
  /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  /home/ricky/WCR_ws/dense_orbslam3/trajectory_d435i.txt
```

```bash
./rgbd_dense_realsense_D435i \
  /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  /home/ricky/WCR_ws/dense_orbslam3/trajectory_d435i.txt \
  --dense
```

### Point cloud tools

```bash
./rotate_pcd
```

```bash
./light_pca_shear_project \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/light_pca_shear_project_corrected.pcd \
  /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/light_pca_shear_project_projection.csv
```

### Room size calculation examples

```bash
./room_size_calc
```

```bash
./room_size_calc_fast_demo
```

```bash
./room_size_calc_min_rect_demo
```

```bash
./room_size_calc_shear_corrected
```

```bash
./room_size_calc_non_destructive_planes
```

For the room-size tools above, you can also pass a custom point cloud path:

```bash
./room_size_calc_fast_demo /absolute/path/to/your_cloud.pcd
./room_size_calc_min_rect_demo /absolute/path/to/your_cloud.pcd
./room_size_calc_shear_corrected /absolute/path/to/your_cloud.pcd
./room_size_calc_non_destructive_planes /absolute/path/to/your_cloud.pcd
```
