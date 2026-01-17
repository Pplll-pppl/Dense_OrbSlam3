# Dense-ORB-SLAM3

This repository is a modified version of [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  

If you want to wrap this project with ROS2, please check [ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git)

--- 

## Modification
- Succesfully tested in **Ubuntu 24.04** and **ROS2 Jazzy**(with OpenCV 4.9.0)
- Update from C++11 to C++17
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

## Note
- This project integrate dense mapping function. Currently, only **rgbd_tum_dense.cc** is available.
To run the dense mapping example, execute:
```
cd ~/dense_orbslam3/Examples/RGB-D ## change to your ws path

./rgbd_tum_dense --tum ~/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:/home/ricky/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt --voc /home/ricky/dense_orbslam3/Vocabulary/ORBvoc.txt --param /home/ricky/dense_orbslam3/Examples/RGB-D/TUM1.yaml ## change to your ws path

```
