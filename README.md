# Dense-ORB-SLAM3

This repository is a modified version of [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  

--- 

## Modification
- Succesfully tested in **Ubuntu 20.04** and **ROS2 Foxy**(with OpenCV 4.2.0)
- Update from C++11 to C++14
- Fixed unexpected <span style="color:red">error</span> when start **STEREO** mode with **Rectified** camera type  

## How to build
Clone the repository:
```
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
```

Install same required dependencies as original version. Then,  
Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

## Note
- 本项目引入了稠密建图的功能，目前仅能使用的例程是rgbd_tum_dense.cc，启动指令为：
```
cd ~/orb_slam3/Examples/RGB-D

./rgbd_tum_dense --tum ~/orb_slam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_xyz:/home/ricky/orb_slam3/Examples/RGB-D/associations/fr1_xyz.txt --voc /home/ricky/orb_slam3/Vocabulary/ORBvoc.txt --param /home/ricky/orb_slam3/Examples/RGB-D/TUM1.yaml

```
