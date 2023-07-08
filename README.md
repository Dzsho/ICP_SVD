本作业旨在实现经典的激光里程计算法ICP/NDT.

---


```bash
# build
catkin_make
```
**启动**

```bash
# set up session:
source install/setup.bash
# launch:
roslaunch lidar_localization front_end.launch
```

```bash
# play ROS bag, full KITTI:
rosbag play kitti_2011_10_03_drive_0027_synced.bag


cd /workspace/assignments/02-lidar-odometry-basic/src/lidar_localization/slam_data/trajectory
```


该目录下会输出:

* Ground Truth的RTK轨迹估计, ground_truth.txt
* Lidar Frontend的轨迹估计, laser_odom.txt

使用上述两个文件, 完成**evo**的评估

此处轨迹会有较大的偏移. 在后续的学习中, 你会了解如何修复这个偏移, 获得更好的轨迹估计.
