# ROS数据展示系统

## 1. 用命令行窗口显示小车的IMU和里程计（odometry）数据
1. 首先使用`roscore`启动ros
2. `rosbag play all.bag`运行`all.bag`
3. `rostopic echo /imu/data_raw`接受imu数据
4. `rostopic echo /odom`接受odometry数据
5. 

## 2. 用图形界面显示颜色相机和深度相机的数据（利用OpenCV库）

## 3. 用图形界面显示激光雷达的点云数据（利用PCL库）