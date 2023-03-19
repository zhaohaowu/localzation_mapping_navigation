#### 1、首先生成定位地图

bag下载地址

链接：https://pan.baidu.com/s/1DE8PUkGEX55W_lFk68A1Vg?pwd=slam 
提取码：slam

首先，生成特征地图flat_cloud_map.pcd和sharp_cloud_map.pcd，关键帧Scans文件夹，SCDs文件夹，关键帧位姿 pose.txt和pose_kitti.txt，保存至lio_sam的data文件夹下。实际sc只需要pose.txt和SCDs文件夹，lidar_localization只需要flat_cloud_map.pcd和sharp_cloud_map.pcd，剩下的pose_kitti.txt和Scans文件夹是粒子滤波使用的，这里删去了

```
roslaunch lio_sam run.launch
rosbag play park.bag
```

然后，将data复制到data_park数据集下，(建议换数据集复制一下，防止被覆盖)

```
cd ~lv_ws/src/relocalization/lio_sam/
cp -r data data_park
```

最后基于地图的定位和重定位（rviz手动重定位和sc自动重定位）

```
roslaunch lidar_localization matching_loam.launch
```

#### 2、定位地图转为导航地图

#将liosam生成的pcd变为栅格地图

cd ~/lv_ws
source devel/setup.bash
roslaunch pcd2map pcd2map_park.launch

#### 3、定位和导航

```
#lidar_driver
cd ~/lv_ws
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

```
#relocalization
cd ~/lv_ws
source devel/setup.bash
roslaunch lidar_localization matching_loam.launch
```

```
#navigation
cd ~/lv_ws
source devel/setup.bash
roslaunch navigation move_base_park.launch
```

#### 4、仿真环境导航

```
roslaunch racecar gazebo_small.launch
roslaunch racecar move_base.launch
```



#### 

