# autoware

## 环境配置

> 参考autoware.test.2的md

编译整个项目：`AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`

编译单个package：`AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --package-select autoware_quickstart_examples`

命令行暂停：`ctrl + s`暂停，输入`ctrl + q`继续

## ndt_mapping

启动建图模块：

```
roslaunch autoware_quickstart_examples my_mapping.launch
rviz
```

播放数据包：小车的frame是/rslidar

```
rosbag play ./bag/20220815test3.bag /rslidar_points:=/points_raw
```

在点云包快结束时使用：

```
rosbag record -O map.bag /ndt_map
```

将bag包转换为pcd文件：

```
rosrun pcl_ros bag_to_pcd map.bag /bdt_map xxx.pcd
```

