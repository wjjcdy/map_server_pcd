# map_server_pcd

同时保存：
*. 3d cloud点云存储为pcd格式；
*. 2d map保存为png格式和对应参数文件yaml；

对应命令如下：
```
rosrun map_server_pcd map_server_pcd _file_name:=test _map_3d_topic:=/laser_cloud_surround _map_2d_topic:=/2d_map
```

