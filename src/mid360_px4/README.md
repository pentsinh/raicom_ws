# mid360_px4

此功能包将FAST_LIO中的位置消息发送到mavros中。

FAST_LIO发出的位置信息话题为**/Odometry**

mavros接受位置信息的话题为**/mavros/vision_pose/pose**

##### 具体使用如下：

开三个终端，依次运行：

```
roslaunch livox_ros_driver2 msg_MID360.launch
```

```
roslaunch fast_lio mapping_mid360.launch
```

```
roslaunch mid360_px4 mid360topx4.launch
```

##### echo话题中的数据：

```
rostopic echo /Odometry
```

```
rostopic echo /mavros/vision_pose/pose
```

