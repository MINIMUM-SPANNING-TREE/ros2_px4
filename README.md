此工程仍在开发中，
uav_mavros2软件包用于存放mavros第层调用接口和遥测数据抓取
fsm软件包暂时用于测试底层代码的供能完整性
mission暂时无作用

v2.0新增功能
新增服务类型，用于无人机服务。
使用服务模式控制无人机动作，上层调用服务即可控制飞行任务，目前offboard移动暂时有运行问题，没法飞到设定位置，但不会导致坠机。
未修改v1.0的代码，老代码依然可以使用，用于对照。测试新功能请勿运行老的uav_ctrl代码

下面介绍使用方法

运行节点前先启动px4仿真（启用前先进入PX4-AutoPilot文件夹）
```
make px4_sitl gz_x500

```
运行 MAVROS 桥接（需要先source install/setup.bash）
```
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

```
运行遥测节点
```
ros2 run uav_mavros2 telemetry_node

```
运行底层控制节点（v1.0版本）
```
ros2 run uav_mavros2 uav_ctrl_node

```
运行底层控制服务(v2.0版本，和上一行的控制节点是相同功能，不要同时启用)
```
ros2 run uav_mavros2 ctrl_server_node

```
按顺序运行起飞，移动，降落节点（v1.0版本，现在暂时不可用）
起飞默认高度2m，移动默认移动至坐标(4.0,4.0,4.0)
```
ros2 run fsm test_takeoff
```
```
ros2 run fsm move
```
```
ros2 run fsm land

```
接口函数介绍

```
arm(True) 
```

解锁无人机，参数为bool，上层控制一般不用

```
set_mode(None)      
```

设置模式，模式类型如下
自动起飞模式    AUTO.TAKEOFF
自动降落模式    AUTO.LAND
自动保持模式    AUTO.LOITER
板外控制模式    OFFBOARD

```
takeoff_auto(relative_altitude = x)  
```

自动起飞到设定高度x

```
move_offboard(x, y, z, yaw: float = None)
```

板外控制飞行到指定坐标，坐标为浮点数，偏航角度默认可不填写

```
land_auto(timeout: float=30)
```

自动降落，参数为超时检测默认30秒
