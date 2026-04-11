此工程仍在开发中。

软件包职责（当前版本）：
- uav_mavros2：MAVROS 底层接口、飞控服务、遥测打印。
- fsm：单功能服务调用测试节点（起飞/移动/降落/返航）。
- mission：交互式任务控制台（已启用）。

## 当前推荐启动方式（v3.0）

### 1) 编译并加载环境
在工作区根目录执行：
```
colcon build
source install/setup.bash
```

### 2) 启动 PX4 仿真（在 PX4-AutoPilot 目录）
```
make px4_sitl gz_x500
```

### 3) 启动 MAVROS 桥接（按场景选择）

仿真（SITL）：
```
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
```

真机（USB 串口）：
```
# 启动前先确认设备名（如 /dev/ttyACM0 或 /dev/ttyACM1）
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM1:57600
```

真机（ESP8266 WiFi bridge）：
```
# 典型配置：Host Port=14550, Client Port=14555, Baudrate=57600
ros2 launch mavros px4.launch fcu_url:=udp://:14550@192.168.4.1:14555
```

说明：
- USB 模式下串口名可能变化，建议每次启动前确认（终端输入ls /dev/ttyACM*u查询）。
- WiFi 模式下若出现 RTT / RADIO_STATUS 告警，但遥测正常，一般不影响基础联通。
- 室内常见 No GPS fix，不代表 MAVROS 或飞控未连接。

### 4) 启动服务端（v2/v3 主控制链路）
```
ros2 run uav_mavros2 ctrl_server_node
```

### 5) 启动遥测与状态打印（可选）
```
ros2 run uav_mavros2 telemetry_node（已弃置）
ros2 run uav_mavros2 print_node
```

### 6) 选择上层控制方式（任选其一）

方式 A：FSM 单功能测试
```
ros2 run fsm test_takeoff_ctrl
ros2 run fsm test_move_ctrl
ros2 run fsm test_land_ctrl
ros2 run fsm test_rtl_ctrl
```

方式 B：Mission 交互控制台（推荐）
```
ros2 run mission mission_console
```
支持命令：
- takeoff <relative_alt>
- move <x> <y> <z> [yaw] [timeout]
- dir <forward|backward|left|right|up|down> <distance> [yaw] [timeout]
- land [timeout]
- rtl [timeout]
- help
- exit

## 当前服务接口（uavserver）

当前上层统一通过服务调用，不再直接调用旧版控制类。

- uav/takeoff：相对高度起飞
- uav/move：移动到目标坐标
- uav/land：自动降落
- uav/rtl：自动返航
- uav/arm：上锁/解锁
- uav/set_mode：模式切换

模式类型：
- AUTO.TAKEOFF
- AUTO.LAND
- AUTO.LOITER
- AUTO.RTL
- OFFBOARD

---

## v2.0 联调问题复盘（本次上下文总结）

### 1. 起飞服务返回成功，但飞机只解锁不起飞
现象：调用 `uav/takeoff` 后日志显示成功，但高度不变化。

原因：
- 起飞流程仅依赖“命令发送成功”，缺少“达到目标高度”的达成判定。
- 起飞服务链路在不同模式下行为不一致，易出现看似成功但未执行。

最终方案：
- 保留 MAVROS `CommandTOL` 起飞方式。
- 在服务层等待高度到达阈值后再返回成功。
- 增加详细日志（success/result）用于定位飞控拒绝原因。

### 2. 降落节点调用超时（看起来像降落失败）
现象：`test_land_ctrl` 很快超时，但服务端可能仍在执行。

原因：
- 客户端等待时间短于服务端内部降落等待时间，导致“假超时”。

最终方案：
- 客户端等待时间改为与请求超时一致（`timeout + buffer`）。
- 后续进一步收敛职责：上层节点简化为“发请求 + 看返回”。

### 3. 起飞/降落顺序执行后，后续服务偶发无响应
现象：第一次动作正常，第二次请求无日志或卡住。

原因：
- 服务回调内嵌套 `spin_once` / `spin_until_future_complete`，与执行器冲突，导致回调调度卡死风险。

最终方案：
- 统一改为轮询 future 的等待方式（`future.done() + sleep`），避免回调中再驱动 executor。
- 保留多线程执行器，并将服务回调组设为可重入。

### 4. move 服务超时，QGC 显示一直等待模式
现象：`uav/move` 超时，飞机悬停不动。

原因：
- move 流程没有可靠进入 `OFFBOARD`。
- 发布 setpoint 时未持续刷新时间戳，可能被飞控忽略。

最终方案：
- move 前先预热发送 setpoint，再切换 `OFFBOARD`。
- `OFFBOARD` 定时发布时每次刷新 `target_pose.header.stamp`。

### 5. 降落逻辑复杂且有冗余分支
现象：`AUTO.LAND` 与 `CommandTOL land` 混用，逻辑重复。

根据当前需求的最终方案：
- 降落仅保留 `AUTO.LAND`。
- 删除额外降落方法分支，减少状态分叉和维护成本。

## 当前最终控制策略（简化版）

- 起飞：`CommandTOL takeoff` + 服务层等待达到目标高度。
- 移动：`OFFBOARD`（预热 setpoint + 切模 + 持续发布新时间戳 setpoint）。
- 降落：仅 `AUTO.LAND` + 服务层等待落地判定。
- 上层测试节点：仅负责调用服务并输出返回结果，不再重复实现状态检测。

## 迭代记录（追加）

### 2026-03-27 第 1 轮
- 修复 move 无法执行问题：在 move 业务流程中增加 OFFBOARD 预热 setpoint 与自动切模。
- 修复 OFFBOARD setpoint 有效性问题：定时发布时刷新 `target_pose.header.stamp`。

### 2026-03-27 第 2 轮
- 精简降落策略：删除 CommandTOL 降落分支，降落仅保留 AUTO.LAND。
- 删除与降落分支相关的冗余客户端初始化与日志分叉。

### 2026-03-27 第 3 轮
- 将 move 到位检测从服务层上移逻辑下沉到业务层：
	- `uav.py` 中 `move()` 新增超时与等待到位参数，内部完成到位判断。
	- `uavserver.py` 中 `handle_move` 只做参数转发与响应封装，不再自建等待循环。
- 结果：控制职责进一步统一，服务层更薄，业务层更集中，后续维护成本更低。

---

## v3.0 迭代总结（本次新增）

### 新增功能

1. 遥测打印节点重构
- print 节点改为复用 Uav/UavBase 数据缓存。
- 日志改为中文结构化输出。
- 输出策略改为“状态变化触发 + 心跳输出”，避免高频刷屏。
- 新增 Home 点信息打印（home xyz + 是否有效）。

2. Home 点能力下沉到底层
- UavBase 新增 HomePosition 订阅与缓存。
- 提供 home 数据 getter（位置与有效性）。

3. 自动返航服务链路打通
- 新增服务接口：uav/rtl。
- Uav 新增 rtl(timeout) 业务方法（切换 AUTO.RTL 并等待返航落地判定）。
- UavServer 注册并暴露 uav/rtl 服务。
- FSM 新增 test_rtl_ctrl 测试节点。

4. FSM 节点职责收敛
- test_move_ctrl 改为纯服务客户端风格，不再做遥测订阅与二次监测。
- 起飞/移动/降落/返航测试节点统一为“发请求 + 等返回”。

5. Mission 软件包启用
- 新增分层式交互控制台节点（mission_console）：
	- 状态缓存层（订阅 MAVROS 原始话题）
	- 动作层（takeoff/move/land/rtl 客户端）
	- 方向移动层（dir -> 绝对目标点换算）
	- 路由层（命令解析）
	- launcher 统一组装与启动

### 本轮发现的关键 Bug 与修复

1. 现象：move 目标为 (5,5,7)，飞机飞向异常位置并超时
- 触发条件：print 节点与控制服务节点同时在线。
- 根因：print 节点继承 Uav 后，继承了 OFFBOARD 定时 setpoint 发布逻辑，导致与控制节点抢控制。
- 修复：在 print 节点覆盖 timer_cb，明确禁止其发布 setpoint，只保留只读遥测打印。

2. 现象：上层节点重复监测导致维护复杂、行为不一致
- 根因：FSM 节点中混入了业务监测逻辑。
- 修复：统一将到位/超时等判定留在 uav 业务层；FSM 只做服务调用。

3. 现象：返航能力缺失，任务链路不完整
- 根因：原服务接口只有起飞/移动/降落。
- 修复：新增 Rtl.srv、uav/rtl 服务、rtl 业务实现和 test_rtl_ctrl 测试入口。

### 当前 v3.0 控制策略（最终）

- 起飞：CommandTOL + 服务层到高确认。
- 移动：OFFBOARD（预热 setpoint + 切模 + 持续发布带新时间戳 setpoint）。
- 降落：AUTO.LAND + 服务层落地判定。
- 返航：AUTO.RTL + 服务层返航落地判定。
- 打印：仅遥测展示，不参与任何飞控命令发布。
