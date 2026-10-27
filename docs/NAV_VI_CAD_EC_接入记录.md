# CAD 模型与 EC 回填数据接入

更新：2026-10-26。按 NAV / VI 负责人要求，使用现有交付资料继续实现。
首次接入仅完成软件与构建；随后按用户要求新增并运行验证，结果见
[软件验证记录](NAV_VI_验证记录.md)。没有进行实车动作或现场标定。

## 本轮直接确定的方案

以下由 NAV / VI 直接采用，不再等待 CAD / EC 重新确认或填写：

| 项目 | 已定方案 |
|---|---|
| 几何基准 | 使用交付 CAD 的实际网格和装配位置，空载轮廓加 10 mm 余量 |
| 车体坐标 | CAD 原点的地面投影为 NAV `base_link`，X 前、Y 左、Z 上 |
| 传感器参考点 | 相机／雷达用组件原点；IMU 用控制板参考点；相机增加标准光学坐标 |
| 通信路线 | 使用 EC 表中已有的 Python 串口桥接，115200 波特率；NAV 在 ROS 侧适配 |
| ROS 接口 | EC 原话题隔离到 `/ec/...`；导航沿用 Twist、Odometry、Imu、Int8 |
| 单位与融合 | EC 已报 SI 单位，避免重复换算；按 10 Hz 输入配置 EKF，只融合 IMU yaw rate |
| IMU 轴向 | 采用 EC 声明的 `base_link` 轴，附加旋转为零；协方差给出可调正值 |
| 速度边界 | 合平移速度 0.5 m/s、角速度 2 rad/s；指令超时 0.25 s、遥测超时 0.5 s |
| 升降过渡方案 | `lift up/down/stop` 可配置映射；目标反馈加操作者观察确认完成 |
| 场地基准 | 采用课程图的 3020 × 2000 mm 和起终区域；墙体占据一格 20 mm |
| 起始摆放约定 | 起点区域中心 (2720, 1000) mm，车头朝向目的地区域，yaw = π |

配置的 `development_baseline_accepted: true` 表示本轮方案已采用，空载入口可直接使用。
`confirmed: false` 专用于区分“完整机械数据已经实物核对”，不是等待其他组批准的开关。
后续由我们现场观察、测量和调参；没有资料支持的实际到位信号、载货外形和比赛成绩不会填成已验证。

## 1. 数据来源与版本

| 来源 | 本次使用的内容 | 状态 |
|---|---|---|
| 小车总装_URDF模型包.zip | 2026-10-25 导出，34 个实体、16 个网格、两份 URDF | 原包作为 `logistics_robot_description` 导入，原始文件保持不变 |
| EC接口回填表_通信岗回填20261025.docx | 固件 `40d3041`、桥接 `tools/ros_serial_bridge.py` v0.1.0 | 依据回填表实现；尚无法读取私有源码进行核对 |
| 主仓库的 EC 子模块 | `2b56582` | 与回填表版本不同，本轮没有改动子模块指针 |
| 课程材料和厂商说明 | Yahboom MicroROS 控制板、板载 ICM42670P | 可确认器件资料，不代表最终固件已按厂商示例实现 |

模型 ZIP 的 SHA256：`4764c8fffd10f3920a90f093ea31f8ef583750e00eecc49a58c616b5c19ab7dc`。
供方的 `validation_report.json` 是随包交付的历史报告，不是本轮新运行的测试结果。
原模型的质量、惯量及未建模物料的安装位置含估计值，保留原始说明。

## 2. CAD 如何接到导航

原包存放于 `ros2_ws/src/nav/logistics_robot_description/`，不修改 CAD 组目录。
导航使用其中全固定的 `model_snapshot_fixed.urdf`，不需要伪造四轮 joint_states。

- 原始模型所有链接增加 `cad_` 前缀，保留网格、碰撞体、惯量及零件间变换。
- NAV 的 `base_link` 采用 **CAD 包围盒中心的地面投影**。X/Y 保持原包定义，
  CAD 原点作为 `cad_base_link`，在 NAV 原点上方约 **111.519853 mm**。
- 不移动网格在装配中的相对位置，不把 60.2 mm 的轮子缩放到 65 mm。
- 保留相机、雷达、控制板各自的 CAD 轴；另外建立导航使用的 `camera_link`、
  `camera_optical_frame`、`laser`、`imu_link`，避免同名链接方向冲突。
- 相机、雷达参考点暂用组件原点；IMU 参考点暂用控制板原点。
  它们有明确的数据来源，但仍属于未标定参考点。

配置：`ros2_ws/src/nav/robot_description/config/mechanical_cad_20261025.yaml`。

| 参数 | 使用值 / 解释 |
|---|---|
| 零位总外形 | 282.500 × 286.567 × 223.040 mm，含模型中的四轮 |
| 空载投影边界 | x ±141.251 mm，y ±143.284 mm，向外取整 |
| 前后轮中心间距 | 140.0 mm，用于模型资料；不覆盖 EC 内部运动学参数 |
| 左右轮中心间距 | 255.066707 mm，同上 |
| 轮子最大网格回转直径 | 60.203649 mm；物料表标称 65 mm 的差异保留 |
| NAV 相机参考点 | (129.750, -1.500, 121.883) mm，body 轴 X 朝前 |
| NAV 雷达参考点 | (108.750, 0, 46.883) mm，沿用 CAD 安装旋转 |
| NAV IMU 参考点 | (29.844, 0, 52.883) mm，控制板参考原点 |
| 开发基准 | `development_baseline_accepted: true`，已采用；空载入口直接使用 |
| 实物记录 | `confirmed: false`；载货外形 `loaded_bounds_mm: null`，现场测量后填入 |

高度 223.040 mm 与课程 180 mm 限制的差异没有靠缩小模型隐藏。
未建模的线束、实际电池等也可能改变实体外形。正式验收仍以实物为准。

### 空载调试模式

新增 `empty_only` 模式：允许使用这个已采用、尚未实测的开发基准启动空载导航，无需通用放行开关。
内部仅以空载轮廓生成两张代价地图；`/nav/set_loaded` 的载货请求会被拒绝。
它不会把空载轮廓写回成载货数据，也不会通过完整任务的实车配置检查。
旧的仿真配置、四货架演示和严格生产入口继续保持各自用途。

## 3. EC 适配做了什么

适配器位于 `robot_bringup/ec_adapter.py`，配置为 `config/ec_20261025.yaml`。

| EC 原接口（经启动器隔离后） | NAV 接口 | 处理 |
|---|---|---|
| `/ec/wheel/odom`，Odometry | `/odom/wheel`，Odometry | 统一原表中的 `/wheel/odom` 与导航名称；检查时间戳、frame、有限值和单位四元数；不发 TF |
| `/ec/imu/data_raw`，Imu | `/imu/data_raw`，Imu | 保留 SI 单位；按配置旋转向量和协方差；不提供姿态；全零协方差使用注明为暂定的正数 |
| `/ec/cmd_vel`，Twist | 从 `/cmd_vel` 接收 | 平移合速度限 0.5 m/s，角速度限 2 rad/s；输入和遥测超时停发运动 |
| `/ec/lift_cmd`，String | 从 `/lift_cmd`，Int8 接收 | 0/1 转换为可配置的放下／抬起字符串；未确认前不重复发同一动作 |
| `/ec/lift_state`，UInt8 | `/ec_adapter/status` | 0/1 只记作目标；2 表示无目标；不将其直接变成到位反馈 |
| 操作者观察后的确认服务 | `/lift_state`，Int8 | 只有存在待确认指令、收到新鲜且匹配的目标上报、操作者确认后才发完成值 |

启动器将 EC 原话题重映射到 `/ec/...`，防止 String/Int8 和 UInt8/Int8 使用同名话题。
**不要再同时启动一份没有重映射的 EC 桥接程序。**

### IMU 的处理依据

回填表明确写：桥接当前 `frame_id=base_link`，角速度 rad/s、加速度 m/s²，且不提供姿态。
因此默认按其声明的车体轴接收，`imu_to_base_rpy_rad=[0,0,0]`，不会再叠加一次控制板 CAD 旋转。
如实际数据保留芯片轴，可在适配器配置中设置旋转和期望输入 frame；不能只改 frame 名而不转数值。
当前 EKF 只融合 IMU yaw rate，板上芯片的毫米级平移偏移不是这一融合量的主要阻塞。
车头左转时 yaw rate 应按约定为正，需在实际上电后观察；此处没有声称已经确认。

回填表上行频率为 10 Hz。本轮提供独立 `ekf_ec_10hz.yaml`，EKF 输出保持 30 Hz，
`sensor_timeout` 取 0.35 s；适配器遥测新鲜度门槛 0.5 s。这些是调试设置，非实测性能结论。

### 速度与升降的边界

- 默认 `enable_motion=false`、`enable_lift=false`，可以先看模型和接收数据。
- 启用运动后，只有里程计和 IMU 都新鲜才转发非零速度。速度指令停止 0.25 s 或遥测过期时，
  发送一次零速然后停止额外补发；不生成 arm 或维持 EC 在线的保活指令。
- 遥测新鲜度包含消息传输途中已经消耗的时间；接收一条接近过期的消息不会重新获得完整 0.5 s。
- 固件表中 500 ms watchdog 会停车并 disarm。重新 arm 依 EC 已有工具处理；本适配器不猜测其命令协议。
- 表格未给出 Python 桥接 String 回调的精确内容约定。默认文本采用明确记载的 UART 命令
  `lift up` / `lift down` / `lift stop`，可配置；若桥接回调会自己加 `lift ` 前缀，应改成 `up/down/stop`。
- 升降没有到位传感器。不会使用目标值、`active` 或自行猜测的延时伪造完成。
  本轮提供的是人工观察确认方式，不能记为比赛自动取放已经完成。
- `stop_lift` 只在显式服务调用时发送；停止脉冲可能影响保持力，所以任务退出时不会自动发放架或停舵机命令。

## 4. 现在在这台电脑看真实 CAD

以下均在 Ubuntu 终端、仓库根目录执行。先结束旧演示，避免同一域出现重复 TF。

```bash
cd '/mnt/e/研究生/sem1课程/7503/dase7503-group-project'
bash scripts/nav.sh build
bash scripts/nav.sh cad
```

`build` 现在构建 8 个包，新增 `logistics_robot_description`。
`cad` 使用本机域 75，只显示模型和适配后的坐标，不需要相机、EC、雷达。
普通完整离线演示用 `bash scripts/nav.sh demo`，显示课程场地和实际扫码；
中央障碍调试场景改用 `bash scripts/nav.sh regression-demo`。

## 5. EC 数据调试入口

```bash
bash scripts/nav.sh hardware
```

默认显示模型、启动适配器和视觉节点、记录日志；不启动外部 EC 脚本、不启动导航或配送任务。
没有相机时视觉等待图像。可加 `vision:=false` 只看模型与 EC 状态。

如果已经有回填表对应的 EC 代码目录，可启动其现有桥接：

```bash
bash scripts/nav.sh hardware start_bridge:=true \
  bridge_root:=/实际EC仓库目录 port:=/dev/ttyUSB0
```

启动器要求该目录确实存在 `tools/ros_serial_bridge.py`，使用系统 Python 和 `python3-serial`。
本机私有仓库访问失败，所以尚未取得或运行此脚本；不会用旧子模块或自编串口帧格式冒充。
若只需要适配器，可用 `bash scripts/nav.sh ec`。外部桥接必须按启动器中的规则 remap。

观察：

```bash
ros2 topic echo /ec_adapter/status
ros2 topic echo /odom/wheel --once
ros2 topic echo /imu/data_raw --once
```

## 6. 空载导航与升降单独调试

实际雷达驱动必须先启动并发布 `/scan`。它的测量 frame 要与模型参考 frame 的定义一致，
不能仅为消除 TF 报错把未知坐标系改名。初始地图位置用现场已知的 m/rad 值替换 X/Y/YAW：

```bash
bash scripts/nav.sh hardware navigation:=true \
  initial_x:=X initial_y:=Y initial_yaw:=YAW \
  start_bridge:=true bridge_root:=/实际EC仓库目录 port:=/dev/ttyUSB0
```

此时仍禁用速度转发。确认实际轴向与定位后，需要运动时在启动参数中加 `enable_motion:=true`。
按 EC 方式 arm 后，可用 RViz 发空载目标；正式货架坐标仍未提供，不会自动开始四架任务。
此入口不启动 `delivery_mission`，不依赖伪造的 START 或 DEMO 货架坐标。

也可以按本轮确定的摆放约定，把车体原点放在课程起点区域中心，车头朝目的地区域，使用：

```bash
bash scripts/nav.sh hardware navigation:=true use_course_start:=true \
  start_bridge:=true bridge_root:=/实际EC仓库目录 port:=/dev/ttyUSB0
```

`use_course_start` 从 `field_config.yaml` 读取 (2.72 m, 1.00 m, π rad)，
与手动填写 `initial_x/y/yaw` 二选一。它只提供定位初值，不能代表机器人已经在该位置。
起点矩形 [2420, 600, 3020, 1400] mm、终点矩形 [0, 300, 800, 1700] mm
来自课程场地图第 2 页；不会作为障碍画入静态地图。
尺寸图未给货架 A/B/C/D 标签，对照示意图无法可靠恢复精确对应关系，所以货架身份和接近位姿现场记录。

```bash
bash scripts/nav.sh robot-doctor --mode navigation --empty-only \
  --geometry-file "$(ros2 pkg prefix --share robot_description)/config/mechanical_cad_20261025.yaml"
```

这个检查只表示输入、TF、导航服务就绪，不确认驱动 arm、实体坐标、升降或载货行为。

单独升降需要启动参数 `enable_lift:=true`，并确认桥接的文本格式匹配。
发出 1（抬起）或 0（放下）后，观察实物达到目标，再调用对应确认服务：

```bash
ros2 topic pub --once /lift_cmd std_msgs/msg/Int8 '{data: 1}'
# 仅在实际抬起完成后：
ros2 service call /ec_adapter/confirm_lift std_srvs/srv/SetBool '{data: true}'
# 放下用 data: 0，下放完成的确认用 data: false。
```

无待确认动作、目标不符、数据过期或超过 60 秒时，服务拒绝确认。
状态记录会明确写 `operator_only`。自动任务原有升降超时仍为 8 秒；本入口不启动它，
也没有通过放宽任务超时或竞争计时来宣称真实自动取放达标。

## 7. 本轮完成与尚不能宣称的内容

已实现：模型导入与 NAV 坐标适配、CAD 参数配置、空载轮廓模式、EC ROS 话题／类型适配、
IMU 协方差与可配置旋转、10 Hz EKF 配置、速度超时处理、人工升降确认、统一入口和状态留档。

尚未完成：运行私有 EC 桥接源码、真实传感器 frame 核对、模型高度问题的实物处理、
载货外形、真实入架／退架、正式场地坐标与二维码、自动升降完成依据、四架实车三分钟验收。
这些由现场测量、观察和现有桥接代码的实际运行推进，不要求其他组重新填写已有模型数据。

原 B6-B10、B21 的排除范围继续保留；本轮没有修改 PID、编码器、EC 运动学或舵机端口／角度／动作时间。
没有改动 `docs/interfaces/` 中的跨组合同，也没有修改 EC 私有固件。
