# NAV／VI 本地运行与对接进度

更新：2026-10-25。环境：Windows + WSL 2 Ubuntu 24.04 + ROS 2 Jazzy。

2026-10-26 补充：已收到并导入 CAD 总装模型，依据 EC 通信岗回填表实现适配器，当前构建包含 8 个包。
使用 `bash scripts/nav.sh cad` 查看新模型；`hardware` 用于空载接入。
最新数据、命令和边界见 [CAD / EC 接入记录](NAV_VI_CAD_EC_接入记录.md)。下文保留上一阶段运行记录。

后续新增的完整联跑、自检和一键脚本见 [一键运行与交付](NAV_VI_一键运行与交付.md)。
当前构建已包含第七个包 `robot_bringup`，推荐在仓库中执行 `bash scripts/nav.sh build`。
以下保留原先各组件的单独运行方法；导航加真实视觉的自动演示使用 `bash scripts/nav.sh demo`。

## 现在完成了什么

| 工作 | 状态与边界 |
|---|---|
| ROS 依赖、编译 | 导航、任务、视觉、模型、机械参数、仿真共六个包可编译 |
| 任务演练 | 真实 ROS 通信，模拟导航到达、二维码和升降反馈；四货架流程及异常分支可运行 |
| 导航仿真 | 实际 EKF、AMCL、Smac、MPPI 和 Nav2，使用二维模拟运动、激光、轮速和 IMU |
| 参数集中管理 | 一份机械 YAML 供机器人模型、传感器 TF、避障轮廓与激光过滤使用 |
| 空载／载货切换 | 升降新反馈之后更新两个代价地图，确认成功才继续导航；失败停止任务推进 |
| 无摄像头识别 | 图片、录像或生成的二维码输入现有视觉节点，实际识别并发布结果 |
| 中断处理 | 任务接到 Ctrl+C 后请求取消导航并停止扫描；真实 ROS 模拟对端已确认响应 |

仿真尺寸、目标坐标和二维码后缀都是示例，不代表比赛数据。二维模型没有真实轮胎打滑、
接触力、载荷运动或取放动作；二维码和升降在导航任务演示中仍由模拟节点返回。
实车急停、定位精度、取放成功率和比赛时间仍需现场验收。

## 这台电脑如何运行

已经准备了 `/home/adminis/dase7503-nav-work` 工作空间，其 `src/nav`、`src/vi`
链接到 Windows 项目目录的源代码；不会另造一份容易混淆的代码副本。
当前 Windows 项目分支为 `nav/offline-integration`。切换源代码分支后需要重新编译。

### 1. 进入 Ubuntu

如果看到 `PS C:\...>`，在 PowerShell 输入：

```powershell
wsl -d Ubuntu-24.04
```

如果已经看到 `adminis@...$`，直接继续。后续全部在 Ubuntu 终端执行。

### 2. 每个新终端加载环境

```bash
source /opt/ros/jazzy/setup.bash
source ~/dase7503-nav-work/install/setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

一次只运行下面一种演示。按 Ctrl+C 结束后再启动另一种。

### 3. 先运行任务演练

```bash
ros2 launch robot_simulation rehearsal.launch.py
```

正常结束会显示四次 `Delivered rack`，最后出现 `Mission state: COMPLETE`。
节点保留在终端便于查看，按 Ctrl+C 退出。这一步用模拟对端验证任务组织和 ROS 通信。

可单独运行故障场景，例如：

```bash
ros2 launch robot_simulation rehearsal.launch.py scenario:=lift_timeout
ros2 launch robot_simulation rehearsal.launch.py scenario:=wrong_qr
ros2 launch robot_simulation rehearsal.launch.py competition_time_s:=2.0
```

这些场景预期进入 `ERROR`，并打印具体原因；出现预期错误才是故障处理成功。

### 4. 打开导航仿真

```bash
ros2 launch robot_simulation navigation_demo.launch.py mission:=true
```

RViz 打开后可以看到地图、机器人、激光和规划轨迹。此命令自动运行四个示例货架任务。
不需要摄像头、ESP32 或真实机器人。关闭 RViz 不等于停止后台节点，结束时在启动终端按 Ctrl+C。

如果只想手动在 RViz 设置导航目标：

```bash
ros2 launch robot_simulation navigation_demo.launch.py
```

若图形界面暂时不可用，可以 `rviz:=false` 在终端运行。初始位置来自演示场景文件，
不用在 RViz 再猜一个初始位置。场地中央障碍物由模拟激光检测，并非静态地图中的固定墙体。

### 5. 不接相机，运行真正的二维码识别

```bash
ros2 launch robot_simulation vision_replay.launch.py payload:=RACKA_DEMO
```

保留窗口，打开第二个 Ubuntu 终端，执行第 2 步的四行环境命令，再输入：

```bash
ros2 service call /qr/start_scan std_srvs/srv/Trigger '{}'
```

扫码窗口应显示 `RACKA_DEMO`。识别确认后自动停止扫描，结果保留。
要换一个码并再次扫描，在第二个终端运行：

```bash
ros2 param set /image_replay payload END
ros2 service call /qr/start_scan std_srvs/srv/Trigger '{}'
```

也可以使用自己的本地图片／录像，路径必须是 Ubuntu 能访问的路径：

```bash
ros2 launch robot_simulation vision_replay.launch.py input_file:=/absolute/path/example.mp4
```

### 6. 源码更新后重新编译

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dase7503-nav-work
colcon build --symlink-install --packages-select robot_geometry robot_description robot_navigation robot_mission robot_vision robot_simulation
source install/setup.bash
```

## 其他电脑首次准备

在装好 ROS 2 Jazzy 的 Ubuntu 24.04 上：

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization ros-jazzy-laser-filters ros-jazzy-xacro ros-jazzy-cv-bridge python3-colcon-common-extensions python3-opencv python3-numpy python3-yaml python3-tk python3-pil python3-pil.imagetk fonts-noto-cjk
```

下载包含本次改动的分支，或在该 PR 合并后下载 main，在仓库的 `ros2_ws` 下编译上述六个包。
把每个终端的环境命令中的工作空间路径换成自己的 `ros2_ws/install/setup.bash`。
这些 NAV／VI 演示不要求下载 EC 私有子模块。

## CAD 已采用；剩余项目由现场验证推进

本轮已导入机械组模型，采用 `mechanical_cad_20261025.yaml` 中的模型尺寸、空载轮廓和传感器参考点。
`development_baseline_accepted: true`，空载入口可以直接使用，不再等待 CAD 逐项确认。
旧的 `mechanical.yaml` 为历史草案，新入口默认使用新文件。

仍需现场处理：检查实际线束等突出物、测量载货包围框、检查自体过滤、记录入架／退架间隙，
以及处理 CAD 当前 223 mm 高度与课程 180 mm 限制的差异。
`confirmed: false` 表示尚无完整实物核对记录；`loaded_bounds_mm: null` 表示货架外形尚未测量。
我们测量后即可更新，不要求 CAD 重新提供整份数据。

## EC 已采用；剩余项目由运行和观察推进

使用通信岗回填表中 `40d3041` / Python 桥接 v0.1.0 的约定：ASCII 串口、115200、SI 单位、
10 Hz 里程计与 IMU。原始话题隔离到 `/ec/...`，NAV 适配代码已实现。
主仓库子模块仍为 `2b56582`，私有源码尚不可读；实际运行需要现有 `tools/ros_serial_bridge.py` 文件。

| 项目 | 已定软件方案 | 现场工作 |
|---|---|---|
| 通信路线 | 现有 EC Python 桥接加 NAV ROS 适配器 | 运行对应桥接文件、连接实际串口 |
| 速度 | Twist，m/s 和 rad/s，限速与超时处理 | 观察前后、横移、旋转和断连停车 |
| 里程计／IMU | 统一话题，采用声明的 base_link 轴，10 Hz EKF 参数 | 观察时间戳、频率、符号和定位效果 |
| 升降命令 | Int8 转可配置 String；默认完整 UART 文本 | 运行时核对桥接是否自行添加 lift 前缀 |
| 升降完成 | UInt8 仅作目标上报；提供人工观察确认服务 | 测量实际动作、取得可用于自动完成判断的依据 |

继续遵守原接口表“不需要管”的范围；不增加 EC 内部 PID、编码器、运动学、舵机引脚／角度／时间调参任务。
人工确认不能记为自动取放已完成；这属于实车功能验证，不再列成等待 EC 填表的事项。

## 场地与跨组验收

- 起终区域已按课程图填入，起点中心摆放约定也已给出；现场记录货架身份、任务接近／放货位姿及四个完整二维码。
- 确认初始定位、空载／载货轮廓、激光过滤不会把需要避让的东西遮掉。
- 验证相机在真实光照、距离和运动模糊下的识别。
- 验证实车横移、旋转、停车精度、升降取放和异常停止。
- 用实际场地和载荷完成四货架全流程计时；仿真耗时不作为比赛达标证据。

原有 PDF 是本轮接入前的状态，不再据此重复向 CAD / EC 索取已有数据。
当前方案、启动命令与现场工作以 [CAD / EC 接入记录](NAV_VI_CAD_EC_接入记录.md) 为准；
构建情况见 `docs/NAV_VI_验证记录.md`。
