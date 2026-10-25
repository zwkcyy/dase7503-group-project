# NAV／VI 本地运行与对接进度

更新：2026-10-25。环境：Windows + WSL 2 Ubuntu 24.04 + ROS 2 Jazzy。

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

## CAD 仍需提供

填写 `ros2_ws/src/nav/robot_description/config/mechanical.yaml` 的对应数据：

- 车体尺寸、离地间隙；轮子尺寸和安装距离用于模型显示。
- 激光、相机、IMU 相对 `base_link` 的位置和方向。
- 包含突出部件的空载包围框，以及完整载货后的包围框。
- 激光自体过滤高度和边界，实际取货、放货所需间隙。

长度用 mm，角度用 rad。载货包围框必须包含空载包围框。
真实配置保留 `confirmed: false`，直到 CAD 核对。当前 `loaded_bounds_mm: null` 是明确待填，
不能用演示尺寸替代。完整实车导航启动会拒绝未确认／未填完整的配置。

## EC 仍需确认

目前小组仓库固定引用 `CaspianW/7503esp32` 的 `2b56582`。
EC README 表示其使用 ASCII 串口，原生 micro-ROS 尚未实现；私有源码目前无读取权限。

| 待确认项 | 需要提供的结果 | 当前状态 |
|---|---|---|
| 固件读取与版本 | 授予读取权限，确认使用哪个提交 | 待 EC |
| 通信路线 | 原生 micro-ROS，或树莓派串口桥接；明确适配代码负责人 | 待 EC／NAV 协商 |
| 若选串口 | 端口／波特率、消息边界、命令字段、单位、反馈格式、错误返回及有效样例 | 待 EC 协议；目前不能猜协议写适配器 |
| 速度输入 | 确认现有 `/cmd_vel` 的 Twist、vx/vy/wz 单位与轴方向如何落到固件 | 待联调 |
| 里程计与 IMU | 对应 `/odom/wheel`、`/imu/data_raw` 的类型、单位、时间戳、坐标系、发布频率 | 待联调 |
| 升降 | 确认 `/lift_cmd` 和 `/lift_state` 的 0/1 含义、何时反馈完成、失败如何表示 | 待 EC |
| 停止行为 | 断连、停止发速度、取消任务后机器人实际如何停止 | 待实车验收 |

继续遵守原接口表中“不需要管”的排除项；不增加 EC 内部 PID、编码器细节、减速比等任务。
升降 Int8 接口本身没有请求编号，迟到反馈的归属问题仍需 EC 完成语义配合。

## 场地与跨组验收

- 根据正式场地图或实测，填写任务接近位姿、放货位姿、四个完整二维码内容。
- 确认初始定位、空载／载货轮廓、激光过滤不会把需要避让的东西遮掉。
- 验证相机在真实光照、距离和运动模糊下的识别。
- 验证实车横移、旋转、停车精度、升降取放和异常停止。
- 用实际场地和载荷完成四货架全流程计时；仿真耗时不作为比赛达标证据。

原有 CAD、EC、跨组验收 PDF 继续用于回填。本次新增软件入口和运行记录详见此文档及
`docs/NAV_VI_验证记录.md`。
