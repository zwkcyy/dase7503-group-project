# NAV／VI 软件验证记录

日期：2026-10-25。Ubuntu 24.04（WSL 2），ROS 2 Jazzy，系统 OpenCV 4.6。

## 已执行

| 验证 | 结果 |
|---|---|
| colcon 构建 | 六个包成功：robot_geometry、robot_description、robot_navigation、robot_mission、robot_vision、robot_simulation |
| 离线单元／回归检查 | 70 项通过，含新增机械参数、几何碰撞、激光射线、轮廓确认等待及失败处理 |
| ROS 任务演练 | 四货架完成；升降指令序列为 `[1,0,1,0,1,0,1,0]` |
| ROS 异常场景 | 错误码、无二维码、升降超时、导航中止、导航拒绝、导航超时、轮廓更新拒绝、比赛总时限均产生预期终止原因 |
| 实际 Nav2 二维仿真 | 四货架完成，收到 642 帧过滤激光、1920 条融合里程计，八次空载／载货切换符合预期 |
| 真实视觉节点 | 从图像消息解出 START、RACKA_DEMO、END；未直接伪造 `/qr/data` 来替代解码 |
| ROS 中断取消 | 中断任务，模拟对端收到扫描停止和导航取消；对端保持运行以完成应答 |
| 轮廓服务 | 两个代价地图确认后成功；其中一个拒绝时保持故障状态，后续请求不再恢复任务 |
| 启动约束 | 实车未确认的机械配置、模拟域号 0 均在启动任何节点之前被拒绝 |
| 静态检查 | 36 个 Python、10 个 YAML、7 个 XML／Xacro 解析通过；git diff 无空白错误 |

本次真实导航演示中的任务用时约 65.4 秒；这是示例坐标、理想运动和模拟升降／扫码下的结果，
不能作为实车三分钟达标的证明。并行验证启动期间出现过一次 EKF 更新周期超时提示，
因此没有将本机调度性能认定为树莓派性能结论。

## 运行中发现并修正

- 原有 AMCL 初始位姿覆盖方式未生效：日志显示仍使用 `(0,0,0)`，造成 `Start occupied`。
  改为重写节点专属 YAML 参数；复跑显示正确的 `(0.45,0.45,0)` 并完成任务。
- 新轮廓节点使用了 rclpy 的只读属性名 `clients`，已改名并复跑。
- 演练启动不再依靠固定延时猜测 Nav2 是否激活；模拟 START 会等 BT navigator 激活。
- 检查器使用持久事件历史，避免 DDS 发现稍晚导致漏掉最早的升降事件。

Nav2 Jazzy SmacPlanner2D 在配置阶段仍有已知的 inflation 提示，原导航配置文件中已有说明。
本次生成的 inflation 半径覆盖载货外接圆；规划、载货轮廓更新与全程导航均已实际运行。

## 复现

见 `ros2_ws/src/common/tests/README.md` 和 `ros2_ws/src/nav/robot_simulation/README.md`。
集成检查脚本为 `ros2_ws/src/common/tests/ros_integration.py`；模式包括 rehearsal、navigation、
vision、cancellation、envelope。输出目录包含 JSON 汇总及各场景完整日志。
本机原始结果位于 `output/nav-validation/`，未将运行日志或构建产物提交进仓库。

实车机械尺寸、EC 通信适配、激光驱动、相机成像、真实取放与停机效果尚待跨组对接。
