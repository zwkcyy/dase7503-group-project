# NAV／VI 一键运行与交付

适用：Ubuntu 24.04、ROS 2 Jazzy。本机使用 WSL 2；所有命令在 Ubuntu 终端执行。
已采用交付 CAD 与 EC 回填数据作为开发基准；硬件驱动和实体行为进入现场验证阶段。
本文列出的离线演示不依赖 EC 私有子仓库。

2026-10-26：已接收 CAD 模型与 EC 通信岗回填表，新增第八个包和 `cad`、`hardware`、`ec` 入口。
当前接入与实物边界见 [CAD / EC 接入记录](NAV_VI_CAD_EC_接入记录.md)。下文七包构建数为上一阶段记录。

## 1. 本机立即运行

先在旧演示的启动终端按 Ctrl+C，一次运行一个演示。在 Ubuntu 中进入共享源码：

```bash
cd '/mnt/e/研究生/sem1课程/7503/dase7503-group-project'
bash scripts/nav.sh demo
```

脚本自动加载 ROS 和 `/home/adminis/dase7503-nav-work/install`，使用本机隔离域 75。
会打开 RViz 和扫码窗口；启动自检通过后才启动任务。无需手动点目标或启动扫描。

完整顺序：

1. 模拟相机发布 START 图片，真正的视觉节点连续确认后，任务开始计时。
2. 实际 Nav2 导航至示例货架；模拟相机按任务状态提供货架二维码图片。
3. 视觉节点从图片中解码；任务核对完整内容，再请求模拟升降。
4. 升降反馈后，确认两个代价地图和自体过滤的载货轮廓更新。
5. 导航至目的地，实际解码 END，模拟放下货架并更新空载轮廓。
6. 重复四个货架，终端显示 `Mission state: COMPLETE`。

图像由任务状态生成，不建模货架在镜头里的实际位置、视野、对位或光照。
运动、激光、IMU、轮速与升降仍为模拟输入；路径规划、定位融合、二维码解码和任务控制使用实际软件。

完成后在启动终端按 Ctrl+C。关闭某个窗口不会停止整套节点。
无图形桌面时：

```bash
bash scripts/nav.sh demo rviz:=false show_results:=false
```

## 2. 更新、安装与编译

源代码更新后：

```bash
bash scripts/nav.sh build
```

其他电脑先安装 Ubuntu 24.04 与 ROS 2 Jazzy，下载仓库，然后执行：

```bash
bash scripts/nav.sh install
bash scripts/nav.sh build
bash scripts/nav.sh demo
```

`install` 安装 NAV／VI 公共依赖，会要求 sudo。它不安装未知型号的雷达、相机或 EC 串口适配器。
`build` 将仓库 nav、vi、common 源码链接到 `~/dase7503-nav-work/src`，构建七个包。
输出留在 Linux 文件系统。若已有源码链接指向另一份仓库，脚本拒绝覆盖。
可用 `NAV_WORKSPACE=/自选路径 bash scripts/nav.sh build` 选择另一个工作空间；后续命令使用相同变量。

Windows 的 PowerShell 不能直接运行上述 Bash 命令；先运行 `wsl -d Ubuntu-24.04`。

## 3. 单独检查是否就绪

保持演示运行，在第二个 Ubuntu 终端进入同一仓库，运行：

```bash
bash scripts/nav.sh doctor
# 如果只运行了二维码图片回放：
bash scripts/nav.sh doctor --mode vision
```

输出 `ready: true` 表示所选检查项持续就绪至少一秒。否则 `issues` 列出问题。
检查只读，不发送速度、升降或扫描命令。

| 检查 | 内容 |
|---|---|
| 图像 | 消息到达、非空图像、frame_id、ROS 时间戳与新鲜度 |
| 定位输入 | 过滤激光、轮里程计、融合里程计、IMU 持续更新 |
| TF | map→odom→base_link，以及 laser、imu_link、camera_optical_frame |
| 导航 | AMCL、规划器、控制器、BT navigator 已 ACTIVE，导航 action 可用 |
| 接口 | 扫码开始／停止、轮廓切换、两个代价地图参数服务可用 |
| 升降 | 指令订阅者、反馈发布者存在；不会为了检查而动作 |
| 扫码来源 | 恰好一个二维码结果发布者，避免混入另一个模拟扫码节点 |

自检是启动门槛，不是持续急停控制器，也不能证明硬件、图像内容或升降完成语义正确。
运行过程仍由现有任务超时、Nav2 故障处理和 EC 停止机制处理；实车停止效果需要现场验收。

## 4. 异常场景和重复验证

错码与空白图像会经过真正的解码节点，任务应进入 ERROR，不应继续升降：

```bash
bash scripts/nav.sh demo image_scenario:=wrong_qr
bash scripts/nav.sh demo image_scenario:=missing_qr
```

每次先 Ctrl+C 结束上一项。完整自动检查有独立入口，不会在普通启动或编译时自动运行：

```bash
bash scripts/nav.sh check offline
bash scripts/nav.sh check robustness
bash scripts/nav.sh check integrated
bash scripts/nav.sh check integrated_faults
bash scripts/nav.sh check preflight
# 包括既有的任务异常、导航、六类二维码、取消、轮廓服务检查：
bash scripts/nav.sh check all
```

检查使用 localhost 域 100–130；不要同时启动两个自动检查套件。
报告默认保存到 `output/nav-validation-current/`。图像鲁棒性报告在其 `robustness/index.html`。
GitHub Actions 执行离线逻辑和复杂图片检查；它不运行实机，也不证明 ROS 集成或树莓派性能。

## 5. 每次运行的证据

完整演示默认在 `~/.ros/nav_vi_runs/integrated-日期-随机后缀/` 新建记录目录。
可以指定 `output_dir:=/某个新目录`，但不要复用已经运行过的目录。

| 文件 | 内容 |
|---|---|
| preflight.json | 启动检查是否通过、缺失项 |
| events.jsonl | 状态切换和每次确认的完整二维码内容、接收时间 |
| summary.json | 最后状态、确认序列、观察者收到 START 后的耗时 |
| qr_scan.csv | 原有视觉节点的逐次扫码日志 |
| qr/*.png、qr/*.json | 真正触发确认的输入图像、图像时间戳、内容、解码器和来源 |

运行后生成方便查看的汇总页面：

```bash
bash scripts/nav.sh report ~/.ros/nav_vi_runs/这里替换成实际运行目录
```

生成 `index.html`、`confirmations.csv`、`evidence_manifest.json`。页面列出 START、A/B/C/D、END
六类是否齐全。反复扫描 END 会保留每次记录。记录中的时间是软件观察值，不是课程正式计时。

## 6. 正式截图留档

课程要求每个二维码解码结果的屏幕截图。保存的输入 PNG 不是窗口截图，不能互相替代。

现在可以用 DEMO 标签演练留档。收到本组正式二维码后：

1. 使用实际相机与本组正式二维码，依次扫描 START、A、B、C、D、END。
2. 每次确认后检查窗口内完整内容、类别和时间；按 Windows `Win+Shift+S` 截取窗口。
3. 将六张图分别命名为 `01_START.png`、`02_RACKA.png` 至 `05_RACKD.png`、`06_END.png`。
4. 同时保留对应记录目录、Git 提交号、相机／环境说明。
5. 使用报告清单核对六类齐全，再交给小组汇总。

DEMO、synthetic、image_replay 来源必须标明离线演示。正式扫码和实车比赛记录另建目录。
软件从不自动把记录标为实车验收通过。

## 7. 实车统一启动入口

已实现 `robot_bringup/system.launch.py`。启动车体、雷达、相机和 EC 驱动前，先取得真实协议和尺寸。
填好并确认 mechanical.yaml、mission_params.yaml；初始位姿必须来自实测。

下面是参数格式示例，`X/Y/YAW` 必须替换成 map 坐标系中的实测 m/rad 值：

```bash
bash scripts/nav.sh robot geometry_file:=/实际路径/mechanical.yaml \
  mission_file:=/实际路径/mission.yaml initial_x:=X initial_y:=Y initial_yaw:=YAW
```

默认使用已经启动的外部相机，且初始轮廓按载货保守处理。
接入确认后的 camera_ros 时可使用 `start_camera:=true`；开窗口加 `show_results:=true`。
完整入口先拒绝未确认配置，再启动 NAV／VI，自检通过后才启动任务。
雷达与 EC 驱动必须独立按实际型号／协议启动；此入口不会猜测串口命令。

实车检查用 `bash scripts/nav.sh robot-doctor`，它保留当前 ROS_DOMAIN_ID 和网络设置；
离线 `doctor` 会切换到本机模拟域。使用自定义图像话题时，启动和检查都传入相同话题。
统一入口使用现有标准二维码话题和服务；需要自定义这些接口时使用各组件 launch。

## 8. 完成范围与剩余对接

本次补齐：真实视觉与导航自动联跑、启动检查、统一入口、安装构建脚本、复杂图片基准、
自动回归、每次确认图像与记录、提交材料的留档流程。

仍待实物／外部数据：CAD 实测尺寸和传感器位姿、EC 权限／协议／适配归属、真实雷达与相机、
正式地图坐标和二维码、货架对位与进入／退出动作确认、实车调参、四货架全流程及异常停止验收。
原接口表明确排除的 EC 内部 PID、编码器和传动细节继续不由 NAV／VI 承担。
