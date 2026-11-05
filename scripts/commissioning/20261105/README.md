# 2026-11-05 无线静态检查工具

这些脚本保存本次实测方法与收尾修复。结果与限制见 [无线静态联调记录](../../../docs/NAV_EC_无线静态联调记录_20261105.md)。

## 环境约定

- Pi 已配置 ROS Jazzy、相机和 MS200 驱动，以及 `~/dase7503-commissioning/session.env` 和 `start-camera.sh`、`start-lidar.sh`、`start-ec.sh`。
- `start-ec.sh` 在本轮使用 `enable_motion:=false enable_lift:=false`；启动前必须确认没有其他控制节点。关闭适配器开关不等于隔离直接 EC 命令。
- 本轮构建目录带日期 `20261105`；这些脚本保留该会话路径。移植环境应先检查路径、设备和配置。
- Windows 接收器保留本轮 IP、账号、SSH 私钥文件**路径**及专用 known-hosts 文件路径；仓库不提供私钥和密码。主机身份通过已验证的原主机别名校验。

## 文件用途

| 文件 | 用途 |
| --- | --- |
| `pi_wireless_static_soak.py` | 在 Pi 启动自有传感器进程，静态观测，逐行输出 JPEG/scan/status JSON；异常写报告并清理自有进程 |
| `receive_wireless.mjs` | Windows 通过 SSH 接收并保存真实数据，检查序号、预览格式、接收间隔与最终报告 |
| `ping-monitor.ps1` | Windows 并行 ICMP 探测；ICMP 结果独立于 SSH 应用数据结果 |
| `lidar_restart_check.py` | Pi 上五次真实 MS200 采集和 SIGINT 退出检查；不会启动底盘或升降 |

观测脚本会主动启动硬件驱动，**不能当作离线单元测试执行**。`--output` 必须是不存在的新目录；不得覆盖先前报告。Pi 观测器和雷达检查均不发布运动命令、不调用使能服务，但它们不能阻止别的程序或固件控制硬件。

本轮部署时把 `pi_wireless_static_soak.py` 复制为 Pi 上的 `~/dase7503-commissioning/wireless-20261105/wireless_static_soak.py`，接收器使用这个路径。执行前先阅读脚本中的路径约定，再选择新运行编号。

实际运行过的示例（输出目录与编号再次执行时必须更换）：

```powershell
node scripts/commissioning/20261105/receive_wireless.mjs output/pi_wireless_validation_20261105 run-20261105T123830Z 120
```

该工具只传约 1 fps 压缩预览与抽样扫描。它不验证移动网络覆盖、物理急停、低速行驶、相机几何标定或落地避障。
