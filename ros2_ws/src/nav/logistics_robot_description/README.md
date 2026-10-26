# 小车总装 URDF（ROS 2）

源模型：`E:/7503A/小车/总装.SLDASM`。只读导出，源总装与零件未保存改写。导出日期：2026-10-25（北京时间）。

## 可直接使用的文件

- `urdf/logistics_robot.urdf`：34个实体部件；四个车轮为连续关节，其余部件保持当前CAD姿态。
- `urdf/model_snapshot_fixed.urdf`：相同外形、位置和质量，全部关节固定，用于严格的静态模型对照。
- `meshes/`：16个共享米制STL，全部来自原生零件显示三角化；车轮恢复自项目内原始STEP。
- `config/mass_estimates.csv`：每项质量、估计范围和依据。
- `config/export_manifest.json`：每个CAD实例与URDF链接的映射、变换、质量、质心和惯量。
- `config/validation_report.json`：真实URDF解析器、正运动学、网格位置/单位、惯量和源文件哈希检查。
- `robot_preview.png`：从导出网格与变换渲染的完整模型，含恢复的四轮。

## base_link 与坐标

`base_link`位于**模型全部实际网格的轴对齐几何包络中心**，包含恢复后的四个车轮；不是估计质心，也不是地面投影。

- CAD坐标中的中心：`(0.005033, 40.817945, 1143.283354) mm`。
- ROS轴：X向前（朝模型相机朝向，等于CAD -X），Y向左（CAD +Z），Z向上（CAD +Y）。
- 零位总外形：约`282.5 × 286.6 × 223.0 mm`（ROS X/Y/Z）。
- 零位最低点在`base_link`下方约`111.52 mm`。
- 所有网格单位为米，URDF mesh scale=`1 1 1`。不要再乘0.001。
- 根`base_link`不设惯量，避免KDL根惯量限制；实体惯量放在对应子链接。额外物料放在独立的固定质量链接中。

## 模型与引用处理

读取源总装时，四个`mecanum wheel.step-*`组件指向失效的SolidWorks临时缓存目录，处于未加载/压缩状态。导出使用项目现有`小车/底盘/mecanum wheel.step`的原始几何，并保留总装中已有的四个实例变换；没有重画、挪动或缩放车轮，也没有改写源总装。

**源车轮当前姿态的X/Z包络约58.3 mm，绕本地Y轴的最大网格回转直径约60.2 mm；物料表写65 mm。按用户“其他都按模型”要求，URDF保留原始网格，不缩放到65 mm。**控制器轮径参数应先核实真实车轮，不能直接把本模型半径当成65 mm车轮的半径。左右轮图样也按源模型原样保留，没有自动镜像成不同手性。

所有其他可用实体均按其模型三角网格和原始组件变换导出。子装配的Transform2已是相对根装配的变换，没有重复乘父装配。外观使用零件级颜色；STL不包含CAD表面纹理或多材质贴图。

## 关节

四个连续关节：

- `front_left_wheel_link_joint`
- `front_right_wheel_link_joint`
- `rear_left_wheel_link_joint`
- `rear_right_wheel_link_joint`

转轴取原车轮几何的本地Y轴（URDF axis=`0 1 0`），各轴在base_link中约朝-Y。电机正反转及控制器符号需据此配置。四轮零位保持CAD的当前旋转角，没有人为归零重排滚子。

**升降机构固定于当前CAD姿态。**CAD中的凸轮接触与托盘导向构成闭环，普通URDF树不能直接表达该非线性闭环。此交付没有编造舵机零位、托盘行程、凸轮函数或线性mimic关系，也没有声称可以直接驱动完整升降仿真。若要运动仿真，应在实际机构运动关系基础上增加SDF约束或控制/接触实现。

相机、雷达和控制板链接保留其模型自身原点与安装旋转。它们不自动等于精确光心、雷达出光中心或IMU裸芯片中心，因此未凭空增加标定用传感器坐标系。

## 质量与惯量估算

**名义整车质量约1.635 kg，保守估计范围约1.285–2.033 kg。不是实称重量。**

- 上、下亚克力板：按CAD实际实体体积×PMMA密度1190 kg/m³；两片合计约304.9 g。
- 打印件：假设PLA密度1240 kg/m³。较厚托盘/底座取60%有效实心率，凸轮70%，较小薄壁支架和立柱90%。这些是估计值，并非实际切片参数。
- 舵盘：假设尼龙1140 kg/m³，按实体体积。
- 310编码器电机：70 g/个，共4个，采用厂商同系列参考规格。
- 麦克纳姆轮及安装适配件：60 g/个，共4个，属于工程估计，范围40–85 g/个。
- MG996R舵机：55 g。
- MS200雷达：40 g。
- 控制板：估计65 g；相机：估计4 g。

源CAD均采用约1000 kg/m³的默认密度，不能直接当真实物料重量。本导出用上述估计质量替换默认质量。已建模件的质心沿用CAD几何质心，惯量按实际模型形状和估计质量缩放，并校正SolidWorks惯性积与URDF惯量张量的符号差异。

总装未独立建模但物料表列出的安装设备，**只增加惯量，不添加虚构外观几何**：

| 物料 | 名义质量 | 质量位置处理 |
|---|---:|---|
| 7.4V、标称9900mAh电池包 | 200 g | 估计位于已建模电池安装块上方；无法仅凭铭牌容量确定真实重量 |
| Raspberry Pi 5+散热器/风扇+SD卡 | 85 g | 用未占用的右侧电路板垫高柱位置估计 |
| 雷达USB转接板 | 10 g | 未建模，暂并入雷达安装位置 |
| USB Wi-Fi接收器 | 8 g | 暂并入树莓派位置 |
| 已安装线束、紧固件等 | 35 g | 未建模，暂分布于整机中心附近 |

这些未建模设备的**质心、惯量也属于估算**，详见manifest。外置充电器、树莓派开发电源、插头转换器、读卡器和外部显示连接线不计入行驶机器人质量。

估计整机质心相对base_link约`(-7.73, -9.69, -33.92) mm`。此偏移来自当前估计质量与几何分布，**不会移动用户指定的base_link**。

## ROS 2 使用

把本目录放入ROS工作空间的`src`下：

```bash
cd ~/ros2_ws
colcon build --packages-select logistics_robot_description
source install/setup.bash
ros2 launch logistics_robot_description display.launch.py
```

显示全固定快照：

```bash
ros2 launch logistics_robot_description display.launch.py model:=model_snapshot_fixed.urdf
```

需要`ament_cmake`、`robot_state_publisher`、`joint_state_publisher_gui`和`rviz2`。没有硬件驱动、ros2_control配置或Gazebo传感器插件。碰撞体也沿用原网格；需要高性能动力学仿真时再单独生成碰撞近似。

## 已完成的检查

- `urdf_parser_py 0.0.4`成功解析两份URDF；唯一根为`base_link`，36个链接、35个关节。
- 每个网格文件存在且二进制STL完整，单位为米。
- 每个实体网格的零位世界位置与CAD源变换一致；最大坐标误差约6.3e-13 m（浮点数舍入）。
- `base_link`在几何包络中心；总尺寸和总质量回读匹配。
- 所有实体惯量正定且满足惯量三角不等式；CAD张量符号已用独立网格体积分核对。
- 19个源文件的SHA-256未变化。
- 已查看源装配四视图和导出网格整机预览。

本机没有启动ROS/RViz或Gazebo；已验证解析、数值一致性和几何，不声称完成了ROS运行或动力学仿真实测。

## 来源

- 用户物料表：`3. Group Projects Materials.pdf`，第1–9页。
- 用户CAD与`小车/打印件`、`小车/亚克力板`文件。
- 310电机厂商参考质量：https://www.yahboom.net/study/wheeled_chassis/14334/
- MG996R厂商55g规格：https://torqpro.com/product/mg996r/
- MS200原厂约40g规格：https://www.orbbec.com/wp-content/uploads/2023/07/MS200-dToF-Lidar-user-manual-A0-20230605-1.pdf
- PMMA密度：https://www.plexiglas.de/en/contact
- PLA密度参考：https://store.bblcdn.com/s7/default/ecb663b46ebb4fb984786d33befb8d2f/PLA_Pure_TDS.pdf
- SolidWorks网格单位：https://help.solidworks.com/2024/english/api/sldworksapi/SolidWorks.Interop.sldworks~SolidWorks.Interop.sldworks.IPartDoc~GetTessTriangles.html

