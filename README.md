# SmartCar 2026 仿真环境

面向 SmartCar 2026 任务三的完整 ROS 1 / Gazebo Classic 工作区源码。它包含车辆、机械臂、摄像头、随机物块、地图、导航栈，以及 `cym_planner` 局部规划器。

## 运行环境

- Ubuntu 20.04
- ROS Noetic（含 Gazebo Classic、navigation、map_server、rviz）
- catkin 工作区

## 快速开始

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 终端 1：启动并完成平滑的机械臂初始姿态设置
roslaunch car3 task3_prepare.launch gui:=true rviz:=true

# 终端 2：收到物品名称后执行取放任务（示例）
source /opt/ros/noetic/setup.bash
source ~/smartcar2026-simulation/devel/setup.bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

任务节点会根据物品类别选择物块，按实时物块位置识别左、上、右区域；到取物点后才启用机械臂控制，抓取成功后以原导航速度的 80% 送往对应加工区。

支持的常用类别为食物、日用品、电子产品；不在别名表中的名称可显式指定类别：

```bash
roslaunch car3 task3_execute.launch cargo_category:="电子产品" cargo_name:="手机"
```

完整任务说明、手动标定、故障排查与参数说明见 [TASK3_RUNBOOK.md](TASK3_RUNBOOK.md)。

## 目录

| 目录 | 内容 |
| --- | --- |
| `src/car3` | 车辆 URDF、机械臂、任务控制、抓取与物块生成脚本 |
| `src/gazebo_map` | Gazebo 世界、地图与加工区资源 |
| `src/gazebo_nav` | `move_base`、代价地图和导航启动文件 |
| `src/cym_planner` | CYM 局部规划器源码与参数 |
| `src/roboticsgroup_gazebo_plugins` | Gazebo 附着/抓取插件 |

## 单独使用规划器

若只需要 `cym_planner`，请使用同目录发布的 `cym_planner_standalone_20260713.zip`；该包不包含车辆模型、Gazebo 场景或任务脚本。

## 注意

- 本仓库仅包含源码，首次使用必须执行 `catkin_make`。
- 官方新模型已作为 `src/car3/urdf/car3.urdf` 生效。
- 请在拥有相关模型、课程与竞赛资料授权的范围内使用和分发。
