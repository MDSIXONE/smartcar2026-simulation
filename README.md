# SmartCar 2026 仿真环境：结构说明

这是 SmartCar 2026 任务三的 ROS 1 / Gazebo Classic 工作区。最终导航栈只有一套：
`cym_planner/CymPlanner`、`gazebo_nav.launch` 和 `config/move_base`。

## 文档

- [部署说明](DEPLOYMENT.md)：环境准备、构建和启动方式。
- [快速操作](TASK3_RUNBOOK.md)：任务三的启动、执行、标定与故障处理。
- 本文：项目目录与启动链路。

## 目录

| 目录 | 内容 |
| --- | --- |
| `src/car3` | 车辆 URDF、机械臂、相机、物块、任务节点与 Gazebo 启动文件 |
| `src/gazebo_map` | Gazebo 世界、静态地图与加工区资源 |
| `src/gazebo_nav` | 最终的 `move_base`、代价地图与导航启动文件 |
| `src/cym_planner` | CYM 局部规划器源码与参数 |
| `src/roboticsgroup_gazebo_plugins` | Gazebo 机械臂/夹爪辅助插件 |
| `datasets/cube_yolov5` | 三类别机械臂静止九宫格样例集、YOLOv5 标签与检查预览 |

## 启动链路

```text
task3_prepare.launch
  └─ car3/gazebo.launch
       └─ gazebo_nav/gazebo_nav.launch
            ├─ move_base + cym_planner
            ├─ map_server
            └─ RViz (rviz/navigation.rviz)
```

`task3_execute.launch` 在准备阶段完成后执行物品取放任务。

## 环境

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic
- catkin 工作区
