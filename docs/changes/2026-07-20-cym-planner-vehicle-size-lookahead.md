# CymPlanner 车长前视距离

## 变更

`CymPlanner` 的 `obstacle_lookahead_distance` 默认值由 `0.8 m` 调整为
`0.36 m`。该距离与 local costmap 的小车 footprint 前后尺寸一致：
`x = -0.18 m` 至 `x = 0.18 m`。

代码中的 ROS 参数缺省值与
`src/cym_planner/config/cym_planner_params.json` 保持一致，未提供该参数时
也会得到同样的车长前视距离。

## RViz 前视车体

`CymPlanner` 会在前视检查实际覆盖的路径末端发布
`/move_base/CymPlanner/lookahead_footprint`。Marker 直接使用 local costmap
当前的 footprint 顶点，因此显示尺寸与碰撞检查使用的车体尺寸一致（默认
`0.36 m × 0.24 m`），并会跟随该路径点的朝向。`v3_cym_nav.rviz` 已默认
启用 **Cym Planner Lookahead Footprint** 显示项。

## 触障行为

本次只改变前视检查距离，不改变触障处理。当前路径点的代价大于或等于
`obstacle_cost_threshold` 时，`CymPlanner` 仍会将 `cmd_vel` 清零并返回
`false`，由 `move_base` 发起全局重规划。

## WSL 验证

验证环境为 WSL Ubuntu 20.04、ROS Noetic 和 Gazebo Classic。工作区在
Linux 文件系统中的 `/home/car/smartcar2026-simulation` 构建，避免使用
Windows 挂载目录进行 CMake 和 Gazebo I/O。

已通过：

- `catkin_make -j2`；
- `python3 src/car3/test/test_cube_mesh_uri.py`；
- `python3 src/car3/test/test_sign_mesh_uri.py`；
- headless `task3_prepare.launch`：`/gazebo`、`/move_base` 正常启动，
  `/move_base/CymPlanner/obstacle_lookahead_distance` 为 `0.36`；
- headless `task3_execute.launch cargo_item:=苹果`：完成识别、导航、抓取、
  携带导航与投放，任务日志输出
  `DONE: 苹果 delivered to 食品加工车间`。

测试使用 `192.168.8.197` 的 WSL loopback ROS Master，未连接车辆网络。
