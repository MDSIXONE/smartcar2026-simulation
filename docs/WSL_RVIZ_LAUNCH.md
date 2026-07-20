# WSL 启动仿真并检查 RViz 前视车体

本文适用于 WSL Ubuntu 20.04 中的
`~/smartcar2026-simulation` 工作区，以及分支
`fix/cym-planner-vehicle-size-lookahead`。

`task3_prepare.launch` 会同时启动 Gazebo、地图、`move_base` 和 RViz。无需
单独启动 `roscore`；未设置远程 ROS Master 时，`roslaunch` 会自动启动本机
Master。

## 1. 首次启动或更新源码后编译

在 WSL 终端执行：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
catkin_make --pkg cym_planner -j2
source devel/setup.bash
```

若该工作区通过 Git 克隆，可先切换到本次分支：

```bash
git fetch origin
git switch fix/cym-planner-vehicle-size-lookahead
git pull --ff-only
```

## 2. 终端 A：干净启动 Gazebo、导航和 RViz

确认没有旧仿真窗口后，在同一终端执行：

```bash
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

等待 Gazebo 场景加载完毕，并在日志中看到：

```text
calibrated initial arm pose applied
```

不要在该命令运行期间再启动 `v3_cym_gazebo.launch` 或第二个
`task3_prepare.launch`，否则会重复启动 Gazebo、控制器和 `move_base`。

## 3. RViz：检查前视车体

RViz 会自动载入 `v3_cym_nav.rviz`。左侧 **Displays** 中应启用：

```text
Cym Planner Lookahead Footprint
```

在 RViz 工具栏选择 **2D Nav Goal**，在地图的可通行区域拖拽一个目标点。
生成全局路径后，会在前视检查路径的末端显示一个青色矩形轮廓：

- 话题：`/move_base/CymPlanner/lookahead_footprint`
- 尺寸：直接读取 local costmap footprint，默认 `0.30 m × 0.20 m`
- 朝向：跟随该路径点朝向
- 显示：最后一次有效的 Marker 会锁存在 RViz 中，便于检查高速导航后的前视位置

没有导航目标时该轮廓不会发布，这是正常现象。可在另一个终端确认发布端：

```bash
source /opt/ros/noetic/setup.bash
source ~/smartcar2026-simulation/devel/setup.bash
rostopic info /move_base/CymPlanner/lookahead_footprint
```

输出中应有一个 publisher，节点为 `/move_base`。

同时会弹出两个 OpenCV 调试窗口：`Map` 显示局部代价地图中的同一青色
footprint，`Plan` 显示车体坐标系中的路径和前视 footprint。它们同样只会在
产生导航路径后更新。

## 4. 终端 B：运行完整取放任务（可选）

终端 A 保持运行，另开一个 WSL 终端：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

任务启动导航后，同样可以在 RViz 中看到前视车体轮廓。实时状态可用：

```bash
rostopic echo /sim_task3/status
```

## 5. 停止

优先在终端 A 按 `Ctrl-C`，等待 `roslaunch` 退出；终端 B 若仍运行，也按
`Ctrl-C`。下次启动前不应保留 `gzserver`、`gzclient`、`move_base`、`rviz` 或
`roslaunch` 进程。

若上次异常退出，可运行仓库根目录的下面命令。它会先清理上述仿真进程，然后
立即重新启动完整 Gazebo 与 RViz：

```bash
./start_v3_clean.sh gui:=true rviz:=true
```
