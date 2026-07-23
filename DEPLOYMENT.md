# 部署说明

## 1. 准备基础环境

在 Ubuntu 20.04 安装 ROS Noetic Desktop-Full，并确保可使用 Gazebo、RViz、`move_base` 与 `catkin_make`。本项目面向 ROS 1 Noetic；不要将预编译二进制从其他 ROS 发行版混用。

任务三的 YOLOv5 权重已经导出为 ONNX。Ubuntu 20.04 需要安装一次视觉运行时：

```bash
# 在工作区根目录执行
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv
python3 -m pip install -r requirements-vision.txt
```

## 2. 构建工作区

```bash
# 在准备放置工作区的父目录执行
git clone https://github.com/MDSIXONE/smartcar2026-simulation.git smartcar2026-simulation
cd smartcar2026-simulation
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如通过 ZIP 安装，解压后先恢复 ROS 节点脚本的可执行权限，再执行同样的构建命令：

```bash
# 在工作区根目录执行
chmod +x start_sim_clean.sh
find src/car3/scripts -type f -name '*.py' -exec chmod +x {} +
```

通过 Git 克隆时可执行权限会自动保留，无需执行这一步。

## 3. 两阶段启动

每个终端都在工作区根目录设置同一个本机 ROS Master，避免任务终端连接到
其他 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

先启动环境并等待机械臂平滑到初始姿态完成：

```bash
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

再在另一个完成相同环境设置的终端执行任务：

```bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

环境启动时机械臂控制器保持关闭，避免车辆行驶抖动；任务节点按左、中、右顺序用摄像头识别物块，并通过视觉闭环完成夹取对位后才开启控制器。

## 4. 可选清理启动脚本

`start_sim_clean.sh` 会先停止本机现有的 Gazebo、RViz、move_base 与 roslaunch 进程，再启动基础 Gazebo 环境。仅在确认这些进程都可关闭时运行：

```bash
./start_sim_clean.sh
```

常规任务运行优先使用上面的 `task3_prepare.launch` / `task3_execute.launch` 两阶段命令。
