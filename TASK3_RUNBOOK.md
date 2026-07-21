# SmartCar Task 3 操作指令

本说明对应当前工作区根目录中的代码。任务流程为：

1. 启动 Gazebo 与导航，并把机械臂停在已标定的初始姿态；
2. 在另一个终端输入物品任务；
3. 程序判定物品类别，读取当前随机物块位置，识别左/上/右区域；
4. 根据标定投影计算车位与朝向，导航、精确对位并夹取；
5. 切换到携带姿态，保持夹爪关闭，低速通过有障碍物的加工区；
6. 抵达对应加工车间后发布完成状态。

> 不要同时运行独立的 `gazebo.launch`、其他任务节点和下面的准备命令。两个 Gazebo/控制器启动器会相互冲突，导致机械臂控制器无法启动。

## 0. 每个终端的环境

在 WSL Ubuntu-20.04 中打开终端，切换到工作区根目录后执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
unset ROS_IP ROS_HOSTNAME
```

所有终端都必须使用相同的 `ROS_MASTER_URI`；否则任务节点无法看到 Gazebo、
`move_base` 和仿真传感器话题。

若上一次仿真异常退出、还有旧节点残留，先执行：

```bash
rosnode kill -a
```

确认没有旧 Gazebo 后，再开始下面的步骤。

## 1. 终端 A：启动仿真并初始化机械臂

可视化运行：

```bash
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

无界面自动测试：

```bash
roslaunch car3 task3_prepare.launch gui:=false rviz:=false
```

等待终端日志出现“`calibrated initial arm pose applied`”，或在终端 B 查询：

```bash
rostopic echo -n 1 /sim_task3/arm_initial_pose_ready
rostopic echo -n 1 /joint_states
rosservice call /controller_manager/list_controllers
```

正确结果应包括：

```text
/sim_task3/arm_initial_pose_ready: True

arm_joint1..5 ≈ [-0.0001, -0.4999, 1.2800, 1.7000, 0.0000]
arm_controller: stopped
gripper_controller: stopped
```

初始姿态为：

```text
[-0.0001, -0.4999, 1.2800, 1.7000, 0.0000]
```

准备阶段会临时启用机械臂控制器，用两个中间阶段在 6 秒内平滑到达初始姿态；随后立即停止控制器。之后仅通过 Gazebo 姿态保持与关闭机械臂重力来固定关节，防止未受控的关节 3、4 在车辆行驶时漂移或看起来“断开”。

## 2. 终端 B：发送物品任务

终端 A 的仿真准备完成后，在新终端重新执行“0. 每个终端的环境”，然后二选一启动任务。

### 方式 A：直接输入常见物品名称

```bash
roslaunch car3 task3_execute.launch cargo_item:="苹果"
```

已内置的常见名称：

| 类别 | 可直接输入的名称 |
| --- | --- |
| 食品 | `苹果`、`香蕉`、`可乐`、`牛奶`、`面包`、`饼干`、`零食`、`饮料` |
| 日用品 | `牙刷`、`毛巾`、`纸巾`、`肥皂`、`洗发水`、`水杯` |
| 电子产品 | `手机`、`平板`、`耳机`、`键盘`、`鼠标`、`相机`、`充电器` |

### 方式 B：明确指定类别（推荐用于未收录的物品名）

```bash
roslaunch car3 task3_execute.launch \
  cargo_category:="电子产品" \
  cargo_name:="待处理物品"
```

可用类别：`food` / `食品`、`daily` / `日用品`、`electronics` / `电子产品`。

例如：

```bash
roslaunch car3 task3_execute.launch cargo_category:=food cargo_name:="苹果"
roslaunch car3 task3_execute.launch cargo_category:="日用品" cargo_name:="牙刷"
roslaunch car3 task3_execute.launch cargo_category:=electronics cargo_name:="耳机"
```

## 3. 自动流程与标定值

物块模型名会被随机生成器随机分配到三个区域，因此**不能**把 `cube_0`、`cube_1`、`cube_2` 当成固定的左、上、右位置。

程序的规则如下：

| 工作 | 规则 |
| --- | --- |
| 类别识别 | 标签模型固定映射：食品→`cube_0`，日用品→`cube_1`，电子产品→`cube_2` |
| 区域识别 | 读取三个物块的实时 Gazebo X 坐标：最小 X=左，中间 X=上，最大 X=右 |
| 抓取车位 | 将左侧实测偏移按每个区域顺时针旋转 90°后投影到目标物块坐标 |
| 最终对位 | `move_base` 到点后，使用低速全向微调，使 XY 误差不超过 `8 mm` |

左侧实测标定基准：

```text
目标 - 车: Δx=-0.319277 m, Δy=+0.000771 m
左侧朝向: yaw=3.141157 rad
```

由此得到的区域方向：

| 区域 | 相对左侧的旋转 | 车头朝向 |
| --- | --- | --- |
| 左 | 0° | `3.141157 rad` |
| 上 | 顺时针 90° | `1.570361 rad` |
| 右 | 顺时针 180° | `-0.000436 rad` |

抓取姿态：

```text
[-0.0001, 1.5000, 0.2800, 1.3000, 0.0000]
```

夹取后携带姿态：

```text
[-0.0001, 0.0000, -1.7200, -0.5000, 0.0000]
```

程序会先尝试真实 `grasp_attach` 夹取；只有夹爪关闭后未检测到重叠时，才将已识别的目标物块对齐 TCP 并再次关闭夹爪作为仿真回退。

## 4. 加工区低速模式

抓取成功且机械臂进入携带姿态后，任务节点发布：

```text
/sim_task3/carry_mode = true
```

`cym_planner` 会在加工区将导航速度降低 `1/5`，即保留原来的 `4/5（80%）`：

| 控制量 | 原上限 | 载物上限 |
| --- | ---: | ---: |
| 前进速度 | `14.0` | `11.2` |
| 转向速度 | `20.5` | `16.4` |
| 终点朝向速度 | `10.2` | `8.16` |

速度倍率位于：

```text
src/cym_planner/config/cym_planner_params.json
carry_speed_scale: 0.80
```

修改该 C++ 规划器或该配置后，重新编译并重启仿真：

```bash
# 在工作区根目录执行
source /opt/ros/noetic/setup.bash
catkin_make --pkg cym_planner -j1
source devel/setup.bash
```

## 5. 激光点云局部控制检查

`CymPlanner` 直接订阅 `/scan`，将二维激光扫描转换到 `base_link` 下的点云，并以点云对候选 `(linear.x, angular.z)` 轨迹做车体扫掠碰撞检测和评分。`local_costmap` 仍会使用同一份激光数据，但只负责辅助全局重规划；它不再是局部速度决策的唯一数据源。

在 RViz 中应能同时看到：

- `Cym Planner Direct Laser Point Cloud`：控制器实际接收并过滤后的点云；
- `Cym Planner Laser Candidate Trajectories`：所有候选局部轨迹；
- `Cym Planner Laser Selected Trajectory`：最终用于输出 `cmd_vel` 的绿色轨迹。

终端 B 可检查控制器状态：

```bash
rostopic echo /move_base/CymPlanner/safety_state
```

正常移动时状态以 `ACTIVE: direct laser rollout selected` 开头。以下两种状态会让车辆主动输出零速度：

```text
STOP: laser scan unavailable or stale
STOP: laser point cloud rejects every local trajectory
```

验证激光数据链路：

```bash
rostopic hz /scan
rostopic echo -n 1 /move_base/CymPlanner/laser_points
```

`/scan` 的频率应稳定高于控制器要求；当前仿真激光雷达为 15 Hz，`scan_timeout` 默认是 0.25 秒。相关安全距离、制动模型、采样数和评分权重都位于：

```text
src/cym_planner/config/cym_planner_params.json
```

## 6. 运行状态检查

在终端 B 观察实时任务状态：

```bash
rostopic echo /sim_task3/status
```

检查是否已夹住物块：

```bash
rostopic echo -n 1 /grasp_attach/state
```

正确夹取时应为：

```text
data: "GRASPING"
```

检查载物低速模式：

```bash
rostopic echo -n 1 /sim_task3/carry_mode
```

任务完成时：

```bash
rostopic echo -n 1 /sim_task3/done
```

应输出：

```text
data: True
```

## 7. 手动标定工具

仅在自动任务没有运行时使用。

### 机械臂标定

```bash
python3 "$(rospack find car3)/scripts/arm_pose_tuner.py"
```

常用命令：

```text
p a1 a2 a3 a4 a5   设置完整姿态
j3 +0.05           微调第 3 轴
open / close        打开或关闭夹爪
cur                 读取实测姿态
show                显示目标与实测姿态
save                发布当前标定姿态
q                   退出
```

### 车辆手动控制与保存车位

```bash
python3 "$(rospack find car3)/scripts/keyboard_drive.py"
```

`p` 默认将当前车位保存到当前用户的 ROS 配置目录。若需要保存在工作区根目录，可传入相对路径：

```bash
python3 "$(rospack find car3)/scripts/keyboard_drive.py" \
  _pose_save_path:=./car3_saved_pose.yaml
```

## 7. 常见故障

### 关节 3、4 初始后漂移或看起来断开

确认使用的是 `task3_prepare.launch`，并检查：

```bash
rostopic echo -n 1 /sim_task3/arm_initial_pose_ready
rosservice call /controller_manager/list_controllers
```

准备阶段必须是 `arm_initial_pose_ready=True` 且两个机械臂控制器均为 `stopped`。不要用独立的 `gazebo.launch` 与准备启动器并行启动。

### 车辆已经到夹取点但任务退出

检查控制器：

```bash
rosservice call /controller_manager/list_controllers
```

任务到达取物点后，`arm_controller` 和 `gripper_controller` 应自动变为 `running`。若存在旧节点，执行 `rosnode kill -a` 后从“1. 启动仿真”重新开始。

### 夹爪没有夹住

```bash
rostopic echo -n 1 /grasp_attach/ready
rostopic echo -n 1 /grasp_attach/offset
rostopic echo -n 1 /grasp_attach/state
```

`ready=True` 表示物块位于抓取容差框内；关闭夹爪后应转为 `GRASPING`。自动任务会在真实重叠失败时启用最后的 TCP 吸附回退。

### 不认识物品名

直接加上 `cargo_category`：

```bash
roslaunch car3 task3_execute.launch cargo_category:="食品" cargo_name:="自定义物品"
```

## 8. 测试记录

在无界面 Gazebo 中已验证过完整流程：准备、动态区域判断、精确对位、真实 `grasp_attach` 夹取、携带姿态、食品加工车间导航和完成信号。一次基准运行从准备启动到完成为 65 秒。
