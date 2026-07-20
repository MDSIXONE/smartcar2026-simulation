# YOLO 物块数据采集

`spawn_cubes.py` 每次启动都会随机分配 `cube_0`、`cube_1`、`cube_2` 到左、上、右
三个区域，并在对应区域内随机取坐标。因此模型编号代表类别，而不代表固定位置：

| 模型 | 类别 / YOLO class id |
| --- | --- |
| `cube_0` | food / `0` |
| `cube_1` | daily / `1` |
| `cube_2` | electronics / `2` |

## 启动与采集

先启动准备场景：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

场景稳定后，在第二个终端运行：

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch car3 capture_cube_dataset.launch \
  dataset_dir:=/home/car/smartcar-yolo-dataset \
  frames_per_cube:=3
```

采集器会依次到达当前场景中的三个物块，打开夹爪、进入校准的**夹取前观察姿势**、
以低速 XY 微调使物块位于 RGB 画面中央后拍照，再切换到真实夹取姿势确认可达性。
接触式夹取姿势会让末端相机贴近 4 cm 物块，无法产生完整 YOLO 框，因此不直接用它
拍照。每个场景默认输出 9 张图像（3 个物块 × 每物块 3 张）。
采集期间不要同时运行 `task3_execute.launch`。

默认 `direct_positioning:=true`，即数据采集时由 Gazebo 把车精确放到与正式
抓取相同的基座位姿，避免随机锥桶导致采集被导航阻塞；物块位置、夹取姿势和相机
画面均保持真实仿真状态。若要连同导航链路一起验证，可显式使用：

```bash
roslaunch car3 capture_cube_dataset.launch direct_positioning:=false
```

## 输出格式

```text
smartcar-yolo-dataset/
├── classes.txt
├── images/*.jpg
├── labels/*.txt
└── metadata.jsonl
```

每张图像都有同名 `.txt`，格式为标准 YOLO：

```text
class_id center_x center_y width height
```

每张图的标签只标注当前居中的目标物块：采集器从 RGB 图中定位其高亮正面轮廓，
再按目标模型写入类别，避免 Gazebo 相机插件坐标轴与 URDF optical frame 的差异造成
错误投影。`metadata.jsonl` 额外保存目标区域、像素框和三个物块的世界坐标，便于复查。

## 收集多组随机场景

每次停止 `task3_prepare.launch` 并重新启动，它都会重新随机生成物块位置。对每个
新场景重复采集命令即可增加位置、相机视角和背景组合。不要把不同场景生成的文件
清空；采集器使用时间戳文件名前缀避免覆盖。
