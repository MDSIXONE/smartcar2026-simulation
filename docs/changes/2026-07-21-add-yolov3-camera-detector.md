# 仿真 RGB 相机接入 YOLOv3-tiny

## 目的

将本机 `darknet_64efa721` 中已训练的三分类 YOLOv3-tiny 模型接入 Task 3
仿真相机。检测器在完整取放流程中并行运行，用于验证相机和模型链路，不改变
已验证的导航、类别映射、抓取或底盘控制决策。

## 涉及文件

- `src/car3/scripts/yolo_detector.py`：常驻启动 `yolo_pipe`，从
  `/camera/rgb/image_raw` 以受限频率读取图像并发布检测 JSON。
- `src/car3/launch/yolo_detector.launch` 与 `task3_prepare.launch`：增加
  `enable_yolo`、模型目录和推理频率参数。
- `src/car3/CMakeLists.txt`、`package.xml`：安装脚本并声明 Python ROS、图像与
  OpenCV bridge 运行依赖。
- `docs/LOCAL_WSL_20.04.md`：记录 WSL 运行库构建、启动、检查和完整任务命令。

## 验证结果

- 已完成 Python 语法、ROS launch XML 和 catkin 包配置的静态检查。
- 已在本机 WSL Ubuntu 20.04 构建 CPU Darknet 运行库，并通过
  `yolo_pipe` 在训练样本上确认三分类权重可推理。
- 已成功 `catkin_make -j2`；Task 3 Gazebo 场景在
  `http://192.168.8.197:11311` 启动。`/odom` 为有限值，TF2 缓冲区确认
  `odom -> base_link` 和 `map -> base_link` 可用。
- 在 `task3_prepare.launch enable_yolo:=true` 下，RGB 相机为 640×480，
  检测话题以约 2 Hz 发布；取件阶段识别到 `food`（置信度 0.6881）。
- 已执行 `task3_execute.launch cargo_item:=苹果`：任务发布 `done=True`，
  最终状态为“苹果 delivered to 食品加工车间”。

## 已知限制

YOLOv3-tiny 目前只发布检测结果，不参与任务控制；这样识别误差不能驱动车辆或
机械臂。运行时仅复制最佳权重，训练图片及中间 checkpoint 保持在本地 Darknet
目录，不进入仿真 Git 仓库。当前为 CPU 运行时，2 Hz 是为 Gazebo 导航预留资源的
保守默认值；若使用 GPU，需要用匹配的 CUDA/CUDNN 配置重建运行库后再提高速率。
