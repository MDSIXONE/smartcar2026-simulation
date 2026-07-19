# CymPlanner 配置与第一阶段调试

`CymPlanner` 是 `nav_core::BaseLocalPlanner` 插件。当前优化分支包含第一至第三阶段的核心链路：

- 从全局路径最近点截取并等距采样局部参考路径；
- 使用机器人完整 Footprint 对路径段做平移、旋转插值扫掠；
- 从 local costmap 建立多源距离场，让左右障碍对横向偏移产生连续梯度；
- 安全时用 Pure Pursuit 跟踪；
- 碰撞时按减速度制动，超时后请求 `move_base` 重规划或恢复；
- 终点停车后单独对准最终朝向。
- 参考路径碰撞后生成左绕/右绕候选，并锁定拓扑方向；
- 在锁定方向内对连续横向偏移做平滑、障碍距离和时间连续性细化；
- 原路径恢复安全后经过保持时间，再指数平滑回归。

## 启用插件

`move_base` 启动文件中需要包含：

```xml
<param name="base_local_planner" value="cym_planner/CymPlanner"/>
<rosparam file="$(find cym_planner)/config/cym_planner_params.json" command="load"/>
```

配置文件根节点保持为 `CymPlanner`。默认本体坐标系是 `base_link`，局部规划坐标系直接使用 local costmap 的 `global_frame`。

## RViz 调试话题

- `/cym_planner/reference_path`：黄色显示较合适的局部参考路径；
- `/cym_planner/left_seed_path`：左绕候选；
- `/cym_planner/right_seed_path`：右绕候选；
- `/cym_planner/selected_path`：当前实际跟踪路径；
- `/cym_planner/predicted_footprints`：红色碰撞 Footprint；
- `/cym_planner/planner_state`：`TRACK`、`STOPPING` 或 `GOAL_ALIGN`。

`reference_path` 和 `selected_path` 使用 `nav_msgs/Path`，Footprint 使用 `visualization_msgs/MarkerArray`。

## 第一轮测试顺序

1. 无障碍直线与转弯，确认 Pure Pursuit 稳定；
2. 让障碍只侵入车体侧边，确认中心路径点未碰撞时仍能发现 Footprint 碰撞；
3. 在左右均有空间时检查候选评分和方向锁定，不应左右来回切换；
4. 测试转角靠墙、距离场梯度和 local costmap 边界；
5. 确认碰撞时速度平滑降为零，持续无解后 `move_base` 能重新规划；
6. 障碍消失后确认路径经过保持时间再平滑回归。

初始 `max_vel_x` 刻意设为 `0.15 m/s`。碰撞预测和制动确认稳定后再逐步提高。
