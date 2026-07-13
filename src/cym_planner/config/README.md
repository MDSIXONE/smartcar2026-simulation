# cym_planner 配置说明

## 这是什么？

`cym_planner` 是一个局部路径规划器插件。它需要知道机器人的两个坐标系名字：
- **机器人本体坐标系**（默认 `base_link`）
- **里程计坐标系**（默认 `odom`）

如果你的机器人坐标系名跟默认值不一样，不用改代码，直接改 JSON 配置文件就行。

---
## 如何使用？

找到你的 move_base 节点，一般在你的启动launch里：

然后找到base_local_planner,改成下面这个


    <param name="base_local_planner" value="cym_planner/CymPlanner" />


</node>


## 注意事项

配置文件里的名字必须跟 TF 树里**完全一致**（大小写、下划线都要对上），否则会报错。

### 方法 ：实时图形界面查看

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

弹出一个窗口，实时显示当前所有坐标系的父子关系。


```
base_footprint──> base_link
              └─> laser_link
```

找到机器人本体对应的那个 base_link 名字，如果和json里的不一样，就需要把json里的改掉。

---

## 怎么改配置

查到实际名字后如果不是，就要编辑文件：

`config/cym_planner_params.json`

```json
{
    "cym_planner/CymPlanner": {
        "base_link_frame": "base_link",
        "odom_frame": "odom"
    }
}
```

把 `base_link_frame` 和 `odom_frame` 后面的值，改成你查到的真实名字。比如你的机器人体坐标系叫 `xxx/base_link`：

```json
{
    "cym_planner/CymPlanner": {
        "base_link_frame": "xxx/base_link",
        "odom_frame": "odom"
    }
}

```

> 改的时候只改引号里的值，外面的 key（`cym_planner/CymPlanner`、`base_link_frame`、`odom_frame`）不要动。



##  加载 JSON 配置

    <rosparam file="$(find cym_planner)/config/cym_planner_params.json"
              command="load" />



