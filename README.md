# FireOwl Sim Drive

基于 CARLA 仿真器和 ROS 2 的自动驾驶仿真工作空间。

## 环境依赖

| 依赖 | 说明 |
|------|------|
| ROS 2 Humble | `source /opt/ros/humble/setup.bash` |
| CARLA 仿真器 | 默认端口 `2000` |
| CARLA Python API | `pip3 install carla` |
| PCL + Eigen | LiDAR 建图节点依赖 |
| pygame | 手动控制节点依赖 |

Python 依赖见 `requirements.txt`。

## 编译

```bash
cd ~/fireowl_ws/fireowl_sim_drive

# 确保 conda 未激活（ROS 2 编译必须使用系统 Python）
conda deactivate

colcon build --symlink-install
source install/setup.bash
```

## 运行

### 1. 启动 CARLA 服务器

在 CARLA 安装目录下运行：

```bash
./CarlaUE4.sh
```

验证连接：

```bash
python3 test_carla_connection.py
```

### 2. 启动 ROS Bridge + 车辆 + 手动控制

```bash
source install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

可选参数：`host:=localhost port:=2000 town:=Town01`

### 3. 启动 LiDAR 建图（新终端）

```bash
conda deactivate
source install/setup.bash
ros2 launch carla_lidar_mapping_ros2 lidar_mapping.launch.py
```

## 手动控制按键

| 按键 | 功能 |
|------|------|
| W / ↑ | 油门 |
| S / ↓ | 刹车 |
| A / D / ← / → | 转向 |
| Q | 切换前进/倒挡 |
| Space | 手刹 |
| P | 开启/关闭自动驾驶 |
| B | 启用手动控制（必须先按此键） |
| M | 切换手动/自动挡 |
| ESC | 退出 |

> 注意：按键操作需要在 pygame 窗口获得焦点时才有效。按 **B** 启用手动控制后，W/S/A/D 才会生效。按 **Q** 可切换前进挡和倒挡。

## 工作空间结构

```
fireowl_sim_drive/
├── src/
│   ├── carla_ros_bridge/          # CARLA ROS Bridge（二次开发）
│   ├── carla_lidar_mapping_ros2/  # 自定义 LiDAR 建图包（详见 LIDAR_MAPPING.md）
│   ├── carla_spawn_objects/       # 车辆/传感器生成
│   ├── carla_manual_control/      # 键盘手动控制
│   ├── carla_ad_agent/            # 自动驾驶 Agent
│   ├── carla_ad_demo/             # 自动驾驶 Demo
│   ├── carla_ackermann_control/   # Ackermann 控制器
│   ├── carla_msgs/                # CARLA 消息定义
│   ├── carla_common/              # 公共工具库
│   ├── carla_waypoint_publisher/  # 路径点发布
│   ├── carla_ros_scenario_runner/ # 场景运行器
│   ├── pcl_recorder/              # 点云录制
│   ├── rviz_carla_plugin/         # RViz CARLA 插件
│   ├── rqt_carla_control/         # RQT 控制面板
│   └── ros-bridge/                # 上游 CARLA ROS Bridge 原始代码
├── test_carla_connection.py       # CARLA 连接测试脚本
└── requirements.txt               # Python 依赖
```

## 常见问题

**编译报错 `CMakeCache.txt` 路径不匹配**

清除构建缓存后重新编译：`rm -rf build/ install/ log/ && colcon build --symlink-install`

**编译报错 `No module named 'em'`**

conda 环境干扰了系统 Python。编译前执行 `conda deactivate` 确保使用系统 Python。

**RViz2 报 `Frame [map] does not exist`**

确保 CARLA ROS Bridge 已成功启动并连接到 CARLA 服务器，`/tf` 话题有数据发布。

**按键无反应**

1. 确认 pygame 窗口已打开并获得焦点
2. 先按 **B** 键启用手动控制
3. 按 **Q** 确认当前是前进挡（gear=1）
