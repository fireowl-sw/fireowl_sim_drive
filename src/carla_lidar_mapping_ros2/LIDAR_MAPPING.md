# carla_lidar_mapping_ros2

基于 CARLA 仿真环境的实时 LiDAR 点云建图 ROS 2 功能包。

订阅 CARLA 中的 LiDAR 点云和里程计数据，通过坐标变换将各帧点云拼接到全局 `map` 坐标系下，构建环境的 3D 点云地图，并在 RViz2 中实时可视化。

## 核心思路

### 整体流程

```
/carla/ego_vehicle/lidar ──→ pointCloudCallback() ──→ 缓存最新点云
                                                              │
/carla/ego_vehicle/odometry ──→ localizationCallback() ──→ 更新当前位姿    │
                                                              ↓
                          timerCallback() (100ms 周期)
                              │
                              ↓
                     有新点云数据？──否──→ 仅发布已有地图
                              │
                             是
                              ↓
                     scanTransformation()
                       ┌─────────────────────────────────┐
                       │ 1. 获取当前位姿 (x, y, z, R, P, Y)  │
                       │ 2. 构建 4×4 齐次变换矩阵            │
                       │ 3. 过滤车身附近点 (vehicle_radius)   │
                       │ 4. 将局部点云变换到 map 坐标系        │
                       │ 5. 累积到全局点云地图                │
                       │ 6. 体素滤波降采样 (0.3m 分辨率)      │
                       └─────────────────────────────────┘
                              │
                              ↓
                     /point_cloud_map ──→ RViz2 可视化
```

### 关键算法说明

#### 1. 坐标变换

核心函数 `transform3D()` 根据当前车辆的位姿（位置 + 姿态角），构建 4×4 齐次变换矩阵：

```
T = | R  t |    R = Rz(yaw) × Ry(pitch) × Rx(roll)
    | 0  1 |    t = [x, y, z]^T
```

每帧 LiDAR 点云中的每个点 `p_local` 通过 `p_map = T × p_local` 变换到全局坐标系。

#### 2. 车身点过滤

变换前先计算每个点到 LiDAR 中心的距离，排除 `distance < vehicle_radius` 的点。这些点是打在车身上的噪点，不应出现在地图中。

#### 3. 体素滤波降采样

使用 PCL 的 `VoxelGrid` 滤波器，以 0.3m 的体素尺寸对累积的全局点云进行降采样，减少冗余点、控制内存占用。

#### 4. 智能扫描控制

为了控制建图密度，实现了距离 + 时间双重条件判断：

- 只有当车辆移动超过 `5.0m` 且距上次扫描超过 `1.0s` 时，才接受新的一帧点云
- 当累积点数超过 `cloud_resolution` 阈值时暂停扫描，等待车辆移动到新位置

#### 5. 地图容量保护

设置 `max_map_points`（默认 5000 万点）上限，防止内存溢出。达到上限后停止累积新点，但仍继续发布已有地图供可视化。

## 话题接口

| 类型 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/carla/ego_vehicle/lidar` | `sensor_msgs/PointCloud2` | LiDAR 点云 |
| 订阅 | `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | 车辆里程计 |
| 发布 | `/point_cloud_map` | `sensor_msgs/PointCloud2` | 累积点云地图 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `new_scan` | `true` | 是否接受新扫描 |
| `vehicle_radius` | `8.0` | 车身过滤半径（米） |
| `cloud_resolution` | `5000` | 单次累积点数阈值 |
| `max_map_points` | `50000000` | 地图最大点数（5000 万） |

## 文件结构

```
carla_lidar_mapping_ros2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── objects.json
├── include/carla_lidar_mapping_ros2/
│   └── lidar_mapping.h          # 类定义、数据结构
├── src/
│   ├── lidar_mapping_node.cpp   # 节点入口（main）
│   └── lidar_mapping.cpp        # 核心算法实现
├── launch/
│   └── lidar_mapping.launch.py  # 启动文件（节点 + RViz2）
├── rviz/
│   └── mapping.rviz             # RViz2 配置
└── maps/
    └── .gitkeep                 # 地图保存目录
```

## 使用方法

```bash
# 确保 CARLA ROS Bridge 已启动并连接到 CARLA
ros2 launch carla_lidar_mapping_ros2 lidar_mapping.launch.py
```

自定义参数：

```bash
ros2 launch carla_lidar_mapping_ros2 lidar_mapping.launch.py \
    vehicle_radius:=5.0 \
    cloud_resolution:=10000
```

## 依赖

- `rclcpp` — ROS 2 C++ 客户端库
- `sensor_msgs` — 点云消息
- `nav_msgs` — 里程计消息
- `tf2` — 坐标变换（四元数转欧拉角）
- `PCL` — 点云处理（VoxelGrid 滤波、PCD 保存）
- `Eigen` — 矩阵运算
