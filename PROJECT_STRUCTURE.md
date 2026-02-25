# bingda_ros1_noetic 项目结构说明

本文档梳理项目目录结构，并说明**各部分如何通过代码串联成机器人可运行的程序**。

---

## 一、项目整体结构

这是一个 **ROS1 Noetic Catkin 工作空间**，没有顶层 `CMakeLists.txt`，各功能以独立 ROS 包形式组织。

```
bingda_ros1_noetic/
├── base_control/          # 底盘控制（串口、里程计、IMU、电池）
├── robot_description/     # 机器人模型（URDF）、Gazebo 仿真
├── robot_navigation/      # 激光导航（SLAM、AMCL、move_base、多机型/多雷达）
├── robot_simulation/      # Stage 仿真（单机/多机、带地图）
├── robot_vision/          # 视觉（opencv_apps、人脸、巡线、相机）
├── robot_vslam/           # 视觉 SLAM（RGB-D + rtabmap）
├── lidar/                 # 多种激光雷达驱动
├── depend_pkg/            # 相机与图像依赖（astra、uvc、jpeg_streamer）
└── astrapro_launch/       # Astra Pro 等 RGB-D 相机 launch
```

**核心思路**：通过 **Launch 文件** 把「底盘 + 雷达 + 地图 + 导航 + 可选视觉」按需组合；通过环境变量 **`BASE_TYPE`**、**`LIDAR_TYPE`** 选择机型和雷达，实现同一套代码适配多机型。

---

## 二、各模块职责与代码入口

### 1. base_control（底盘控制）

| 内容 | 说明 |
|------|------|
| **入口** | `base_control/script/base_control.py`（Python 节点） |
| **启动** | `base_control/launch/base_control.launch` |
| **作用** | 通过串口与下位机通信，把 ROS 速度指令发到底盘，并发布里程计、IMU、电池等 |

**代码如何变成“可用的底盘”**：

- **订阅**：`/cmd_vel`（`geometry_msgs/Twist`）或 `/ackermann_cmd_topic`（阿克曼车）
- **发布**：`/odom`（`nav_msgs/Odometry`）、`battery`、可选 `imu`；若开启 `boardcast_odom_tf`，会发布 `odom → base_footprint` 的 TF
- **串口协议**：115200 波特率，固定帧头 `0x5a`，功能码 0x01（速度）、0x15（阿克曼）等，详见 `base_control/README.md`
- **参数**：端口、波特率、话题名、是否发布 IMU 等来自 `base_control.launch` 和 rosparam

因此：**导航/键盘遥控** 发 `cmd_vel` → **base_control 节点** 解析并写成串口协议 → **下位机** 驱动轮子；同时下位机回传编码器/IMU → 同一节点发布 `odom`/`imu`，供定位与导航使用。

---

### 2. 雷达（lidar/）

| 内容 | 说明 |
|------|------|
| **入口** | 各包内的 C++ 节点（如 `rplidarNode`、`nvilidar_node` 等） |
| **启动** | 由 `robot_navigation/launch/lidar.launch` 按 `LIDAR_TYPE` include 对应子 launch |
| **作用** | 发布激光扫描 `sensor_msgs/LaserScan`，并提供 `base_footprint → base_laser_link` 的 TF（在 lidar.launch 里用 `static_transform_publisher` 按机型配置） |

**代码如何变成“可用的激光数据”**：

- 环境变量 **`LIDAR_TYPE`** 决定 include 哪个文件，例如：`rplidar`、`ydlidar`、`nvilidar`、`ld19`、`vp300`、`vp350`、`rplidar_s2`、`sclidar` 等（见 `robot_navigation/launch/lidar/` 下同名 `.launch`）。
- **`BASE_TYPE`** 在 `lidar.launch` 里用于选择不同机型的雷达安装位置（不同 `static_transform_publisher` 的 xyz、yaw），这样同一份 launch 能适配 NanoRobot、NanoCar、NanoRobot_Pro、NanoCar_Pro、NanoOmni 等。

即：**雷达驱动节点** 发布 `scan`（或类似）→ **lidar.launch** 根据机型发布 TF → 导航栈才能把激光数据正确变换到 `odom`/`map` 下使用。

---

### 3. robot_navigation（激光建图与导航）

| 内容 | 说明 |
|------|------|
| **入口** | 多个 launch 文件，见下表 |
| **作用** | 提供地图、定位（AMCL）、路径规划（move_base）、以及多种激光 SLAM |

**代码如何变成“可用的导航程序”**：

- **robot_lidar.launch**  
  - include `base_control` + `lidar.launch`  
  - 即：**一次启动** 就拉起底盘节点 + 对应型号雷达，形成「底盘 + 激光」的最小可运行组合。

- **robot_navigation.launch**（实机导航）  
  - 若 `simulation:=false`：先 include `robot_lidar.launch`（底盘+雷达），再启动：
    - `map_server`：加载 `robot_navigation/maps/map.yaml`
    - `amcl`：参数来自 `robot_navigation/param/$(BASE_TYPE)/amcl_params.yaml`
    - `move_base.launch`：加载同目录下 costmap、局部/全局规划器（DWA 或 TEB）等
  - 若 `simulation:=true`：改为 include `robot_simulation` 的带地图仿真，不再启动 robot_lidar。

- **move_base.launch**  
  - 启动 `move_base` 节点，通过 `param/$(BASE_TYPE)/` 下的 yaml 加载：
    - `costmap_common_params.yaml`、`local/global_costmap_params.yaml`
    - `move_base_params.yaml`
    - `dwa_local_planner_params.yaml` 或 `teb_local_planner_params.yaml`
  - 并 remap `cmd_vel`、`odom` 到当前配置的话题。

- **robot_slam_laser.launch**  
  - 实机：先 `robot_lidar.launch`；仿真：则 `robot_simulation`。
  - 再按需 include gmapping/hector/karto/cartographer 等 SLAM，可选带 move_base。

数据流概括：  
**雷达 scan** + **底盘 odom**（及 TF）→ **AMCL** 得到 map→base_footprint → **move_base** 订阅目标点、发布 **cmd_vel** → **base_control** 接收并驱动底盘。  
建图时：**scan** + **odom** → SLAM 节点 → 地图；导航时：地图 + AMCL + move_base 形成闭环。

---

### 4. robot_simulation（Stage 仿真）

| 内容 | 说明 |
|------|------|
| **入口** | `simulation_one_robot.launch`、`simulation_one_robot_with_map.launch` 等 |
| **作用** | 用 Stage 模拟机器人和环境，可带地图、AMCL、move_base，与实机导航共用同一套 param |

**代码如何接入“可运行程序”**：

- `robot_navigation.launch` 里当 `simulation:=true` 时，会 include `robot_simulation/launch/simulation_one_robot_with_map.launch`。
- 仿真 launch 会启动 stage、map_server、AMCL（其中 AMCL 等参数仍来自 `robot_navigation/param/$(BASE_TYPE)/`），因此**不需要真实底盘和雷达**，就能跑完整导航栈。
- 多机仿真则有单独的 multi_robot 相关 launch。

---

### 5. robot_vision（视觉）

| 内容 | 说明 |
|------|------|
| **入口** | `robot_vision/scripts/` 下如 `face_detector.py`、`line_detector.py` 等 + `robot_vision/launch/robot_camera.launch` |
| **作用** | 相机驱动与视觉应用（人脸、巡线等） |

**代码如何变成“可用的视觉”**：

- 通过 **base_startup.launch**（在 base_control 下）可同时启动：base_control + lidar + `robot_vision/launch/robot_camera.launch`。
- 即：一个 launch 把「底盘 + 雷达 + 相机」都拉起来，视觉节点从相机话题取图做检测或巡线。

---

### 6. robot_vslam（视觉 SLAM）

| 内容 | 说明 |
|------|------|
| **入口** | `robot_vslam/launch/` 下如 `rtabmap_rgbd_lidar.launch` 等 |
| **作用** | 用 RGB-D 相机 + rtabmap 做视觉 SLAM，深度转激光、再与 move_base 结合 |

**代码如何变成“可用的 vSLAM 程序”**：

- 启动相机（如 astrapro_launch 或 astra/kinect 等）、深度转激光、rtabmap、move_base。
- 参数在 `robot_vslam/param/$(BASE_TYPE)/`，与激光导航类似，通过 BASE_TYPE 选机型配置。
- 与 robot_navigation 共用 move_base 和部分 param 思路，只是前端从“纯激光”换成“RGB-D + 点云/深度”。

---

### 7. robot_description、astrapro_launch、depend_pkg

- **robot_description**：URDF、Gazebo world，用于仿真与 RViz 显示，不直接参与“底盘+导航”的控制逻辑。
- **astrapro_launch / depend_pkg**：相机驱动与 launch，被 robot_vision、robot_vslam 的 launch 通过 include 使用，提供图像和深度话题。

---

## 三、Launch 串联关系（谁调谁）

```
robot_navigation.launch
├── simulation==true  → robot_simulation/simulation_one_robot_with_map.launch
│                        （Stage + map_server + AMCL，param 仍用 robot_navigation）
└── simulation==false → robot_navigation/robot_lidar.launch
│                        ├── base_control/launch/base_control.launch  → base_control.py
│                        └── robot_navigation/launch/lidar.launch
│                             └── lidar/$(LIDAR_TYPE).launch  → 对应雷达节点
│                             + static_transform_publisher（按 BASE_TYPE 选 TF）
├── map_server
├── amcl（param/$(BASE_TYPE)/amcl_params.yaml）
├── move_base.launch（param/$(BASE_TYPE)/*）
└── 可选 rviz
```

```
base_control/base_startup.launch（底盘+雷达+相机）
├── base_control.launch
├── robot_navigation/launch/lidar.launch
└── robot_vision/launch/robot_camera.launch
```

---

## 四、配置与多机型（BASE_TYPE / LIDAR_TYPE）

| 机制 | 说明 |
|------|------|
| **BASE_TYPE** | 环境变量，如 NanoRobot、NanoCar、4WD、4WD_OMNI、NanoRobot_Pro、NanoCar_Pro、NanoOmni 等。用于：雷达 TF（lidar.launch）、AMCL/costmap/move_base 等 param 路径 `param/$(BASE_TYPE)/`。 |
| **LIDAR_TYPE** | 环境变量，对应 `robot_navigation/launch/lidar/` 下某雷达的 `.launch`，决定启动哪一款雷达驱动。 |

因此：**同一套 launch 和代码**，通过不同 **BASE_TYPE + LIDAR_TYPE** 组合，变成不同机型、不同雷达的“可运行程序”，无需改代码，只改环境变量和 param 文件。

---

## 五、从“代码”到“机器人可运行程序”的总结

1. **底盘**：`base_control.py` 通过串口协议把 `cmd_vel` 转为下位机指令，并发布 odom/TF，成为导航的速度执行端与里程计来源。
2. **雷达**：各 lidar 驱动节点发布 scan；`lidar.launch` 按 BASE_TYPE 发布 base_footprint→base_laser_link，使 scan 能正确参与建图与定位。
3. **导航**：`robot_navigation.launch` 按“仿真/实机”选择是否带底盘+雷达，然后统一启动 map_server、AMCL、move_base；move_base 订阅目标、发布 cmd_vel，形成“定位 + 规划 + 执行”的闭环。
4. **视觉 / vSLAM**：通过 base_startup 或 robot_vslam 的 launch 把相机和（可选）rtabmap、move_base 接进来，与底盘、雷达一样，最终都通过话题和 TF 融入同一 ROS 图。

整体上，**可运行程序 = 若干 launch 文件按需组合节点 + 通过 BASE_TYPE/LIDAR_TYPE 选择机型与雷达 + param 目录下对应机型的 yaml 配置**；代码层面主要通过 **include、remap、rosparam** 串联，而不是 package.xml 里的包依赖。
