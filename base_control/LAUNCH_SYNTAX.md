# Launch 文件中 pkg、type、param 从哪里来？如何查？

本文说明：`<node>` 里的 **pkg**、**type** 以及 **传入的变量（arg/param）** 是在哪里定义的，如何知道该写什么、该传什么；以及哪些是**本仓库作者写的**，哪些是 **ROS 官方/第三方**、有文档可查。

---

## 一、为什么整份 launch 会变成灰色？

可能原因：

1. **编辑器没有把 `.launch` 当 XML 解析**  
   - 在 Cursor/VS Code 右下角点击当前语言（如 “Plain Text” 或 “ROS Launch”），手动改为 **XML**，代码就会按 XML 高亮。
2. **文件缺少 XML 声明**  
   - 已在 `base_control.launch` 首行加上：`<?xml version="1.0" encoding="UTF-8"?>`，便于编辑器识别为 XML。
3. **ROS 扩展的高亮异常**  
   - 若安装了 ROS 扩展，有时会对 `.launch` 用自定义高亮；切到 “XML” 语言模式可避免整篇变灰。

**建议**：对 `*.launch` 使用 **XML** 语言模式，既标准又稳定。

---

## 二、pkg 和 type 是在哪里定义的？

### 1. `pkg`（包名）

- **定义位置**：每个 ROS 包的 **`package.xml`** 里有一行 `<name>包名</name>`，那就是 **pkg** 的取值。
- **本仓库示例**：`base_control/package.xml` 里有 `<name>base_control</name>`，所以 launch 里写 `pkg="base_control"`。
- **谁定义的**：**包作者**在创建包时在 `package.xml` 里写的；ROS 通过 `rospack find 包名` 或 `$(find 包名)` 找到该包路径。

**如何查**：

- 本仓库：直接打开对应包的 `package.xml` 看 `<name>...</name>`。
- 系统/第三方包：`rospack find 包名` 能找到就说明包存在，包名就是 `pkg` 要写的名字。

### 2. `type`（可执行文件名 / 节点程序名）

- **含义**：要启动的**可执行文件**的名字。通常有两种情况：
  - **Python 节点**：一般是 **脚本文件名**，如 `base_control.py`，即 `type="base_control.py"`。该脚本会在包的 `script/` 或 `scripts/` 下，并在 `CMakeLists.txt` 里用 `catkin_install_python_programs` 或 `install(PROGRAMS ...)` 安装为可执行。
  - **C++ 节点**：是 **CMake 里 `add_executable(目标名 ...)` 的第一个参数**，即“可执行目标名”，例如某包写 `add_executable(my_node src/my_node.cpp)`，则 `type="my_node"`。
- **本仓库 base_control**：没有 C++ 可执行，只有 Python 脚本 `script/base_control.py`，因此 `type="base_control.py"`，由**本包作者**定义。
- **谁定义的**：**包作者**——要么在 `CMakeLists.txt` 里声明 C++ 可执行，要么提供并安装 Python 脚本；**type 必须和实际安装的可执行名一致**。

**如何查**：

- **本仓库**：看该包的 `script/` 或 `scripts/` 下有哪些 `.py`，以及 `CMakeLists.txt` 里是否 `install(PROGRAMS script/xxx.py ...)`；或看 `add_executable(...)` 的目标名。
- **系统/第三方包**：  
  - 用 `rosrun 包名 Tab` 补全，会列出该包下可执行名（即 type）。  
  - 或查官方文档：**wiki.ros.org/包名**，例如 [robot_pose_ekf](http://wiki.ros.org/robot_pose_ekf) 会写可执行名。

### 3. 本 launch 里出现的 pkg/type 分别是谁的？

| pkg                 | type                        | 来源说明 |
|---------------------|-----------------------------|----------|
| `base_control`      | `base_control.py`           | 本仓库作者写的包，脚本在 `base_control/script/base_control.py` |
| `robot_pose_ekf`    | `robot_pose_ekf`            | ROS 官方/社区包，需 `sudo apt install ros-noetic-robot-pose-ekf`，文档：wiki.ros.org/robot_pose_ekf |
| `tf`                | `static_transform_publisher`| ROS 核心包，随 ROS 安装，文档：wiki.ros.org/tf |

---

## 三、`rospy.get_param` 和 launch 里的 `<param>` 是怎么建立联系的？

**不是**和 `<arg>` 建立联系，而是和 **`<param>`** 建立联系。

### 1. 流程简述

1. **Launch 启动节点时**：`<node>` 里的每一个 `<param name="参数名" value="值"/>` 会被 ROS 写到 **参数服务器（Parameter Server）** 上。
2. **参数名在服务器上的完整路径**：会加上**节点的私有命名空间**。例如节点名是 `base_control`，则：
   - `<param name="battery_topic" value="battery"/>` → 参数服务器上的名字是 **`/base_control/battery_topic`**。
3. **Python 里**：`rospy.get_param('~battery_topic', 'battery')` 中：
   - **`~`** 表示“当前节点的私有命名空间”，所以会解析为 `/base_control/battery_topic`；
   - 若该参数已在 launch 里通过 `<param>` 设置，则读到 launch 里写的值；
   - 若**没有**在 launch 里设置（例如没写 `battery_freq`），则用 `rospy.get_param` 的**第二个参数**作为默认值（如 `'1'`）。

所以：**launch 里的 `<param name="xxx" value="yyy"/>` 和 代码里的 `rospy.get_param('~xxx', 默认值)` 通过“参数名”对应**；参数名一致（且都在同一节点下），就会读到 launch 里设的值。

### 2. 对应关系小结

| Launch 写法 | 参数服务器上的名字（节点名为 base_control 时） | Python 读取方式 |
|-------------|------------------------------------------------|-----------------|
| `<param name="battery_topic" value="battery"/>` | `/base_control/battery_topic` | `rospy.get_param('~battery_topic', 'battery')` |
| 未在 launch 中写 `<param name="battery_freq" .../>` | 无此参数 | `rospy.get_param('~battery_freq', '1')` → 得到默认值 `'1'` |

因此：**launch 里没写的参数，节点照样能运行**，因为代码里给了默认值；若希望在 launch 里可配置（如改电池发布频率），就需要在 launch 里补上对应的 `<arg>` 和 `<param name="battery_freq" value="$(arg battery_freq)"/>`。已在 `base_control.launch` 中为 `battery_freq` 补全。

### 3. `<arg>` 和 `<param>` 的区别

- **`<arg>`**：只在 **launch 文件内部** 使用，用于在 launch 里传参（例如从命令行传入、或在 `<include>` 时传入）。**节点代码无法直接读 arg**。
- **`<param>`**：会把名和值**写入参数服务器**，节点通过 `rospy.get_param('~参数名', 默认值)` **读取**。

所以典型用法是：在 launch 里用 `<arg name="battery_freq" default="1"/>` 声明变量，再在 `<node>` 里用 `<param name="battery_freq" value="$(arg battery_freq)"/>` 把该值赋给参数服务器，节点里用 `rospy.get_param('~battery_freq', '1')` 读取。

---

## 四、`<arg>` 和 `<param>` 传入的变量指代什么？如何知道要传什么？

### 1. `<arg>`（launch 层变量，仅 launch 内可见）

- **定义**：在本 launch 或**被 include 的 launch** 里用 `<arg name="变量名" default="默认值"/>` 声明。
- **指代什么**：由**写 launch 的人**约定，例如 `base_type` 表示“底盘型号”，`cmd_vel_topic` 表示“速度话题名”。**没有全局规范**，只能看该 launch 或同包内其它 launch 的注释和用法。
- **如何知道要传什么**：
  - 看本文件或同包 launch 里的注释、`default` 取值。
  - 若 launch 被别的文件 `include`，看调用处 `<arg name="xxx" value="yyy"/>` 传了什么。

### 2. `<param>`（节点参数，写入参数服务器，供 rospy.get_param 读取）

- **定义**：在节点内用 `<param name="参数名" value="值"/>` 设置；在代码里用 `rospy.get_param('~参数名', 默认值)` 读取（`~` 表示私有）。
- **指代什么**：由**写该节点的人**在代码里决定。例如 `base_control.py` 里会 `rospy.get_param('~port', '/dev/ttyUSB0')`，那 launch 里传的 `port` 就是串口设备。
- **如何知道要传什么**：
  - **本仓库节点**：打开对应脚本（如 `base_control.py`），搜索 `rospy.get_param('~` 或 `rospy.get_param("~`，看到的名称就是可用的 `<param name="...">`。
  - **ROS 官方/第三方节点**：查该包在 **wiki.ros.org/包名** 的说明，通常有 “Parameters” 或 “ROS API” 列表。

---

## 五、ROS 官方/第三方包的文档在哪里查？

- **ROS 官方/常用包**：  
  **https://wiki.ros.org/包名**  
  例如：  
  - https://wiki.ros.org/robot_pose_ekf  
  - https://wiki.ros.org/tf  
  - https://wiki.ros.org/roslaunch  
  页面里会有 Nodes、Parameters、Topics 等说明。
- **命令行**：  
  - `rosrun 包名 可执行名` 可运行节点；  
  - `rosrun 包名 可执行名 --help` 部分节点会打印参数说明；  
  - `rospack find 包名` 可确认包是否安装、路径在哪。

---

## 六、小结

| 内容         | 在哪里定义           | 如何知道写什么/传什么 |
|--------------|----------------------|------------------------|
| **pkg**      | 包的 `package.xml` 的 `<name>` | 看 package.xml 或 `rospack find` |
| **type**     | 本包：Python 脚本名或 CMake 的 `add_executable` 目标名 | 看 script/ 与 CMakeLists.txt；系统包用 `rosrun 包名 Tab` 或 wiki.ros.org |
| **arg**      | 本 launch 或被 include 的 launch 里 `<arg>` | 看该 launch 注释与 default，以及 include 处传参 |
| **param**    | 节点代码里 `rospy.get_param('~xxx')` | 本仓库看脚本；系统包看 wiki.ros.org 的 Parameters |

**pkg/type 有的是本仓库作者写的（如 base_control、base_control.py），有的是 ROS 官方的（如 tf、robot_pose_ekf）；官方包都有 wiki.ros.org 文档可查。**

---

## 七、机器人/无人艇常用 ROS 官方包推荐（ROS1 Noetic）

以下包在 wheeled robot、无人艇、自主导航场景中出现频率极高，建议重点熟悉其 wiki 文档和参数：

### 1. 定位与状态估计（EKF / UKF / 融合）

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `robot_pose_ekf` | 融合 odom + IMU（扩展卡尔曼滤波） | 轮式机器人里程计修正 | wiki.ros.org/robot_pose_ekf |
| `robot_localization` | EKF/UKF 多传感器融合（支持 GPS、IMU、odom、vo 等） | 更通用的状态估计，支持多机/无人机 | wiki.ros.org/robot_localization |
| `imu_filter_madgwick` | Madgwick 互补滤波（IMU 姿态解算） | 低成本 IMU 姿态估计 | wiki.ros.org/imu_filter_madgwick |
| `amcl` | 自适应蒙特卡洛定位（粒子滤波） | 2D 地图中的全局定位 | wiki.ros.org/amcl |

### 2. 导航与路径规划

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `move_base` | 全局/局部路径规划 + 避障 + 速度控制 | 轮式机器人导航核心 | wiki.ros.org/move_base |
| `dwa_local_planner` / `teb_local_planner` | 局部路径规划器（DWA / TEB） | 动态避障、无人艇/机器人 | wiki.ros.org/dwa_local_planner |
| `global_planner` | 全局路径规划（A* / Dijkstra） | 与 move_base 配合使用 | wiki.ros.org/global_planner |
| `costmap_2d` | 代价地图（障碍物、膨胀层） | 导航必需 | wiki.ros.org/costmap_2d |

### 3. TF 与坐标变换

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `tf` / `tf2` | 坐标系变换广播与查询 | 所有机器人必备 | wiki.ros.org/tf |
| `tf2_ros` | TF2 核心（推荐使用） | 静态/动态 TF 发布 | wiki.ros.org/tf2_ros |
| `robot_state_publisher` | 从 URDF 发布 TF | 机械臂/移动底盘 TF 自动发布 | wiki.ros.org/robot_state_publisher |

### 4. 传感器驱动与处理

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `urg_node` | Hokuyo 激光雷达驱动 | 2D 激光 | wiki.ros.org/urg_node |
| `rplidar_ros` | 思岚 RPLIDAR 驱动 | 低成本激光 | wiki.ros.org/rplidar_ros |
| `velodyne_pointcloud` | Velodyne 3D 激光驱动 | 无人车/无人艇 3D 感知 | wiki.ros.org/velodyne_pointcloud |
| `usb_cam` | USB 摄像头驱动 | 视觉 | wiki.ros.org/usb_cam |
| `image_proc` | 图像预处理（去畸变、双目） | 视觉导航前置 | wiki.ros.org/image_proc |

### 5. 无人艇 / 水面机器人特有

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `marine_ros` / `usv_base_control` | 无人艇底层控制（常需自研） | 推进器、舵机控制 | 社区/自研 |
| `nmea_navsat_driver` | GPS NMEA 解析 | 无人艇定位 | wiki.ros.org/nmea_navsat_driver |
| `marine_radar` | 船用雷达驱动 | 避障 | 社区包 |

### 6. 工具与可视化

| 包名 | 主要功能 | 典型用途 | 文档 |
|---|---|---|---|
| `rqt` / `rqt_graph` | 运行时图形化调试 | 节点/话题关系查看 | wiki.ros.org/rqt |
| `rviz` | 3D 可视化 | TF、激光、地图、路径显示 | wiki.ros.org/rviz |
| `rosbag` | 数据录制与回放 | 实验数据保存 | wiki.ros.org/rosbag |

**记忆建议**：先掌握 `tf`、`robot_pose_ekf`、`move_base`、`amcl`、`costmap_2d` 这 5 个核心包，再按传感器类型补充驱动包。所有官方包均可在 `wiki.ros.org/包名` 找到完整参数说明。

---

## 八、单机模式 vs 多机模式（robot_name 控制）

### 1. 什么是“单机”与“多机”？

在 `base_control.launch` 中，通过 `<arg name="robot_name">` 的值来区分两种运行模式：

| 模式 | `robot_name` 的值 | 进入条件 | 命名空间 | 典型话题示例 |
|---|---|---|---|---|
| **单机模式** | 空字符串 `""`（默认） | `if="$(eval robot_name == '')"` | 根命名空间 `/` | `/odom`、`/cmd_vel`、`/imu` |
| **多机模式** | 非空（如 `robot1`、`r2`） | `unless="$(eval robot_name == '')"` | 带前缀的命名空间 | `/robot1/odom`、`/robot1/cmd_vel` |

- **单机**：同一台物理机器（或仿真环境）上只运行**一个**机器人实例，所有话题和 TF 都在根命名空间下。
- **多机**：同一台机器（或多台机器）上运行**多个**机器人实例，每个机器人被分配一个独立的命名空间，避免话题和 TF 名字冲突。

### 2. launch 文件中的实现逻辑

```xml
<!-- 单机：robot_name 为空时执行 -->
<group if="$(eval robot_name == '')">
    <node name="base_control" .../>
    <node pkg="robot_pose_ekf" .../>
    ...
</group>

<!-- 多机：robot_name 非空时执行 -->
<group unless="$(eval robot_name == '')">
    <group ns="$(arg robot_name)">          <!-- 关键：给整个机器人加命名空间 -->
        <node name="base_control" .../>
        <node pkg="robot_pose_ekf" .../>
        ...
    </group>
</group>
```

- 外层 `<group unless>`：判断是否进入多机分支
- 内层 `<group ns="...">`：真正给该机器人下的**所有节点、话题、TF** 加上前缀
- 两层嵌套配合，实现了“默认单机、传参即多机”的灵活切换

### 3. 命令行如何切换模式？

```bash
# 单机模式（默认）
roslaunch base_control base_control.launch

# 多机模式（给 robot_name 传值）
roslaunch base_control base_control.launch robot_name:=robot1
roslaunch base_control base_control.launch robot_name:=robot2
```

启动后可用 `rostopic list` / `rosnode list` / `rqt_graph` 验证：

- 单机：看到 `/odom`、`/base_control` 等
- 多机：看到 `/robot1/odom`、`/robot1/base_control` 等

### 4. 多机模式下的话题与 TF 变化

| 项目 | 单机模式 | 多机模式（robot_name=robot1） |
|---|---|---|
| 里程计话题 | `/odom` | `/robot1/odom` |
| 速度指令话题 | `/cmd_vel` | `/robot1/cmd_vel` |
| TF 父子关系 | `odom → base_footprint` | `robot1/odom → robot1/base_footprint` |
| 参数服务器路径 | `/base_control/xxx` | `/robot1/base_control/xxx` |

**注意**：`odom_topic`、`imu_topic` 等在多机时**没有**显式加 `robot_name` 前缀，是因为节点本身已经在 `ns="$(arg robot_name)"` 的命名空间内，ROS 会自动把相对话题解析为带前缀的全局话题。

而 `ackermann_cmd_topic` 显式写了 `$(arg robot_name)$(arg ackermann_cmd_topic)`，是因为它通常由**外部遥控节点**发布，需要明确指定目标命名空间。

### 5. 适用场景

| 场景 | 推荐模式 | 理由 |
|---|---|---|
| 实验室单机器人调试 | 单机 | 话题简单、易观察 |
| Gazebo 多机器人仿真 | 多机 | 避免多个机器人话题冲突 |
| 真实多机器人编队/协同 | 多机 | 每个机器人独立命名空间，便于分别控制 |
| 教学演示（一个 launch 同时启动两个机器人） | 多机 | `robot_name:=robot1` 和 `robot_name:=robot2` 分别启动两次 |
| 跨机器通信（一台 PC 控制多台机器人） | 多机 | 通过 `ROS_MASTER_URI` + 命名空间实现 |

### 6. 记忆口诀

- **robot_name 为空** → 单机 → 所有东西都在 `/` 下
- **robot_name 非空** → 多机 → 所有东西都在 `/robotX/` 下
- **想快速切换** → 命令行传 `robot_name:=xxx` 即可，无需改 launch 文件

这样设计的好处是：**同一份 launch 文件，既能单机调试，又能多机部署**，极大地提高了代码复用性。

---

## 九、ROS1 Python 开发搜索速查与官方文档资源

### 1. 命令行快速搜索（日常最常用）

#### 1.1 搜索 ROS 消息结构

```bash
# 查看消息完整字段定义（最常用）
rosmsg show geometry_msgs/Twist
rosmsg show nav_msgs/Odometry
rosmsg show sensor_msgs/Imu

# 列出某个包的所有消息
rosmsg list | grep geometry_msgs

# 找到 .msg 源文件位置
rospack find geometry_msgs
# 输出示例：/opt/ros/noetic/share/geometry_msgs
# 然后进入 msg/ 目录查看原始 Twist.msg 等文件
```

#### 1.2 搜索 rospy / roscpp 函数文档

```bash
# 查看 rospy 函数帮助
python3 -c "import rospy; help(rospy.get_param)"
python3 -c "import rospy; help(rospy.Publisher)"

# 找到 rospy 源码位置
python3 -c "import rospy; print(rospy.__file__)"
# 典型输出：/opt/ros/noetic/lib/python3/dist-packages/rospy/__init__.py
# 源码在同目录下的 client.py、param.py 等文件中
```

#### 1.3 交互式 Python 探索

```python
python3

>>> from geometry_msgs.msg import Twist
>>> help(Twist)          # 查看完整文档和字段
>>> t = Twist()
>>> print(t)             # 查看默认值
>>> dir(t)               # 查看所有属性和方法

>>> import rospy
>>> help(rospy.get_param)
```

---

### 2. IDE 跳转配置（Cursor / VS Code）

在项目根目录的 `.vscode/settings.json` 中加入：

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/noetic/lib/python3/dist-packages"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/noetic/lib/python3/dist-packages"
  ]
}
```

配置后：
- 光标放到 `Twist` 上 → 按 `F12` 跳转到定义
- 光标放到 `rospy.get_param` 上 → 直接跳转到源码

---

### 3. 网页与帮助文档搜索资源（重点）

ROS1 没有单一的“官方帮助中心”，而是分散在以下几个网站和工具中：

#### 3.1 wiki.ros.org（最重要、最权威）

**网址**：https://wiki.ros.org/

**用途**：
- 每个 ROS 包都有独立页面：`https://wiki.ros.org/包名`
- 包含：Nodes、Parameters、Topics、Services、API 文档
- 示例：
  - https://wiki.ros.org/geometry_msgs
  - https://wiki.ros.org/rospy
  - https://wiki.ros.org/robot_pose_ekf

**搜索技巧**：
- 直接在浏览器地址栏输入 `wiki.ros.org/你想查的包名或函数名`
- 例如：`wiki.ros.org/Twist`、`wiki.ros.org/get_param`

#### 3.2 answers.ros.org（社区问答，强烈推荐）

**网址**：https://answers.ros.org/

**用途**：
- ROS 开发者遇到问题时的问答社区
- 搜索“rospy get_param”、“Twist message”、“serial communication”等关键词
- 很多老问题已有详细解答

**搜索技巧**：
- 直接在首页搜索框输入问题关键词
- 按标签过滤（如 `rospy`、`python`、`serial`）

#### 3.3 docs.ros.org（ROS2 为主，ROS1 也有部分）

**网址**：https://docs.ros.org/

**说明**：
- ROS2 的官方文档站点
- ROS1 的部分包文档也迁移到了这里
- 主要用于查找较新的 API 说明

#### 3.4 GitHub 仓库（源码 + README + Issues）

**常用仓库**：
- https://github.com/ros/ros_comm（rospy、roscpp 核心）
- https://github.com/ros/common_msgs（geometry_msgs、sensor_msgs 等）
- https://github.com/ros/robot_state_publisher
- https://github.com/ros/robot_localization

**搜索技巧**：
- 在仓库首页按 `t` 键可快速搜索文件
- 在 Issues 里搜索报错信息，常有 workaround

#### 3.5 ROS Answers + Google 搜索技巧

当你在 Google / Bing 搜索时，建议加上以下关键词：

```
site:answers.ros.org rospy get_param
site:wiki.ros.org Twist message
site:github.com ros serial communication 115200
```

这样可以精准定位到 ROS 社区的讨论。

---

### 4. 实际搜索示例

#### 示例 1：想知道 `Twist` 消息有哪些字段

1. 命令行：`rosmsg show geometry_msgs/Twist`
2. 网页：打开 https://wiki.ros.org/geometry_msgs → 找到 Twist 链接
3. IDE：`from geometry_msgs.msg import Twist` → F12 跳转

#### 示例 2：想知道 `rospy.get_param` 的默认值行为

1. 命令行：`python3 -c "import rospy; help(rospy.get_param)"`
2. 源码：`python3 -c "import rospy; print(rospy.__file__)"` → 打开 `param.py` 搜索 `def get_param`
3. 网页：https://wiki.ros.org/rospy → 找到 Parameters 章节

#### 示例 3：想知道 `base_control.py` 里用到的 `BatteryState` 消息结构

1. 命令行：`rosmsg show sensor_msgs/BatteryState`
2. 网页：https://wiki.ros.org/sensor_msgs → BatteryState
3. 源码位置：`/opt/ros/noetic/share/sensor_msgs/msg/BatteryState.msg`

---

### 5. 记忆口诀

- **想看消息结构** → `rosmsg show 包名/消息名`
- **想看 rospy 函数** → `python3 -c "import rospy; help(rospy.xxx)"`
- **想找官方文档** → `wiki.ros.org/包名`（最优先）
- **遇到问题不会** → `answers.ros.org` 搜索关键词
- **想看源码实现** → IDE 配置好路径 + F12，或直接读 `/opt/ros/noetic/lib/python3/dist-packages/`

掌握以上 5 个入口，ROS1 开发中的 90% 搜索需求都能快速解决。
