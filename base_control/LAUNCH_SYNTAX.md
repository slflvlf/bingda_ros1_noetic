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
