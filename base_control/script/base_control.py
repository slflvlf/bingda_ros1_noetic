#!/usr/bin/python
# coding=gbk
# -*- coding: utf-8 -*-

# Copyright 2019 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@liu.fuzhi@bingda-robot.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
底盘控制节点：ROS 与 STM 底盘串口通信，订阅 cmd_vel 发送速度指令，发布 odom/IMU/电池等。

================================================================================
一、ROS 与 STM 串口通信概览
================================================================================
  - 串口参数：115200 波特率，8N1，无校验（在 launch 中 port/baudrate 可配置）。
  - 通信角色：本节点为上位机，STM 底盘控制板为下位机；本节点主动发“查询/控制”帧，
    下位机回复“数据/应答”帧。
  - 帧结构（每帧必含）：[帧头 0x5A][帧长度][ID][功能码][数据 0~250 字节][预留 0x00][CRC-8]
    详见 base_control/PROTOCOL_README.md 与 README.md。
  - 功能码约定：奇数=上位机→下位机（本节点发送），偶数=下位机→上位机（本节点接收并解析）。

二、数据格式与单位
  - 线速度 vx/vy：m/s，传输时 ×1000 存为 int16_t，大端序。
  - 角速度 vz：rad/s，×1000 存为 int16_t。
  - 航向角 yaw：度，×100 存为 int16_t（部分阿克曼转向角为弧度×1000）。
  - 电池电压/电流：V 与 A，×1000 存为 uint16_t。
  - IMU 陀螺/加速度：×100000 存为 int32_t；四元数 ×10000 存为 int16_t。

三、传输与同步细节
  - 发送：所有“写串口”前通过 serialIDLE_flag 互斥，避免与接收解析冲突；发送前等待
    out_waiting 清空，保证一帧完整发出后再发下一帧。
  - 接收：1kHz 定时器 timerCommunicationCB 中 read 到的字节先入环形队列 Circleloop，
    再按 0x5A 帧头 + 帧长度 组帧，CRC 校验通过后按功能码解析，更新 Vx/Vy/Vyaw/电池/IMU 等。
  - 连接保持：下位机超过 1000ms 未收到协议内数据会断开并停电机，故需持续发查询或指令
    （本节点通过定时器周期发 0x09/0x11、0x07、0x13 等维持连接）。
"""

# ---------- 标准库与 ROS 依赖 ----------
import os
import rospy
import tf
import time
import sys
import math
import serial      # 串口通信（与嵌入式板通信的核心库）
import string
import ctypes      # 用于 int16/int32 有符号解析（下位机数据为大端有符号数）
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range

# ---------- 环境变量：底盘型号与传感器配置 ----------
# 从环境变量读取底盘型号（如 NanoRobot、NanoCar），用于选择订阅 cmd_vel 或 ackermann 等
base_type = os.getenv('BASE_TYPE')
# 超声波数量，未设置则默认为 0（与下位机超声波功能码 0x1a 对应）
if os.getenv('SONAR_NUM') is None:
    sonar_num = 0
else:
    sonar_num = int(os.getenv('SONAR_NUM'))


# ================================================================================
# 环形队列（Ring Buffer）：串口接收流式数据缓存，用于可靠组帧
# ================================================================================
#
# 【为什么需要环形队列】
# 串口是流式传输：一次 read() 可能读到任意长度（例如 3 字节、15 字节），不一定刚好是一整帧。
# 若直接按“帧头 0x5A + 第二字节帧长度”去解析，会出现两种问题：
#   1) 半帧：一帧有 12 字节，第一次只读到 5 字节 → 长度不够，无法解析，若丢弃则丢数据。
#   2) 粘包：第一次读到 8 字节（含下一帧的前 2 字节）→ 若按 12 字节截取，会把下一帧的头
#      误当作本帧数据，导致错位，后续帧全部错乱。
# 解决方式：把每次 read() 得到的字节**按顺序全部入队**，在队列里始终从队首检查“是否出现
# 0x5A”以及“当前队列长度是否 ≥ 该帧长度”；只有凑齐一整帧才出队解析，这样帧边界永远正确。
#
# 【为什么叫“环形”？环形体现在哪里？】
# 物理上 array 是一段 4KB 的连续内存，并没有首尾相接的导线。所谓“环形”是指**下标的使用方式**：
#   - 入队时 rear 自增，若 rear 已是 capacity-1，下一次入队会写到 rear=(capacity-1+1)%capacity=0，
#     即“写满末尾后，接着从下标 0 写”，逻辑上把数组首尾连成环。
#   - 出队时 front 同理：front 从 0 一直增到 capacity-1 后，下一次出队 front 变为 0。
# 这样不需要在“队首出队后”把后面所有数据往前搬移，只需移动 front/rear 指针，用 % capacity
# 实现“从末尾回到 0”的环。例如 capacity=8 时：
#
#     下标:  0   1   2   3   4   5   6   7
#           [  ][  ][  ][  ][  ][  ][  ][  ]
#             ^rear 可再次写入的“下一个位置”若从 7 再写，就回到 0
#           逻辑上: 7 的下一个 = 0，形成环。
#
# 【“收齐后解析”在哪里？解析后的数据放到哪里？】
# 本 class 只负责“存字节、按队首出队”，不负责解析。收齐判断与解析在 timerCommunicationCB 中：
#   (1) 收齐判断：get_front()==0x5A 且 get_queue_length() >= get_front_second()（即长度 L）。
#   (2) 取出整帧：for i in range(length): databuf.append(get_front()); dequeue()
#       即先读队首字节放入 databuf，再 dequeue 把该字节从队列中“逻辑移除”；循环 L 次后 databuf 即为整帧。
#   (3) 解析与去向：对 databuf 做 CRC 校验后，根据 databuf[3] 功能码分支，将数据段写入 self 的成员，
#       例如 0x08 电池 → self.Vvoltage、self.Icurrent；0x0a/0x12 里程计 → self.Vx、self.Vy、self.Yawz、
#       self.Vyaw；0xf2 版本 → self.movebase_hardware_version 等，供 timerOdomCB/timerBatteryCB 等发布。
#
# 【dequeue 并没有把 array 里的数据擦掉，“完整帧”算移除吗？】
# 是的，dequeue 只做两件事：size 减 1，front 前移（环到 0）。array 里原来那几位并不会被清零。
# “移除”是**逻辑上的**：队列约定“有效数据只在 [front, rear) 区间内”，front 前移后，原来 front
# 指向的那一格就不再被看作队列的一部分，之后 get_front() 会读到新的队首。那 6 个字节的格子
# 仍然保留旧值，直到将来某次 enqueue 时 rear 绕回并覆盖到这些位置。所以“完整帧”的取出过程是：
# 每次 get_front() 把当前队首（即那一帧的一个字节）复制到 databuf，然后 dequeue() 让 front 前进，
# 这样该字节就不再参与后续的 get_front()，相当于从队列里“取走”了；循环 6 次后，整帧在 databuf 中，
# 队列里这 6 格已被逻辑“消费”，不再被使用，直到被后续入队覆盖。
#
# 【与本协议的关系】
# 协议帧格式：[0x5A][帧长度 L][ID][功能码][数据...][预留][CRC]。帧长度 L = 整帧字节数。
# 组帧逻辑（见 timerCommunicationCB）：
#   - 若 get_front() == 0x5A 且 get_queue_length() >= get_front_second()，则收齐一帧；
#   - 连续 L 次：get_front() 放入 databuf，再 dequeue()；然后对 databuf 校验并按功能码解析。
#
# 【举例说明】设 capacity=8（实际为 4096），协议一帧 6 字节：5A 06 01 07 00 E4（查询电池）
#
#   (1) 初始：front=rear=0, size=0，队列空
#       array: [ _, _, _, _, _, _, _, _ ]
#                ^front,rear
#
#   (2) 串口读到 3 字节 [0x5A, 0x06, 0x01] 依次 enqueue
#       array: [5A, 06, 01, _, _, _, _, _ ]
#                ^front        ^rear
#       get_front()==0x5A，get_front_second()==0x06 即长度 6，get_queue_length()==3 < 6，未收齐，不解析
#
#   (3) 再读到 3 字节 [0x07, 0x00, 0xE4] 入队
#       array: [5A, 06, 01, 07, 00, E4, _, _ ]
#                ^front                 ^rear
#       get_queue_length()==6 >= 6，收齐一帧。在 timerCommunicationCB 中：循环 6 次
#       get_front()→databuf、dequeue()，得到 databuf=[5A,06,01,07,00,E4]；校验后按功能码 0x08 解析，
#       写入 self.Vvoltage、self.Icurrent 等。6 次 dequeue 后 front 移到 6，队列逻辑上只剩 0 字节，
#       array 内 [0..5] 仍存 5A,06,01,07,00,E4，但已不在“有效区间”内，不会被再次读取。
#
#   (4) 若下一次 read() 一次收到 12 字节（两帧）：[5A,06,01,07,00,E4, 5A,0C,01,01,01,F4,...]
#       前 6 字节解析完后，队首变为 0x5A，长度 0x0C=12，队列里还剩 6 字节 < 12，不解析；
#       等后续字节入队凑齐 12 字节再解析第二帧。这样不会把两帧混在一起。
#
# ================================================================================
class queue:
    def __init__(self, capacity=1024 * 4):
        self.capacity = capacity   # 队列容量（默认 4KB），可缓存多帧，防止高速接收时溢出
        self.size = 0              # 当前元素个数，用于判空/判满
        self.front = 0             # 队首下标：下一次出队或“读队首”的位置
        self.rear = 0              # 队尾下标：下一次入队写入的位置（当前为空位）
        self.array = [0] * capacity # 存储字节的固定长度数组，环形使用

    def is_empty(self):
        """队列是否为空（size==0）。空时不应 dequeue 或取队首。"""
        return 0 == self.size

    def is_full(self):
        """队列是否已满（size==capacity）。满时 enqueue 会抛异常。"""
        return self.size == self.capacity

    def enqueue(self, element):
        """入队：将单字节 element 写入 rear 位置，rear 前移（环到 0）。串口读到的每个字节都应入队。"""
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity   # 到末尾后从 0 开始

    def dequeue(self):
        """出队：从队首移除一字节，front 前移（环到 0）。解析一帧时连续调用 L 次取出整帧。"""
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity

    def get_front(self):
        """取队首字节（不删除）。用于判断队首是否为帧头 0x5A，是则再看长度是否收齐。"""
        return self.array[self.front]

    def get_front_second(self):
        """取队首的下一字节，即协议中的“帧长度”字节。帧长度 = 整帧字节数（含帧头到 CRC）。"""
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        """当前队列中有效字节数。公式 (rear - front + capacity) % capacity 在环形下等价于 size。"""
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        """调试用：遍历数组（本实现未实际打印）。"""
        for i in range(self.capacity):
            pass
        print(' ')


# ================================================================================
# 底盘控制主类：与嵌入式板（STM 底盘控制板）串口通信、速度指令下发、里程计/IMU/电池上报
# ================================================================================
class BaseControl:
    def __init__(self):
        # ---------- 串口接收环形缓冲：用于组帧，避免半帧解析 ----------
        self.Circleloop = queue(capacity=1024 * 4)

        # ---------- 从 launch 的 rosparam 读取的私有参数（~ 表示节点私有） ----------
        self.baseId = rospy.get_param('~base_id', 'base_footprint')
        self.odomId = rospy.get_param('~odom_id', 'odom')
        self.device_port = rospy.get_param('~port', '/dev/ttyUSB0')   # 与嵌入式板连接的串口设备
        self.baudrate = int(rospy.get_param('~baudrate', '115200'))   # 协议规定 115200，8N1
        self.odom_freq = int(rospy.get_param('~odom_freq', '50'))     # 里程计查询与发布频率（Hz）
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.battery_topic = rospy.get_param('~battery_topic', 'battery')
        self.battery_freq = float(rospy.get_param('~battery_freq', '1'))
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd_topic')
        self.pub_imu = bool(rospy.get_param('~pub_imu', False))
        if self.pub_imu:
            self.imuId = rospy.get_param('~imu_id', 'imu')
            self.imu_topic = rospy.get_param('~imu_topic', 'imu')
            self.imu_freq = float(rospy.get_param('~imu_freq', '50'))
            if self.imu_freq > 100:
                self.imu_freq = 100
        self.pub_sonar = bool(rospy.get_param('~pub_sonar', False))
        self.sub_ackermann = bool(rospy.get_param('~sub_ackermann', False))
        self.boardcast_odom_tf = bool(rospy.get_param('~boardcast_odom_tf', True))

        # ---------- 内部状态与协议解析用的变量（由下位机应答帧更新） ----------
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        # 串口忙标志（互斥用）：0=空闲；1=等版本/里程计应答；3=电池/IMU 查询；4=发速度指令
        self.serialIDLE_flag = 0
        self.trans_x = 0.0   # 待下发的线速度 x（m/s）
        self.trans_y = 0.0
        self.rotat_z = 0.0   # 待下发的角速度（rad/s）
        self.speed = 0.0
        self.steering_angle = 0.0
        self.sendcounter = 0
        self.ImuErrFlag = False
        self.EncoderFlag = False
        self.BatteryFlag = False
        self.OdomTimeCounter = 0
        self.BatteryTimeCounter = 0
        # 以下由 timerCommunicationCB 解析下位机应答后写入（原始整型，需除 1000/100 等还原）
        self.Vx = 0          # X 轴线速度 ×1000（int16 大端）
        self.Vy = 0
        self.Vyaw = 0        # 角速度 ×1000（rad/s）
        self.Yawz = 0        # 航向角 ×100（度），来自下位机
        self.Vvoltage = 0    # 电池电压 ×1000（V）
        self.Icurrent = 0    # 电池电流 ×1000（A）
        self.Gyro = [0, 0, 0]   # 陀螺仪 ×100000（int32）
        self.Accel = [0, 0, 0]  # 加速度 ×100000
        self.Quat = [0, 0, 0, 0]  # 四元数 ×10000（int16）
        self.Sonar = [0, 0, 0, 0]
        self.movebase_firmware_version = [0, 0, 0]
        self.movebase_hardware_version = [0, 0, 0]
        self.movebase_type = ["NanoCar", "NanoRobot", "4WD_OMNI", "4WD", "RC_ACKERMAN"]
        self.motor_type = ["25GA370", "37GB520", "TT48", "RS365", "RS540"]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_ackermann_cmd_time = rospy.Time.now()

        # ---------- 里程计协方差矩阵（6x6，行优先存储）----------
        # ROS nav_msgs/Odometry 中 pose.covariance 与 twist.covariance 对应 6 个自由度：
        #   位姿/速度维度顺序：x, y, z, roll(绕x), pitch(绕y), yaw(绕z)
        #   对角线索引：0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        # 设计原则：
        #   (1) 本节点为 2D 平面里程计，只估计 x/y/yaw（及 vx/vy/vyaw），z/roll/pitch 不观测，
        #       故对应对角线设为 1e6，表示“未知/不可用”，EKF 等融合时不会错误信任这些维度。
        #   (2) 小方差（1e-3、1e-9）表示该维度有观测、相对可信；数值越小越“确定”。
        #   (3) pose 与 twist 各有两套：带 1e-3 的表示运动时略大不确定性（轮式积分漂移）；
        #       带 1e-9 的表示更自信。当前发布逻辑中两分支均使用 *_covariance2。
        #   (4) 非常确定时对角线不要设为 0：EKF 等会用到协方差矩阵的逆，对角为 0 会导致
        #       奇异/除零或数值不稳定，故用 1e-9 表示“几乎完全确定”而不用 0。
        self.odom_pose_covariance = [1e-3,    0,    0,   0,   0,    0, 
                                                0, 1e-3,    0,   0,   0,    0,
                                                0,    0,  1e6,   0,   0,    0,
                                                0,    0,    0, 1e6,   0,    0,
                                                0,    0,    0,   0, 1e6,    0,
                                                0,    0,    0,   0,   0,  1e3 ]

        self.odom_pose_covariance2  = [1e-9,    0,    0,   0,   0,    0, 
                                                    0, 1e-3, 1e-9,   0,   0,    0,
                                                    0,    0,  1e6,   0,   0,    0,
                                                    0,    0,    0, 1e6,   0,    0,
                                                    0,    0,    0,   0, 1e6,    0,
                                                    0,    0,    0,   0,   0, 1e-9 ]

        self.odom_twist_covariance  = [1e-3,    0,    0,   0,   0,    0, 
                                                    0, 1e-3,    0,   0,   0,    0,
                                                    0,    0,  1e6,   0,   0,    0,
                                                    0,    0,    0, 1e6,   0,    0,
                                                    0,    0,    0,   0, 1e6,    0,
                                                    0,    0,    0,   0,   0,  1e3 ]
                                                    
        self.odom_twist_covariance2 = [1e-9,    0,    0,   0,   0,    0, 
                                                    0, 1e-3, 1e-9,   0,   0,    0,
                                                    0,    0,  1e6,   0,   0,    0,
                                                    0,    0,    0, 1e6,   0,    0,
                                                    0,    0,    0,   0, 1e6,    0,
                                                    0,    0,    0,   0,   0, 1e-9]

        # ---------- 打开与嵌入式板通信的串口 ----------
        try:
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=10)
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()   # 清空上电残留数据
            except Exception:
                rospy.loginfo("Opening Serial Try Faild")
                pass
        except Exception:
            rospy.logerr("Can not open Serial" + self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")

        # ---------- 订阅与发布：阿克曼车可订阅 ackermann，否则订阅 cmd_vel ----------
        if ('NanoCar' in base_type) and (self.sub_ackermann is True):
            from ackermann_msgs.msg import AckermannDriveStamped
            self.sub = rospy.Subscriber(self.ackermann_cmd_topic, AckermannDriveStamped, self.ackermannCmdCB, queue_size=20)
        else:
            self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmdCB, queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.battery_pub = rospy.Publisher(self.battery_topic, BatteryState, queue_size=3)
        if self.boardcast_odom_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()
        # 定时器：里程计查询(0x09/0x11)、电池查询(0x07)、通信解析(1kHz)
        self.timer_odom = rospy.Timer(rospy.Duration(1.0 / self.odom_freq), self.timerOdomCB)
        self.timer_battery = rospy.Timer(rospy.Duration(1.0 / self.battery_freq), self.timerBatteryCB)
        self.timer_communication = rospy.Timer(rospy.Duration(1.0 / 1000), self.timerCommunicationCB)

        if self.pub_imu:
            self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
            self.timer_imu = rospy.Timer(rospy.Duration(1.0 / self.imu_freq), self.timerIMUCB)

        # ---------- 启动时与嵌入式板握手：先读版本，再读 SN、配置 ----------
        self.getVersion()
        # 等待下位机返回版本后再继续；老固件 IMU 初始化约 2s 会阻塞
        while self.movebase_hardware_version[0] == 0:
            pass
        if self.movebase_hardware_version[0] < 2:
            time.sleep(2.0)
        self.getSN()
        time.sleep(0.01)
        self.getInfo()

    # ============================================================================
    # 与嵌入式板协议：CRC-8 校验（CRC-8/MAXIM）
    # ============================================================================
    # 校验范围：整帧从帧头到预留位（不含 CRC 自身）。与下位机 STM 协议一致，发送时填最后一字节，
    # 接收时用前 length-1 字节计算 CRC 与最后一字节比较，不一致则丢帧。
    def crc_1byte(self, data):
        """单字节 CRC-8/MAXIM 迭代（多项式 0x31，即 0x18 反序）。"""
        crc_1byte = 0
        for i in range(0, 8):
            if (crc_1byte ^ data) & 0x01:
                crc_1byte ^= 0x18
                crc_1byte >>= 1
                crc_1byte |= 0x80
            else:
                crc_1byte >>= 1
            data >>= 1
        return crc_1byte

    def crc_byte(self, data, length):
        """对 data 前 length 字节计算 CRC-8。发送时 length=len-1 并填最后一字节；接收时校验用。"""
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret ^ data[i])
        return ret

    # ============================================================================
    # 上位机→下位机：速度控制指令（功能码 0x01）
    # ============================================================================
    # 协议帧：[0x5A][0x0C][0x01][0x01][vx_H vx_L][vy_H vy_L][vz_H vz_L][0x00][CRC]
    # 数据：vx/vy 单位 m/s，vz 单位 rad/s；传输时 ×1000 存为 int16_t，大端（高字节在前）。
    # 例：0.5 m/s 前进 → vx=500 → 0x01F4，帧：5A 0C 01 01 01 F4 00 00 00 00 00 56
    def cmdCB(self, data):
        """订阅 /cmd_vel 的回调：将 Twist 转为协议帧并发送给嵌入式板。"""
        self.trans_x = data.linear.x
        self.trans_y = data.linear.y
        self.rotat_z = data.angular.z
        self.last_cmd_vel_time = rospy.Time.now()
        outputdata = [0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        # 数据段索引 4~9：vx(4,5)、vy(6,7)、vz(8,9)，有符号×1000，大端
        outputdata[4] = (int(self.trans_x * 1000.0) >> 8) & 0xff
        outputdata[5] = int(self.trans_x * 1000.0) & 0xff
        outputdata[6] = (int(self.trans_y * 1000.0) >> 8) & 0xff
        outputdata[7] = int(self.trans_y * 1000.0) & 0xff
        outputdata[8] = (int(self.rotat_z * 1000.0) >> 8) & 0xff
        outputdata[9] = int(self.rotat_z * 1000.0) & 0xff
        crc_8 = self.crc_byte(outputdata, len(outputdata) - 1)
        outputdata[11] = crc_8
        # 互斥：等待串口空闲再发，避免与接收解析/其他查询冲突
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass   # 等待上一帧完全发出
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0

    # ============================================================================
    # 上位机→下位机：阿克曼车速度控制（功能码 0x15）
    # ============================================================================
    # 帧结构同 0x01，数据：speed×1000(int16)、2 字节占位、steering_angle 弧度×1000(int16)
    def ackermannCmdCB(self, data):
        """订阅 ackermann 话题的回调：下发速度与转向角给阿克曼底盘。"""
        self.speed = data.drive.speed
        self.steering_angle = data.drive.steering_angle
        self.last_ackermann_cmd_time = rospy.Time.now()
        outputdata = [0x5a, 0x0c, 0x01, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        outputdata[4] = (int(self.speed * 1000.0) >> 8) & 0xff
        outputdata[5] = int(self.speed * 1000.0) & 0xff
        outputdata[8] = (int(self.steering_angle * 1000.0) >> 8) & 0xff
        outputdata[9] = int(self.steering_angle * 1000.0) & 0xff
        crc_8 = self.crc_byte(outputdata, len(outputdata) - 1)
        outputdata[11] = crc_8
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0

    # ============================================================================
    # 上位机→下位机：查询版本号（功能码 0xF1）
    # ============================================================================
    # 无数据段。下位机回复 0xF2，数据 6 字节：硬件版主.次.修订(3) + 固件版(3)，用于区分 0x09/0x11 等
    def getVersion(self):
        outputdata = [0x5a, 0x06, 0x01, 0xf1, 0x00, 0xd7]
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Get Version Command Send Faild")
        self.serialIDLE_flag = 0

    # ============================================================================
    # 上位机→下位机：查询主板 SN（功能码 0xF3）
    # ============================================================================
    # 下位机回复 0xF4，数据 12 字节为 SN（十六进制），仅用于日志打印
    def getSN(self):
        outputdata = [0x5a, 0x06, 0x01, 0xf3, 0x00, 0x46]
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Get SN Command Send Faild")
        self.serialIDLE_flag = 0

    # ============================================================================
    # 上位机→下位机：获取底盘配置信息（功能码 0x21）
    # ============================================================================
    # 下位机回复 0x22：底盘类型(1)、电机类型(1)、减速比×10(2)、轮径×10(2)，大端
    def getInfo(self):
        outputdata = [0x5a, 0x06, 0x01, 0x21, 0x00, 0x8f]
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Get info Command Send Faild")
        self.serialIDLE_flag = 0

    # ============================================================================
    # 定时器：请求里程计（0x09/0x11）并发布 Odometry 与 TF
    # ============================================================================
    # 0x09：旧固件，下位机回复 0x0a（vx, yaw×100, vz）；0x11：新固件，回复 0x12（vx, vy, yaw, vz）。
    # 应答在 timerCommunicationCB 中解析并写入 self.Vx/Vy/Yawz/Vyaw，此处用其积分位姿并发布。
    def timerOdomCB(self, event):
        if self.movebase_firmware_version[1] == 0:
            outputdata = [0x5a, 0x06, 0x01, 0x09, 0x00, 0x38]  # 请求旧版速度/航向（无 Vy）
        else:
            outputdata = [0x5a, 0x06, 0x01, 0x11, 0x00, 0xa2]  # 请求新版（含 Vy，全向底盘）
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Odom Command Send Faild")
        self.serialIDLE_flag = 0
        # 将下位机返回的整型还原为物理量（int16 需用 ctypes 按有符号解析）
        Vx = float(ctypes.c_int16(self.Vx).value / 1000.0)
        Vy = float(ctypes.c_int16(self.Vy).value / 1000.0)
        Vyaw = float(ctypes.c_int16(self.Vyaw).value / 1000.0)
        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value / 100.0)
        self.pose_yaw = self.pose_yaw * math.pi / 180.0   # 度 → 弧度
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
        self.previous_time = self.current_time
        # 平面运动学积分：在 odom 系下根据线速度/角速度更新 pose_x, pose_y
        self.pose_x = self.pose_x + Vx * math.cos(self.pose_yaw) * dt - Vy * math.sin(self.pose_yaw) * dt
        self.pose_y = self.pose_y + Vx * math.sin(self.pose_yaw) * dt + Vy * math.cos(self.pose_yaw) * dt
        pose_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose_yaw)
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id =self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = Vx
        msg.twist.twist.linear.y = Vy
        msg.twist.twist.angular.z = Vyaw
        if abs(Vx) < 0.001 and abs(Vy) < 0.001:
            msg.pose.covariance = self.odom_pose_covariance2
            msg.twist.covariance = self.odom_twist_covariance2
        else:
            msg.pose.covariance = self.odom_pose_covariance2
            msg.twist.covariance = self.odom_twist_covariance2
        self.pub.publish(msg)
        if self.boardcast_odom_tf:
            self.tf_broadcaster.sendTransform((self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)

    # ============================================================================
    # 定时器：查询电池（功能码 0x07），下位机 0x08 应答后发布 BatteryState
    # ============================================================================
    # 应答数据 4 字节：电压×1000、电流×1000（uint16 大端），在 timerCommunicationCB 中解析到 Vvoltage/Icurrent
    def timerBatteryCB(self, event):
        outputdata = [0x5a, 0x06, 0x01, 0x07, 0x00, 0xe4]
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Battery Command Send Faild")
        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage/1000.0)
        msg.current = float(self.Icurrent/1000.0)
        self.battery_pub.publish(msg)

    # ============================================================================
    # 定时器：查询 IMU（功能码 0x13），下位机 0x14 应答后发布 Imu
    # ============================================================================
    # 应答 32 字节：陀螺/加速度各 int32×1e5，四元数 int16×1e4，在 timerCommunicationCB 中解析到 Gyro/Accel/Quat
    def timerIMUCB(self, event):
        outputdata = [0x5a, 0x06, 0x01, 0x13, 0x00, 0x33]
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(outputdata)
        except Exception:
            rospy.logerr("Imu Command Send Faild")
        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.imuId
        msg.angular_velocity.x = float(ctypes.c_int32(self.Gyro[0]).value / 100000.0)
        msg.angular_velocity.y = float(ctypes.c_int32(self.Gyro[1]).value / 100000.0)
        msg.angular_velocity.z = float(ctypes.c_int32(self.Gyro[2]).value / 100000.0)
        msg.linear_acceleration.x = float(ctypes.c_int32(self.Accel[0]).value / 100000.0)
        msg.linear_acceleration.y = float(ctypes.c_int32(self.Accel[1]).value / 100000.0)
        msg.linear_acceleration.z = float(ctypes.c_int32(self.Accel[2]).value / 100000.0)
        msg.orientation.w = float(ctypes.c_int16(self.Quat[0]).value / 10000.0)
        msg.orientation.x = float(ctypes.c_int16(self.Quat[1]).value / 10000.0)
        msg.orientation.y = float(ctypes.c_int16(self.Quat[2]).value / 10000.0)
        msg.orientation.z = float(ctypes.c_int16(self.Quat[3]).value / 10000.0)
        self.imu_pub.publish(msg)

    # ================================================================================
    # 【核心】通信定时器回调（1kHz）：从嵌入式板接收数据并解析协议帧
    # ================================================================================
    # 流程简述：
    #   1) 将串口 in_waiting 读出的所有字节入环形队列 Circleloop，避免半帧/粘包导致错位。
    #   2) 若队首为帧头 0x5A，用第二字节“帧长度”判断队列中是否已收齐一整帧。
    #   3) 收齐则出队到 databuf，对前 length-1 字节做 CRC-8 校验；不通过则丢弃本帧。
    #   4) 按 databuf[3] 功能码分支解析（下位机→上位机均为偶数功能码），更新 Vx/Vy/Vyaw/
    #      Yawz/Vvoltage/Icurrent/Gyro/Accel/Quat/版本/SN/配置 等，供里程计/电池/IMU 发布使用。
    # ================================================================================
    def timerCommunicationCB(self, event):
        # ---------- 步骤 1：将串口缓冲区中待读字节全部入队 ----------
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading) != 0:
                for i in range(0, len(reading)):
                    data = reading[i]
                    try:
                        self.Circleloop.enqueue(data)
                    except Exception:
                        pass

        # ---------- 步骤 2~4：若队首是帧头则尝试组帧并解析 ----------
        if not self.Circleloop.is_empty():
            data = self.Circleloop.get_front()
            if data == 0x5a:  # 帧头固定 0x5A（协议规定）
                length = self.Circleloop.get_front_second()  # 帧长度 = 整帧字节数（含帧头到 CRC）
                if length > 1:
                    # 队列中字节数是否足够一帧（“收齐”条件，参见 class queue 注释）
                    if self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                        # 从环形队列中取出整帧：每次 get_front() 复制队首到 databuf，dequeue() 逻辑移除该字节
                        databuf = []
                        for i in range(length):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()

                        # CRC 校验：最后一字节为 CRC，校验范围为 databuf[0..length-2]
                        if databuf[length - 1] != self.crc_byte(databuf, length - 1):
                            return  # CRC 不通过，丢弃本帧，防止错误数据写入状态

                        # ---------- 下位机→上位机：按功能码 databuf[3] 解析数据段 databuf[4..]，结果写入 self 成员 ----------
                        # 解析后的数据去向：Vx/Vy/Vyaw/Yawz → 里程计 timerOdomCB；Vvoltage/Icurrent → 电池 timerBatteryCB；
                        # Gyro/Accel/Quat → IMU timerIMUCB；版本/SN/配置 → 仅日志。数据段均大端序。

                        if databuf[3] == 0x04:
                            # 0x04：当前速度应答，6 字节。Vx Vy Vyaw 各 int16，×1000 → m/s、rad/s
                            self.Vx = databuf[4] * 256 + databuf[5]
                            self.Vy = databuf[6] * 256 + databuf[7]
                            self.Vyaw = databuf[8] * 256 + databuf[9]

                        elif databuf[3] == 0x06:
                            # 0x06：IMU 欧拉角应答，6 字节。Pitch/Roll/Yaw×1000；此处仅取 Yaw（字节 4~5 为 Pitch，6~7 为 Roll，8~9 为 Yaw）
                            self.Yawz = databuf[8] * 256 + databuf[9]

                        elif databuf[3] == 0x08:
                            # 0x08：电池应答，4 字节。电压×1000、电流×1000（uint16 大端）
                            self.Vvoltage = databuf[4] * 256 + databuf[5]
                            self.Icurrent = databuf[6] * 256 + databuf[7]

                        elif databuf[3] == 0x0a:
                            # 0x0a：里程计应答（旧固件），6 字节。vx×1000, yaw×100(度), vz×1000
                            self.Vx = databuf[4] * 256 + databuf[5]
                            self.Yawz = databuf[6] * 256 + databuf[7]
                            self.Vyaw = databuf[8] * 256 + databuf[9]

                        elif databuf[3] == 0x12:
                            # 0x12：里程计应答（新固件），8 字节。vx vy yaw×100(度) vz，各 int16 大端
                            self.Vx = databuf[4] * 256 + databuf[5]
                            self.Vy = databuf[6] * 256 + databuf[7]
                            self.Yawz = databuf[8] * 256 + databuf[9]
                            self.Vyaw = databuf[10] * 256 + databuf[11]

                        elif databuf[3] == 0x14:
                            # 0x14：IMU 原始数据应答，32 字节。GyroX/Y/Z、AccelX/Y/Z 各 int32×1e5；Quat W/X/Y/Z 各 int16×1e4，大端
                            self.Gyro[0] = int(((databuf[4]&0xff)<<24)|((databuf[5]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[7]&0xff))
                            self.Gyro[1] = int(((databuf[8]&0xff)<<24)|((databuf[9]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[11]&0xff))
                            self.Gyro[2] = int(((databuf[12]&0xff)<<24)|((databuf[13]&0xff)<<16)|((databuf[14]&0xff)<<8)|(databuf[15]&0xff))
                            self.Accel[0] = int(((databuf[16]&0xff)<<24)|((databuf[17]&0xff)<<16)|((databuf[18]&0xff)<<8)|(databuf[19]&0xff))
                            self.Accel[1] = int(((databuf[20]&0xff)<<24)|((databuf[21]&0xff)<<16)|((databuf[22]&0xff)<<8)|(databuf[23]&0xff))
                            self.Accel[2] = int(((databuf[24]&0xff)<<24)|((databuf[25]&0xff)<<16)|((databuf[26]&0xff)<<8)|(databuf[27]&0xff))
                            self.Quat[0] = int((databuf[28]&0xff)<<8|databuf[29])
                            self.Quat[1] = int((databuf[30]&0xff)<<8|databuf[31])
                            self.Quat[2] = int((databuf[32] & 0xff) << 8 | databuf[33])
                            self.Quat[3] = int((databuf[34] & 0xff) << 8 | databuf[35])

                        elif databuf[3] == 0x1a:
                            # 0x1a：超声波应答，4 字节，单位 cm
                            self.Sonar[0] = databuf[4]
                            self.Sonar[1] = databuf[5]
                            self.Sonar[2] = databuf[6]
                            self.Sonar[3] = databuf[7]

                        elif databuf[3] == 0xf2:
                            # 0xf2：版本号应答（getVersion 的回复）。Byte4~6 硬件版 xx.yy.zz，Byte7~9 固件版 aa.bb.cc
                            self.movebase_hardware_version[0] = databuf[4]
                            self.movebase_hardware_version[1] = databuf[5]
                            self.movebase_hardware_version[2] = databuf[6]
                            self.movebase_firmware_version[0] = databuf[7]
                            self.movebase_firmware_version[1] = databuf[8]
                            self.movebase_firmware_version[2] = databuf[9]
                            version_string = "Move Base Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d"\
                                %(self.movebase_hardware_version[0],self.movebase_hardware_version[1],self.movebase_hardware_version[2],\
                                self.movebase_firmware_version[0],self.movebase_firmware_version[1],self.movebase_firmware_version[2])
                            rospy.loginfo(version_string)

                        elif databuf[3] == 0xf4:
                            # 0xf4：SN 应答（getSN 的回复）。Byte4~15 为 12 字节 SN，十六进制打印
                            sn_string = "SN:"
                            for i in range(4, 16):
                                sn_string = "%s%02x" % (sn_string, databuf[i])
                            rospy.loginfo(sn_string)

                        elif databuf[3] == 0x22:
                            # 0x22：配置信息应答（getInfo 的回复）。类型(1)、电机(1)、减速比×10(2)、轮径×10(2)，大端
                            fRatio = float(databuf[6]<<8|databuf[7])/10
                            fDiameter = float(databuf[8]<<8|databuf[9])/10
                            info_string = "Type:%s Motor:%s Ratio:%.01f WheelDiameter:%.01f"\
                                %(self.movebase_type[databuf[4]-1],self.motor_type[databuf[5]-1],fRatio,fDiameter)
                            rospy.loginfo(info_string)
                        else:
                            pass
                else:
                    pass
            else:
                # 队首不是帧头，丢弃一字节以便下次可能对齐到 0x5A
                self.Circleloop.dequeue()
        else:
            pass


# ================================================================================
# 程序入口：初始化 ROS 节点并创建底盘控制实例，rospy.spin() 保持运行
# ================================================================================
if __name__ == "__main__":
    try:
        rospy.init_node('base_control', anonymous=True)
        if base_type is not None:
            rospy.loginfo('%s base control ...' % base_type)
        else:
            rospy.loginfo('base control ...')
            rospy.logerr('PLEASE SET BASE_TYPE ENV FIRST')
        # 创建底盘控制对象：打开串口、订阅 cmd_vel/ackermann、启动定时器（里程计/电池/通信解析）
        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")



