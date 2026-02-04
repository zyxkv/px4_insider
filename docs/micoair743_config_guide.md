# MicoAir743v2 + uXRCE-DDS 完整配置指南

## 📋 硬件配置概览

### 飞控板: MicoAir743v2
- **主控**: STM32H743VIT6, 480MHz, 2MB Flash
- **双IMU**: BMI088 + BMI270
- **气压计**: SPL06
- **磁力计**: QMC5883L
- **串口映射**:
  - USB (ttyACM0) → 用于 uXRCE-DDS
  - UART1 (ttyS0) → TELEM1
  - UART2 (ttyS1) → GPS2
  - UART3 (ttyS2) → GPS1
  - UART4 (ttyS3) → TELEM2
  - UART5 (ttyS4) → TELEM3
  - UART6 (ttyS5) → RC
  - UART7 (ttyS6) → URT6
  - UART8 (ttyS7) → TELEM4/Bluetooth

### 外接传感器: MTF01 光流测距一体传感器
- **接口**: LVTTL 串口 (3.3V), 波特率 115200
- **数据频率**: 100Hz
- **测距范围**: 0.02-8m
- **光流测量**: 最大 7m/s (1米高度)
- **协议**: Mavlink_px4 (PX4版本)

---

## ✅ uXRCE-DDS 支持状态

### 当前情况
**PX4 1.14.3 (Minderring fork)**:
- ⚠️ **micoair_h743-v2 board 支持不存在**
- uXRCE-DDS 模块存在于源码中: `src/modules/uxrce_dds_client`
- 但没有针对 MicoAir743v2 的预配置

**PX4 Main Branch (官方)**:
- ✅ **支持 micoair_h743-v2**
- 官方仓库: https://github.com/PX4/PX4-Autopilot/tree/main/boards/micoair/h743-v2

### 推荐解决方案

#### 方案 1: 使用官方 PX4 Main 分支 (推荐)
```bash
# 克隆官方仓库
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# 切换到 main 分支 (已有 micoair 支持)
git checkout main

# 更新子模块
git submodule update --init --recursive

# 编译
make micoair_h743-v2_default
```

#### 方案 2: 使用 Matek H743 作为基础手动添加
基于 Matek H743 配置进行修改：
```bash
# Matek H743 配置参考
cd ~/projects/px4_insider/px4_src/boards/matek/h743/
cat default.px4board | grep "UXRCE\|dds"
# 输出: CONFIG_MODULES_UXRCE_DDS_CLIENT=y ✅
```

---

## 🔧 PX4 1.14.3 固件编译

### 1. 获取源码
```bash
cd ~/projects/px4_insider/
git clone --branch micoair743-v1.14.3 https://github.com/Minderring/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
```

### 2. 编译 Bootloader
```bash
make micoair_h743-v2_bootloader
```

### 3. 编译固件
```bash
make micoair_h743-v2_default
```

### 4. 烧录固件
编译完成后在 `build/micoair_h743-v2_default/` 目录生成 `.px4` 文件

---

## 📡 MTF01 光流传感器配置

### 传感器设置
使用微空助手将 MTF-01 协议设置为 **Mavlink_px4**

### PX4 参数配置 (1.14.x)
```bash
# 连接飞控后，在 QGC 或通过 MAVShell 设置

# 1. 配置串口 (以 UART4/TELEM2 为例，对应 ttyS3)
param set MAV_1_CONFIG 102  # TELEM2
param save

# 重启飞控

# 2. 设置波特率
param set SER_TEL3_BAUD 115200  # 8N1

# 3. 启用光流和测距辅助
param set EKF2_OF_CTRL 1        # 启用光流
param set EKF2_RNG_CTRL 1       # 启用测距
param set EKF2_HGT_REF 3        # 使用距离传感器作为高度参考

# 4. 设置光流旋转角度 (根据实际安装方向)
param set SENS_FLOW_ROT 0       # No rotation (默认)

# 5. 启用 EKF2 光流融合
param set EKF2_AID_MASK 2       # use optical flow

# 重启飞控
```

### 验证配置
在 QGC 中打开 MAVLink Inspector，检查是否有：
- `OPTICAL_FLOW_RAD` 消息 (光流数据)
- `DISTANCE_SENSOR` 消息 (测距数据)

---

## 🚀 uXRCE-DDS 配置与启动

### 1. 机载电脑安装依赖
```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-fast-dds
sudo apt install microxrce-dds-agent
```

### 2. 启动 uXRCE-DDS Agent
```bash
# USB 连接飞控
# 设备通常为 /dev/ttyACM0

MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

### 3. PX4 端启动 uXRCE-DDS Client
通过 MAVShell 或参数设置：
```bash
# 设置 uXRCE-DDS 串口配置
param set UXRCE_DDS_CFG 1    # 使用 USB/Serial0

# 启动客户端
uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
```

### 4. ROS 2 订阅示例
```python
#!/usr/bin/env python3
# px4_sensor_bridge.py

import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleAttitude, OpticalFlow, DistanceSensor

class PX4Bridge(Node):
    def __init__(self):
        super().__init__('px4_sensor_bridge')
        
        # 订阅传感器融合数据
        self.sensor_sub = self.create_subscription(
            SensorCombined, '/fmu/sensor_combined/out', 
            self.sensor_callback, 10)
        
        # 订阅姿态数据
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/vehicle_attitude/out',
            self.attitude_callback, 10)
        
        # 订阅光流数据
        self.flow_sub = self.create_subscription(
            OpticalFlow, '/fmu/optical_flow/out',
            self.flow_callback, 10)
        
        # 订阅测距数据
        self.distance_sub = self.create_subscription(
            DistanceSensor, '/fmu/distance_sensor/out',
            self.distance_callback, 10)
        
        self.get_logger().info('PX4 Sensor Bridge started')
    
    def sensor_callback(self, msg):
        """处理传感器原始数据"""
        self.get_logger().info(
            f'IMU: acc=[{msg.accelerometer_m_s2[0]:.2f}, '
            f'{msg.accelerometer_m_s2[1]:.2f}, '
            f'{msg.accelerometer_m_s2[2]:.2f}]')
    
    def attitude_callback(self, msg):
        """处理姿态数据"""
        # 四元数姿态
        q = msg.q
        self.get_logger().info(f'Attitude q: [{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}]')
    
    def flow_callback(self, msg):
        """处理光流数据"""
        self.get_logger().info(
            f'Flow: integration_time={msg.integration_time_us}, '
            f'delta_angle=[{msg.delta_angle[0]:.4f}, {msg.delta_angle[1]:.4f}]')
    
    def distance_callback(self, msg):
        """处理测距数据"""
        self.get_logger().info(f'Distance: {msg.current_distance}m')

def main(args=None):
    rclpy.init(args=args)
    bridge = PX4Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔌 USB 连接拓扑

```
┌─────────────────────────────────────────────────────────────┐
│                    MicoAir743v2 Flight Controller            │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │ BMI088  │  │ BMI270  │  │  SPL06  │  │ QMC5883L│       │
│  │  IMU1   │  │  IMU2   │  │ Baro    │  │  Mag    │       │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘       │
│       │            │            │            │              │
│       └────────────┴─────┬──────┴────────────┘              │
│                          │                                  │
│                   ┌──────┴──────┐                           │
│                   │   EKF2      │                           │
│                   │  Fusion     │                           │
│                   └──────┬──────┘                           │
│                          │                                  │
│              ┌───────────┼───────────┐                      │
│              │           │           │                      │
│       ┌──────┴──────┐   │    ┌──────┴──────┐               │
│       │ uXRCE-DDS   │   │    │  MAVLink    │               │
│       │  Client     │   │    │  (TELEM)    │               │
│       └──────┬──────┘   │    └─────────────┘               │
│              │           │                                  │
│              └─────┬─────┘                                  │
│                    │ USB CDC-ACM                            │
│                    ↓ (ttyACM0)                              │
├─────────────────────────────────────────────────────────────┤
│                    Onboard Computer                         │
│  ┌─────────────────────┐  ┌─────────────────────┐          │
│  │ uXRCE-DDS Agent     │  │  ROS 2 Humble       │          │
│  │ - serial→DDS bridge │  │  - Subscriber       │          │
│  └──────────┬──────────┘  │  - Data fusion      │          │
│             │             │  - Publisher        │          │
│             ↓ DDS         └──────────┬──────────┘          │
│    ┌─────────────────────────────┐   │                     │
│    │   Fast-DDS / CycloneDDS     │   │                     │
│    └─────────────────────────────┘   │                     │
│                                      ↓                     │
│                           ┌─────────────────────┐          │
│                           │  Zenoh Bridge       │          │
│                           │  (Optional)         │          │
│                           └─────────────────────┘          │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 传感器数据流

### PX4 内部数据流
```
IMU Raw Data (BMI088 + BMI270)
         ↓
   SensorHub / SensorFusion
         ↓
   ┌──────┴──────┐
   ↓            ↓
EKF2         uXRCE-DDS
Fusion       Client
   ↓            ↓
   ↓        ┌────┴────┐
   ↓        ↓         ↓
Position  /fmu/sensor_combined/out
Estimation /fmu/vehicle_attitude/out
   ↓       /fmu/optical_flow/out
GPS       /fmu/distance_sensor/out
融合            ↓
   ↓      USB → Agent → DDS → ROS 2
   ↓
MAVLink → Ground Station
```

---

## ⚙️ 推荐参数配置

### 完整 PX4 参数设置
```bash
# 串口配置
MAV_1_CONFIG = 102    # TELEM2 (UART4)
SER_TEL3_BAUD = 115200  # MTF01 波特率

# 光流和测距配置
EKF2_OF_CTRL = 1       # 启用光流
EKF2_RNG_CTRL = 1      # 启用测距
EKF2_HGT_REF = 3       # 距离传感器作为高度参考
SENS_FLOW_ROT = 0      # 光流无旋转

# uXRCE-DDS 配置
UXRCE_DDS_CFG = 1      # Serial0 (USB)
UXRCE_DDS_DOM_ID = 0   # DDS Domain ID
UXRCE_DDS_KEY = 1      # Session Key

# 磁力计配置 (板载罗盘)
CAL_MAG0_PRIO = -1     # 禁用板载罗盘 (如有干扰)
```

---

## 🔍 验证步骤

### 1. 飞控连接验证
```bash
# 检查 USB 设备
ls -la /dev/ttyACM*

# 读取飞控参数
param show MAV_1_CONFIG
param show EKF2_OF_CTRL
```

### 2. uXRCE-DDS 测试
```bash
# 终端 1: 启动 Agent
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600

# 终端 2: 监听 DDS 话题
ros2 topic list
ros2 topic echo /fmu/sensor_combined/out
```

### 3. 数据完整性检查
```bash
# 检查消息频率
ros2 topic hz /fmu/sensor_combined/out
ros2 topic hz /fmu/optical_flow/out
ros2 topic hz /fmu/distance_sensor/out
```

---

## 📁 项目文件结构

```
px4_insider/
├── docs/
│   ├── uxrce_dds_research.md          # uXRCE-DDS 基础调研
│   └── micoair743_config_guide.md     # 本配置指南
├── scripts/
│   ├── start_uxrce_agent.sh           # 启动 uXRCE Agent
│   └── setup_sensors.sh               # 传感器配置脚本
├── src/
│   ├── px4_bridge/                    # ROS 2 桥接节点
│   │   ├── px4_bridge/
│   │   │   ├── __init__.py
│   │   │   ├── node.py
│   │   │   └── subscribers.py
│   │   └── setup.py
│   └── zenoh_bridge/                  # Zenoh 桥接 (可选)
├── config/
│   ├── px4_params.txt                 # 推荐 PX4 参数
│   └── sensor_calibration.yaml        # 传感器标定参数
└── README.md
```

---

## ⚠️ 注意事项

1. **串口选择**: 建议使用 USB (ttyACM0) 作为 uXRCE-DDS 专用端口
2. **波特率**: USB 建议 921600，MTF01 使用 115200
3. **光流安装**: 确保光流传感器方向与飞控方向一致
4. **板载罗盘**: 如有磁干扰，可通过 `CAL_MAG0_PRIO = -1` 禁用
5. **EKF2 调优**: 根据实际飞行环境调整 EKF2 参数

---

## 📚 参考链接

- [MicoAir743v2 用户手册](https://micoair.cn/docs/MicoAir743V2-fei-kong-yong-hu-shou-ce)
- [MTF01 光流传感器手册](https://micoair.cn/docs/MTF01-guang-liu-ce-ju-yi-ti-chuan-gan-qi-yong-hu-shou-ce)
- [PX4 uXRCE-DDS 文档](https://docs.px4.io/main/en/middleware/uxrce_dds/)
- [ROS 2 PX4 集成](https://docs.px4.io/main/en/ros/ros2_comm/)
- [Minderring PX4 Fork (1.14.3)](https://github.com/Minderring/PX4-Autopilot/tree/micoair743-v1.14.3/boards/micoair)
- [官方 PX4 Main 分支](https://github.com/PX4/PX4-Autopilot/tree/main/boards/micoair/h743-v2)

---

*文档创建时间: 2026-02-02*
*Author: OpenClaw Agent*
