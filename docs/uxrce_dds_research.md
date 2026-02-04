# PX4 uXRCE-DDS 调研报告

## 1. uXRCE-DDS 概述

### 1.1 什么是 uXRCE-DDS?
- **uXRCE-DDS** = Micro XRCE-DDS (DDS for eXtremely Resource-Constrained Environments)
- 由 eProsima 开发，是 DDS (Data Distribution Service) 的轻量级版本
- 专为资源受限的嵌入式系统设计
- PX4 飞控中用于与地面站/GCS 和机载电脑通信

### 1.2 架构
```
PX4 Flight Stack (uXRCE-DDS Client)
          │
    Serial/USB Connection
          │
   uXRCE-DDS Agent (机载电脑端)
          │
    DDS (ROS 2 / Zenoh)
```

## 2. PX4 中的 uXRCE-DDS 实现

### 2.1 关键组件
- **uxrce_dds_client**: PX4 飞控上的客户端模块
- **uxrce_dds_agent**: 机载电脑上的代理服务

### 2.2 PX4 固件配置
```bash
CONFIG_MODULES_UXRCE_DDS_CLIENT=y
```

## 3. 多传感器数据融合方案

### 3.1 PX4 传感器数据流
```
IMU → SensorFusion (EKF2) → 姿态/位置估计
  ↓
Barometer → 高度估计
  ↓
GPS → 位置估计
  ↓
所有数据 → uXRCE-DDS → 机载电脑
```

### 3.2 传输优化
- 使用 CDDR 减小消息大小
- 调整 QoS 配置
- 批量发送减少开销

## 4. ROS 2 Humble 集成

### 4.1 安装
```bash
sudo apt install ros-humble-ros-base
sudo apt install ros-humble-fast-dds
```

### 4.2 启动 Agent
```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 4.3 ROS 2 订阅示例
```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined

class PX4Bridge(Node):
    def __init__(self):
        super().__init__('px4_bridge')
        self.sub = self.create_subscription(
            SensorCombined, '/fmu/sensor_combined/out', 
            self.callback, 10)
    
    def callback(self, msg):
        # 处理传感器数据
        pass
```

## 5. Zenoh 集成

### 5.1 Zenoh 优势
- 更低延迟
- 支持 UDP/TCP/QUIC
- 更好资源效率

### 5.2 Zenoh-DDS 桥接
```bash
zenoh-bridge-dds
```

## 6. USB 接口配置

### 6.1 串口参数
- 波特率: 921600
- 设备: /dev/ttyACM0 (CDC-ACM)

### 6.2 检查连接
```bash
ls -la /dev/ttyACM*
dmesg | grep ttyACM
```

## 7. 推荐架构

```
┌─────────────────────┐
│    PX4 Flight       │
│  ┌───────────────┐  │
│  │ Sensors→Fusion│  │
│  └───────┬───────┘  │
└──────────┼──────────┘
           │ USB CDC-ACM
           ↓
┌─────────────────────┐
│    机载电脑          │
│  ┌───────────────┐  │
│  │uXRCE-DDS Agent│  │
│  └───────┬───────┘  │
│          ↓          │
│  ┌───────────────┐  │
│  │ROS2 / Zenoh   │  │
│  └───────────────┘  │
└─────────────────────┘
```

## 8. 性能优化建议

1. 只订阅需要的主题
2. 调整发布频率
3. 使用 UDP 替代 TCP
4. 配置 Fast-DDS SHM

## 9. 参考文献

- https://docs.px4.io/main/en/middleware/uxrce_dds/
- https://micro-xrce-dds.readthedocs.io/
- https://zenoh.io/docs/

## 10. 下一步

- [ ] 配置 PX4 开发环境
- [ ] 测试 USB 串口通信
- [ ] 实现 ROS 2 桥接节点
- [ ] 集成 Zenoh

---
*Created: 2026-02-02*
