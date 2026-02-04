# TODO - PX4 Development Setup

## P0 - Environment Setup (当前阶段)
- [x] 克隆 PX4 官方仓库 v1.14.3
- [x] 创建 micoair_h743-v2 board 配置
- [ ] 安装编译工具链 (ARM GCC)
- [ ] 安装系统依赖
- [ ] 配置 USB udev 规则
- [ ] 编译 bootloader 测试环境

## P1 - Firmware Build
- [ ] 编译 micoair_h743-v2 bootloader
- [ ] 编译 micoair_h743-v2 固件
- [ ] 验证固件烧录

## P2 - Sensor Integration
- [ ] 配置 MTF01 光流传感器
- [ ] 测试 EKF2 光流融合
- [ ] 验证传感器数据流

## P3 - uXRCE-DDS Integration
- [ ] 测试 USB 串口通信
- [ ] 启动 uXRCE-DDS Agent
- [ ] 验证 DDS 话题发布

## 已完成 ✅
- [x] 创建项目结构
- [x] 调研 uXRCE-DDS 文档
- [x] 调研 MicoAir743v2 硬件配置
- [x] 调研 MTF01 光流传感器配置
- [x] 克隆 PX4 v1.14.3 源码
- [x] 创建 micoair_h743-v2 board 配置 (自定义)
- [x] 编写开发环境配置文档
- [x] 编写自动安装脚本
