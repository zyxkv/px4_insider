# PX4 v1.14.3 开发环境配置指南

## 📋 系统要求

- **操作系统**: Ubuntu 20.04/22.04 (推荐) 或 macOS
- **内存**: 8GB+ (编译需要)
- **存储**: 20GB+ 可用空间
- **Python**: 3.8+

## 🔧 Ubuntu 22.04 安装步骤

### 1. 安装编译工具链

```bash
# 更新系统
sudo apt update
sudo apt upgrade -y

# 安装依赖
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    libssl-dev \
    python3 \
    python3-pip \
    wget \
    zip \
    unzip \
    avahi-daemon \
    libavahi-compat-libdnssd-dev

# 安装 ARM GCC 工具链
cd ~/
wget https://github.com/PX4/toolchain/releases/download/v0.24/PX4-toolchain-0.24-linux-x64.deb
sudo apt install -y ./PX4-toolchain-0.24-linux-x64.deb
rm PX4-toolchain-0.24-linux-x64.deb

# 验证安装
arm-none-eabi-gcc --version
```

### 2. 安装 Python 依赖

```bash
pip3 install --user pyserial pandas toml numpy
```

### 3. 安装 Fast DDS (ROS 2 集成需要)

```bash
sudo apt install -y ros-humble-fast-dds
```

---

## 📦 macOS 安装步骤

### 1. 安装 Homebrew

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### 2. 安装依赖

```bash
brew install cmake ninja python3 avahi wget zip unzip

# 安装 ARM GCC 工具链
brew install --cask gcc-arm-embedded
```

---

## 🐙 克隆 PX4 源码

### 1. 克隆官方仓库并切换到 v1.14.3

```bash
cd ~/projects/px4_insider
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.3
```

### 2. 更新子模块

```bash
git submodule update --init --recursive
```

---

## ⚙️ MicoAir743v2 Board 配置

### 已创建自定义 Board 配置

```bash
# 位置
ls -la boards/micoair/h743-v2/
```

Board 配置文件:
- `default.px4board` - 主要配置
- `bootloader.px4board` - Bootloader 配置
- `nuttx-config/` - NuttX 内核配置

**已包含:**
- ✅ BMI088 + BMI270 双IMU支持
- ✅ SPL06 气压计
- ✅ QMC5883L 磁力计
- ✅ 8x UART 串口配置
- ✅ uXRCE-DDS 客户端
- ✅ EKF2 传感器融合

---

## 🔨 编译固件

### 1. 编译 Bootloader

```bash
cd ~/projects/px4_insider/PX4-Autopilot
make micoair_h743-v2_bootloader
```

### 2. 编译固件

```bash
# 完整编译
make micoair_h743-v2_default

# 仅编译 (不链接)
make micoair_h743-v2_default ARCHIVER=ar
```

### 3. 编译输出

编译成功后，固件位于:
```
build/micoair_h743-v2_default/micoair_h743-v2.px4
```

---

## 📁 项目结构

```
px4_insider/
├── PX4-Autopilot/              # PX4 官方源码 (v1.14.3)
│   └── boards/micoair/h743-v2/ # 自定义 Board 配置
│       ├── default.px4board
│       ├── bootloader.px4board
│       ├── nuttx-config/
│       └── init/
├── docs/                       # 文档
│   ├── micoair743_config_guide.md
│   └── px4_dev_setup.md        # 本文档
├── scripts/                    # 工具脚本
└── config/                     # 配置文件
```

---

## 🔌 USB 驱动安装

### Linux (udev rules)

```bash
# 创建 udev 规则
sudo nano /etc/udev/rules.d/50-px4.rules
```

添加以下内容:
```
# STM32 DFU
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666", GROUP="plugdev"

# FTDI Serial
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="0666", GROUP="plugdev"

# USB Serial (CDC-ACM)
SUBSYSTEM=="usb", ATTR{idVendor}=="26ac", MODE="0666", GROUP="plugdev"
```

重新加载 udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### macOS

无需额外驱动，使用系统内置 CDC-ACM 驱动。

---

## 🧪 测试编译

### 验证 Board 配置

```bash
cd ~/projects/px4_insider/PX4-Autopilot

# 检查 board 是否被识别
make list_defconfigs | grep micoair

# 应该输出:
# micoair_h743-v2_default
```

### 编译测试

```bash
# 快速编译测试 (不完整编译)
make micoair_h743-v2_default configure

# 完整编译
make micoair_h743-v2_default
```

---

## 🚀 烧录固件

### 1. 进入 DFU 模式

1. 按住飞控 **BOOT** 按钮
2. 连接 USB
3. 保持 2 秒后松开

### 2. 使用 QGC 烧录

1. 打开 QGroundControl
2. 连接飞控
3. 固件页面选择 "Advanced"
4. 选择本地 `.px4` 文件

### 3. 命令行烧录 (macOS/Linux)

```bash
cd ~/projects/px4_insider/PX4-Autopilot
make micoair_h743-v2_default upload
```

---

## 📚 参考文档

- [PX4 官方编译文档](https://docs.px4.io/main/en/dev_setup/building_px4.html)
- [PX4 源码结构](https://docs.px4.io/main/en/concept/architecture/)
- [NuttX 配置](https://docs.px4.io/main/en/config/)
- [MicoAir743v2 用户手册](https://micoair.cn/docs/MicoAir743V2-fei-kong-yong-hu-shou-ce)

---

## ⚠️ 常见问题

### Q: 编译报错 "arm-none-eabi-gcc: command not found"
A: 工具链未正确安装，参考 "安装编译工具链" 部分

### Q: 编译内存不足
A: 增加 swap 空间或使用 Ninja 构建器:
```bash
make micoair_h743-v2_default ninja
```

### Q: 找不到 micoair_h743-v2 target
A: Board 配置未正确创建，检查:
```bash
ls -la boards/micoair/h743-v2/
```

### Q: USB 连接问题
A: 检查 udev 规则或尝试:
```bash
sudo chmod 666 /dev/ttyACM0
```

---

*文档更新时间: 2026-02-02*
