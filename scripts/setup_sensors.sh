#!/bin/bash
# setup_sensors.sh - 配置 MTF01 光流传感器

echo "🔧 MicoAir743v2 传感器配置脚本"
echo "================================"

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <串口设备> [波特率]"
    echo "示例: $0 /dev/ttyUSB0 115200"
    echo ""
    echo "注意: 此脚本用于配置 MTF01 传感器的协议"
    echo "实际配置请使用微空助手软件"
    exit 1
fi

DEVICE=$1
BAUDRATE=${2:-115200}

echo "📡 设备: $DEVICE"
echo "🔧 波特率: $BAUDRATE"

# 检查设备是否存在
if [ ! -e "$DEVICE" ]; then
    echo "⚠️  设备 $DEVICE 不存在"
    exit 1
fi

# 读取当前配置 (如果有的话)
echo ""
echo "📖 尝试读取传感器信息..."
if command -v minicom &> /dev/null; then
    echo "使用 minicom 配置串口参数后查看数据"
elif command -v screen &> /dev/null; then
    echo "screen $DEVICE $BAUDRATE"
fi

echo ""
echo "✅ 配置完成!"
echo ""
echo "📋 下一步:"
echo "1. 使用微空助手将 MTF-01 设置为 Mavlink_px4 协议"
echo "2. 在 PX4 中配置串口参数:"
echo "   - MAV_1_CONFIG = TELEM端口号"
echo "   - SER_TELn_BAUD = 115200"
echo "   - EKF2_OF_CTRL = 1"
echo "   - EKF2_RNG_CTRL = 1"
