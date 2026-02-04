#!/bin/bash
# start_uxrce_agent.sh - 启动 uXRCE-DDS Agent

DEVICE="/dev/ttyACM0"
BAUDRATE="921600"

echo "🚀 启动 uXRCE-DDS Agent..."
echo "📡 设备: $DEVICE"
echo "🔧 波特率: $BAUDRATE"

# 检查设备
if [ ! -e "$DEVICE" ]; then
    echo "⚠️  设备 $DEVICE 不存在"
    exit 1
fi

# 启动 Agent
MicroXRCEAgent serial --dev "$DEVICE" -b "$BAUDRATE"
