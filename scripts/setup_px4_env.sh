#!/bin/bash
# setup_px4_env.sh - PX4 v1.14.3 开发环境自动配置脚本 (macOS)

set -e

echo "🚀 PX4 v1.14.3 开发环境配置"
echo "================================"

# 检查系统
if [[ "$OSTYPE" != "darwin"* ]]; then
    echo "⚠️  此脚本仅支持 macOS"
    exit 1
fi

# 检查是否为 root 用户
if [[ $EUID -eq 0 ]]; then
    echo "⚠️  请不要使用 root 用户运行"
    exit 1
fi

# 安装 ARM GCC 工具链
echo ""
echo "🔨 安装 ARM GCC 工具链..."
ARM_PKG="/tmp/arm-gnu-toolchain.pkg"
if [ -f "$ARM_PKG" ]; then
    echo "✅ 找到已下载的工具链: $ARM_PKG"
    echo ""
    echo "📋 请手动安装:"
    echo "   1. 双击打开 $ARM_PKG"
    echo "   2. 按照安装向导完成安装"
    echo "   3. 安装路径通常是: /Applications/ArmGNU-toolchain/"
    echo ""
    echo "   或者在终端运行:"
    echo "   sudo installer -pkg $ARM_PKG -target /"
    echo ""
    read -p "按回车键继续 (安装完成后)..."
    
    # 验证安装
    if [ -f "/Applications/ArmGNU-toolchain/bin/arm-none-eabi-gcc" ]; then
        export PATH="/Applications/ArmGNU-toolchain/bin:$PATH"
        echo "✅ ARM GCC 已安装"
        /Applications/ArmGNU-toolchain/bin/arm-none-eabi-gcc --version | head -1
    elif command -v arm-none-eabi-gcc &> /dev/null; then
        echo "✅ ARM GCC 已安装"
        arm-none-eabi-gcc --version | head -1
    else
        echo "❌ ARM GCC 未正确安装"
        exit 1
    fi
else
    echo "📥 下载 ARM GCC 工具链..."
    curl -L -o /tmp/arm-gnu-toolchain.pkg \
        "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.pkg"
    
    echo ""
    echo "📋 请手动安装下载的工具包"
    echo "   sudo installer -pkg /tmp/arm-gnu-toolchain.pkg -target /"
    exit 0
fi

# 安装 Homebrew (如果未安装)
echo ""
echo "🍺 检查 Homebrew..."
if ! command -v brew &> /dev/null; then
    echo "📥 安装 Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
else
    echo "✅ Homebrew 已安装"
fi

# 安装系统依赖
echo ""
echo "📦 安装系统依赖..."
brew install cmake ninja python3 wget zip unzip

# 安装 Python 依赖
echo ""
echo "🐍 安装 Python 依赖..."
pip3 install --user pyserial pandas toml numpy

# 克隆 PX4 源码
echo ""
echo "📥 克隆 PX4 源码..."
if [ ! -d "PX4-Autopilot" ]; then
    git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    git submodule update --init --recursive
else
    echo "⚠️  PX4-Autopilot 已存在"
    cd PX4-Autopilot
fi

# 检查 micoair board
echo ""
echo "🔍 检查 micoair_h743-v2 board..."
if [ -d "boards/micoair/h743-v2" ]; then
    echo "✅ micoair_h743-v2 board 已配置"
    ls -la boards/micoair/h743-v2/
else
    echo "❌ micoair_h743-v2 board 不存在，需要创建"
    exit 1
fi

echo ""
echo "================================"
echo "✅ PX4 开发环境配置完成!"
echo ""
echo "📋 下一步操作:"
echo "1. cd PX4-Autopilot"
echo "2. source /opt/homebrew/bin/brew shellenv  # 如果需要"
echo "3. export PATH="/Applications/ArmGNU-toolchain/bin:\$PATH""
echo "4. make micoair_h743-v2_bootloader"
echo "5. make micoair_h743-v2_default"
echo ""
echo "💡 提示: 编译可能需要 10-30 分钟，取决于网络和硬件"
