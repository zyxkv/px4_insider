# PX4 Development Environment Setup

## Quick Start

### 1. Clone Repository

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/zyxkv/px4_insider.git
cd px4_insider

# If already cloned without submodules, initialize them
git submodule update --init --recursive
```

### 2. Install Dependencies (Ubuntu 22.04)

```bash
# Run automatic setup
cd ~/projects/px4_insider
chmod +x scripts/setup_px4_env.sh
./scripts/setup_px4_env.sh
```

### 3. Manual Installation

```bash
# System dependencies
sudo apt update
sudo apt install -y build-essential cmake git ninja-build \
    libssl-dev python3 python3-pip wget zip unzip \
    avahi-daemon libavahi-compat-libdnssd-dev

# ARM GCC toolchain
wget https://github.com/PX4/toolchain/releases/download/v0.24/PX4-toolchain-0.24-linux-x64.deb
sudo apt install -y ./PX4-toolchain-0.24-linux-x64.deb

# Python dependencies
pip3 install --user pyserial pandas toml numpy
```

### 4. Build Firmware

```bash
cd PX4-Autopilot

# Build bootloader
make micoair_h743-v2_bootloader

# Build main firmware
make micoair_h743-v2_default
```

## Board Configuration

**MicoAir743v2 (Custom Board)**
- Location: `boards/micoair/h743-v2/`
- Target: `micoair_h743-v2_default`
- Processor: STM32H743VIT6, 480MHz, 2MB Flash
- Sensors: BMI088, BMI270, SPL06, QMC5883L

## File Structure

```
px4_insider/
├── PX4-Autopilot/              # PX4 source (v1.14.3 submodule)
│   └── boards/micoair/h743-v2/ # Board config
├── MicoAir743V2-Manual/        # Hardware manual and images
├── config/
│   └── px4_params.txt          # PX4 parameters
├── docs/
│   ├── px4_dev_setup.md
│   ├── micoair743_config_guide.md
│   └── uxrce_dds_research.md
└── scripts/
    ├── setup_px4_env.sh        # Auto setup script
    ├── start_uxrce_agent.sh
    └── setup_sensors.sh
```

**Note:** PX4-Autopilot is included as a git submodule pointing to v1.14.3. When cloning this repository, use `--recurse-submodules` flag or run `git submodule update --init --recursive` after cloning.

## Troubleshooting

### Compilation Errors
```bash
# Clean build
make distclean
make micoair_h743-v2_default
```

### USB Connection Issues
```bash
# Check device
ls -la /dev/ttyACM*

# Set permissions
sudo chmod 666 /dev/ttyACM0
```

### Toolchain Not Found
```bash
# Verify installation
arm-none-eabi-gcc --version

# Reinstall if needed
# See: https://docs.px4.io/main/en/dev_setup/building_px4.html
```

## References

- [PX4 Build Guide](https://docs.px4.io/main/en/dev_setup/building_px4.html)
- [MicoAir743v2 Manual](https://micoair.cn/docs/MicoAir743V2-fei-kong-yong-hu-shou-ce)
