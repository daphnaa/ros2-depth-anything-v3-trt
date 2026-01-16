#!/bin/bash
# ============================================================
# 键盘控制小车启动脚本
# ============================================================
# 功能：
#   1. 启动小车底盘控制节点（不启动RViz）
#   2. 启动键盘遥控节点
#
# 使用方法：
#   ./run_keyboard_control.sh                    # 使用默认速度
#   ./run_keyboard_control.sh --linear-speed 0.3 # 设置线速度
#   ./run_keyboard_control.sh -l 0.3 -a 0.8      # 设置线速度和角速度
#
# 参数：
#   -l, --linear-speed  : 初始线速度 (m/s), 默认: 0.5
#   -a, --angular-speed : 初始角速度 (rad/s), 默认: 1.0
#   -h, --help          : 显示帮助信息
#
# 控制按键：
#   W : 前进    S : 后退
#   A : 左转    D : 右转
#   Q : 左平移  E : 右平移 (全向底盘)
#   空格 : 紧急停止
#   +/- : 调整速度
#   Ctrl+C : 退出
# ============================================================

set -e

# 默认参数
LINEAR_SPEED=0.5
ANGULAR_SPEED=1.0

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示帮助
show_help() {
    echo "============================================================"
    echo "  键盘控制小车启动脚本"
    echo "============================================================"
    echo ""
    echo "使用方法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -l, --linear-speed <速度>   初始线速度 (m/s), 默认: 0.5"
    echo "  -a, --angular-speed <速度>  初始角速度 (rad/s), 默认: 1.0"
    echo "  -h, --help                  显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                          # 使用默认速度"
    echo "  $0 -l 0.3                   # 设置线速度为 0.3 m/s"
    echo "  $0 -l 0.3 -a 0.8            # 设置线速度和角速度"
    echo "  $0 --linear-speed 0.5       # 使用长参数名"
    echo ""
    echo "控制按键:"
    echo "  W : 前进    S : 后退"
    echo "  A : 左转    D : 右转"
    echo "  Q : 左平移  E : 右平移 (全向底盘)"
    echo "  空格 : 紧急停止"
    echo "  +/- : 调整速度"
    echo "  Ctrl+C : 退出"
    echo "============================================================"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -l|--linear-speed)
            LINEAR_SPEED="$2"
            shift 2
            ;;
        -a|--angular-speed)
            ANGULAR_SPEED="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知参数 $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  键盘控制小车启动脚本${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在停止所有节点...${NC}"
    
    # 停止键盘控制节点
    pkill -f "keyboard_teleop.py" 2>/dev/null || true
    
    # 停止小车控制节点
    pkill -f "xnode_vehicle" 2>/dev/null || true
    pkill -f "xnode_comm" 2>/dev/null || true
    
    # 等待进程结束
    sleep 1
    
    echo -e "${GREEN}所有节点已停止${NC}"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未设置${NC}"
    echo "请先运行: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo -e "${GREEN}✓ ROS2环境: $ROS_DISTRO${NC}"

# Source工作空间
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo -e "${GREEN}✓ 工作空间已加载${NC}"
else
    echo -e "${YELLOW}警告: 未找到工作空间setup.bash，尝试继续...${NC}"
fi

# 检查键盘控制脚本
KEYBOARD_SCRIPT="$SCRIPT_DIR/keyboard_teleop.py"
if [ ! -f "$KEYBOARD_SCRIPT" ]; then
    echo -e "${RED}错误: 未找到键盘控制脚本: $KEYBOARD_SCRIPT${NC}"
    exit 1
fi

# 确保脚本可执行
chmod +x "$KEYBOARD_SCRIPT"

echo ""
echo -e "${BLUE}配置信息:${NC}"
echo -e "  线速度: ${GREEN}$LINEAR_SPEED${NC} m/s"
echo -e "  角速度: ${GREEN}$ANGULAR_SPEED${NC} rad/s"
echo ""

# 清理可能存在的旧进程
echo -e "${YELLOW}清理旧进程...${NC}"
pkill -f "xnode_vehicle" 2>/dev/null || true
pkill -f "xnode_comm" 2>/dev/null || true
pkill -f "keyboard_teleop.py" 2>/dev/null || true
sleep 2

# 获取配置文件路径
VEHICLE_INI_PATH=""
if [ -d "$SCRIPT_DIR/install/xpkg_vehicle/share/xpkg_vehicle/ini" ]; then
    VEHICLE_INI_PATH="$SCRIPT_DIR/install/xpkg_vehicle/share/xpkg_vehicle/ini/device_id_list.ini"
fi

echo -e "${BLUE}启动小车控制节点...${NC}"

# 启动通信节点 (后台运行)
echo -e "  启动 xnode_comm..."
ros2 run xpkg_comm xnode_comm \
    --ros-args \
    -p dev_list:=false \
    -p com_enable:=true \
    -p com_channel_common:=false \
    -p com_channel_xstd:=true \
    &
COMM_PID=$!
echo -e "  ${GREEN}✓ xnode_comm 已启动 (PID: $COMM_PID)${NC}"

sleep 1

# 启动车辆控制节点 (后台运行)
echo -e "  启动 xnode_vehicle..."
if [ -n "$VEHICLE_INI_PATH" ] && [ -f "$VEHICLE_INI_PATH" ]; then
    ros2 run xpkg_vehicle xnode_vehicle \
        --ros-args \
        -p ini_path:="$VEHICLE_INI_PATH" \
        -p show_path:=false \
        -p show_loc:=false \
        -p calc_speed:=false \
        -p mode_can_lock:=false \
        -p rate_x:=1.0 \
        -p rate_y:=1.0 \
        -p rate_z:=1.0 \
        -p rate_az:=1.0 \
        &
else
    ros2 run xpkg_vehicle xnode_vehicle \
        --ros-args \
        -p show_path:=false \
        -p show_loc:=false \
        -p calc_speed:=false \
        -p mode_can_lock:=false \
        -p rate_x:=1.0 \
        -p rate_y:=1.0 \
        -p rate_z:=1.0 \
        -p rate_az:=1.0 \
        &
fi
VEHICLE_PID=$!
echo -e "  ${GREEN}✓ xnode_vehicle 已启动 (PID: $VEHICLE_PID)${NC}"

# 等待节点启动
echo ""
echo -e "${YELLOW}等待节点初始化...${NC}"
sleep 3

# 检查节点是否正常运行
if ! kill -0 $COMM_PID 2>/dev/null; then
    echo -e "${RED}错误: xnode_comm 启动失败${NC}"
    cleanup
    exit 1
fi

if ! kill -0 $VEHICLE_PID 2>/dev/null; then
    echo -e "${RED}错误: xnode_vehicle 启动失败${NC}"
    cleanup
    exit 1
fi

echo -e "${GREEN}✓ 小车控制节点启动成功${NC}"
echo ""

# 启动键盘控制节点 (前台运行)
echo -e "${BLUE}启动键盘控制节点...${NC}"
echo ""

python3 "$KEYBOARD_SCRIPT" \
    --linear-speed "$LINEAR_SPEED" \
    --angular-speed "$ANGULAR_SPEED"

# 正常退出时清理
cleanup
