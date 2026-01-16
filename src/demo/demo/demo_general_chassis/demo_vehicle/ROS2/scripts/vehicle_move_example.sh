#!/bin/bash

# 车辆移动控制示例脚本
# 使用前请确保已经启动了 bringup_basic_ctrl.launch.py

echo "=== 车辆移动控制示例 ==="
echo

# 检查ROS2环境
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "警告: ROS2环境可能未正确设置"
fi

echo "使用方法:"
echo "1. 按时间控制移动:"
echo "   ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.5 --time 3.0"
echo "   # 以0.5m/s速度向前移动3秒"
echo

echo "2. 按距离控制移动:"
echo "   ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.5 --distance 2.0"
echo "   # 以0.5m/s速度向前移动2米"
echo

echo "3. 向后移动:"
echo "   ros2 run xpkg_demo vehicle_move_control.py -- --speed -0.3 --time 2.0"
echo "   # 以0.3m/s速度向后移动2秒"
echo

echo "4. 快速移动:"
echo "   ros2 run xpkg_demo vehicle_move_control.py -- --speed 1.0 --distance 5.0"
echo "   # 以1.0m/s速度向前移动5米"
echo

echo "参数说明:"
echo "  --speed: 速度 (m/s), 正值向前，负值向后"
echo "  --time:  时间 (秒), 与--distance二选一"
echo "  --distance: 距离 (米), 与--time二选一"
echo

echo "安全提示:"
echo "- 请确保车辆周围有足够的空间"
echo "- 建议先用较小的速度测试"
echo "- 随时准备按Ctrl+C紧急停止"
echo

# 询问用户是否要运行示例
read -p "是否要运行一个简单的测试示例？(y/n): " choice
if [[ $choice == "y" || $choice == "Y" ]]; then
    echo "正在执行: 以0.2m/s速度向前移动2秒..."
    ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.2 --time 2.0
fi
