# 车辆移动控制脚本

这个脚本允许你精确控制底盘按正前方移动，可以指定速度、时间或距离。

## 前提条件

1. 首先启动车辆控制系统：
```bash
ros2 launch ~/sdk_echo_ws/src/demo/demo/demo_general_chassis/demo_vehicle/ROS2/launch/bringup_basic_ctrl.launch.py
```

2. 确保车辆周围有足够的安全空间

## 使用方法

### 1. 按时间控制移动

控制车辆以指定速度移动指定时间：

```bash
# 基本语法
ros2 run xpkg_demo vehicle_move_control.py -- --speed <速度> --time <时间>

# 示例：以0.5m/s速度向前移动3秒
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.5 --time 3.0

# 示例：以0.3m/s速度向后移动2秒
ros2 run xpkg_demo vehicle_move_control.py -- --speed -0.3 --time 2.0
```

### 2. 按距离控制移动

控制车辆以指定速度移动指定距离：

```bash
# 基本语法
ros2 run xpkg_demo vehicle_move_control.py -- --speed <速度> --distance <距离>

# 示例：以0.5m/s速度向前移动2米
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.5 --distance 2.0

# 示例：以0.2m/s速度向后移动1米
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.2 --distance -1.0
```

## 参数说明

- `--speed`: 速度（米/秒）
  - 正值：向前移动
  - 负值：向后移动
  - 建议：0.1-2.0 m/s 范围内

- `--time`: 移动时间（秒）
  - 必须大于0
  - 与`--distance`参数二选一

- `--distance`: 移动距离（米）
  - 正值：向前移动
  - 负值：向后移动
  - 与`--time`参数二选一

## 使用示例

### 保守测试（推荐首次使用）
```bash
# 很慢向前移动1秒
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.1 --time 1.0

# 慢速向前移动0.5米
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.2 --distance 0.5
```

### 正常使用
```bash
# 中速向前移动3秒
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.5 --time 3.0

# 中速向前移动5米
ros2 run xpkg_demo vehicle_move_control.py -- --speed 0.8 --distance 5.0
```

### 快速移动（谨慎使用）
```bash
# 快速向前移动2秒
ros2 run xpkg_demo vehicle_move_control.py -- --speed 1.5 --time 2.0

# 快速向前移动10米
ros2 run xpkg_demo vehicle_move_control.py -- --speed 1.2 --distance 10.0
```

## 安全注意事项

1. **首次使用**：建议先用很小的速度（如0.1m/s）和短时间测试
2. **空间检查**：确保车辆前方有足够的移动空间
3. **随时停止**：按`Ctrl+C`可以立即停止车辆
4. **监控车辆**：移动过程中保持对车辆的观察
5. **速度控制**：避免使用过高的速度，建议不超过2.0m/s

## 工作原理

脚本通过向`/cmd_vel`话题发布`geometry_msgs/Twist`消息来控制车辆：

- `linear.x`: 设置线速度（正值向前，负值向后）
- `linear.y`, `linear.z`: 设置为0（不进行侧向或垂直移动）
- `angular.x`, `angular.y`, `angular.z`: 设置为0（不进行旋转）

脚本会以10Hz的频率持续发布控制命令，达到指定时间或距离后自动发送停止命令。

## 故障排除

如果车辆没有响应：

1. 检查是否正确启动了`bringup_basic_ctrl.launch.py`
2. 确认`/cmd_vel`话题是否存在：`ros2 topic list | grep cmd_vel`
3. 检查节点是否正常运行：`ros2 node list | grep vehicle`
4. 查看脚本日志输出，确认是否有错误信息

## 扩展功能

你可以修改脚本来实现更复杂的控制，如：
- 添加转向控制
- 实现轨迹跟踪
- 添加障碍物检测
- 集成传感器反馈
