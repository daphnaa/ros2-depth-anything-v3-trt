# 从 YAML 文件读取相机标定参数

## 概述

系统现在支持从 YAML 文件读取相机标定参数，不再需要在代码或脚本中硬编码参数。

## YAML 文件格式

相机标定文件位于：`camera_info_example.yaml`

```yaml
# 图像分辨率
image_width: 1920
image_height: 1536

# 相机内参
fx: 824.147361  # 焦距 x
fy: 823.660879  # 焦距 y
cx: 958.275200  # 主点 x
cy: 767.389372  # 主点 y

# 鱼眼畸变系数
distortion_model: "fisheye"
k1: 1.486308    # 径向畸变系数 1
k2: -13.386609  # 径向畸变系数 2
p1: 21.409334   # 切向畸变系数 1
p2: 3.817858    # 切向畸变系数 2
k3: 0.0         # 径向畸变系数 3
```

## 使用方法

### 方法 1：使用 YAML 文件（推荐）

```bash
# 使用默认的 camera_info_example.yaml 文件
CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh

# 或使用自定义路径
CAMERA_INFO_FILE=/path/to/your/camera_calibration.yaml ./run_camera_depth.sh
```

### 方法 2：使用环境变量（旧方法，仍然支持）

```bash
# 使用硬编码的标定参数
USE_CALIBRATION=1 ./run_camera_depth.sh
```

### 方法 3：不使用标定（使用估计值）

```bash
# 直接运行，使用估计的相机参数
./run_camera_depth.sh
```

## 优先级

参数读取优先级（从高到低）：

1. **YAML 文件** - 如果指定了 `CAMERA_INFO_FILE` 且文件存在
2. **环境变量** - 如果设置了 `USE_CALIBRATION=1`
3. **默认值** - 代码中的默认值（估计值）

## 创建自己的标定文件

### 1. 复制示例文件

```bash
cp camera_info_example.yaml my_camera_calibration.yaml
```

### 2. 修改参数

编辑 `my_camera_calibration.yaml`，填入你的相机标定参数：

```yaml
# 你的相机分辨率
image_width: 1920
image_height: 1536

# 你的相机内参（从标定工具获得）
fx: 824.147361
fy: 823.660879
cx: 958.275200
cy: 767.389372

# 你的畸变系数
k1: 1.486308
k2: -13.386609
p1: 21.409334
p2: 3.817858
k3: 0.0
```

### 3. 使用你的标定文件

```bash
CAMERA_INFO_FILE=my_camera_calibration.yaml ./run_camera_depth.sh
```

## 鱼眼畸变矫正 (Fisheye Undistortion)

### 概述

系统支持鱼眼镜头畸变矫正功能，可以将鱼眼图像转换为直线图像（rectilinear），使直线在图像中显示为直线。

### 启用畸变矫正

```bash
# 基本用法：启用畸变矫正
ENABLE_UNDISTORTION=1 CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh

# 带平衡参数：控制裁剪与视野的权衡
ENABLE_UNDISTORTION=1 UNDISTORTION_BALANCE=0.5 CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `ENABLE_UNDISTORTION` | 启用畸变矫正 | `0` (禁用) |
| `UNDISTORTION_BALANCE` | 平衡参数 (0.0-1.0) | `0.0` |

### 平衡参数 (Balance)

- **0.0** - 完全裁剪：只保留有效像素区域，无黑边，但视野较小
- **1.0** - 完整视野：保留完整视野，但边缘会有黑边
- **0.5** - 折中方案：平衡裁剪和视野

### 工作原理

1. 读取相机标定参数（内参和畸变系数）
2. 使用 OpenCV fisheye 模块计算畸变矫正映射表
3. 对每帧图像应用畸变矫正
4. 更新相机内参为矫正后的参数
5. 使用矫正后的图像进行深度估计和点云生成

### 性能

- 畸变矫正使用预计算的映射表，运行时只需执行 `cv::remap`
- 典型延迟：< 5ms @ 1920x1536 分辨率
- 可通过 ROS2 DEBUG 日志查看实际耗时：
  ```bash
  ros2 run depth_anything_v3 camera_depth_node --ros-args --log-level debug
  ```

### 注意事项

- **必须提供标定参数**：畸变矫正需要准确的相机标定参数
- **分辨率匹配**：标定参数应与实际相机分辨率匹配
- **畸变模型**：使用 OpenCV fisheye 模型（4 个畸变系数）

## 标定相机

如果你需要重新标定相机，可以使用 ROS 2 标定工具：

```bash
# 安装标定工具
sudo apt install ros-humble-camera-calibration

# 运行标定（鱼眼模式）
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  --fisheye \
  image:=/camera/image \
  camera:=/camera
```

标定完成后，将输出的参数填入 YAML 文件。

## 验证标定

### 检查日志输出

启动节点时，会显示加载的标定参数：

```
[camera_depth_node]: Loading calibration from: camera_info_example.yaml
[camera_depth_node]: ✓ Calibration loaded successfully
[camera_depth_node]:   fx=824.15, fy=823.66, cx=958.28, cy=767.39
[camera_depth_node]:   Distortion: k1=1.4863, k2=-13.3866, p1=21.4093, p2=3.8179, k3=0.0000
```

启用畸变矫正时，还会显示：

```
[camera_depth_node]: Fisheye undistortion: ENABLED (balance=0.00)
[camera_depth_node]: ✓ Fisheye undistorter initialized
[camera_depth_node]:   Undistorted size: 1920x1536
[camera_depth_node]:   New intrinsics: fx=xxx.xx, fy=xxx.xx, cx=xxx.xx, cy=xxx.xx
```

### 检查点云质量

1. 启动系统并打开 RViz
2. 查看点云是否有重叠
3. 检查直线是否笔直（启用畸变矫正后）
4. 验证深度缩放是否正确

## 故障排除

### 文件未找到

```
[camera_depth_node]: Failed to load calibration file: ...
[camera_depth_node]: Will use default or parameter-based calibration
```

**解决方法**：
- 检查文件路径是否正确
- 确保文件存在且可读
- 使用绝对路径或相对于工作目录的路径

### YAML 格式错误

```
[camera_depth_node]: Failed to load calibration file: bad conversion
```

**解决方法**：
- 检查 YAML 语法是否正确
- 确保所有参数都是数字类型
- 检查缩进是否正确

### 畸变矫正初始化失败

```
[camera_depth_node]: Failed to initialize fisheye undistorter
```

**可能原因**：
1. 标定参数不正确或极端
2. 畸变系数导致数值不稳定
3. 相机矩阵无效

**解决方法**：
1. 检查标定参数是否合理
2. 尝试重新标定相机
3. 检查畸变系数是否在合理范围内

### 点云仍然重叠

**可能原因**：
1. 标定参数不正确
2. 相机分辨率与标定时不匹配
3. 降采样因子未正确应用

**解决方法**：
1. 重新标定相机
2. 确保 YAML 文件中的分辨率与实际相机分辨率匹配
3. 检查降采样设置

## 多相机系统

对于多相机系统，每个相机可以有自己的标定文件。需要修改 launch 文件为每个相机指定不同的标定文件。

## 优势

使用 YAML 文件的优势：

1. ✅ **易于管理** - 所有参数集中在一个文件中
2. ✅ **易于共享** - 可以轻松分享标定文件
3. ✅ **版本控制** - 可以用 Git 跟踪标定文件的变化
4. ✅ **多相机支持** - 每个相机可以有独立的标定文件
5. ✅ **无需重新编译** - 修改参数不需要重新编译代码
6. ✅ **标准格式** - 使用 ROS 标准的 YAML 格式
7. ✅ **向后兼容** - 旧的 `USE_CALIBRATION=1` 方法仍然有效

## 使用示例

### 示例 1：使用 YAML 文件（推荐）

```bash
CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh
```

### 示例 2：使用 YAML 文件 + 畸变矫正

```bash
CAMERA_INFO_FILE=camera_info_example.yaml ENABLE_UNDISTORTION=1 ./run_camera_depth.sh
```

### 示例 3：使用 YAML 文件 + 畸变矫正 + 自定义平衡

```bash
CAMERA_INFO_FILE=camera_info_example.yaml ENABLE_UNDISTORTION=1 UNDISTORTION_BALANCE=0.5 ./run_camera_depth.sh
```

### 示例 4：使用环境变量（旧方法）

```bash
USE_CALIBRATION=1 ./run_camera_depth.sh
```

### 示例 5：不使用标定

```bash
./run_camera_depth.sh
```

## 参考

- [ROS Camera Calibration](http://wiki.ros.org/camera_calibration)
- [OpenCV Fisheye Camera Model](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)
- [YAML-CPP Documentation](https://github.com/jbeder/yaml-cpp)
