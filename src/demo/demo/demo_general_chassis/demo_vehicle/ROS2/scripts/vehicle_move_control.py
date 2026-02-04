#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import argparse
import sys

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('车辆控制器节点已启动')
    
    def move_forward(self, linear_speed, duration):
        """
        控制车辆正前方移动
        
        Args:
            linear_speed (float): 线速度 (m/s)，正值向前，负值向后
            duration (float): 移动时间 (秒)
        """
        self.get_logger().info(f'开始控制车辆: 速度={linear_speed} m/s, 时间={duration} s')
        
        # 创建Twist消息
        twist = Twist()
        twist.linear.x = linear_speed  # 正前方线速度
        twist.linear.y = 0.0           # 侧向速度为0
        twist.linear.z = 0.0           # 垂直速度为0
        twist.angular.x = 0.0          # 横滚角速度为0
        twist.angular.y = 0.0          # 俯仰角速度为0
        twist.angular.z = 0.0          # 偏航角速度为0（不转向）
        
        # 计算移动距离
        distance = abs(linear_speed * duration)
        direction = "向前" if linear_speed > 0 else "向后"
        
        self.get_logger().info(f'车辆开始{direction}移动，预计距离: {distance:.2f} 米')
        
        # 发布控制命令
        start_time = time.time()
        rate = self.create_rate(10)  # 10Hz发布频率
        
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            rate.sleep()
        
        # 停止车辆
        self.stop_vehicle()
        
        self.get_logger().info(f'移动完成! 实际移动时间: {time.time() - start_time:.2f} 秒')
    
    def move_distance(self, linear_speed, distance):
        """
        控制车辆移动指定距离
        
        Args:
            linear_speed (float): 线速度 (m/s)
            distance (float): 移动距离 (米)，正值向前，负值向后
        """
        if linear_speed == 0:
            self.get_logger().error('速度不能为0')
            return
        
        # 计算所需时间
        duration = abs(distance / linear_speed)
        
        # 调整速度方向以匹配距离方向
        adjusted_speed = linear_speed if distance > 0 else -abs(linear_speed)
        
        self.get_logger().info(f'移动距离控制: 目标距离={distance} m, 速度={adjusted_speed} m/s, 预计时间={duration} s')
        
        self.move_forward(adjusted_speed, duration)
    
    def stop_vehicle(self):
        """停止车辆运动"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # 发布几次停止命令确保车辆停止
        for _ in range(3):
            self.publisher.publish(twist)
            time.sleep(0.1)
        
        self.get_logger().info('车辆已停止')

def main():
    rclpy.init()
    
    try:
        controller = VehicleController()
        
        # 解析命令行参数
        parser = argparse.ArgumentParser(description='车辆移动控制脚本')
        group = parser.add_mutually_exclusive_group(required=True)
        
        # 时间控制模式
        group.add_argument('-t', '--time', type=float, metavar='SECONDS',
                          help='按时间控制移动（秒）')
        
        # 距离控制模式
        group.add_argument('-d', '--distance', type=float, metavar='METERS',
                          help='按距离控制移动（米）')
        
        # 速度参数
        parser.add_argument('-s', '--speed', type=float, required=True, metavar='M/S',
                          help='移动速度（米/秒），正值向前，负值向后')
        
        args = parser.parse_args()
        
        # 验证参数
        if args.speed == 0:
            print("错误: 速度不能为0")
            sys.exit(1)
        
        if args.time is not None and args.time <= 0:
            print("错误: 时间必须大于0")
            sys.exit(1)
        
        # 执行移动控制
        if args.time:
            controller.move_forward(args.speed, args.time)
        elif args.distance:
            controller.move_distance(args.speed, args.distance)
        
    except KeyboardInterrupt:
        print("\n用户中断，正在停止车辆...")
        if 'controller' in locals():
            controller.stop_vehicle()
    except Exception as e:
        print(f"错误: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
