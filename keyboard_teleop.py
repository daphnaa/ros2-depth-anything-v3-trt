#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘遥控小车节点
使用 W/A/S/D 控制小车前进/左转/后退/右转
使用 Q/E 控制小车左平移/右平移（如果支持全向移动）
使用空格键紧急停止
使用 +/- 调整速度
按 Ctrl+C 退出

作者: Kiro AI Assistant
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# 控制说明
HELP_MSG = """
========================================
    键盘遥控小车控制器
========================================
控制按键:
    W : 前进
    S : 后退
    A : 左转
    D : 右转
    Q : 左平移 (全向底盘)
    E : 右平移 (全向底盘)
    
速度调节:
    + / = : 增加速度 10%
    - / _ : 减少速度 10%
    
其他:
    空格 : 紧急停止
    H    : 显示帮助
    Ctrl+C : 退出程序

当前速度: 线速度={linear:.2f} m/s, 角速度={angular:.2f} rad/s
========================================
"""


class KeyboardTeleop(Node):
    """键盘遥控节点"""
    
    def __init__(self, linear_speed=0.5, angular_speed=1.0):
        super().__init__('keyboard_teleop')
        
        # 声明参数
        self.declare_parameter('linear_speed', linear_speed)
        self.declare_parameter('angular_speed', angular_speed)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        # 获取参数
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # 创建发布者
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # 当前速度状态
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info(f'键盘遥控节点已启动')
        self.get_logger().info(f'发布话题: {cmd_vel_topic}')
        self.get_logger().info(f'初始线速度: {self.linear_speed} m/s')
        self.get_logger().info(f'初始角速度: {self.angular_speed} rad/s')
    
    def get_key(self, timeout=0.1):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def print_help(self):
        """打印帮助信息"""
        print(HELP_MSG.format(linear=self.linear_speed, angular=self.angular_speed))
    
    def publish_velocity(self):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z
        self.publisher.publish(twist)
    
    def stop(self):
        """停止小车"""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self.publish_velocity()
    
    def run(self):
        """主循环"""
        self.print_help()
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '':
                    # 没有按键，保持当前状态
                    self.publish_velocity()
                    continue
                
                # 处理按键
                if key.lower() == 'w':
                    # 前进
                    self.current_linear_x = self.linear_speed
                    self.current_linear_y = 0.0
                    self.current_angular_z = 0.0
                    print(f'\r前进: {self.linear_speed:.2f} m/s          ', end='')
                    
                elif key.lower() == 's':
                    # 后退
                    self.current_linear_x = -self.linear_speed
                    self.current_linear_y = 0.0
                    self.current_angular_z = 0.0
                    print(f'\r后退: {self.linear_speed:.2f} m/s          ', end='')
                    
                elif key.lower() == 'a':
                    # 左转
                    self.current_linear_x = 0.0
                    self.current_linear_y = 0.0
                    self.current_angular_z = self.angular_speed
                    print(f'\r左转: {self.angular_speed:.2f} rad/s       ', end='')
                    
                elif key.lower() == 'd':
                    # 右转
                    self.current_linear_x = 0.0
                    self.current_linear_y = 0.0
                    self.current_angular_z = -self.angular_speed
                    print(f'\r右转: {self.angular_speed:.2f} rad/s       ', end='')
                    
                elif key.lower() == 'q':
                    # 左平移（全向底盘）
                    self.current_linear_x = 0.0
                    self.current_linear_y = self.linear_speed
                    self.current_angular_z = 0.0
                    print(f'\r左平移: {self.linear_speed:.2f} m/s       ', end='')
                    
                elif key.lower() == 'e':
                    # 右平移（全向底盘）
                    self.current_linear_x = 0.0
                    self.current_linear_y = -self.linear_speed
                    self.current_angular_z = 0.0
                    print(f'\r右平移: {self.linear_speed:.2f} m/s       ', end='')
                    
                elif key == ' ':
                    # 紧急停止
                    self.stop()
                    print('\r紧急停止!                    ', end='')
                    
                elif key in ['+', '=']:
                    # 增加速度
                    self.linear_speed *= 1.1
                    self.angular_speed *= 1.1
                    print(f'\r速度增加: 线速度={self.linear_speed:.2f} m/s, 角速度={self.angular_speed:.2f} rad/s', end='')
                    
                elif key in ['-', '_']:
                    # 减少速度
                    self.linear_speed *= 0.9
                    self.angular_speed *= 0.9
                    # 设置最小速度
                    self.linear_speed = max(0.05, self.linear_speed)
                    self.angular_speed = max(0.1, self.angular_speed)
                    print(f'\r速度减少: 线速度={self.linear_speed:.2f} m/s, 角速度={self.angular_speed:.2f} rad/s', end='')
                    
                elif key.lower() == 'h':
                    # 显示帮助
                    self.print_help()
                    
                elif key == '\x03':
                    # Ctrl+C
                    break
                
                else:
                    # 未知按键，停止
                    self.stop()
                
                # 发布速度
                self.publish_velocity()
                
        except Exception as e:
            self.get_logger().error(f'错误: {e}')
        finally:
            # 确保停止
            self.stop()
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print('\n程序退出')


def main(args=None):
    """主函数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='键盘遥控小车')
    parser.add_argument('--linear-speed', '-l', type=float, default=0.5,
                        help='初始线速度 (m/s), 默认: 0.5')
    parser.add_argument('--angular-speed', '-a', type=float, default=1.0,
                        help='初始角速度 (rad/s), 默认: 1.0')
    parser.add_argument('--topic', '-t', type=str, default='/cmd_vel',
                        help='速度命令话题, 默认: /cmd_vel')
    
    # 分离ROS参数和自定义参数
    parsed_args, ros_args = parser.parse_known_args()
    
    # 初始化ROS
    rclpy.init(args=ros_args)
    
    try:
        # 创建节点
        node = KeyboardTeleop(
            linear_speed=parsed_args.linear_speed,
            angular_speed=parsed_args.angular_speed
        )
        
        # 运行
        node.run()
        
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
