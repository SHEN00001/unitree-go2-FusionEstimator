#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import time
import math
import matplotlib.pyplot as plt
from threading import Lock, Thread

class SquareWalker:
    def __init__(self):
        # 初始化ROS2节点
        rclpy.init()
        self.node = Node('square_walker')
        
        # 参数配置
        self.side_length = 1.0    # 正方形边长（米）
        self.linear_speed = 0.3   # 线速度（m/s）
        self.angular_speed = 0.5  # 角速度（rad/s）
        self.control_interval = 0.1  # 控制周期（秒）
        
        # 计算运动时间
        self.side_duration = self.side_length / self.linear_speed
        self.turn_duration = math.pi/2 / self.angular_speed
        
        # 初始化控制状态
        self.phase = 0  # 0: 前进，1: 右转
        self.phase_start_time = time.time()
        self.is_running = True
        
        # 初始化发布订阅
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/SMXFE/Odom', self.odom_callback, 10)
        
        # 数据记录
        self.odom_data = []
        self.data_lock = Lock()
        
        # 启动控制线程
        self.control_thread = Thread(target=self.control_loop)
        self.control_thread.start()
        
        # 启动绘图线程
        self.plot_thread = Thread(target=self.real_time_plot)
        self.plot_thread.start()

    def odom_callback(self, msg):
        with self.data_lock:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.odom_data.append((timestamp, x, y))

    def control_loop(self):
        while self.is_running and rclpy.ok():
            twist = Twist()
            elapsed = time.time() - self.phase_start_time
            
            # 状态机控制
            if self.phase % 2 == 0:  # 直线阶段
                if elapsed < self.side_duration:
                    twist.linear.x = self.linear_speed
                else:
                    self.phase += 1
                    self.phase_start_time = time.time()
                    print("进入转弯阶段")
            else:  # 转弯阶段
                if elapsed < self.turn_duration:
                    twist.angular.z = self.angular_speed
                else:
                    self.phase += 1
                    self.phase_start_time = time.time()
                    print("进入直线阶段" if self.phase < 8 else "完成正方形路径")
            
            self.cmd_pub.publish(twist)
            time.sleep(self.control_interval)
            
            # 完成四次边角后停止
            if self.phase >= 8:
                self.stop()

    def real_time_plot(self):
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'b-')
        ax.set_title('实时轨迹')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True)
        ax.axis('equal')
        
        while self.is_running:
            with self.data_lock:
                if len(self.odom_data) > 1:
                    x = [d[1] for d in self.odom_data]
                    y = [d[2] for d in self.odom_data]
                    line.set_data(x, y)
                    ax.relim()
                    ax.autoscale_view()
            plt.pause(0.05)
        
        plt.ioff()

    def stop(self):
        self.is_running = False
        # 发送停止指令
        twist = Twist()
        self.cmd_pub.publish(twist)
        
        # 保存数据
        self.save_data()
        rclpy.shutdown()

    def save_data(self):
        filename = f"square_path_{time.strftime('%Y%m%d-%H%M%S')}.csv"
        with open(filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'X', 'Y'])
            writer.writerows(self.odom_data)
        print(f"数据已保存至 {filename}")
        
        # 绘制最终轨迹
        plt.figure()
        x = [d[1] for d in self.odom_data]
        y = [d[2] for d in self.odom_data]
        plt.plot(x, y, 'b-', label='实际轨迹')
        plt.plot([0,1,1,0,0], [0,0,1,1,0], 'r--', label='理想轨迹')
        plt.legend()
        plt.title('轨迹对比')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(f"trajectory_{time.strftime('%Y%m%d-%H%M%S')}.png")
        plt.show()

if __name__ == '__main__':
    try:
        walker = SquareWalker()
        # 保持主线程运行
        while walker.is_running:
            rclpy.spin_once(walker.node, timeout_sec=0.1)
        walker.control_thread.join()
        walker.plot_thread.join()
    except KeyboardInterrupt:
        walker.stop()