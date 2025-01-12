"""
本代码用于UR5出现不可控姿态时的位姿重置
"""

import struct
import socket
import numpy as np
import time
import threading

# 机械臂IP地址和端口，不变量
HOST = "192.168.3.6"
PORT = 30003

# 定义机械臂的常量
tool_acc = 1  # Safe: 0.5
tool_vel = 0.5  # Safe: 0.2
PI = 3.141592653589
fmt1 = '<I'
fmt2 = '<6d'
BUFFER_SIZE = 1108

class Robot:
    def __init__(self, host=None, port=None):
        # 创建socket对象，然后TCP连接
        self.recv_buf = []
        if host is None and port is None:
            host = HOST
            port = PORT
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((HOST, PORT))
        self.control_mode = None  # 'pose' or 'angle'

    def robot_pose_control(self, target_tcp):
        self.control_mode = 'pose'
        tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % \
                      (target_tcp[0] / 1000, target_tcp[1] / 1000, target_tcp[2] / 1000,
                       target_tcp[3], target_tcp[4], target_tcp[5],
                       tool_acc, tool_vel)
        print(tcp_command)
        self.tcp_socket.send(str.encode(tcp_command))

    def robot_angle_control(self, target_tcp):
        self.control_mode = 'angle'
        self.target_rad = [angle * PI / 180.0 for angle in target_tcp]  # 存储弧度值用于比较
        tcp_command = "movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % \
                      (self.target_rad[0], self.target_rad[1], self.target_rad[2],
                       self.target_rad[3], self.target_rad[4], self.target_rad[5],
                       tool_acc, tool_vel)
        print(tcp_command)
        self.tcp_socket.send(str.encode(tcp_command))

    def get_current_tcp(self):
        robot_angle, robot_pos = self.robot_msg_recv()
        if self.control_mode == 'pose':
            return [float(value) for value in robot_pos]
        else:  # angle mode
            return [float(angle) * PI / 180.0 for angle in robot_angle]  # 转换为弧度

    def has_reached_target(self, current_values, target_values):
        """
        检查当前位置是否达到目标位置
        根据控制模式使用不同的容差值
        """
        if self.control_mode == 'pose':
            # 位置精度(mm)和姿态精度(rad)
            position_tolerance = 1.0  # 1mm
            orientation_tolerance = 0.01  # ~0.57度
            
            # 分别检查位置和姿态
            position_reached = all(
                abs(current_values[i] - target_values[i]) < position_tolerance 
                for i in range(3)
            )
            orientation_reached = all(
                abs(current_values[i] - target_values[i]) < orientation_tolerance 
                for i in range(3, 6)
            )
            return position_reached and orientation_reached
        else:  # angle mode
            angle_tolerance = 0.01  # ~0.57度
            return all(
                abs(current - target) < angle_tolerance 
                for current, target in zip(current_values, target_values)
            )

    def wait_until_reached(self, target_values, timeout=30, check_interval=0.5):
        """
        等待直到机械臂到达目标位置或超时
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_values = self.get_current_tcp()
            if self.has_reached_target(current_values, target_values):
                return True
            time.sleep(check_interval)
        return False

    def robot_msg_recv(self):
        self.recv_buf = []
        self.recv_buf = self.tcp_socket.recv(BUFFER_SIZE)

        if len(self.recv_buf) == 1108:
            pack_len = struct.unpack(fmt1, self.byte_swap(self.recv_buf[:4]))[0]

            # 解析机器人角度数据
            pos1 = 12
            pos2 = pos1 + 48
            data1 = self.byte_swap(self.recv_buf[pos1:pos2])
            data2 = np.frombuffer(data1, dtype=fmt2)
            new_data1 = np.around(np.rad2deg(data2[::-1]), 2)
            robot_angle = [str(value) for value in new_data1[0][::-1]]

            # 解析机器人位置数据
            pos3 = 444
            pos4 = pos3 + 48
            data3 = self.byte_swap(self.recv_buf[pos3:pos4])
            data4 = np.frombuffer(data3, dtype=fmt2)
            new_data2 = np.around(data4[::-1] * 1000, 2)
            robot_pos = [f"{value / 1000:.2f}" if i >= len(new_data2[0]) - 3
                         else str(value) for i, value in enumerate(new_data2[0][::-1])]

            return robot_angle, robot_pos

    def byte_swap(self, data):
        return data[::-1]

def keyboard_input():
    # 定义复位角度
    reset_angles = [0, -90, -90, -90, 90, 0]
    
    while True:
        user_input = input("请输入数字(1: 位置1, 2: 位置2, 3：位置3，0: 复位): ")
        
        if user_input == '1':
            target_pose1 = [0, -120, 0, 180, 90, 90]
            print("正在移动到位置1...")
            ur5.robot_angle_control(target_pose1)
            
            if ur5.wait_until_reached(ur5.target_rad):
                print("已到达位置1!")
                reset = input("是否需要复位? (0: 是, 其他: 否): ")
                if reset == '0':
                    print("正在移动到复位位置...")
                    ur5.robot_angle_control(reset_angles)
                    if ur5.wait_until_reached([x * PI / 180.0 for x in reset_angles]):
                        print("已完成复位!")
                    else:
                        print("复位超时，请检查机械臂状态")
            else:
                print("移动超时，请检查机械臂状态")
        
        elif user_input == '2':
            target_pose2 = [0, -123, -79, -70, 93, 90]
            print("正在移动到位置2...")
            ur5.robot_angle_control(target_pose2)
            
            if ur5.wait_until_reached(ur5.target_rad):
                print("已到达位置2!")
                reset = input("是否需要复位? (0: 是, 其他: 否): ")
                if reset == '0':
                    print("正在移动到复位位置...")
                    ur5.robot_angle_control(reset_angles)
                    if ur5.wait_until_reached([x * PI / 180.0 for x in reset_angles]):
                        print("已完成复位!")
                    else:
                        print("复位超时，请检查机械臂状态")
            else:
                print("移动超时，请检查机械臂状态")
        
        elif user_input == '3':
            target_pose3 = [0, -120, -90, 60, 90, -68]
            print("正在移动到位置3（标定位置）...")
            ur5.robot_angle_control(target_pose3)
            
            if ur5.wait_until_reached(ur5.target_rad):
                print("已到达位置3!")
                reset = input("是否需要复位? (0: 是, 其他: 否): ")
                if reset == '0':
                    print("正在移动到复位位置...")
                    ur5.robot_angle_control(reset_angles)
                    if ur5.wait_until_reached([x * PI / 180.0 for x in reset_angles]):
                        print("已完成复位!")
                    else:
                        print("复位超时，请检查机械臂状态")
            else:
                print("移动超时，请检查机械臂状态")

        elif user_input == '0':
            print("正在移动到复位位置...")
            ur5.robot_angle_control(reset_angles)
            if ur5.wait_until_reached([x * PI / 180.0 for x in reset_angles]):
                print("已完成复位!")
            else:
                print("复位超时，请检查机械臂状态")
        
        else:
            print("无效输入，请输入数字：")
            print("0: 直接复位")
            print("1: 移动到位置1")
            print("2: 移动到位置2")


if __name__ == "__main__":
    # 连接机械臂
    ur5 = Robot()
    # 循环检测键盘输入,守护子线程
    keyboard_thread = threading.Thread(target=keyboard_input)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    # 循环获取机械臂姿态
    while True:
        ur5.robot_msg_recv()




