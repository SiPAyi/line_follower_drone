#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import sys
import tty
import termios
import select
import threading
import math

from geometry_msgs.msg import Vector3


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.ctrl_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos)

        # State variables
        self.vehicle_status = VehicleStatus()
        self.current_z = -2.0              # Fixed height
        self.current_yaw = 0.0             # Yaw in radians
        
        self.max_speed = 1.0
        self.speed = 0.0
        self.step = 0.1                    # Max speed step per update
        
        self.vx = 0.0
        self.vy = 0.0

        self.counter = 0                   # Used for initial offboard setup

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.print_instructions()


    def status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Drone Armed")

    def disarm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Drone Disarmed")

    def engage_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard Mode Engaged")

    def land(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing...")

    def send_cmd(self, cmd_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = cmd_id
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def publish_ctrl_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = True     # control in z
        msg.velocity = True     # control in x, y

        self.ctrl_mode_pub.publish(msg)


    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.vx = self.speed * math.cos(self.current_yaw)
        self.vy = self.speed * math.sin(self.current_yaw)
        
        #print(f"{self.speed:.2f}, {self.current_yaw:.2f}, {self.vx:.2f}, {self.vy:.2f}\n")

        msg.velocity = [self.vx, self.vy, float('nan')]
        msg.position = [float('nan'), float('nan'), self.current_z]
        msg.yaw = self.current_yaw

        self.sp_pub.publish(msg)


    def timer_callback(self):
        self.publish_ctrl_mode()
        self.publish_setpoint()

        if self.counter == 10:
            self.engage_offboard()
            self.arm()
        if self.counter < 11:
            self.counter += 1

    def keyboard_loop(self):
        try:
            while True:
                key = self.get_key()

                if key == 'w':
                    self.speed = min(self.max_speed, self.speed+self.step) 
                elif key == 's':
                    self.speed = 0 
                elif key == 'x':
                    self.speed = max(0.0, self.speed-self.step)

                    
                elif key == 'j':
                    self.current_yaw -= 0.1
                elif key == 'k':
                    self.current_yaw = 0.0
                elif key == 'l':
                    self.current_yaw += 0.1

                elif key == 'r':
                    self.current_z -= self.step
                elif key == 'f':
                    self.current_z += self.step
                elif key == 't':
                    self.arm()
                elif key == 'y':
                    self.disarm()
                elif key == 'l':
                    self.land()
                elif key == '\x03':  # Ctrl+C
                    self.get_logger().info("Keyboard exit signal received")
                    break

        except Exception as e:
            print(f"Keyboard Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_instructions(self):
        print("PX4 Velocity-Yaw Keyboard Controller ")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

