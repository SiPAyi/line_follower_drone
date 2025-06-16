#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from geometry_msgs.msg import Vector3

import math

class OffboardControl(Node):
    def __init__(self):
        super().__init__('control')

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
        self.subscription = self.create_subscription(Vector3, '/velocity_vector', self.listener_callback, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
  
        # State variables
        self.vehicle_status = VehicleStatus()
        self.current_z = 0.0              # Fixed height
        self.desired_z = -2.0              # Fixed height
        self.desired_yaw = 0.0             # Yaw in radians
        
        self.max_speed = 1.0
        self.speed = 0.0
        self.step = 0.1                    # Max speed step per update
        
        self.vx = 0.0
        self.vy = 0.0

        self.counter = 0                   # Used for initial offboard setup

        self.timer = self.create_timer(0.1, self.timer_callback)
        

    def listener_callback(self, msg):
        self.speed = msg.x
        self.desired_yaw = msg.y
        #self.get_logger().info(f'Received velocity: vx={self.vx:.2f}, vy={self.vy:.2f}')

    def status_callback(self, msg):
        self.vehicle_status = msg
        
    def odom_callback(self, msg):
        self.current_z = msg.position[2]

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
        
        self.vx = self.speed * math.cos(self.desired_yaw)
        self.vy = self.speed * math.sin(self.desired_yaw)
        
        tolerance = 0.2  # meters, you can adjust this as needed
        if abs(self.current_z - self.desired_z) >  tolerance:
            self.vx = 0.0
            self.vy = 0.0
    
        msg.velocity = [self.vx, self.vy, float('nan')]
        msg.position = [float('nan'), float('nan'), self.desired_z]
        msg.yaw = self.desired_yaw

        self.sp_pub.publish(msg)
        self.speed = 0.0


    def timer_callback(self):
        
        if (self.vehicle_status.pre_flight_checks_pass):
            if(self.counter == 0):
                print("pre-flight checks pass -> True")  
            
            self.publish_ctrl_mode()
            self.publish_setpoint()

            if self.counter == 10:
                self.engage_offboard()
                self.arm()
            if self.counter < 11:
                self.counter += 1
                
        else:
            print("waiting for pre-arm checks", end="\r", flush=True)
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

