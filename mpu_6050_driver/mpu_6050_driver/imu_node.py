#!/usr/bin/env python

import time
import smbus
import struct
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

temp_pub = None
imu_pub = None

class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.temp_publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        imu_timer_period = 0.02  # seconds
        temp_timer_period = 10  # seconds
        self.timer = self.create_timer(imu_timer_period, self.publish_imu(self))
        self.timer = self.create_timer(temp_timer_period, self.publish_temp(self))
    
    # read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
    def read_word(self, adr):
        high = bus.read_byte_data(ADDR, adr)
        low = bus.read_byte_data(ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def publish_temp(self, timer_event, Node):
        temp_msg = Temperature()
        temp_msg.header.frame_id = IMU_FRAME
        temp_msg.temperature = self.read_word_2c(TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = Node.Time.now()
        temp_pub.publish(temp_msg)


    def publish_imu(self, timer_event, Node):
        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME

        # Read the acceleration vals
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0

        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))

        # Read the gyro vals
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0

        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = 0.0

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = Node.get_clock().now()

        imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)

    imu_node = ImuNode()

    bus = smbus.SMBus(rclpy.get_parameter('~bus', 1))
    ADDR = rclpy.get_parameter('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rclpy.get_parameter('~imu_frame', 'imu_link')
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    rclpy.spin(imu_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
