#!/usr/bin/env python3
"""
ROS Node for MPU6050 IMU Sensor
Publishes sensor_msgs/Imu messages

Author: Your Name
Date: 2026-01-16
"""

import rospy
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Vector3
import smbus
import math

class MPU6050:
    """Driver for MPU6050 IMU sensor via I2C"""
    
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    
    def __init__(self, bus_number=1, address=0x68):
        """
        Initialize MPU6050
        
        Args:
            bus_number: I2C bus number (1 for Raspberry Pi, 0 for Orange Pi)
            address: I2C address of MPU6050 (default 0x68)
        """
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        
        # Wake up the MPU6050 (it starts in sleep mode)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
        
        # Configure accelerometer range (±2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        
        # Configure gyroscope range (±250°/s)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        
        # Set sample rate divider
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x07)
        
        # Scaling factors
        self.accel_scale = 16384.0  # LSB/g for ±2g range
        self.gyro_scale = 131.0     # LSB/(°/s) for ±250°/s range
        
        rospy.loginfo("MPU6050 initialized on I2C bus %d at address 0x%02X", bus_number, address)
    
    def read_word_2c(self, reg):
        """Read 16-bit signed value from two registers"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def get_accel_data(self):
        """Get accelerometer data in m/s²"""
        accel_x = self.read_word_2c(self.ACCEL_XOUT_H) / self.accel_scale * 9.81
        accel_y = self.read_word_2c(self.ACCEL_YOUT_H) / self.accel_scale * 9.81
        accel_z = self.read_word_2c(self.ACCEL_ZOUT_H) / self.accel_scale * 9.81
        return accel_x, accel_y, accel_z
    
    def get_gyro_data(self):
        """Get gyroscope data in rad/s"""
        gyro_x = self.read_word_2c(self.GYRO_XOUT_H) / self.gyro_scale * (math.pi / 180.0)
        gyro_y = self.read_word_2c(self.GYRO_YOUT_H) / self.gyro_scale * (math.pi / 180.0)
        gyro_z = self.read_word_2c(self.GYRO_ZOUT_H) / self.gyro_scale * (math.pi / 180.0)
        return gyro_x, gyro_y, gyro_z
    
    def get_temp(self):
        """Get temperature in Celsius"""
        temp_raw = self.read_word_2c(self.TEMP_OUT_H)
        temp_c = (temp_raw / 340.0) + 36.53
        return temp_c


class MPU6050Node:
    """ROS Node for MPU6050 IMU"""
    
    def __init__(self):
        rospy.init_node('mpu6050_imu_node', anonymous=False)
        
        # Parameters
        self.bus_number = rospy.get_param('~i2c_bus', 1)
        self.address = rospy.get_param('~i2c_address', 0x68)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~rate', 50)  # Hz
        
        # Initialize MPU6050
        try:
            self.mpu = MPU6050(self.bus_number, self.address)
        except Exception as e:
            rospy.logerr("Failed to initialize MPU6050: %s", str(e))
            rospy.signal_shutdown("IMU initialization failed")
            return
        
        # Publishers
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.temp_pub = rospy.Publisher('imu/temperature', Temperature, queue_size=10)
        
        rospy.loginfo("MPU6050 node started. Publishing at %d Hz", self.publish_rate)
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            try:
                # Read sensor data
                accel_x, accel_y, accel_z = self.mpu.get_accel_data()
                gyro_x, gyro_y, gyro_z = self.mpu.get_gyro_data()
                temp = self.mpu.get_temp()
                
                # Create and publish IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = self.frame_id
                
                # Linear acceleration
                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                
                # Angular velocity
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z
                
                # Orientation not available (no magnetometer)
                # Set covariance to -1 to indicate unknown
                imu_msg.orientation_covariance[0] = -1
                
                # Publish
                self.imu_pub.publish(imu_msg)
                
                # Publish temperature
                temp_msg = Temperature()
                temp_msg.header.stamp = imu_msg.header.stamp
                temp_msg.header.frame_id = self.frame_id
                temp_msg.temperature = temp
                self.temp_pub.publish(temp_msg)
                
            except Exception as e:
                rospy.logerr("Error reading IMU data: %s", str(e))
            
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MPU6050Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
