#!/usr/bin/env python3
import rospy
import smbus2
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf


imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

# Dirección del sensor I2C
I2C_ADDR = 0x68

# Inicializar bus I2C
bus = smbus2.SMBus(1)

# Inicializar el sensor (MPU6050 en este caso)

bus.write_byte_data(I2C_ADDR, 0x6B, 0)  # Despertar el sensor


def read_word(reg):
    high = bus.read_byte_data(I2C_ADDR, reg)
    low = bus.read_byte_data(I2C_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:  # Convert to signed
        value -= 0x10000
    return value


def read_sensor():
    """Reads sensor data and converts to physical units."""
    raw_accel_x = read_word(0x3B)
    raw_accel_y = read_word(0x3D)
    raw_accel_z = read_word(0x3F)
    raw_gyro_x = read_word(0x43)
    raw_gyro_y = read_word(0x45)
    raw_gyro_z = read_word(0x47)

    # Convert raw data to real-world units
    accel_x = (raw_accel_x / 16384.0) * 9.81  # m/s²
    accel_y = (raw_accel_y / 16384.0) * 9.81
    accel_z = (raw_accel_z / 16384.0) * 9.81
    gyro_x = raw_gyro_x / 131.0  # °/s
    gyro_y = raw_gyro_y / 131.0
    gyro_z = raw_gyro_z / 131.0

    return [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]


def sensor_publisher():
    rospy.init_node('sensor_node', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        sensor_values = read_sensor()
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = sensor_values

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # Convert gyroscope data to rad/s
        imu_msg.angular_velocity.x = gyro_x * (3.14159 / 180.0)
        imu_msg.angular_velocity.y = gyro_y * (3.14159 / 180.0)
        imu_msg.angular_velocity.z = gyro_z * (3.14159 / 180.0)

        # Convert acceleration data
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        # Convert to Quaternion (assuming no magnetometer, using a simple placeholder)
        quaternion = tf.transformations.quaternion_from_euler(gyro_x, gyro_y, gyro_z)
        imu_msg.orientation = Quaternion(*quaternion)

        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_publisher()
    except rospy.ROSInterruptException:
        pass


