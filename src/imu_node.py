import rospy
import smbus
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

class MPU6050Node:
    def __init__(self):
        rospy.init_node('mpu6050_node')
        
        # MPU6050 address and registers
        self.address = 0x68
        self.power_mgmt_1 = 0x6b
        self.bus = smbus.SMBus(1)
        
        # Initialize MPU6050
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
        
        # Publishers
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz
        
    def read_word_2c(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        return value
        
    def read_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        # Read accelerometer data
        accel_x = self.read_word_2c(0x3b) / 16384.0
        accel_y = self.read_word_2c(0x3d) / 16384.0
        accel_z = self.read_word_2c(0x3f) / 16384.0
        
        # Read gyroscope data
        gyro_x = self.read_word_2c(0x43) / 131.0
        gyro_y = self.read_word_2c(0x45) / 131.0
        gyro_z = self.read_word_2c(0x47) / 131.0
        
        # Calculate orientation
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        
        # Convert to quaternion
        quaternion = quaternion_from_euler(roll, pitch, 0)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Set linear acceleration
        imu_msg.linear_acceleration.x = accel_x * 9.81
        imu_msg.linear_acceleration.y = accel_y * 9.81
        imu_msg.linear_acceleration.z = accel_z * 9.81
        
        # Set angular velocity
        imu_msg.angular_velocity.x = math.radians(gyro_x)
        imu_msg.angular_velocity.y = math.radians(gyro_y)
        imu_msg.angular_velocity.z = math.radians(gyro_z)
        
        return imu_msg
        
    def run(self):
        while not rospy.is_shutdown():
            imu_msg = self.read_imu_data()
            self.imu_pub.publish(imu_msg)
            self.rate.sleep()