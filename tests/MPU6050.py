from mpu6050 import mpu6050

sensor = mpu6050(0x68)  # Replace with 0x69 if needed
try:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    print("Accelerometer:", accel_data)
    print("Gyroscope:", gyro_data)
except OSError as e:
    print("I2C Communication Error:", e)