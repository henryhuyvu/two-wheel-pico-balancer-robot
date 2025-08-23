# Import packages
import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Configure GPIO pins for I2C communications/signals
I2C_SCL_PIN = board.GP17
I2C_SDA_PIN = board.GP16
SAMPLE_FREQ = 400000

# I2C setup
try:
    i2c = busio.I2C(I2C_SCL_PIN, I2C_SDA_PIN, frequency=SAMPLE_FREQ)
    bno = BNO08X_I2C(i2c)
    print("BNO085 initialized successfully.")
except ValueError as e:
    print(f"Error initializing BNO085: {e}")
    while True:
        time.sleep(1)

# Enable BNO085 IMU report features
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
# bno.enable_feature(BNO_REPORT_GYROSCOPE)
# bno.enable_feature(BNO_REPORT_MAGNETOMETER)
# bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

# Constant loop to output IMU reports
MODES = ["Datastream","Plotter"] 
MODE = MODES[0]

if MODE == "Datastream":
    while True:
        time.sleep(0.5)
        accel_x, accel_y, accel_z = bno.acceleration
        print("Acceleration:\nX: %0.6f Y: %0.6f Z: %0.6f m/s^2 \n" % (accel_x, accel_y, accel_z))

        # gyro_x, gyro_y, gyro_z = bno.gyro
        # print("Gyro:\nX: %0.6f Y: %0.6f Z: %0.6f rads/s \n" % (gyro_x, gyro_y, gyro_z))
        
        # mag_x, mag_y, mag_z = bno.magnetic
        # print("Magnetometer:\nX: %0.6f Y: %0.6f Z: %0.6f uT \n" % (mag_x, mag_y, mag_z))
        
        # g_quat_i, g_quat_j, g_quat_k, g_quat_real = bno.game_quaternion
        # print("Game Rotation Vector Quaternion:\nI: %0.6f J: %0.6f K: %0.6f Real: %0.6f \n" % (g_quat_i, g_quat_j, g_quat_k, g_quat_real))
elif MODE == "Plotter":
    while True:
        accel_x, accel_y, accel_z = bno.acceleration
        g_quat_i, g_quat_j, g_quat_k, g_quat_real = bno.game_quaternion
        print("%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f" % (accel_x, accel_y, accel_z, g_quat_i, g_quat_j, g_quat_k, g_quat_real))