# Import packages
import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# Configure GPIO pins for I2C communications/signals
I2C_SCL_PIN = board.GP15
I2C_SDA_PIN = board.GP14
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
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

# Constant loop to output IMU reports

while True:
    last_time = time.monotonic()
    time.sleep(1) # in seconds; 10ms delay for ~100Hz output
    accel_x, accel_y, accel_z = bno.acceleration    
    g_quat_i, g_quat_j, g_quat_k, g_quat_real = bno.game_quaternion
    
    current_time = time.monotonic()
    elapsed_time = current_time - last_time
    print(f"Accel - X:{accel_x:.3f}, Y:{accel_y:.3f}, Z:{accel_z:.3f} m/s^2;; GRVQuat - I:{g_quat_i:.3f}, J:{g_quat_j:.3f}, K:{g_quat_k:.3f}, ℝ:{g_quat_real:.3f}, ΔT:{elapsed_time:.3f} s")
