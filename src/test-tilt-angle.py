# Self-Balancing Robot Control Code
# Using Raspberry Pi Pico (RP2040), BNO085 IMU, L298N motor driver, and JGA25-371 Motors with encoders
# Adapted from various sources and examples
# Author: HHV
# Date: 2024-06-10
# License: MIT License

'''
This script is used to focus on finding the tilt angle (roll about the x-axis) from the BNO085 IMU. It is designed to capture the Euler angle for roll using the game rotation vector report from the IMU. The script initializes the IMU, continuously reads the quaternion data, computes the roll angle in degrees, and stores this data in a separate data file for plotting and analysis. 

This is useful for testing and calibrating the tilt sensing capabilities of the IMU in a self-balancing robot application while avoiding real-time motor control complexities.
'''


# ============ Imports  ============
# ==================================
import time                                 # For delays and timing loops
import board                                # For accessing board pins
import busio                                # For I2C communication with IMU
import math                                 # For mathematical functions (e.g., atan2, asin, degrees)
import sys
# Adafruit library for BNO085 IMU reports
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
)                   
from adafruit_bno08x.i2c import BNO08X_I2C  # I2C interface for BNO085



# ============ Constants ============
# ===================================

# ====== IMU REPORTS ======

REPORTS = [
    BNO_REPORT_GAME_ROTATION_VECTOR,
]

# ====== IMU constants ======
SAMPLE_FREQ = 400000                        # I2C frequency for IMU communication
# I2C pins for Raspberry Pi Pico
I2C_SCL_PIN = board.GP15
I2C_SDA_PIN = board.GP14

# ============ Functions ============
# ===================================

# ====== IMU Helper Functions ======
def initialize_imu():
    """Initializes the BNO085 IMU and enables desired reports."""
    try:
        i2c = busio.I2C(I2C_SCL_PIN, I2C_SDA_PIN, frequency=SAMPLE_FREQ)
        bno = BNO08X_I2C(i2c)
        REPORT_COUNT = 0
        for report in REPORTS:
            bno.enable_feature(report)
            REPORT_COUNT += 1
        sys.stdout.write(f"BNO085 initialized, {REPORT_COUNT} reports successfully.\n")
        # print(f"BNO085 initialized {REPORT_COUNT} reports successfully.")
        return bno
    except ValueError as e:
        sys.stderr.write(f"Error initializing BNO085: {e}\n")
        return None

# ====== IMU Initialization ======
bno = initialize_imu()
if bno is None:
    sys.stderr.write("IMU initialization failed. Trying again.\n")
    N_Retries = 10
    for _ in range(N_Retries):  # Retry initialization a few times
        bno = initialize_imu()
        if bno is not None:
            break
    else:
        sys.stderr.write(f"IMU initialization failed after {N_Retries} retries. Exiting script now\nGoodbye.\n")
        exit()
sys.stdout.write("IMU initialized.\n")

# Print a header line for the data output (useful for later analysis)
print("timestamp_s,roll_deg")

# ============ Main program Loop - START ============
# ===================================================

# Constant loop to output IMU reports
imu_delay = 0.025

start_time = time.monotonic()
while True:
    try:
        # Check if the IMU is ready to read at the desired frequency
        time.sleep(imu_delay)
        # Time elapsed since loop start
        current_time = time.monotonic() - start_time 
        (
            game_quat_i,
            game_quat_j,
            game_quat_k,
            game_quat_real,
        ) = bno.game_quaternion

        # Rename for clarity with standard formula (x, y, z, w)
        x = game_quat_i
        y = game_quat_j
        z = game_quat_k
        w = game_quat_real

        # Calculate Roll (rotation around X-axis)
        roll_rad = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))

        # Convert to Degrees for easier understanding
        roll_deg = math.degrees(roll_rad)

        # --- DATA STREAMING LINE ---
        # Print the data in a CSV-like format for easy parsing on the laptop
        # Use an f-string for precise formatting
        print(f"{current_time:0.4f},{roll_deg:0.4f}") 
        
        # Manually flush the output buffer to ensure the data is sent immediately
        # This is very important for high-frequency data streaming!
        # sys.stdout.flush()
    except KeyboardInterrupt:
        sys.stderr.write("\nCtrl+C detected. Cleaning up and exiting.\n")
        break
    except Exception as e:
        sys.stderr.write(f"An error occurred: {e}\n")
        break


# ============ Main program Loop - END ============
# ===================================================