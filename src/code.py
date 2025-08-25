# ============ Import packages
import time
import board
import digitalio
import pwmio
import busio

# ============ Initialization Status
# Keep track of which modules are successfully or unsuccessfully initialized
# IMU
# Motor 1, Motor 2
# Other?

# ============ Initialization for BNO085 IMU data via I2C
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



# ============ Initialization for Motor control
# Assigning GPIO pins for motor control
MOTOR1_ENA_PIN = board.GP5      # PWM pin for Motor 1
MOTOR1_IN1_PIN = board.GP6      
MOTOR1_IN2_PIN = board.GP7

MOTOR2_IN3_PIN = board.GP8
MOTOR2_IN4_PIN = board.GP9
MOTOR2_ENB_PIN = board.GP10     # PWM pin for Motor 2

# Constants
PWM_FREQ = 5000 # Hz. Frequency for PWM control of motors
MAX_DUTY_CYCLE = 65535  # Maximum duty cycle for PWM on RP2040

# Initialize motor control pins
# Motor 1 Direction Pins
MOTOR1_IN1 = digitalio.DigitalInOut(MOTOR1_IN1_PIN)
MOTOR1_IN1.direction = digitalio.Direction.OUTPUT
MOTOR1_IN2 = digitalio.DigitalInOut(MOTOR1_IN2_PIN)
MOTOR1_IN2.direction = digitalio.Direction.OUTPUT

# Motor 1 Enable Pin
MOTOR1_ENA = pwmio.PWMOut(MOTOR1_ENA_PIN, frequency=PWM_FREQ, duty_cycle=0)

# Motor 2 Direction Pins
MOTOR2_IN3 = digitalio.DigitalInOut(MOTOR2_IN3_PIN)
MOTOR2_IN3.direction = digitalio.Direction.OUTPUT
MOTOR2_IN4 = digitalio.DigitalInOut(MOTOR2_IN4_PIN)
MOTOR2_IN4.direction = digitalio.Direction.OUTPUT

# Motor 2 Enable Pin
MOTOR2_ENB = pwmio.PWMOut(MOTOR2_ENB_PIN, frequency=PWM_FREQ, duty_cycle=0)

# Helper functions
def set_motor_speed(motor_pwm, speed_percent):
    "Sets the speed of the motor using PWM."
    if not 0 <= speed_percent <= 100:
        raise ValueError("Speed must be between 0 and 100 percent.")
        return
    # Convert speed percentage to duty cycle
    duty_cycle = int((speed_percent / 100) * MAX_DUTY_CYCLE)
    motor_pwm.duty_cycle = duty_cycle

def set_motor_direction(motor_in1, motor_in2, direction):
    "Sets the direction of the motor."
    if direction == "forward":
        motor_in1.value = True
        motor_in2.value = False
    elif direction == "reverse":
        motor_in1.value = False
        motor_in2.value = True
    elif direction == "stop":
        motor_in1.value = False
        motor_in2.value = False
    elif direction == "brake":
        motor_in1.value = True
        motor_in2.value = True
    else:
        print(f"Invalid direction: {direction}. Use 'forward', 'reverse', 'stop', or 'brake'.")

def control_motor(motor_num, speed_percent, direction):
    "Controls the specified motor with given direction and speed."
    if motor_num == 1:
        set_motor_direction(MOTOR1_IN1, MOTOR1_IN2, direction)
        set_motor_speed(MOTOR1_ENA, speed_percent)
    elif motor_num == 2:
        set_motor_direction(MOTOR2_IN3, MOTOR2_IN4, direction)
        set_motor_speed(MOTOR2_ENB, speed_percent)
    else:
        print(f"Invalid motor number: {motor_num}. Use 1 or 2.")

try:
    # Test Motor 1
    print("Motor 1: Forward at 50% speed for 2 seconds")
    control_motor(1, 50, "forward")
    time.sleep(2)

    print("Motor 1: Backward at 75% speed for 2 seconds")
    control_motor(1, 75, "reverse")
    time.sleep(2)

    print("Motor 1: Braking...")
    control_motor(1, 0, "brake") # Speed 0, but apply brake
    time.sleep(1)
    control_motor(1, 0, "stop") # Ensure PWM is off and inputs are low
    time.sleep(1)

    # Test Motor 2
    print("Motor 2: Forward at 60% speed for 2 seconds")
    control_motor(2, 60, "forward")
    time.sleep(2)

    print("Motor 2: Backward at 40% speed for 2 seconds")
    control_motor(2, 40, "reverse")
    time.sleep(2)

    print("Motor 2: Stopping (coasting)...")
    control_motor(2, 0, "stop") # Speed 0, coast to stop
    time.sleep(1)

    # Simultaneous control (e.g., turning a robot)
    print("Turning left: Motor 1 Forward, Motor 2 Reverse (both 80%)")
    control_motor(1, 80, "forward")
    control_motor(2, 80, "reverse")
    time.sleep(3)

    print("Stopping all motors...")
    control_motor(1, 0, "stop")
    control_motor(2, 0, "stop")
    time.sleep(1)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Always ensure motors are stopped and pins are deinitialized on exit
    print("Cleaning up pins and stopping motors.")
    control_motor(1, 0, "stop")
    control_motor(2, 0, "stop")
    
    MOTOR1_ENA.deinit()
    MOTOR1_IN1.deinit()
    MOTOR1_IN2.deinit()
    MOTOR2_IN3.deinit()
    MOTOR2_IN4.deinit()
    MOTOR2_ENB.deinit()
    print("Demonstration finished.")



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