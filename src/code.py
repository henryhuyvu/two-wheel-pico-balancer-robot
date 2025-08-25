# ============ Imports  ============
# ==================================
import time
import board
import digitalio
import pwmio
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C



# ============ Constants ============
# ===================================

# ====== Motor control Constants ======
PWM_FREQ = 5000 # Hz. Frequency for PWM control of motors
MAX_DUTY_CYCLE = 65535  # Maximum duty cycle for PWM on RP2040
MOTOR2_DIRECTION_FLIP = True # Change direction of rotation via software

# ====== Motor control GPIO pin definitions ======
MOTOR1_ENA_PIN = board.GP5      # PWM pin for Motor 1
MOTOR1_IN1_PIN = board.GP6      
MOTOR1_IN2_PIN = board.GP7
if MOTOR2_DIRECTION_FLIP == True:
    MOTOR2_IN3_PIN = board.GP9
    MOTOR2_IN4_PIN = board.GP8
    print("Motor 2 direction flipped.\n")
else:
    MOTOR2_IN3_PIN = board.GP8
    MOTOR2_IN4_PIN = board.GP9
    print("Motor 2 direction normal.\n")
MOTOR2_ENB_PIN = board.GP10     # PWM pin for Motor 2

# ====== IMU constants
# Configure GPIO pins for I2C communications/signals
I2C_SCL_PIN = board.GP17
I2C_SDA_PIN = board.GP16
SAMPLE_FREQ = 400000



# ============ Initialization ============
# ========================================

# ====== Motor control Initialization
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

# ====== IMU Initialization ======
try:
    i2c = busio.I2C(I2C_SCL_PIN, I2C_SDA_PIN, frequency=SAMPLE_FREQ)
    bno = BNO08X_I2C(i2c)
    print("BNO085 initialized successfully.\n")
except ValueError as e:
    print(f"Error initializing BNO085: {e}")
    while True:
        time.sleep(1)



# ============ Helper functions ============
# ==========================================

# ====== Motor Helper Functions ======
def set_motor_speed(motor_pwm, speed_percent):
    "Sets the speed of the motor using PWM."
    if not 0 <= speed_percent <= 100:
        raise ValueError("Speed must be between 0 and 100 percent.")
    # Convert speed percentage to duty cycle
    duty_cycle = int((speed_percent / 100) * MAX_DUTY_CYCLE)
    motor_pwm.duty_cycle = duty_cycle

def set_motor_direction(motor_in1, motor_in2, direction):
    "Sets the direction of the motor."
    if direction == "FWD":
        motor_in1.value = True
        motor_in2.value = False
    elif direction == "REV":
        motor_in1.value = False
        motor_in2.value = True
    elif direction == "COAST":
        motor_in1.value = False
        motor_in2.value = False
    elif direction == "BRAKE":
        motor_in1.value = True
        motor_in2.value = True
    else:
        print(f"Invalid direction: {direction}. Use 'FWD', 'REV', 'COAST', or 'BRAKE'.")

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

# ============ Main program loop ============
# ===========================================
'''
1. Connect power.
2. Remove from mount and hand balance on a surface.
3. Calibrate the accelerometer set points?
    Capture the neutral position.
    Store the neutral position as the target position.
4. Poll the accelerometer data and calculate PID feedback
    Determine the error between the target position and the current position.
    Calculate the PID output based on the error.
5. Direct the motor speeds based on PID output.

'''

# Enable BNO085 IMU report features
bno.enable_feature(BNO_REPORT_ACCELEROMETER)

try:
    print("Motors 1 and 2: Forward at 60% speed for 2 seconds.\n")
    control_motor(1, 50, "FWD")
    control_motor(2, 50, "FWD")
    time.sleep(2)

    print("Motors 1 and 2: Backward at 40% speed for 2 seconds.\n")
    control_motor(1, 75, "REV")
    control_motor(2, 75, "REV")
    time.sleep(2)

    print("Motors 1 and 2: Coasting then breaking\n")
    control_motor(1, 0, "COAST") # Ensure PWM is off and inputs are low
    control_motor(2, 0, "COAST") # Ensure PWM is off and inputs are low
    time.sleep(2)
    control_motor(1, 0, "BRAKE") # Speed 0, but apply brake
    control_motor(2, 0, "BRAKE") # Speed 0, but apply brake
    time.sleep(1)

    print("Turning left: Motor 1 FWD, Motor 2 REV.\n")
    control_motor(1, 80, "FWD")
    control_motor(2, 80, "REV")
    time.sleep(2)
    print("Turning right: Motor 1 REV, Motor 2 FWD.\n")
    control_motor(1, 80, "REV")
    control_motor(2, 80, "FWD")
    time.sleep(2)

    print("Stopping all motors.\n")
    control_motor(1, 0, "COAST")
    control_motor(2, 0, "COAST")
    time.sleep(1)
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Always ensure motors are stopped and pins are deinitialized on exit
    print("Cleaning up pins and stopping motors.")
    control_motor(1, 0, "COAST")
    control_motor(2, 0, "COAST")
    MOTOR1_ENA.deinit()
    MOTOR1_IN1.deinit()
    MOTOR1_IN2.deinit()
    MOTOR2_IN3.deinit()
    MOTOR2_IN4.deinit()
    MOTOR2_ENB.deinit()
    print("Demonstration finished.")

# Constant loop to output IMU reports
count = 0
while count <= 40:
        time.sleep(0.1)
        accel_x, accel_y, accel_z = bno.acceleration
        print("Acceleration:\nX: %0.6f Y: %0.6f Z: %0.6f m/s^2 \n" % (accel_x, accel_y, accel_z))
        count += 1
