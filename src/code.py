# Self-Balancing Robot Control Code
# Using Raspberry Pi Pico (RP2040), BNO085 IMU, L298N motor driver, and JGA25-371 Motors with encoders
# Adapted from various sources and examples
# Author: HHV
# Date: 2024-06-10
# License: MIT License

'''
TODO: 
Reading data from your IMU.
Controlling your motor driver.
Reading data from your encoders.
Implementing the balancing algorithm (PID control, etc.).
'''
     

# ============ Imports  ============
# ==================================
import time                                 # For delays and timing loops
import board                                # For accessing board pins
import digitalio                            # For setting pin modes (input/output)
import pwmio                                # For generating PWM signals for motor speed control
import busio                                # For I2C communication with IMU
# Adafruit library for BNO085 with IMU reports; Library found online
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)                   
from adafruit_bno08x.i2c import BNO08X_I2C  # I2C interface for BNO085



# ============ Constants ============
# ===================================

# ====== IMU constants ======
SAMPLE_FREQ = 400000                        # I2C frequency for IMU communication
# I2C pins for Raspberry Pi Pico
I2C_SCL_PIN = board.GP15
I2C_SDA_PIN = board.GP14

# ====== Motor control Constants ======
PWM_FREQ = 5000                             # Hz. Frequency for PWM control of motors
MAX_DUTY_CYCLE = 65535                      # Maximum duty cycle for PWM on RP2040; 16-bit resolution
MOTOR2_DIRECTION_FLIP = False               # Change direction of rotation via software
# ====== Motor control GPIO pin definitions ======
MOTOR1_ENA_PIN = board.GP0                  # PWM pin for Motor 1
MOTOR1_IN1_PIN = board.GP1                  # Direction pin 1, for Motor 1
MOTOR1_IN2_PIN = board.GP2                  # Direction pin 2, for Motor 1
if MOTOR2_DIRECTION_FLIP == True:           # Swap direction pins for motor 2 if needed
    MOTOR2_IN3_PIN = board.GP4              # Direction pin 3, for Motor 2
    MOTOR2_IN4_PIN = board.GP3              # Direction pin 4, for Motor 2
else:                                       # Alternately, swap the direction pins on flip condition
    MOTOR2_IN3_PIN = board.GP3
    MOTOR2_IN4_PIN = board.GP4
MOTOR2_ENB_PIN = board.GP5                  # PWM pin for Motor 2

# ====== Encoder Constants ======
# Raspberry Pi Pico GPIO pins for Hall Sensor encoders A and B for Motor 1 and Motor 2
MOTOR1_ENCODER_A_PIN = board.GP9
MOTOR1_ENCODER_B_PIN = board.GP8
MOTOR2_ENCODER_A_PIN = board.GP12
MOTOR2_ENCODER_B_PIN = board.GP11
# Encoder resolution (Counts Per Revolution). JGA25-371 docs suggest 12 CPR resulting in 48 pulses per revolution (12 * 4)
ENCODER_CPR = 48

# ====== PID Control Constants ======
Kp = 10.0                                   # Proportional gain
Ki = 0.0                                    # Integral gain
Kd = 0.01                                   # Derivative gain
last_error = 0.0
integral = 0.0
derivative = 0.0 
previous_time = time.monotonic()
# Balancing Target Constants
TARGET_ANGLE = 0.0                          # Desired angle (e.g., 0 degrees for upright balance)
TARGET_ANGLE_TOLERANCE = 5.0                # Tolerance in degrees
# You might also want to define a tolerance for how close it needs to be.



# ============ Global Variables ============
# ==========================================

# IMU Data
acceleration = (0.0, 0.0, 0.0)
# For orientation-based balancing, you'd use quaternion or Euler angles:
# orientation = (0.0, 0.0, 0.0) # Yaw, Pitch, Roll (Euler)

# Encoder Variables
motor1_encoder_count = 0
motor1_encoder_direction = 0  # 1 for forward, -1 for backward (relative to motor controller's definition)
motor2_encoder_count = 0
motor2_encoder_direction = 0

# Robot State
is_balancing = False



# ============ Functions ============
# ===================================

# ====== IMU Helper Functions ======
def initialize_imu():
    """Initializes the BNO085 IMU and enables desired reports."""
    try:
        i2c = busio.I2C(I2C_SCL_PIN, I2C_SDA_PIN, frequency=SAMPLE_FREQ)
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        print("BNO085 initialized successfully.")
        return bno
    except ValueError as e:
        print(f"Error initializing BNO085: {e}")
        return None

def read_imu_data(bno):
    """Reads and returns acceleration data from the IMU."""
    if bno is None:
        return (0.0, 0.0, 0.0) # Return default if IMU failed to init
    accel_x, accel_y, accel_z = bno.acceleration
    # If using rotation vector for orientation:
    # g_quat_i, g_quat_j, g_quat_k, g_quat_real = bno.game_quaternion
    # You'd then convert quaternion to Euler angles (pitch) for balancing.

    # For this example, we'll use acceleration along the Y-axis as a proxy for tilt.
    # This is a simplification and might not be stable.
    # A better approach is to use the angle derived from the gyroscope or rotation vector.
    return accel_x, accel_y, accel_z

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

# ====== PID Controller Functions ======
def calculate_pid_output(current_value, target_value):
    """Calculates the PID output based on current and target values."""
    global last_error, integral, previous_time

    current_time = time.monotonic()
    dt = current_time - previous_time
    previous_time = current_time

    error = target_value - current_value
    integral += error * dt
    # Limit integral windup if necessary (e.g., if integral exceeds a certain value)
    # if abs(integral) > INTEGRAL_LIMIT:
    #     integral = INTEGRAL_LIMIT * (1 if integral > 0 else -1)

    derivative = (error - last_error) / dt
    last_error = error

    # Calculate PID output
    pid_output = (Ki * error) + (Ki * integral) + (Kd * derivative)

    return pid_output



# ============ System Initialization ============
# ===============================================

# ====== Motor control Initialization
# Motor 1 Direction Pins
MOTOR1_IN1 = digitalio.DigitalInOut(MOTOR1_IN1_PIN)     # Motor 1 Direction Pin 1
MOTOR1_IN1.direction = digitalio.Direction.OUTPUT       # Set pin as output
MOTOR1_IN2 = digitalio.DigitalInOut(MOTOR1_IN2_PIN)     # Motor 1 Direction Pin 2
MOTOR1_IN2.direction = digitalio.Direction.OUTPUT       # Set pin as output
# Motor 1 Enable Pin
MOTOR1_ENA = pwmio.PWMOut(MOTOR1_ENA_PIN, frequency=PWM_FREQ, duty_cycle=0) # PWM for Motor 1

# Motor 2 Direction Pins
MOTOR2_IN3 = digitalio.DigitalInOut(MOTOR2_IN3_PIN)     # Motor 2 Direction Pin 1
MOTOR2_IN3.direction = digitalio.Direction.OUTPUT       # Set pin as output
MOTOR2_IN4 = digitalio.DigitalInOut(MOTOR2_IN4_PIN)     # Motor 2 Direction Pin 2
MOTOR2_IN4.direction = digitalio.Direction.OUTPUT       # Set pin as output
# Motor 2 Enable Pin
MOTOR2_ENB = pwmio.PWMOut(MOTOR2_ENB_PIN, frequency=PWM_FREQ, duty_cycle=0) # PWM for Motor 2
print("\nMotor direction and PWM pins initialized.\n")

# ====== IMU Initialization ======
bno = initialize_imu()
if bno is None:
    print("IMU initialization failed. Trying again.")
    N_Retries = 10
    for _ in range(N_Retries):  # Retry initialization a few times
        bno = initialize_imu()
        if bno is not None:
            break
    else:
        print(f"IMU initialization failed after {N_Retries} retries. Exiting script now\nGoodbye.")
        exit()
print("\nIMU initialized.\n")


# ============ Main program Loop - START ============
# ===================================================

# Motor Testing
MOTOR_TESTING = False
if MOTOR_TESTING == True:
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
while count <= 100:
    time.sleep(0.5)
    accel_x, accel_y, accel_z = bno.acceleration
    print("Acceleration:\nX: %0.6f Y: %0.6f Z: %0.6f m/s^2 \n" % (accel_x, accel_y, accel_z))
    (
        game_quat_i,
        game_quat_j,
        game_quat_k,
        game_quat_real,
    ) = bno.game_quaternion
    print(
        "Game Rotation Vector:\nI: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f"
        % (game_quat_i, game_quat_j, game_quat_k, game_quat_real)
    )
    print("")
    count += 1

try:
    print("Starting balancing sequence.")
    is_balancing = False
    while is_balancing:
        # Read sensor data
        accel_x, accel_y, accel_z = read_imu_data(bno)

        # --- Balancing Logic ---
        # This is the core of your balancing algorithm.
        # You need to determine the "angle" or "tilt" of the robot.
        #
        # Simplistic approach using acceleration (proxy for tilt):
        # If the robot is tilted forward, accel_y might be positive (depending on sensor orientation).
        # You want to drive motors forward to counteract this tilt.
        # If tilted backward, accel_y might be negative, drive motors backward.
        #
        # A more robust approach uses gyroscope data or rotation vectors to get actual angle.
        # For example, if using GAME_ROTATION_VECTOR, you'd convert the quaternion to Euler angles.
        # Let's assume for now we're using accel_y as a simple tilt indicator:
        current_tilt_proxy = accel_y # This is a simplification!

        # Calculate PID output
        # We want to move motors to counteract the tilt.
        # If current_tilt_proxy is positive (robot leaning forward), we need to move forward.
        # The PID output will be positive, which we'll map to motor speed.
        motor_command = calculate_pid_output(current_tilt_proxy, TARGET_ANGLE)

        # Map PID output to motor speed and direction
        # The PID output can be positive or negative.
        # Positive output means move forward, negative means move backward.
        # The magnitude of the output determines the speed.
        # You'll need to scale and clamp the motor command.

        # Example scaling and clamping:
        # Let's assume PID output ranges roughly from -100 to 100 for full speed.
        # You'll need to tune this based on your robot's behavior.
        speed_percent = abs(motor_command)
        speed_percent = min(speed_percent, 100) # Clamp speed to 0-100%

        if motor_command > 0: # Robot leaning forward, move forward
            direction = "FWD"
        elif motor_command < 0: # Robot leaning backward, move backward
            direction = "REV"
        else: # Robot is balanced or stationary
            direction = "COAST"
            speed_percent = 0 # Ensure no movement if command is zero

        # Apply motor commands to both motors
        # In a balanced robot, both motors should generally move in the same direction
        # to propel it forward or backward to maintain balance.
        control_motor(1, speed_percent, direction)
        control_motor(2, speed_percent, direction)

        # Optional: Print debug information
        # print(f"Tilt Proxy: {current_tilt_proxy:.2f}, PID Output: {motor_command:.2f}, Speed: {speed_percent:.1f}% {direction}")

        # Control the loop frequency. You might need to adjust this.
        # A higher frequency can lead to more responsive control but might consume more CPU.
        time.sleep(0.01) # Adjust this value for your control loop frequency
except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping motors and cleaning up.")
    is_balancing = False
except Exception as e:
    print(f"An error occurred during balancing: {e}")
    is_balancing = False
    

# ============ Main program Loop - END ============
# ===================================================