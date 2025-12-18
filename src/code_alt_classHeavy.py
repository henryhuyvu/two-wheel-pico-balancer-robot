# Self-Balancing Robot Control Code - Refactored
# Platform: Raspberry Pi Pico (CircuitPython)
# Hardware: BNO085 IMU, L298N Driver, JGA25-371 Motors

import time
import board
import digitalio
import pwmio
import busio
import math
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# ==========================================
# CLASS DEFINITIONS
# ==========================================

class PIDController:
    """
    Handles the math for maintaining balance.
    Stores its own error history (integral/derivative) internally.
    """
    def __init__(self, kp, ki, kd, target_angle=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target_angle
        
        # internal state variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.monotonic()

    def compute(self, current_angle):
        now = time.monotonic()
        dt = now - self.last_time
        
        # Prevent division by zero if called too fast
        if dt <= 0.0: 
            return 0.0
            
        self.last_time = now
        
        error = self.target - current_angle
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (sum of errors over time)
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term (rate of change of error)
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        self.last_error = error
        
        return p_term + i_term + d_term


class Motor:
    """
    Controls a single motor via L298N driver.
    """
    def __init__(self, pwm_pin, in1_pin, in2_pin, frequency=5000):
        # Setup Direction Pins
        self.in1 = digitalio.DigitalInOut(in1_pin)
        self.in1.direction = digitalio.Direction.OUTPUT
        
        self.in2 = digitalio.DigitalInOut(in2_pin)
        self.in2.direction = digitalio.Direction.OUTPUT
        
        # Setup Speed (PWM) Pin
        self.pwm = pwmio.PWMOut(pwm_pin, frequency=frequency, duty_cycle=0)
        self.max_duty = 65535

    def set_speed(self, speed_percent):
        """Sets speed from 0 to 100%."""
        if speed_percent < 0: speed_percent = 0
        if speed_percent > 100: speed_percent = 100
        
        duty = int((speed_percent / 100) * self.max_duty)
        self.pwm.duty_cycle = duty

    def drive(self, speed_percent, direction):
        """
        Direction: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
        """
        # Set Direction Pins
        if direction < 0:       # Forward
            self.in1.value = True
            self.in2.value = False
        elif direction > 0:     # Reverse
            self.in1.value = False
            self.in2.value = True
        else:                   # Brake/Stop
            self.in1.value = False 
            self.in2.value = False # Coast (set both True for Brake)

        self.set_speed(speed_percent)

    def stop(self):
        self.drive(0, 0)
        
    def deinit(self):
        """Release pins to be safe"""
        self.pwm.deinit()
        self.in1.deinit()
        self.in2.deinit()


class IMU:
    """
    Wrapper for the BNO085 Sensor.
    Handles initialization and math conversion (Quaternion -> Euler).
    """
    def __init__(self, i2c_scl, i2c_sda, frequency=400000):
        self.i2c = busio.I2C(i2c_scl, i2c_sda, frequency=frequency)
        self.sensor = None
        self.init_sensor()

    def init_sensor(self):
        try: 
            REPORT_INTERVAL = const(10000) # in microseconds; 100Hz
            self.sensor = BNO08X_I2C(self.i2c)
            self.sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, REPORT_INTERVAL)
            # self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER) # Enable if needed
            print("IMU Initialized successfully.")
        except Exception as e:
            print(f"IMU Init failed: {e}")

    def get_tilt_angle(self):
        """
        Returns the Roll angle (X-axis rotation) in degrees.
        Adjusted for robot frame where +Y is UP, +Z is FWD.
        """
        if not self.sensor:
            return 0.0
        try:
            # Get Quaternion (x, y, z, w) = (i, j, k, real)
            quat_i, quat_j, quat_k, quat_real = self.sensor.game_quaternion
            x, y, z, w = quat_i, quat_j, quat_k, quat_real

            # Calculate Roll (Rotation around X-axis)
            # This is the "Tilt" for your specific mounting
            roll_rad = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
            return math.degrees(roll_rad)
        except Exception:
            # Sometimes I2C fails momentarily, return 0 or last known
            return 0.0

# ==========================================
# MAIN SETUP
# ==========================================

# 1. Create IMU Object
imu = IMU(board.GP15, board.GP14)

# 2. Create Motor Objects
# Note: You can easily swap pins here without hunting through the code
motor_left = Motor(pwm_pin=board.GP0, in1_pin=board.GP1, in2_pin=board.GP2)
motor_right = Motor(pwm_pin=board.GP5, in1_pin=board.GP3, in2_pin=board.GP4)

# 3. Create PID Controller Object
# Tune these values! 
initial_target_angle = 91.5  # Upright position
pid = PIDController(kp=9, ki=0.165, kd=0.34, target_angle=initial_target_angle)
# pid = PIDController(kp=9, ki=0.045, kd=0.34, target_angle=initial_target_angle)
# pid = PIDController(kp=8, ki=1.095, kd=0.3, target_angle=initial_target_angle)

# ==========================================
# CONSTANTS FOR SAFETY AND CONTROL
# ==========================================

# Angle beyond which the robot is considered "fallen" and should stop motors.
# Since 90 is upright, 45 degrees of tilt is usually a good cutoff (90 +/- 45).
MAX_TILT_DEGREES = 45.0 
TILT_HIGH = initial_target_angle + MAX_TILT_DEGREES
TILT_LOW = initial_target_angle - MAX_TILT_DEGREES

print("Setup Complete. Main Loop starting in:")
print("3...")
time.sleep(1)
print("2...")
time.sleep(1)
print("1...")
time.sleep(1)
print("Starting!")

# ==========================================
# MAIN LOOP
# ==========================================

# Variables for non-blocking printing (Define these before the while True loop)
last_print_time = time.monotonic()
print_interval = 0.2  # Print only 5 times a second (every 200ms)

try:
    while True:
        # A. Read Sensor
        tilt_angle = imu.get_tilt_angle()

        # B. Check Safety Cutoff
        if tilt_angle > TILT_HIGH or tilt_angle < TILT_LOW:
            # Robot has fallen too far! Stop the motors immediately.
            motor_left.stop()
            motor_right.stop()
            now = time.monotonic()
            if now - last_print_time > print_interval:
                # This print statement is now safe because it runs infrequently.
                print(f"Safety Cutoff: {tilt_angle:.2f} deg outside of {TILT_LOW} to {TILT_HIGH:.1f}. Motors stopped.")
                last_print_time = now
            time.sleep(1.5) # Give a moment to read the message
            continue # Skip PID calculation for this loop and wait

        # C. Calculate PID Output
        control_signal = pid.compute(tilt_angle)
        
        # D. Convert PID signal to Motor Speed/Direction
        # 1. Determine direction
        if control_signal > 0:
            direction = 1   # Forward
        else:
            direction = -1  # Reverse
            
        # 2. Determine absolute speed (clamped to 0-100%)
        # CALCULATE MINIMUM POWER
        min_power = 15  # Start with 15-20%. This is the % power required just to get wheels turning.
        
        abs_signal = abs(control_signal)
        # If we need to move, add the min_power so wheels actually turn immediately
        if abs_signal > 0.5: # Small threshold to ignore noise
            speed = min_power + abs_signal
        else:
            speed = 0
        if speed > 100: speed = 100
        
        # E. Drive Motors
        motor_left.drive(speed, direction)
        motor_right.drive(speed, direction)

        # Use the timer to control when you print, not a sleep or print every loop.
        now = time.monotonic()
        if now - last_print_time > print_interval:
            # This print statement is now safe because it runs infrequently.
            print(f"Tilt: {tilt_angle:.2f} deg | Out: {control_signal:.2f} err")
            last_print_time = now

except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping motors and exiting.")
    
except Exception as e:
    print(f"\nCritical Error: {e}. Stopping motors.")
    
finally:
    # This ensures a clean shutdown regardless of how the loop exits.
    motor_left.stop()
    motor_right.stop()
    motor_left.deinit()
    motor_right.deinit()
    print("Cleanup complete. Motors disabled.")

