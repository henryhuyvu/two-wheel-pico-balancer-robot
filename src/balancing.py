# Self-Balancing Robot Control Code
# CPU: Raspberry Pi Pico (CircuitPython)
# Hardware: BNO085 IMU, L298N Driver, 2x JGA25-371 Motors + Encoders

import time
import board
import digitalio
import pwmio
import busio
import math
from adafruit_bno08x import (
    # BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class IMU:
    """
    Wrapper for the BNO085 Sensor.
    Handles initialization and math conversion (Quaternion -> Euler).
    """
    SAMPLE_FREQ = 400000  # I2C frequency for sensor communication
    REPORT_INTERVAL = const(10000) # in microseconds; 100Hz update rate for the IMU reports

    def __init__(self, I2C_SCL, I2C_SDA, frequency=SAMPLE_FREQ):
        self.I2C = busio.I2C(I2C_SCL, I2C_SDA, frequency=frequency)
        self.sensor = None
        self.init_sensor()

    def init_sensor(self):
        try: 
            self.sensor = BNO08X_I2C(self.I2C)
            self.sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, IMU.REPORT_INTERVAL)
            print("Initialized successfully: Game Rotation Vector.")
        except Exception as e:
            print(f"IMU Init failed: {e}")

    def get_tilt_angle(self):
        """
        Returns the Roll angle (X-axis rotation) in degrees.
        Adjusted for robot frame where +Y is UP, +Z is FWD.
        """
        # If sensor isn't initialized, return 0.0 as a safe default.
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
            return 0.0

class Motor:
    """
    Controls a single motor via L298N driver.
    """
    PWM_FREQUENCY = 5000  # 5 kHz is a common choice for motor control
    motorDeadzonePercent = 10 # Minimum power to overcome motor deadzone.
    def __init__(self, PWM_PIN, IN1_PIN, IN2_PIN, frequency=PWM_FREQUENCY):
        # Setup Direction Input Pins
        self.IN1 = digitalio.DigitalInOut(IN1_PIN)
        self.IN2 = digitalio.DigitalInOut(IN2_PIN)
        self.IN1.direction = digitalio.Direction.OUTPUT
        self.IN2.direction = digitalio.Direction.OUTPUT
        
        # Setup Speed (PWM) Pin
        self.pwm = pwmio.PWMOut(PWM_PIN, frequency=frequency, duty_cycle=0)
        self.max_duty = 65535

    def set_speed(self, speed_percent):
        """Sets speed from 0 to 100%."""
        if speed_percent < 0: speed_percent = 0
        if (speed_percent < Motor.motorDeadzonePercent) and (speed_percent > 0): speed_percent = motorDeadzonePercent
        if speed_percent > 100: speed_percent = 100
        
        duty = int((speed_percent / 100) * self.max_duty)
        self.pwm.duty_cycle = duty

    def drive(self, speed_percent, direction):
        """
        Direction: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
        """
        # Set Direction Pins
        if direction < 0:       # Forward
            self.IN1.value = True
            self.IN2.value = False
        elif direction > 0:     # Reverse
            self.IN1.value = False
            self.IN2.value = True
        else:                   # Brake/Stop
            self.IN1.value = False 
            self.IN2.value = False # Coast (set both True for Brake)

        self.set_speed(speed_percent)

    def stop(self):
        self.drive(0, 0)
        
    def deinit(self):
        """Release pins to be safe"""
        self.pwm.deinit()
        self.IN1.deinit()
        self.IN2.deinit()

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


def main():
    initial_target_angle = 91.5  # Upright position

    # 1. Create IMU Object
    imu = IMU(board.GP15, board.GP14)

    # 2. Create Motor Objects
    motor_left = Motor(PWM_PIN=board.GP0, IN1_PIN=board.GP1, IN2_PIN=board.GP2)
    motor_right = Motor(PWM_PIN=board.GP5, IN1_PIN=board.GP3, IN2_PIN=board.GP4)

    # 3. Create PID Controller Object
    PID_Control = PIDController(kp=9, ki=0.165, kd=0.34, target_angle=initial_target_angle)

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
                # This print statement is now safe because it runs infrequently.
                print(f"Safety Cutoff: {tilt_angle:.2f} deg outside of {TILT_LOW} to {TILT_HIGH:.1f}. Motors stopped.")
                time.sleep(1.5) # Give a moment to read the message
                continue # Skip PID calculation for this loop and wait

            # C. Calculate PID Output
            control_signal = PID_Control.compute(tilt_angle)
            
            # D. Convert PID signal to Motor Speed/Direction
            # 1. Determine direction
            if control_signal > 0:
                direction = 1   # Forward
            elif control_signal < 0:
                direction = -1  # Reverse
            elif control_signal == 0:
                continue # No movement needed, skip to next loop iteration
                
            # 2. Determine absolute speed (clamped to 0-100%)
            # CALCULATE MINIMUM POWER
            min_power = Motor.motorDeadzonePercent  # Start with 15-20%. This is the % power required just to get wheels turning.
            
            abs_control_signal = abs(control_signal)
            # If we need to move, add the min_power so wheels actually turn immediately
            if abs_control_signal > 0.5: # Small threshold to ignore noise
                speed = min_power + abs_control_signal
            elif speed > 100: speed = 100
            else: speed = 0
            
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


if __name__ == "__main__":
    main()

