import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# --- Configuration ---
# Find your Pico's serial port. On macOS, it's usually /dev/cu.usbmodemXXXXX
# You can find it by running `ls /dev/cu.usbmodem*` in your terminal when Pico is connected.
# >>> IMPORTANT: REPLACE THIS PLACEHOLDER WITH YOUR ACTUAL PICO SERIAL PORT <<<
SERIAL_PORT = '/dev/tty.usbmodem101'
BAUD_RATE = 115200

# Data points to display on the plot
MAX_PLOT_POINTS = 200

# Lists to store data for plotting
time_data = []
accel_x_data = []
accel_y_data = []
accel_z_data = []
pitch_data = []
roll_data = []
yaw_data = []
abs_data = []

# --- Matplotlib Setup ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle('BNO085 IMU Data Live Plot')

# Subplot 1: Linear Acceleration
line_accel_x, = ax1.plot([], [], label='Accel X', color='cyan')
line_accel_y, = ax1.plot([], [], label='Accel Y', color='magenta')
line_accel_z, = ax1.plot([], [], label='Accel Z', color='yellow')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Acceleration (m/s^2)')
ax1.set_title('Linear Acceleration')
ax1.legend()
ax1.grid(True)
ax1.set_ylim(-12, 12) # Adjust as needed

# Subplot 2: Pitch, Roll, Yaw
line_pitch, = ax2.plot([], [], label='Pitch', color='blue')
line_roll, = ax2.plot([], [], label='Roll', color='green')
line_yaw, = ax2.plot([], [], label='Yaw', color='red')
line_abs, = ax2.plot([], [], label='Abs', color='grey')
ax2.set_ylabel('Angle (degrees)')
ax2.set_title('Orientation')
ax2.legend()
ax2.grid(True)
ax2.set_ylim(-1, 1) # Adjust as needed for your expected range

start_time = time.time()


# --- Animation function ---
def animate(i):
    # print(f"Animating frame {i}...") # Debug: Confirm animate function is running
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            # print(f"Received raw line: '{line}'") # Debug: See what's being read
            if line:
                try:
                    # Parse CSV data
                    # Ensure the number of values matches what Pico is sending
                    accel_x, accel_y, accel_z, pitch, roll, yaw, abs = map(float, line.split(','))

                    current_time = time.time() - start_time
                    time_data.append(current_time)
                    accel_x_data.append(accel_x)
                    accel_y_data.append(accel_y)
                    accel_z_data.append(accel_z)
                    pitch_data.append(pitch)
                    roll_data.append(roll)
                    yaw_data.append(yaw)
                    abs_data.append(abs)

                    # Keep only the last MAX_PLOT_POINTS data points
                    if len(time_data) > MAX_PLOT_POINTS:
                        time_data.pop(0)
                        accel_x_data.pop(0)
                        accel_y_data.pop(0)
                        accel_z_data.pop(0)
                        pitch_data.pop(0)
                        roll_data.pop(0)
                        yaw_data.pop(0)
                        abs_data.pop(0)

                    # Update plot data
                    line_accel_x.set_data(time_data, accel_x_data)
                    line_accel_y.set_data(time_data, accel_y_data)
                    line_accel_z.set_data(time_data, accel_z_data)

                    line_pitch.set_data(time_data, pitch_data)
                    line_roll.set_data(time_data, roll_data)
                    line_yaw.set_data(time_data, yaw_data)
                    line_abs.set_data(time_data, abs_data)

                    # Rescale x-axis dynamically based on current data range
                    ax1.set_xlim(time_data[0], time_data[-1] + 0.1) # Add a small buffer
                    ax2.set_xlim(time_data[0], time_data[-1] + 0.1)

                    # Optional: Adjust y-axis limits dynamically if data goes out of bounds
                    # This can make plots less stable, but ensures data is always visible
                    # ax1.set_ylim(min(accel_x_data + accel_y_data + accel_z_data) - 1, max(accel_x_data + accel_y_data + accel_z_data) + 1)
                    # ax2.set_ylim(min(pitch_data + roll_data + yaw_data) - 5, max(pitch_data + roll_data + yaw_data) + 5)

                except ValueError as e:
                    print(f"Error parsing data (ValueError): '{line}' - {e}")
                except IndexError as e:
                    print(f"Error parsing data (IndexError - likely missing values): '{line}' - {e}")
                except Exception as e:
                    print(f"Unexpected error during data processing: {e} for line '{line}'")
            else:
                pass # print("Received empty line (might be expected during startup or noise)")
        else:
            pass # print("No data in serial buffer yet.") # Debug: Only print if you suspect no data is coming

    except serial.SerialException as e:
        print(f"Serial communication error in animate: {e}")
        ani.event_source.stop() # Stop animation on serial error
        plt.close(fig) # Close the plot window
    except Exception as e:
        print(f"An unexpected error occurred in animate: {e}")
        ani.event_source.stop()
        plt.close(fig)

# --- Main execution ---
ser = None # Initialize ser to None
try:
    print(f"Attempting to connect to serial port: {SERIAL_PORT} at {BAUD_RATE} baud.")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # Add a timeout
    print(f"Successfully connected to {SERIAL_PORT}.")

    # Clear any old data in the serial buffer
    ser.flushInput()
    print("Serial input buffer flushed.")

    # Set up the animation
    # interval=50 means update every 50ms (20 frames per second)
    # cache_frame_data=False is important for live plotting to prevent memory issues
    ani = animation.FuncAnimation(fig, animate, interval=5, cache_frame_data=False)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap
    plt.show()

except serial.SerialException as e:
    print(f"\n--- ERROR: Could not open serial port ---")
    print(f"Reason: {e}")
    print(f"1. Is the Pico connected and powered on?")
    print(f"2. Is '{SERIAL_PORT}' the correct serial port? (Check `ls /dev/cu.usbmodem*` in Terminal)")
    print(f"3. Is Thonny (or any other program) disconnected from the Pico's serial port?")
except Exception as e:
    print(f"\n--- An unexpected error occurred during setup: {e} ---")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")