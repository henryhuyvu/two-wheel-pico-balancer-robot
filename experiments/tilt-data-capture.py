import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/tty.usbmodem101'  # !!! CHANGE THIS to your Pico's serial port !!!
BAUD_RATE = 115200           # CircuitPython's default is 115200
OUTPUT_FILENAME = 'imu_tilt_data.csv'
# ---------------------

def capture_serial_data():
    print(f"Connecting to port {SERIAL_PORT} at {BAUD_RATE} baud...")

    try:
        # Open the serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        # Give the connection a moment to establish
        time.sleep(2) 
        print("Connection established. Starting data capture. Press Ctrl+C to stop.")

        with open(OUTPUT_FILENAME, 'w') as outfile:
            while True:
                # Read all available lines from the serial buffer
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        # Print to console for monitoring
                        print(line) 
                        # Write to file
                        outfile.write(line + '\n') 

    except serial.SerialException as e:
        print(f"Error communicating with serial port: {e}")
        print("Please check the port name and ensure the Pico is connected.")
    except KeyboardInterrupt:
        print("\nData capture stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Serial port closed. Data saved to {OUTPUT_FILENAME}")

if __name__ == '__main__':
    capture_serial_data()