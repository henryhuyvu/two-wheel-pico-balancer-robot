import board
import digitalio
import time

# Initialize the LED pin
# Note: Use 'board.LED' for the built-in LED on most boards
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

print("LED starts flashing...")

try:
    while True:
        # Toggle the LED
        led.value = not led.value
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping...")
finally:
    # Ensure the LED is off when the script ends
    led.value = False
    print("Finished.")
