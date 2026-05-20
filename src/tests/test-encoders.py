import board
import rotaryio
import time

# --- Configuration ---
# Assign Left and Right Encoders to Pico Pins
encoder_L = rotaryio.IncrementalEncoder(board.GP7, board.GP8)
encoder_R = rotaryio.IncrementalEncoder(board.GP12, board.GP11)


print("\n\n\n--- Encoder Diagnostic Mode ---")
print("Instructions:")
print("1. Rotate wheels FORWARD (the way the robot moves to balance a forward fall).")
print("2. If the count INCREASES (becomes more positive), that encoder is correct.")
print("3. If the count DECREASES, you must swap the pins in your main code.")
print("-------------------------------\n")


# Encoder position tracking
lastPosition_L = 0
lastPosition_R = 0
encoderCheckFrequency = 100  # Hz
encoderWaitInterval = 1/encoderCheckFrequency

# Print statement padding
currentPositionPadding = 6  # Number of spaces for current position
changePadding = 3           # Number of spaces for change in position

while True:
    currPosition_L = encoder_L.position
    currPosition_R = encoder_R.position
    
    # Calculate position change since last check
    diff_L = currPosition_L - lastPosition_L
    diff_R = currPosition_R - lastPosition_R
    
    lastPosition_L = currPosition_L
    lastPosition_R = currPosition_R

    if diff_L != 0 or diff_R != 0:
        print(f"L: {currPosition_L:{currentPositionPadding}} (Change: {diff_L:{changePadding}}) | R: {currPosition_R:{currentPositionPadding}} (Change: {diff_R:{changePadding}})")

    time.sleep(encoderWaitInterval)
