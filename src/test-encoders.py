import board
import rotaryio
import time

# --- Configuration ---
# Use the pins from your existing setup
# Encoder Left (Encoder 1 in your previous script)
enc_l = rotaryio.IncrementalEncoder(board.GP7, board.GP8)

# Encoder Right (Encoder 2 in your previous script)
enc_r = rotaryio.IncrementalEncoder(board.GP12, board.GP11)

print("--- Encoder Diagnostic Mode ---")
print("Instructions:")
print("1. Rotate wheels FORWARD (the way the robot moves to balance a forward fall).")
print("2. If the count INCREASES (becomes more positive), that encoder is correct.")
print("3. If the count DECREASES, you must swap the pins in your main code.")
print("-------------------------------")

last_l = 0
last_r = 0

while True:
    curr_l = enc_l.position
    curr_r = enc_r.position
    
    # Calculate change since last check
    diff_l = curr_l - last_l
    diff_r = curr_r - last_r
    
    if diff_l != 0 or diff_r != 0:
        print(f"L: {curr_l:6} (Change: {diff_l:3}) | R: {curr_r:6} (Change: {diff_r:3})")
    
    last_l = curr_l
    last_r = curr_r
    
    time.sleep(0.1)