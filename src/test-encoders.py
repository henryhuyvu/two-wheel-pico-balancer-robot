import board
import digitalio
import time

# --- Configuration ---
# Encoder 1
ENCODER1_PIN_A = board.GP11
ENCODER1_PIN_B = board.GP12

# Encoder 2
ENCODER2_PIN_A = board.GP7
ENCODER2_PIN_B = board.GP8

# Number of pulses per full motor revolution (this is a key parameter you'll need to determine)
# For many encoders, this is a multiple of the number of magnetic poles or slots.
# Start with a placeholder and adjust later.
PULSES_PER_REVOLUTION_1 = 20  # Placeholder for Encoder 1
PULSES_PER_REVOLUTION_2 = 20  # Placeholder for Encoder 2

# --- Pin Setup ---
# Encoder 1
encoder1_pin_a = digitalio.DigitalInOut(ENCODER1_PIN_A)
encoder1_pin_a.direction = digitalio.Direction.INPUT
encoder1_pin_a.pull = digitalio.Pull.UP  # Use internal pull-up resistors

encoder1_pin_b = digitalio.DigitalInOut(ENCODER1_PIN_B)
encoder1_pin_b.direction = digitalio.Direction.INPUT
encoder1_pin_b.pull = digitalio.Pull.UP

# Encoder 2
encoder2_pin_a = digitalio.DigitalInOut(ENCODER2_PIN_A)
encoder2_pin_a.direction = digitalio.Direction.INPUT
encoder2_pin_a.pull = digitalio.Pull.UP

encoder2_pin_b = digitalio.DigitalInOut(ENCODER2_PIN_B)
encoder2_pin_b.direction = digitalio.Direction.INPUT
encoder2_pin_b.pull = digitalio.Pull.UP

# --- State Variables ---
# Store previous states to detect changes
encoder1_state_a_prev = encoder1_pin_a.value
encoder1_state_b_prev = encoder1_pin_b.value

encoder2_state_a_prev = encoder2_pin_a.value
encoder2_state_b_prev = encoder2_pin_b.value

# Counters for pulses
encoder1_count = 0
encoder2_count = 0

# Direction indicators (0: stationary, 1: forward, -1: backward)
encoder1_direction = 0
encoder2_direction = 0

print("Starting encoder monitoring. Rotate the wheels manually.")
print("Press Ctrl+C to stop.")

# --- Main Loop ---
while True:
    # Read current states
    encoder1_state_a_current = encoder1_pin_a.value
    encoder1_state_b_current = encoder1_pin_b.value

    encoder2_state_a_current = encoder2_pin_a.value
    encoder2_state_b_current = encoder2_pin_b.value

    # --- Process Encoder 1 ---
    # Check for changes on Encoder 1
    if encoder1_state_a_current != encoder1_state_a_prev or encoder1_state_b_current != encoder1_state_b_prev:
        # A pulse has occurred. Now determine direction.
        # This logic assumes a quadrature encoder where A and B are 90 degrees out of phase.
        # We'll detect a full cycle of A and B to increment the count.
        # A common way is to check the state of B when A changes, or vice-versa.

        # Let's use a common approach: check B's state when A changes.
        # If A goes HIGH and B is HIGH, it's one direction.
        # If A goes HIGH and B is LOW, it's the other direction.
        # Similarly, if A goes LOW and B is LOW, one direction.
        # If A goes LOW and B is HIGH, the other direction.

        # We'll focus on detecting a full cycle of a quadrature encoder for simplicity
        # and to avoid double counting. A common way to get a clean count is to count
        # on one edge of one channel, but only if the other channel is in a specific state.
        # For example, count when A goes HIGH, if B is LOW. This gives 1 pulse per phase change.
        # A full quadrature cycle will give 4 such events.

        # Simplified approach: detect any transition and infer direction.
        # This might overcount if not careful, but for initial testing, it's good.

        # Let's use a method that increments on one edge of A and checks B for direction.
        # If A goes from LOW to HIGH, and B is LOW, increment forward.
        # If A goes from LOW to HIGH, and B is HIGH, increment backward.
        # This is a simplified way to get direction.

        # A more robust way to detect a full pulse (e.g., one "detent" or unique position)
        # is to count when one signal (say, A) transitions, and then observe the state of B.
        # For example, when A transitions from LOW to HIGH:
        # If B is LOW -> increment forward
        # If B is HIGH -> increment backward

        # Let's simplify for now and just count transitions and try to infer direction from the pattern.
        # This is a basic quadrature decoding.

        # When A changes state:
        if encoder1_state_a_current != encoder1_state_a_prev:
            if encoder1_state_a_current:  # A went from LOW to HIGH
                if encoder1_state_b_current:
                    # A HIGH, B HIGH (or A HIGH, B was LOW and is now HIGH)
                    # This combination implies one direction.
                    # Let's assume A->HIGH, B->HIGH means forward
                    encoder1_count += 1
                    encoder1_direction = 1
                else:
                    # A HIGH, B LOW
                    # This combination implies the other direction.
                    # Let's assume A->HIGH, B->LOW means backward
                    encoder1_count -= 1
                    encoder1_direction = -1
            else:  # A went from HIGH to LOW
                if encoder1_state_b_current:
                    # A LOW, B HIGH
                    # This combination implies one direction.
                    # Let's assume A->LOW, B->HIGH means backward
                    encoder1_count -= 1
                    encoder1_direction = -1
                else:
                    # A LOW, B LOW
                    # This combination implies the other direction.
                    # Let's assume A->LOW, B->LOW means forward
                    encoder1_count += 1
                    encoder1_direction = 1
        # If B changes state and A did not, we can also infer direction and update count.
        # This makes it more robust, but also more complex to avoid double counting.
        # For this example, we'll rely on A's transitions primarily for simplicity.
        # A more complete quadrature decoder would handle both A and B transitions.

    # Update previous states for Encoder 1
    encoder1_state_a_prev = encoder1_state_a_current
    encoder1_state_b_prev = encoder1_state_b_current

    # --- Process Encoder 2 ---
    # Check for changes on Encoder 2
    if encoder2_state_a_current != encoder2_state_a_prev or encoder2_state_b_current != encoder2_state_b_prev:
        # Similar logic for Encoder 2
        if encoder2_state_a_current != encoder2_state_a_prev:
            if encoder2_state_a_current:  # A went from LOW to HIGH
                if encoder2_state_b_current:
                    encoder2_count += 1
                    encoder2_direction = 1
                else:
                    encoder2_count -= 1
                    encoder2_direction = -1
            else:  # A went from HIGH to LOW
                if encoder2_state_b_current:
                    encoder2_count -= 1
                    encoder2_direction = -1
                else:
                    encoder2_count += 1
                    encoder2_direction = 1

    # Update previous states for Encoder 2
    encoder2_state_a_prev = encoder2_state_a_current
    encoder2_state_b_prev = encoder2_state_b_current

    # --- Display Output ---
    # Print the current counts and inferred direction
    # We'll print every so often to avoid flooding the console.
    # A simple way is to print when a change is detected, but for a stream,
    # we can print periodically.
    # For this example, let's print if there's movement or every second.

    # To create a constantly updating stream, we can print every iteration,
    # but it might be too fast. Let's add a small delay and print.
    # Or, we can only print if the count has changed or direction is non-zero.

    # Let's print if there's any movement detected or periodically
    if encoder1_direction != 0 or encoder2_direction != 0:
        print(f"Enc1: Count={encoder1_count}, Dir={encoder1_direction} | Enc2: Count={encoder2_count}, Dir={encoder2_direction}")
    else:
        # If no movement, we can print less frequently or not at all.
        # For a constant stream, we'll print even if stationary, but maybe with a marker.
        # For now, let's just print if there's movement.
        # If you want a true constant stream, uncomment the line below:
        # print(f"Enc1: Count={encoder1_count}, Dir={encoder1_direction} | Enc2: Count={encoder2_count}, Dir={encoder2_direction}")
        pass # Do nothing if stationary to keep output cleaner.

    # Add a small delay to avoid overwhelming the CPU and the serial output.
    # Adjust this value based on how fast you want the updates.
    # A smaller value means faster updates but more CPU usage.
    time.sleep(0.01) # 10 milliseconds
