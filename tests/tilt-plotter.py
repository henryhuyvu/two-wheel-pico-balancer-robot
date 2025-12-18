import pandas as pd
import matplotlib.pyplot as plt
import io

# --- CONFIGURATION ---
INPUT_FILENAME = 'imu_tilt_data.csv' 
# This is the name of the file where your laptop script saved the serial data.
# If you used a simple terminal program, it might be a .txt file, adjust if necessary.
# ---------------------

def plot_imu_data(file_path):
    print(f"Reading data from: {file_path}")
    
    # 1. Read the raw data from the file
    with open(file_path, 'r') as f:
        raw_data = f.read()

    # 2. Clean the data to isolate only the CSV-formatted content
    # We look for the header line to start parsing the data.
    try:
        data_start_index = raw_data.index("timestamp_s,roll_deg")
    except ValueError:
        print("Error: Could not find the expected header 'timestamp_s,roll_deg'.")
        print("Please ensure the data file contains this header and is correctly formatted.")
        return

    # Extract only the relevant CSV part (header + data)
    csv_data = raw_data[data_start_index:].strip()

    # 3. Use the 'io.StringIO' to treat the string data as a file
    # This lets pandas read the clean CSV data directly from memory.
    data_io = io.StringIO(csv_data)
    
    # 4. Read the data into a pandas DataFrame
    try:
        df = pd.read_csv(data_io)
    except pd.errors.EmptyDataError:
        print("Error: Data file is empty or contains no data after the header.")
        return
    except Exception as e:
        print(f"An error occurred while parsing the CSV data: {e}")
        return

    # 5. Plot the data
    
    # Set the plotting style for a clean look
    plt.style.use('ggplot')
    
    # Create the figure and axis objects
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot the roll angle vs. time
    ax.plot(df['timestamp_s'], df['roll_deg'], label='Tilt Angle (Roll)', color='teal', linewidth=1)

    # Add titles and labels
    ax.set_title('BNO085 Roll Angle Over Time')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Roll Angle (degrees)')
    
    # Add a horizontal line at 90 degrees (often the equilibrium point)
    ax.axhline(y=90, color='r', linestyle='--', alpha=0.6, label='Equilibrium (90°)')
    
    # Add a grid and legend
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend()

    # Automatically adjust plot limits to ensure all data is visible
    plt.tight_layout()
    
    # Show the plot
    plt.show()

# Execute the plotting function
if __name__ == '__main__':
    plot_imu_data(INPUT_FILENAME)