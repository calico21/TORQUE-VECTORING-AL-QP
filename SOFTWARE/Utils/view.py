import serial
import matplotlib.pyplot as plt
import numpy as np
import threading
from matplotlib.animation import FuncAnimation

# Open serial port
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)  # Adjust baud rate and timeout if necessary

# Create lists to store the data for plotting
yaw_data = []
pitch_data = []
roll_data = []
a_rate_y_data = []
a_rate_p_data = []
a_rate_r_data = []
acc_x_data = []
acc_y_data = []
acc_z_data = []

# Create figure and axes for each data type
fig, axs = plt.subplots(3, 3, figsize=(15, 10))
axs[0, 0].set_title('Yaw')
axs[0, 1].set_title('Pitch')
axs[0, 2].set_title('Roll')
axs[1, 0].set_title('Angular Rate Y')
axs[1, 1].set_title('Angular Rate P')
axs[1, 2].set_title('Angular Rate R')
axs[2, 0].set_title('Acc X')
axs[2, 1].set_title('Acc Y')
axs[2, 2].set_title('Acc Z')

# Initialize lines to be updated
lines = [
    axs[0, 0].plot([], [], 'r-')[0],  # yaw
    axs[0, 1].plot([], [], 'g-')[0],  # pitch
    axs[0, 2].plot([], [], 'b-')[0],  # roll
    axs[1, 0].plot([], [], 'r-')[0],  # angular rate y
    axs[1, 1].plot([], [], 'g-')[0],  # angular rate p
    axs[1, 2].plot([], [], 'b-')[0],  # angular rate r
    axs[2, 0].plot([], [], 'r-')[0],  # acc x
    axs[2, 1].plot([], [], 'g-')[0],  # acc y
    axs[2, 2].plot([], [], 'b-')[0],  # acc z
]

# Set fixed axis limits for each plot
fixed_limits = {
    'yaw': (-180, 180),  # Example range for yaw in degrees
    'pitch': (-180, 180),  # Example range for pitch in degrees
    'roll': (-180, 180),   # Example range for roll in degrees
    'a_rate_y': (-100, 100),  # Example range for angular rate in deg/s
    'a_rate_p': (-100, 100),  # Example range for angular rate in deg/s
    'a_rate_r': (-100, 100),  # Example range for angular rate in deg/s
    'acc_x': (-2, 2),  # Example range for accelerometer in g
    'acc_y': (-2, 2),  # Example range for accelerometer in g
    'acc_z': (-2, 2),  # Example range for accelerometer in g
}

# Apply fixed limits to each axis
axs[0, 0].set_ylim(fixed_limits['roll'])
axs[0, 1].set_ylim(fixed_limits['pitch'])
axs[0, 2].set_ylim(fixed_limits['yaw'])
axs[1, 0].set_ylim(fixed_limits['a_rate_r'])
axs[1, 1].set_ylim(fixed_limits['a_rate_p'])
axs[1, 2].set_ylim(fixed_limits['a_rate_y'])
axs[2, 0].set_ylim(fixed_limits['acc_x'])
axs[2, 1].set_ylim(fixed_limits['acc_y'])
axs[2, 2].set_ylim(fixed_limits['acc_z'])

# Initialize the x-axis range
for ax in axs.flat:
    ax.set_xlim(0, 100)  # Set x-axis limit (time or data points count)

# Read data from serial port
def read_data():
    """Reads a line from the serial port and parses it."""
    line = ser.readline().decode('utf-8').strip()  # Read and decode line
    try:
        data = list(map(float, line.split(',')))
        if len(data) == 9:
            return data  # Returns the full list: yaw, pitch, roll, a_rate_ypr[], acc_xyz[]
    except ValueError:
        return None

# Update the data lists in the background
def update_data():
    """Function to continuously read data in a separate thread."""
    global yaw_data, pitch_data, roll_data, a_rate_y_data, a_rate_p_data, a_rate_r_data
    global acc_x_data, acc_y_data, acc_z_data

    while True:
        data = read_data()
        if data:
            roll, pitch, yaw = data[0], data[1], data[2]
            a_rate_r, a_rate_p, a_rate_y = data[3], data[4], data[5]
            acc_x, acc_y, acc_z = data[6], data[7], data[8]

            # Append new data to the lists
            yaw_data.append(yaw)
            pitch_data.append(pitch)
            roll_data.append(roll)
            a_rate_y_data.append(a_rate_y)
            a_rate_p_data.append(a_rate_p)
            a_rate_r_data.append(a_rate_r)
            acc_x_data.append(acc_x)
            acc_y_data.append(acc_y)
            acc_z_data.append(acc_z)

            # Limit data length for plotting
            max_length = 100
            if len(yaw_data) > max_length:
                yaw_data.pop(0)
                pitch_data.pop(0)
                roll_data.pop(0)
                a_rate_y_data.pop(0)
                a_rate_p_data.pop(0)
                a_rate_r_data.pop(0)
                acc_x_data.pop(0)
                acc_y_data.pop(0)
                acc_z_data.pop(0)

# Update the plot with new data
def update_plot(frame):
    """Update the plot with new data."""
    lines[0].set_data(np.arange(len(yaw_data)), yaw_data)
    lines[1].set_data(np.arange(len(pitch_data)), pitch_data)
    lines[2].set_data(np.arange(len(roll_data)), roll_data)
    lines[3].set_data(np.arange(len(a_rate_y_data)), a_rate_y_data)
    lines[4].set_data(np.arange(len(a_rate_p_data)), a_rate_p_data)
    lines[5].set_data(np.arange(len(a_rate_r_data)), a_rate_r_data)
    lines[6].set_data(np.arange(len(acc_x_data)), acc_x_data)
    lines[7].set_data(np.arange(len(acc_y_data)), acc_y_data)
    lines[8].set_data(np.arange(len(acc_z_data)), acc_z_data)

    return lines

# Start a separate thread to continuously read data
data_thread = threading.Thread(target=update_data, daemon=True)
data_thread.start()

# Set up the plot animation
ani = FuncAnimation(fig, update_plot, interval=10)

# Display the plot
plt.tight_layout()
plt.show()

