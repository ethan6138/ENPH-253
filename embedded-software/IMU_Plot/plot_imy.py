import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

SERIAL_PORT = "/dev/ttyACM0" #change to correct serial port
BAUD_RATE = 115200
MAX_POINTS = 200  # how many samples to keep visible on screen at once

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

roll_data = deque(maxlen=MAX_POINTS)
pitch_data = deque(maxlen=MAX_POINTS)

fig, ax = plt.subplots()
roll_line, = ax.plot([], [], label="Roll (deg)")
pitch_line, = ax.plot([], [], label="Pitch (deg)")
ax.legend()
ax.set_ylim(-90, 90)

def parse_line(line):
    # Expects format: "RollDeg:12.3 PitchDeg:-4.5"
    try:
        parts = line.strip().split(" ")
        roll = float(parts[0].split(":")[1])
        pitch = float(parts[1].split(":")[1])
        return roll, pitch
    except (IndexError, ValueError):
        return None, None

def update(frame):
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore")
        roll, pitch = parse_line(line)
        if roll is not None:
            roll_data.append(roll)
            pitch_data.append(pitch)

    roll_line.set_data(range(len(roll_data)), roll_data)
    pitch_line.set_data(range(len(pitch_data)), pitch_data)
    ax.set_xlim(0, MAX_POINTS)
    return roll_line, pitch_line

ani = FuncAnimation(fig, update, interval=50)
plt.show()