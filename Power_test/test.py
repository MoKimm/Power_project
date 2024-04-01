import numpy as np
import smbus
import time

# Raspberry Pi I2C channel (usually 1)
I2C_CHANNEL = 1
# MCP4725 default I2C address
MCP4725_ADDRESS = 0x60

# Initialize I2C (SMBus)
bus = smbus.SMBus(I2C_CHANNEL)

# Function to set voltage on MCP4725
def set_voltage(dac_address, voltage, vref=3.3):
    # Convert voltage to DAC value and write to MCP4725
    dac_value = int((voltage / vref) * 4095)
    bus.write_i2c_block_data(dac_address, 0x40, [dac_value >> 4, (dac_value & 15) << 4])

def HeartBeatSignal(t):
    # Generates a simplified heartbeat waveform for demonstration.
    # You may need to adjust this to match your actual desired heartbeat shape.
    A = 1.0 - np.abs(t)  # Basic triangular shape
    return np.maximum(A, 0)

def scale01(a):
    # Scale an array to be within the 0 to 1 range
    return (a - np.min(a)) / (np.max(a) - np.min(a))

def HeartBeat_pattern(t, bpm):
    # Calculate interval and steps per beat based on BPM
    interval = 60 / bpm  # Interval in seconds between beats
    dt = t[1] - t[0]  # Time step in the t array
    steps_per_beat = int(round(interval / dt))
    
    V = np.zeros_like(t)
    for i in range(0, len(t), steps_per_beat):
        # Calculate the length of the heartbeat waveform to ensure it fits within the array bounds
        beat_length = min(steps_per_beat, len(t) - i)
        # Generate a heartbeat waveform for each beat and add it to the signal
        V[i:i+beat_length] += HeartBeatSignal(np.linspace(-1, 1, beat_length))
    
    return scale01(V)

# Define the time array and desired BPM
nt = 1001  # Number of time points
t = np.linspace(-5, 5, nt)  # Time array spanning from -5 to 5 seconds
desired_bpm = 60  # Example BPM

# Generate the heartbeat pattern with the desired BPM
hbp = HeartBeat_pattern(t, desired_bpm)

# Scale the heartbeat pattern to the 0-3.3V range
scaled_hbp = scale01(hbp) * 3.3

# Output the signal to the DAC at the desired sample rate
sample_rate = 0.01  # Sample rate in seconds, adjust as needed
try:
    for voltage in scaled_hbp:
        set_voltage(MCP4725_ADDRESS, voltage)
        time.sleep(sample_rate)
finally:
    bus.close()  # Ensure the I2C bus is closed even if an error occurs
