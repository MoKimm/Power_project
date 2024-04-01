import numpy as np
import smbus
import time

# Raspberry Pi I2C channel (usually 1)
I2C_CHANNEL = 1
# MCP4725 default I2C address
MCP4725_ADDRESS = 0x60

# Initialize I2C (SMBus)
bus = smbus.SMBus(I2C_CHANNEL)

def set_voltage(dac_address, voltage, vref=3.3):
    if np.isnan(voltage):
        print("Warning: Attempted to set NaN voltage, skipping.")
        return
    # Convert voltage to DAC value and write to MCP4725
    dac_value = int((voltage / vref) * 4095)
    bus.write_i2c_block_data(dac_address, 0x40, [dac_value >> 4, (dac_value & 15) << 4])

def HeartBeatSignal(t):
    # Simplified ECG waveform components: P wave, QRS complex, and T wave
    P_wave = np.exp(-((t - 0.2)**2) / (2 * 0.02**2))
    Q_wave = -0.5 * np.exp(-((t - 0.4)**2) / (2 * 0.01**2))
    R_wave = np.exp(-((t - 0.45)**2) / (2 * 0.01**2))
    S_wave = -0.3 * np.exp(-((t - 0.5)**2) / (2 * 0.01**2))
    T_wave = 0.5 * np.exp(-((t - 0.7)**2) / (2 * 0.04**2))
    heartbeat = P_wave + Q_wave + R_wave + S_wave + T_wave
    heartbeat = heartbeat / np.max(np.abs(heartbeat))  # Normalize
    return heartbeat

def scale01(a):
    min_a = np.min(a)
    range_a = np.max(a) - min_a
    if range_a == 0:
        return np.zeros_like(a)  # Or return a constant array if more appropriate
    else:
        return (a - min_a) / range_a


def HeartBeat_pattern(t, bpm):
    interval = 60 / bpm
    dt = t[1] - t[0]
    steps_per_beat = int(round(interval / dt))
    V = np.zeros_like(t)
    for i in range(0, len(t), steps_per_beat):
        beat_length = min(steps_per_beat, len(t) - i)
        V[i:i+beat_length] += HeartBeatSignal(np.linspace(-1, 1, beat_length))
    return scale01(V)

# Define the time array and desired BPM
nt = 1001
t = np.linspace(-5, 5, nt)
desired_bpm = 60  # Desired BPM

# Generate the heartbeat pattern with the desired BPM
hbp = HeartBeat_pattern(t, desired_bpm)

# Scale the heartbeat pattern to the 0-3.3V range
scaled_hbp = scale01(hbp) * 3.3

# Output the signal to the DAC at the desired sample rate
sample_rate = 0.7  # Adjust as needed
if np.isnan(scaled_hbp).any():
    print("Error: The scaled heartbeat pattern contains NaN values.")
else:
    try:
        for voltage in scaled_hbp:
            set_voltage(MCP4725_ADDRESS, voltage)
            time.sleep(sample_rate)
    finally:
        bus.close()  # Ensure
