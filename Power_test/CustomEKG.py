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
    dac_value = int((voltage / vref) * 4095)
    bus.write_i2c_block_data(dac_address, 0x40, [dac_value >> 4, (dac_value & 15) << 4])


# Customization variables for the EKG components
P_amplitude = 0.1
Q_amplitude = -0.05
R_amplitude = 0.25
S_amplitude = -0.15
T_amplitude = 0.1

PR_interval = 0.16  # seconds
QT_interval = 0.4  # seconds
QRS_complex = 0.08  # seconds
ST_segment = 0.12  # seconds


# Simplified function to generate waveform components, adjust as needed for accuracy
def generate_waveform_component(t, amplitude, duration, start_time):
    end_time = start_time + duration
    return np.where((t >= start_time) & (t < end_time), amplitude, 0)


# Customized HeartBeatSignal function
def HeartBeatSignal_customized(t):
    P_wave = generate_waveform_component(t, P_amplitude, 0.1, 0)
    Q_wave = generate_waveform_component(t, Q_amplitude, 0.05, PR_interval)
    R_wave = generate_waveform_component(t, R_amplitude, 0.1, PR_interval + 0.05)
    S_wave = generate_waveform_component(t, S_amplitude, 0.05, PR_interval + 0.15)
    T_wave = generate_waveform_component(t, T_amplitude, 0.1, PR_interval + QRS_complex + ST_segment)

    heartbeat_signal = P_wave + Q_wave + R_wave + S_wave + T_wave
    return heartbeat_signal


# Main loop for generating and outputting the heartbeat signal
bpm = 96  # Example BPM, adjust as needed
seconds_per_beat = 60.0 / bpm

try:
    while True:
        nt, it, thor = 301, np.linspace(0, 1, nt), 30
        t = thor * 2 * np.pi * it

        hbp_customized = HeartBeatSignal_customized(t)

        # Scale the customized heartbeat pattern to the 0-3.3V range
        scaled_hbp_customized = (hbp_customized - np.min(hbp_customized)) * (
                    3.3 / (np.max(hbp_customized) - np.min(hbp_customized)))

        sample_rate = seconds_per_beat / len(scaled_hbp_customized)

        for voltage in scaled_hbp_customized:
            set_voltage(MCP4725_ADDRESS, voltage)
            time.sleep(sample_rate)

except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    bus.close()  # Ensure the I2C bus is closed properly