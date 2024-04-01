import numpy as np
import smbus
import time
import threading

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)  # Assuming channel 1 for Raspberry Pi

# MCP4725 DAC address
MCP4725_ADDRESS = 0x60

# Shared variable for BPM, with a default value
bpm = 96

def set_voltage(dac_address, voltage, vref=3.3):
    dac_value = int((voltage / vref) * 4095)
    bus.write_i2c_block_data(dac_address, 0x40, [dac_value >> 4, (dac_value & 15) << 4])

def generate_heartbeat_signal(t, current_bpm):
    # Simplified signal generation based on BPM
    # For demonstration,  sinusoidal waveform
    frequency = current_bpm / 60.0  # Convert BPM to Hz
    return np.sin(2 * np.pi * frequency * t)

def input_listener():
    global bpm
    while True:
        print("Current BPM: ", bpm)
        print("Enter new BPM (or 'exit' to stop): ")
        input_value = input().strip()
        if input_value.lower() == 'exit':
            print("Exiting BPM adjustment.")
            break
        try:
            new_bpm = int(input_value)
            if new_bpm > 0:
                bpm = new_bpm
                print(f"BPM updated to {bpm}.")
            else:
                print("Please enter a positive integer.")
        except ValueError:
            print("Invalid input. Please enter a valid integer.")

def main():
    global bpm
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()

    try:
        while True:
            # Simulate a continuous time axis
            t = np.linspace(0, 1, 100)  # 1 second divided into 100 parts

            current_bpm = bpm  # Copy the current BPM value to minimize locking

            # Generate and scale the heartbeat signal
            heartbeat_signal = generate_heartbeat_signal(t, current_bpm)
            scaled_signal = (heartbeat_signal + 1) * 1.65  # Scaling to 0-3.3V

            # Output the scaled signal
            for voltage in scaled_signal:
                set_voltage(MCP4725_ADDRESS, voltage)
                time.sleep(0.01)  # Assuming a fixed sample rate for simplicity

    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()