import numpy as np
import smbus
import time
import threading

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)  # Assuming channel 1 for Raspberry Pi

# MCP4725 DAC address
MCP4725_ADDRESS = 0x60


def set_voltage(dac_address, voltage, vref=3.3):
    dac_value = int((voltage / vref) * 4095)
    bus.write_i2c_block_data(dac_address, 0x40, [dac_value >> 4, (dac_value & 15) << 4])


ekg_settings = {
    "P_amplitude": 0.1,
    "Q_amplitude": -0.05,
    "R_amplitude": 0.25,
    "S_amplitude": -0.15,
    "T_amplitude": 0.1,
    "PR_interval": 0.16,  # seconds
    "QT_interval": 0.4,  # seconds
    "QRS_complex": 0.08,  # seconds
    "ST_segment": 0.12,  # seconds
    "bpm": 96  # Beats per minute
}


# Function to generate a placeholder waveform component
def generate_waveform_component(t, amplitude, duration, start_time):
    end_time = start_time + duration
    return np.where((t >= start_time) & (t < end_time), amplitude, 0)


def HeartBeatSignal_customized(t, settings):
    P_wave = generate_waveform_component(t, settings["P_amplitude"], 0.1, 0)
    Q_wave = generate_waveform_component(t, settings["Q_amplitude"], 0.05, settings["PR_interval"])
    R_wave = generate_waveform_component(t, settings["R_amplitude"], 0.1, settings["PR_interval"] + 0.05)
    S_wave = generate_waveform_component(t, settings["S_amplitude"], 0.05, settings["PR_interval"] + 0.15)
    T_wave = generate_waveform_component(t, settings["T_amplitude"], 0.1,
                                         settings["PR_interval"] + settings["QRS_complex"] + settings["ST_segment"])

    heartbeat_signal = P_wave + Q_wave + R_wave + S_wave + T_wave
    return heartbeat_signal


def input_listener():
    global ekg_settings
    while True:
        print("Enter the variable you want to change (e.g., bpm, P_amplitude): ")
        var_name = input().strip()
        if var_name in ekg_settings:
            print(f"Current value of {var_name}: {ekg_settings[var_name]}")
            print(f"Enter new value for {var_name}: ")
            new_value = float(input().strip())
            ekg_settings[var_name] = new_value
            print(f"Updated {var_name} to {new_value}")
        else:
            print("Variable not recognized. Available variables: " + ", ".join(ekg_settings.keys()))


def main():
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()

    try:
        while True:
            nt = 301
            it = np.linspace(0, 1, nt)
            thor = 30
            t = thor * 2 * np.pi * it

            # Generate heartbeat signal using current settings
            hbp_customized = HeartBeatSignal_customized(t, ekg_settings)

            # Scale the customized heartbeat pattern to the 0-3.3V range
            scaled_hbp_customized = (hbp_customized - np.min(hbp_customized)) * (
                        3.3 / (np.max(hbp_customized) - np.min(hbp_customized)))

            # Calculate time between beats and sample rate
            seconds_per_beat = 60.0 / ekg_settings["bpm"]
            sample_rate = seconds_per_beat / len(scaled_hbp_customized)

            for voltage in scaled_hbp_customized:
                set_voltage(MCP4725_ADDRESS, voltage)
                time.sleep(sample_rate)

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        bus.close()


if __name__ == "__main__":
    main()