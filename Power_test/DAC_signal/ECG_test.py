import pandas as pd
import matplotlib.pyplot as plt

# The path to the CSV file uploaded by the user
csv_file_path = 'c:/Users/14709/Power_project/Power_test/100.csv'

# Read the CSV file, using the first column as the index if it represents time
ecg_data = pd.read_csv(csv_file_path)

# Plotting the 'MLII' lead of the ECG
plt.figure(figsize=(20, 5))  # Set a wide figure to visualize the ECG waveform clearly
ecg_data['MLII'][0:1000].plot()  # Assuming 'MLII' is the name of the column for ECG data

# Providing labels and title for the plot
plt.title('First 4 Heartbeats of Patient 100\'s EKG')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')

# Displaying a grid for better readability
plt.grid(True)

# Display the plot
plt.show()
