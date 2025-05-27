import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import json

# Load the action log file and extract tx_power and beacon_rate
action_log_file = 'custom/logs/action.log'
tx_power, beacon_rate = [], []

with open(action_log_file, 'r') as file:
    action_data = ""
    for line in file:
        if 'Batch' in line:
            # If we already have action data, parse and store it
            if action_data:
                try:
                    # Extract tx_power and beacon_rate from lines 2 and 3
                    action_lines = action_data.split('\n')
                    tx_power.append(float(action_lines[1].strip().replace(',', '')))  # Line 2: tx_power
                    beacon_rate.append(float(action_lines[2].strip().replace(',', '')))  # Line 3: beacon_rate
                except Exception as e:
                    print(f"Error processing action data: {e}")
                action_data = ""  # Reset for the next batch
            
            # Start accumulating action data from the new batch line
            action_data = line.split('Batch')[1].strip()  # Skip the timestamp and batch number

        else:
            # Accumulate action data spread over multiple lines
            action_data += '\n' + line.strip()

    # Process the last batch data after the loop ends
    if action_data:
        try:
            action_lines = action_data.split('\n')
            tx_power.append(float(action_lines[1].strip().replace(',', '')))  # Line 2: tx_power
            beacon_rate.append(float(action_lines[2].strip().replace(',', '')))  # Line 3: beacon_rate
        except Exception as e:
            print(f"Error processing last batch data: {e}")
            
# Convert the action lists into numpy arrays
tx_power = np.array(tx_power)
beacon_rate = np.array(beacon_rate)

# Load performance metrics from CSV
performance_metrics_file = 'custom/logs/performance_metrics.csv'
performance_data = pd.read_csv(performance_metrics_file)

# Plot histograms for tx_power and beacon_rate
plt.figure(figsize=(10, 6))  # Increased figure size to accommodate more plots

# Histogram for tx_power
plt.subplot(2, 2, 1)  # Changed to 2x2 grid, position 1
plt.hist(tx_power, bins=20, color='skyblue', edgecolor='black')
plt.title("Histogram of Transmission Power (tx_power)")
plt.xlabel("Transmission Power (tx_power)")
plt.ylabel("Frequency")

# Histogram for beacon_rate
plt.subplot(2, 2, 2)  # Changed to 3x2 grid, position 2
plt.hist(beacon_rate, bins=20, color='lightgreen', edgecolor='black')
plt.title("Histogram of Beacon Rate (beacon_rate)")
plt.xlabel("Beacon Rate (beacon_rate)")
plt.ylabel("Frequency")

# Plot line charts for CBR, SNR, and reward
plt.subplot(2, 2, 3)  # Position 3
plt.plot(performance_data['Batch'], performance_data['CBR'], color='blue')
plt.title("CBR Over Time")
plt.xlabel("Batch")
plt.ylabel("CBR")

plt.subplot(2, 2, 4)  # Position 4
plt.plot(performance_data['Batch'], performance_data['SNR'], color='green')
plt.title("SNR Over Time")
plt.xlabel("Batch")
plt.ylabel("SNR")

# Adjust layout and display the first figure (excluding Reward plot)
plt.tight_layout()

# Now, create a new figure for the Reward plot
plt.figure(figsize=(10, 6))  # New figure for Reward plot
plt.plot(performance_data['Batch'], performance_data['Reward'], color='red')
plt.title("Reward Over Time")
plt.xlabel("Batch")
plt.ylabel("Reward")

# Display the Reward plot in a new window (new tab)
plt.show()