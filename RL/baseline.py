import socket
import torch
import logging
import json
import os
from datetime import datetime

LOG_RECEIVED_PATH = 'receive_data.log'
LOG_SENT_PATH = 'sent_data.log'

def log_data(log_path, data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(log_path, 'a') as log_file:
        log_file.write(f"[{timestamp}] {data}\n")

# Fungsi untuk menyesuaikan MCS berdasarkan SNR
def adjust_mcs_based_on_snr(snr):
    if 0 <= snr < 5:
        return 0  # MCS 0: BPSK 1/2
    elif 5 <= snr < 10:
        return 1  # MCS 1: QPSK 1/2
    elif 10 <= snr < 15:
        return 2  # MCS 2: QPSK 3/4
    elif 15 <= snr < 20:
        return 3  # MCS 3: 16-QAM 1/2
    elif 20 <= snr < 25:
        return 4  # MCS 4: 16-QAM 3/4
    elif 25 <= snr < 30:
        return 5  # MCS 5: 64-QAM 2/3
    elif 30 <= snr < 35:
        return 6  # MCS 6: 64-QAM 3/4
    elif 35 <= snr < 40:
        return 7  # MCS 7: 64-QAM 5/6
    elif 40 <= snr < 45:
        return 8  # MCS 8: 256-QAM 3/4
    elif 45 <= snr < 50:
        return 9  # MCS 9: 256-QAM 5/6
    elif snr >= 50:
        return 10 # MCS 10: 1024-QAM 3/4
    else:
        return 0 # SNR out of range

logging.basicConfig(level=logging.INFO)

# Membuka soket untuk menerima data
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("localhost", 5000))
server.listen(1)
logging.info("Listening on port 5000...")
conn, addr = server.accept()
logging.info(f"Connected by {addr}")

# Loop untuk menerima dan memproses batch data
batch_counter = 0
while True:
    data = conn.recv(65536)
    if not data:
        logging.info("No data received, closing connection.")
        break
    try:
        batch_counter += 1
        # Parse incoming batch data from client
        batch_data = json.loads(data.decode())
        log_data(LOG_RECEIVED_PATH, f"Batch {batch_counter}: {json.dumps(batch_data, indent=4)}")
        logging.info(f"Received data for batch {batch_counter}:\n{json.dumps(batch_data, indent=4)}")

        # Prepare response dictionary
        responses = {}

        for veh_id, vehicle_data in batch_data.items():
            responses[veh_id] = {
                "transmissionPower": float(vehicle_data['transmissionPower']),
                "beaconRate": float(vehicle_data['beaconRate']),
                "MCS": adjust_mcs_based_on_snr(vehicle_data['SNR'])
            }

        # Send the response batch to the client
        response_data = json.dumps(responses).encode('utf-8')
        conn.sendall(response_data)
        formatted_response = json.dumps(responses, indent=4)
        log_data(LOG_SENT_PATH, f"Batch {batch_counter}: {formatted_response}")
        logging.info(f"Sending response to client for batch {batch_counter}: {formatted_response}")

    except Exception as e:
        logging.error(f"Error: {e}")
        continue

# Menutup koneksi dan server
conn.close()
server.close()
logging.info("Server closed.")
