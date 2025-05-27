import socket
import torch
import numpy as np
import logging
import json
import os
import csv
from datetime import datetime
from ppo_agent import PPOAgent # Changed from SACAgent

# --- Configuration Constants ---
# TODO: Consider moving to a config file or command-line args for more flexibility
SERVER_HOST = "localhost"
SERVER_PORT = 5001
SOCKET_BUFFER_SIZE = 65536
SOCKET_TIMEOUT = 10.0 # Seconds

MIN_TX_POWER = 1.0  # dBm
MAX_TX_POWER = 30.0 # dBm
MIN_MCS_IDX = 0
MAX_MCS_IDX = 7
# --- End Configuration Constants ---


# --- PPO Specific Hyperparameters ---
# TODO: Tune these hyperparameters
NUM_ROLLOUT_STEPS = 2048  # Number of transitions to collect before a PPO update
PPO_EPOCHS = 10           # Number of epochs to run PPO updates for
PPO_BATCH_SIZE = 64       # Mini-batch size for PPO updates
GAMMA = 0.99              # Discount factor
GAE_LAMBDA = 0.95         # GAE lambda parameter
CLIP_EPSILON = 0.2        # PPO clipping epsilon
ENT_COEF = 0.01           # Entropy coefficient
VF_COEF = 0.5             # Value function loss coefficient
LR_ACTOR = 3e-4           # Learning rate for actor
LR_CRITIC = 1e-3          # Learning rate for critic
# --- End PPO Specific Hyperparameters ---

LOG_DIR = 'custom_ppo/logs/'
MODEL_DIR = 'custom_ppo/model/'

LOG_RECEIVED_PATH = os.path.join(LOG_DIR, 'receive_data_ppo.log')
LOG_SENT_PATH = os.path.join(LOG_DIR, 'sent_data_ppo.log')
LOG_DEBUG_ACTION_PATH = os.path.join(LOG_DIR, 'action_ppo.log')
LOG_DEBUG_REWARD_PATH = os.path.join(LOG_DIR, 'reward_debug_ppo.log')
PERFORMANCE_LOG_PATH = os.path.join(LOG_DIR, 'performance_metrics_ppo.csv')
# MODEL_SAVE_PATH was removed as actor/critic are saved separately by agent

# Ensure directories exist
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)

# TODO: For performance, consider in-memory buffering for logs and flushing periodically,
# or use an asynchronous logger instead of writing on every call.
def log_data(log_path, data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(log_path, 'a') as log_file:
        log_file.write(f"[{timestamp}] {data}\n")

def write_performance_metrics(cbr, snr, mcs_chosen, data_rate, tx_power, reward, batch_number, total_steps_in_buffer, file_path=PERFORMANCE_LOG_PATH):
    file_exists = os.path.exists(file_path)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(['Batch', 'Total_Steps_In_Buffer', 'Timestamp', 'CBR', 'SNR', 'MCS_Chosen', 'Data_Rate_Mbps', 'Tx_Power_dBm', 'Reward'])
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer.writerow([batch_number, total_steps_in_buffer, timestamp, cbr, snr, mcs_chosen, data_rate, tx_power, reward])

# Reusing this function from train.py
def get_data_rate_and_sr(mcs):
    """
    Returns data rate (Mbps) and receiver sensitivity (Sr, dBm) based on MCS index.
    """
    table = [
        (3, -85), (4.5, -84), (6, -82), (9, -80),
        (12, -77), (18, -73), (24, -69), (27, -68),
    ]
    if 0 <= mcs < len(table):
        return table[mcs]
    else:
        # Default to lowest MCS if out of bounds, or handle error
        logging.warning(f"MCS index {mcs} out of range. Defaulting to MCS 0.")
        return table[0]


# TODO: CRITICAL - Implement the reward function from the paper
# "Simultaneous Data Rate and Transmission Power Adaptation in V2V Communications: A Deep Reinforcement Learning Approach"
def calculate_reward_datarate_power_paper(achieved_data_rate, chosen_tx_power, snr, cbr, num_neighbors, mcs_idx, prev_tx_power, prev_mcs_idx, sr_at_mcs):
    """
    Placeholder for the reward function based on the specified paper.
    You need to implement this function according to the paper's formulas.

    Parameters might include:
    - achieved_data_rate (float): Data rate in Mbps for the chosen MCS.
    - chosen_tx_power (float): Transmission power in dBm.
    - snr (float): Achieved Signal-to-Noise Ratio.
    - cbr (float): Channel Busy Ratio.
    - num_neighbors (int): Number of neighbors.
    - mcs_idx (int): Chosen MCS index.
    - prev_tx_power (float): Transmission power from the previous state.
    - prev_mcs_idx (int): MCS index from the previous state.
    - sr_at_mcs (float): Receiver sensitivity for the chosen MCS.
    
    Returns:
    - reward (float): Calculated reward.
    """
    logging.info(f"[REWARD_DEBUG_PAPER] Inputs: DR={achieved_data_rate}, TxPwr={chosen_tx_power}, SNR={snr}, CBR={cbr}, Neighbors={num_neighbors}, MCS={mcs_idx}, PrevTxPwr={prev_tx_power}, PrevMCS={prev_mcs_idx}, Sr={sr_at_mcs}")
    
    # Example components (replace with paper's actual formula):
    reward_data_rate = achieved_data_rate * 0.1 # Encourage higher data rate
    
    # Penalty for high power (example)
    reward_power_efficiency = - (chosen_tx_power / 30.0) * 0.5 
    
    # SNR quality (example: aim for a target SNR, e.g., 20 dB)
    target_snr = 20.0
    snr_penalty = -abs(snr - target_snr) * 0.05
    
    # CBR penalty (example: keep CBR low, e.g., below 0.6)
    cbr_target = 0.6
    cbr_penalty_val = 0
    if cbr > cbr_target:
        cbr_penalty_val = -(cbr - cbr_target) * 1.0
        
    # Power stability (example: penalize large changes)
    power_change_penalty = -abs(chosen_tx_power - prev_tx_power) * 0.02
    
    # MCS stability (example: penalize MCS changes)
    mcs_change_penalty = -abs(mcs_idx - prev_mcs_idx) * 0.1 if prev_mcs_idx is not None else 0
    
    # TODO: Ensure this reward accurately reflects the paper's objectives.
    # The current SNR, CBR, etc., are pre-action. Ideally, reward is based on post-action outcomes.
    reward = reward_data_rate + reward_power_efficiency + snr_penalty + cbr_penalty_val + power_change_penalty + mcs_change_penalty
    
    debug_reward_components = {
        "reward_total": reward,
        "reward_data_rate": reward_data_rate,
        "reward_power_efficiency": reward_power_efficiency,
        "snr_penalty": snr_penalty,
        "cbr_penalty": cbr_penalty_val,
        "power_change_penalty": power_change_penalty,
        "mcs_change_penalty": mcs_change_penalty
    }
    log_data(LOG_DEBUG_REWARD_PATH, json.dumps(debug_reward_components, indent=4))
    return float(reward)


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Agent Initialization ---
# State: [current_tx_power, current_mcs_idx, cbr_value, num_neighbors, snr_value]
STATE_DIM = 5
# Action: [tx_power_control, mcs_control] (both continuous, then scaled/discretized)
ACTION_DIM = 2

agent = PPOAgent(
    state_dim=STATE_DIM,
    action_dim=ACTION_DIM,
    lr_actor=LR_ACTOR,
    lr_critic=LR_CRITIC,
    gamma=GAMMA,
    clip_epsilon=CLIP_EPSILON,
    ppo_epochs=PPO_EPOCHS,
    batch_size=PPO_BATCH_SIZE, # This is mini-batch size for PPO updates
    gae_lambda=GAE_LAMBDA,
    ent_coef=ENT_COEF,
    vf_coef=VF_COEF,
    hidden_dim=256 # Default from ppo_agent
)

# Load existing models if they exist
actor_model_path = os.path.join(MODEL_DIR, "ppo_actor.pth")
critic_model_path = os.path.join(MODEL_DIR, "ppo_critic.pth")

if os.path.exists(actor_model_path) and os.path.exists(critic_model_path):
    agent.load_models(actor_model_path, critic_model_path)
    logging.info("Loaded existing PPO actor and critic models.")
else:
    logging.info("Initialized new PPO model.")


# --- Socket Communication Setup ---
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((SERVER_HOST, SERVER_PORT))
server.listen(1)
logging.info(f"PPO Training Server: Listening on port {SERVER_PORT}...")
conn, addr = server.accept()
conn.settimeout(SOCKET_TIMEOUT) # Set socket timeout
logging.info(f"PPO Training Server: Connected by {addr}")

# --- Training Loop ---
batch_counter = 0
total_transitions_collected_since_last_train = 0
# Store the next_state of the absolute last transition before buffer becomes full
# This is needed for the PPO agent's train() method's last_value_estimate_for_rollout
# Initialize with a dummy state or handle carefully if buffer is empty at first train call
last_overall_next_state = np.zeros(STATE_DIM) 

# Keep track of previous power and mcs for each vehicle for reward calculation
vehicle_previous_tx_power = {}
vehicle_previous_mcs_idx = {}


try:
    while True:
        try:
            data = conn.recv(SOCKET_BUFFER_SIZE)
        except socket.timeout:
            logging.warning(f"Socket timeout after {SOCKET_TIMEOUT}s. No data received. Waiting for new connection or data...")
            # Option: try to re-accept connection or simply continue waiting if client auto-reconnects
            # For simplicity, we might break here or try to re-accept if client is expected to disconnect/reconnect
            # For now, let's assume client might send data again or a new client connects after old one drops.
            # If client is gone, the script might idle here or eventually be stopped manually.
            # Consider a more robust re-connection logic if needed.
            # Re-accepting logic (simplified example, might need more robust handling):
            # conn.close()
            # logging.info("PPO Training Server: Listening again for new connection...")
            # conn, addr = server.accept()
            # conn.settimeout(SOCKET_TIMEOUT)
            # logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue # Continue to the next iteration to try recv() again or handle a new connection if re-accept is added
        except ConnectionResetError:
            logging.warning("Connection reset by peer. Waiting for new connection...")
            # Potentially try to re-accept or break
            conn.close()
            logging.info("PPO Training Server: Listening again for new connection...")
            conn, addr = server.accept()
            conn.settimeout(SOCKET_TIMEOUT)
            logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue

        if not data:
            logging.info("No data received (client disconnected gracefully?), closing this connection. Waiting for new one.")
            conn.close()
            logging.info("PPO Training Server: Listening again for new connection...")
            conn, addr = server.accept()
            conn.settimeout(SOCKET_TIMEOUT)
            logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue
        
        batch_counter += 1
        try:
            batch_vehicle_data = json.loads(data.decode())
            log_data(LOG_RECEIVED_PATH, f"Batch {batch_counter}: {json.dumps(batch_vehicle_data, indent=4)}")
            logging.info(f"Received data for PPO batch {batch_counter} ({len(batch_vehicle_data)} vehicles):
{json.dumps(batch_vehicle_data, indent=2)}")

            responses = {}

            for veh_id, vehicle_data in batch_vehicle_data.items():
                # --- 1. Extract Current State ---
                # If it's the first time we see this vehicle, initialize previous values
                # The state includes the *previous* action's outcome (power and MCS)
                # If first step, use current reported values as "previous" for state representation
                prev_tx_power = vehicle_previous_tx_power.get(veh_id, vehicle_data['transmissionPower'])
                prev_mcs_idx = vehicle_previous_mcs_idx.get(veh_id, vehicle_data['MCS'])
                
                cbr = vehicle_data['CBR']
                neighbors = vehicle_data['neighbors']
                snr = vehicle_data['SNR']
                
                # State: [current_tx_power, current_mcs_idx, cbr_value, num_neighbors, snr_value]
                current_state = np.array([prev_tx_power, prev_mcs_idx, cbr, neighbors, snr], dtype=np.float32)

                # --- 2. Agent Selects Action ---
                # Action from PPO agent is (raw_action_values, log_prob, value_estimate)
                # raw_action_values are continuous, typically from a Gaussian distribution
                raw_action, log_prob_old, value_old = agent.select_action_for_rollout(current_state)
                
                # --- 3. Scale and Discretize Actions ---
                # Action 0: Transmission Power (e.g., scale to [1, 30] dBm)
                # Raw action is in range [-1, 1] due to tanh in policy_net.sample
                # (val + 1) / 2 maps [-1, 1] to [0, 1]
                # Then scale to [min_power, max_power]
                # Using constants defined at the top
                chosen_tx_power = (raw_action[0] + 1) / 2 * (MAX_TX_POWER - MIN_TX_POWER) + MIN_TX_POWER
                chosen_tx_power = np.clip(chosen_tx_power, MIN_TX_POWER, MAX_TX_POWER)

                # Action 1: MCS Index (e.g., scale to [0, 7])
                # Using constants defined at the top
                continuous_mcs_val = (raw_action[1] + 1) / 2 * (MAX_MCS_IDX + 1 - MIN_MCS_IDX) + MIN_MCS_IDX - 0.5
                chosen_mcs_idx = int(np.round(np.clip(continuous_mcs_val, MIN_MCS_IDX -0.49, MAX_MCS_IDX + 0.49)))
                chosen_mcs_idx = np.clip(chosen_mcs_idx, MIN_MCS_IDX, MAX_MCS_IDX)


                action_details_log = {
                    "veh_id": veh_id,
                    "raw_action": [float(a) for a in raw_action],
                    "chosen_tx_power": float(chosen_tx_power),
                    "chosen_mcs_idx": int(chosen_mcs_idx)
                }
                log_data(LOG_DEBUG_ACTION_PATH, json.dumps(action_details_log))

                # --- 4. Simulate Environment / Get Outcomes ---
                # For simulation, chosen_tx_power and chosen_mcs_idx are applied.
                # SNR might change based on new power, environment might provide new CBR, neighbors.
                # For now, we assume the next 'snr', 'cbr', 'neighbors' will come from the *next* batch of data for this vehicle.
                # The 'achieved_data_rate' and 'sr_at_mcs' depend on 'chosen_mcs_idx'.
                achieved_data_rate, sr_at_mcs = get_data_rate_and_sr(chosen_mcs_idx)

                # --- 5. Calculate Reward ---
                # The reward depends on the outcomes of the chosen actions.
                # We use 'prev_tx_power' and 'prev_mcs_idx' from the *start* of this step for stability terms in reward.
                reward = calculate_reward_datarate_power_paper(
                    achieved_data_rate=achieved_data_rate,
                    chosen_tx_power=chosen_tx_power,
                    snr=snr, # This is SNR before action, ideally, we'd get SNR *after* action
                    cbr=cbr, # This is CBR before action
                    num_neighbors=neighbors,
                    mcs_idx=chosen_mcs_idx,
                    prev_tx_power=prev_tx_power, 
                    prev_mcs_idx=prev_mcs_idx, # This was from vehicle_data or previous step
                    sr_at_mcs=sr_at_mcs
                )
                
                write_performance_metrics(cbr, snr, chosen_mcs_idx, achieved_data_rate, chosen_tx_power, reward, batch_counter, len(agent.rollout_buffer) ) # Using len() directly


                # --- 6. Define Next State ---
                # TODO: CRITICAL - 'next_state_for_buffer' uses cbr, neighbors, snr from the *current* observation period.
                # Ideally, these should be the values *after* the action is applied in the environment.
                # This might require changes to the simulation client or a more complex state update mechanism
                # if true post-action states are only available in the *next* batch from the client.
                # For now, this is a known simplification.
                next_state_for_buffer = np.array([chosen_tx_power, chosen_mcs_idx, cbr, neighbors, snr], dtype=np.float32)
                # This `next_state_for_buffer` will become `current_state` for this vehicle in the next time step it sends data.
                
                last_overall_next_state = next_state_for_buffer # Keep track for PPO train

                # --- 7. Store Transition ---
                # `done` is usually False in continuous V2V scenarios unless an episode explicitly ends.
                # TODO: If your environment has natural episode terminations (e.g., vehicle disconnects, task completion),
                # set `done = True` here. This will allow GAE to correctly handle terminal states
                # and avoid bootstrapping from V(s_terminal), which should be 0.
                done = False 
                # The `raw_action` is stored, as PPO needs to evaluate it with the current policy.
                agent.store_transition(current_state, raw_action, reward, next_state_for_buffer, done, log_prob_old, value_old)
                total_transitions_collected_since_last_train +=1

                # Update previous action trackers for this vehicle for the *next* step
                vehicle_previous_tx_power[veh_id] = chosen_tx_power
                vehicle_previous_mcs_idx[veh_id] = chosen_mcs_idx
                
                # --- 8. Prepare Response ---
                responses[veh_id] = {
                    "transmissionPower": float(chosen_tx_power),
                    "MCS": int(chosen_mcs_idx) 
                    # "beaconRate" is not controlled by this PPO agent
                }

            # --- 9. PPO Training Step (if buffer is full) ---
            if total_transitions_collected_since_last_train >= NUM_ROLLOUT_STEPS:
                logging.info(f"Collected {total_transitions_collected_since_last_train} transitions. Starting PPO training...")
                
                # Estimate value of the very last 'next_state' encountered in the rollout
                # This is 'last_overall_next_state'
                # Must ensure last_overall_next_state is a valid np array for FloatTensor
                if not isinstance(last_overall_next_state, np.ndarray) or last_overall_next_state.shape != (STATE_DIM,):
                     # Fallback or error if last_overall_next_state is not properly formed
                    logging.warning(f"last_overall_next_state is not a valid array: {last_overall_next_state}. Using zero vector.")
                    final_s_tensor = torch.FloatTensor(np.zeros(STATE_DIM)).unsqueeze(0).to(agent.device)
                else:
                    final_s_tensor = torch.FloatTensor(last_overall_next_state).unsqueeze(0).to(agent.device)

                with torch.no_grad():
                    last_value_est = agent.value_net(final_s_tensor).cpu().item()
                
                # If the (conceptual) episode ended with the last transition, last_value_est should be 0.
                # Assuming 'done' is always False for now for simplicity in this continuous task.
                # If agent.rollout_buffer.dones[-1] is True, GAE handles it.
                # This estimate is for V(S_T) if the rollout truncates mid-episode.
                # The PPO agent's GAE calculation will use 0 if the internal dones flag for the last step is True.
                
                agent.train(last_value_estimate_for_rollout=last_value_est)
                logging.info("PPO training finished for this rollout.")
                total_transitions_collected_since_last_train = 0 # Reset counter
                
                # Save models periodically after training
                agent.save_models(actor_model_path, critic_model_path)


            # Send responses for the batch
            if responses:
                response_data = json.dumps(responses).encode('utf-8')
                conn.sendall(response_data)
                formatted_response = json.dumps(responses, indent=2)
                log_data(LOG_SENT_PATH, f"Batch {batch_counter}: {formatted_response}")
                logging.info(f"Sent PPO RL response to client for batch {batch_counter}")
            else:
                logging.info(f"No vehicles in batch {batch_counter} or no responses generated.")

        except json.JSONDecodeError as e:
            logging.error(f"JSON Decode Error: {e}. Data received: {data[:200]}...") # Log first 200 chars
            # conn.sendall(json.dumps({"error": "Malformed JSON received"}).encode('utf-8')) # Optional: inform client
        except Exception as e:
            logging.error(f"Error processing PPO batch {batch_counter}: {e}", exc_info=True)
            # If error during processing one vehicle, try to continue with others or next batch.
            # Consider sending error response for the specific vehicle or batch.
            # For now, we continue to the next batch cycle.
            continue

finally:
    # Save model one last time upon exit/error
    logging.info("Attempting to save PPO models on exit...")
    agent.save_models(actor_model_path, critic_model_path)

    conn.close()
    server.close()
    logging.info("PPO Training Server closed.") 