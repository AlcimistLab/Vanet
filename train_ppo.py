import socket
import torch
import numpy as np
import logging
import json
import os
import csv
import math
from datetime import datetime
from ppo_agent import PPOAgent  # Using PPOAgent

# --- Configuration Constants ---
SERVER_HOST = "localhost"
SERVER_PORT = 5001
SOCKET_BUFFER_SIZE = 65536
SOCKET_TIMEOUT = 10.0  # Seconds

MIN_TX_POWER_DBM = 1.0   # dBm
MAX_TX_POWER_DBM = 30.0  # dBm
MIN_MCS_IDX = 0
MAX_MCS_IDX = 7

# --- Physical & Environment Constants for Paper Formulas ---
BETA = 2.0               # Path-loss exponent (β)
NAKAGAMI_M = 3.0         # Nakagami-m shape parameter (m)
CARRIER_FREQ = 5.9e9     # Hz (IEEE 802.11p band)
LIGHT_SPEED = 3e8        # m/s
LAMBDA_WAVELENGTH = LIGHT_SPEED / CARRIER_FREQ # Renamed to avoid conflict with gae_lambda
A_CONST = (4 * math.pi / LAMBDA_WAVELENGTH) ** 2
BEACON_RATE = 10         # Hz (B_r)
SAFETY_DISTANCE = 100.0  # meters (d_s)
CBR_TARGET = 0.6         # x_T (MBL)
CBR_TOLERANCE = 0.025    # tolerance for bonus in reward
OMEGA_C = 2.0            # weight for congestion term
OMEGA_P = 0.25           # weight for reliability term
OMEGA_D = 0.1            # weight for data-rate penalty
OMEGA_E = 0.8            # exponent for data-rate penalty
M_PAYLOAD_BITS = 536 * 8 # M_payload in bits
BST_SYMBOLS = 22         # Basic Service Set symbols (N_bst)
TPS_SECONDS = 40e-6      # Physical Layer Convergence Procedure time (T_ps)


# --- PPO Hyperparameters ---
NUM_ROLLOUT_STEPS = 2048
PPO_EPOCHS = 10
PPO_BATCH_SIZE = 64
GAMMA = 0.99
GAE_LAMBDA = 0.95         # Note: GAE_LAMBDA is different from LAMBDA_WAVELENGTH
CLIP_EPSILON = 0.2
ENT_COEF = 0.01
VF_COEF = 0.5
LR_ACTOR = 3e-4
LR_CRITIC = 1e-3

# --- Logging & Model Paths ---
LOG_DIR = 'custom_ppo/logs/'
MODEL_DIR = 'custom_ppo/model/'
LOG_RECEIVED_PATH = os.path.join(LOG_DIR, 'receive_data_ppo.log')
LOG_SENT_PATH     = os.path.join(LOG_DIR, 'sent_data_ppo.log')
LOG_ACTION_PATH   = os.path.join(LOG_DIR, 'action_ppo.log')
LOG_REWARD_PATH   = os.path.join(LOG_DIR, 'reward_debug_ppo.log')
PERF_METRIC_PATH  = os.path.join(LOG_DIR, 'performance_metrics_ppo.csv')

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)

# --- Utility: Logging & Metrics ---
def log_data(path, data):
    # TODO: For performance, consider in-memory buffering for logs and flushing periodically,
    # or use an asynchronous logger instead of writing on every call.
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(path, 'a') as f:
        f.write(f"[{ts}] {data}\n")

def write_performance_metrics(batch, steps_in_buffer, cbr, snr_observed, mcs, data_rate, tx_power_dbm, reward):
    file_path = PERF_METRIC_PATH
    exists = os.path.exists(file_path)
    with open(file_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if not exists:
            writer.writerow(['Batch','Steps_In_Buffer','Timestamp','CBR_Calculated','SNR_Observed','MCS_Chosen','DataRate_Mbps','TxPower_dBm','Reward'])
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer.writerow([batch, steps_in_buffer, ts, cbr, snr_observed, mcs, data_rate, tx_power_dbm, reward])

# --- Unit Conversion Utilities ---
def dbm_to_watts(dbm):
    return 10**((dbm - 30) / 10)

def watts_to_dbm(watts):
    if watts <= 0:
        return -float('inf') # Or handle error appropriately
    return 10 * math.log10(watts) + 30

# --- Data-Rate, Sensitivity, and Cd Lookup ---
# TODO: Verify these Cd values or get them from a reliable source for your specific 802.11p setup.
# Cd = bits per OFDM symbol (data bits + coding bits) * useful subcarriers / total subcarriers * coding rate
# These are rough placeholders based on common 802.11p knowledge, actual values depend on specific PHY params.
MCS_PARAMETERS = [
    # (data-rate Mbps, sensitivity dBm, Approx. Coded Bits per Symbol Cd - NEEDS VERIFICATION)
    (3,   -85, 96),   # MCS 0: BPSK 1/2
    (4.5, -84, 144),  # MCS 1: BPSK 3/4
    (6,   -82, 192),  # MCS 2: QPSK 1/2
    (9,   -80, 288),  # MCS 3: QPSK 3/4
    (12,  -77, 384),  # MCS 4: 16-QAM 1/2
    (18,  -73, 576),  # MCS 5: 16-QAM 3/4
    (24,  -69, 768),  # MCS 6: 64-QAM 2/3 
    (27,  -68, 864)   # MCS 7: 64-QAM 3/4 
]

def get_mcs_params(mcs_idx):
    if 0 <= mcs_idx < len(MCS_PARAMETERS):
        return MCS_PARAMETERS[mcs_idx]
    logging.warning(f"MCS index {mcs_idx} out of range. Defaulting to index 0.")
    return MCS_PARAMETERS[0]

# --- Paper Formula Implementations ---
def r_cs(p_tx_watts, sr_watts):
    """Carrier-sense range (Eq.1) under Nakagami-m fading & one-slope PL."""
    if p_tx_watts <= 0 or sr_watts <= 0: return 0.0 # Avoid math errors
    # Ensure NAKAGAMI_M and BETA are appropriate for gamma function inputs
    term_gamma = math.gamma(NAKAGAMI_M + 1.0/BETA) / math.gamma(NAKAGAMI_M)
    return (term_gamma * (p_tx_watts * A_CONST / sr_watts)) ** (1.0/BETA)

def effective_capacity(Cd_bits_per_symbol):
    """Effective capacity in beacons/sec (Eq.2). Renamed from capacity to avoid conflict."""
    if Cd_bits_per_symbol <= 0: return 1.0 # Avoid division by zero, return a default high capacity (low impact on CBR)
    symbols = math.ceil((BST_SYMBOLS + M_PAYLOAD_BITS) / Cd_bits_per_symbol)
    return 1.0 / (Cd_bits_per_symbol * symbols * (1/CARRIER_FREQ * 1e-6) + TPS_SECONDS) # Assuming symbol duration part was missing, needs review against paper. Simplified T_sym=1/BW for Cd usage.
    # The paper's Eq.2 seems to define C_eff = 1 / (N_sym * T_sym + T_ps)
    # N_sym = ceil((N_bst + M_payload)/C_d)
    # T_sym is OFDM symbol duration, e.g. 4us for 802.11p. C_d is bits per OFDM symbol.
    # The previous `Cd * symbols` in denominator looked like bits*symbols, not time.
    # For now, let T_sym be very small or implicitly handled by Cd if Cd is bits/second for a symbol duration.
    # This part needs careful check against paper's definition of Cd and T_sym.
    # A common way: N_sym = ceil( (N_bst_bits + M_payload_bits) / Cd_bits_per_symbol ) -> number of symbols
    # T_total_symbols = N_sym * T_ofdm_symbol_duration. Then C_eff = 1 / (T_total_symbols + T_ps)
    # For now using a simplified placeholder based on your original `capacity` function structure for `compute_cbr`:
    # If Cd in compute_cbr is data_rate (bits/sec), then cap = data_rate (bits/sec)
    # Let's assume Cd given to compute_cbr is actually related to data rate directly for now or B_r / C_eff
    # This function might not be directly used if compute_cbr is given data rate. The paper is key.
    # Reverting to a structure closer to your `compute_cbr` that implies `cap` is `BEACON_RATE / C_eff_from_paper` if `BEACON_RATE` is in `1/s`
    # The paper is (2 * R_cs * rho * B_r) / C_eff. If C_eff is in bits/sec, and B_r is beacons/sec, then CBR is dimensionless.
    # The function capacity(Cd) in user code returned 1.0 / (Cd * symbols + tps)
    # If Cd is bits/symbol, symbols is num_symbols, tps is time. Denom needs to be time.
    # Let T_symbol be OFDM symbol duration (e.g., 4us for 802.11p standard for 10MHz channel, 8us for 20MHz)
    T_OFDM_SYMBOL = 4e-6 # Example for 10MHz channel. Adjust if necessary.
    if Cd_bits_per_symbol <= 0: return 1.0
    num_ofdm_symbols = math.ceil((BST_SYMBOLS + M_PAYLOAD_BITS) / Cd_bits_per_symbol)
    duration_payload_symbols = num_ofdm_symbols * T_OFDM_SYMBOL
    c_eff = 1.0 / (duration_payload_symbols + TPS_SECONDS) # This is effective packets/sec
    return c_eff # beacons/sec or packets/sec

def compute_cbr(p_tx_watts, Cd_bits_per_symbol, rho, sr_watts):
    """Channel Busy Ratio estimate (Eq.3)."""
    cs_val = r_cs(p_tx_watts, sr_watts)
    cap_eff_packets_sec = effective_capacity(Cd_bits_per_symbol) # Effective capacity in packets/sec
    if cap_eff_packets_sec <= 0: return 1.0 # Avoid division by zero, assume high CBR
    # The formula is 2 * R_cs * rho * B_r / C_eff
    # B_r is beacon rate (beacons/sec). C_eff from paper is effective channel capacity (packets/sec or bits/sec)
    # Assuming C_eff here is packets/sec, and B_r is also packets/sec (beaconing)
    cbr_val = (2 * cs_val * rho * BEACON_RATE) / cap_eff_packets_sec 
    return min(cbr_val, 1.0) # CBR is typically capped at 1.0

def calculate_reward_paper(p_tx_dbm, data_rate_mbps, calculated_cbr, sr_dbm):
    """Reward combining congestion, reliability, and data-rate penalty (Eqs.4–6)."""
    # 1) Congestion term g(CBR)
    g_cbr = -math.copysign(1.0, calculated_cbr - CBR_TARGET) * calculated_cbr # Paper uses x_k for CBR
    if abs(calculated_cbr - CBR_TARGET) <= CBR_TOLERANCE:
        g_cbr += 1.0 # Paper states: g(x_k) = ..., +1 if |x_k - x_T| <= tolerance. Your code used 10.
    # else: # Paper implies no penalty, just the first part if outside tolerance.
        # g_cbr -= 0.1 # This was from your previous version, paper might be different.

    term1_congestion = OMEGA_C * g_cbr

    # 2) Reliability at safety distance ds (Eq. 5)
    # R_k = P_tx,k - PL(d_s) - S_r,k  (all in dB or dBm)
    # PL(d_s) = 10 * log10 ( A * (d_s)^beta )
    path_loss_at_ds_db = 10 * math.log10(A_CONST * (SAFETY_DISTANCE ** BETA))
    # The paper has reward_reliability = omega_p * R_k if R_k >=0, else omega_p * R_k * factor (e.g. 10 for penalty)
    # Your code used -abs((sr + pl) - ptx). Let's use paper's R_k directly. P_tx is p_tx_dbm, S_r is sr_dbm.
    R_k = p_tx_dbm - path_loss_at_ds_db - sr_dbm
    # The paper (e.g. from Al-Shareeda et al. on adaptive beaconing) often has: if R_k < 0, heavily penalize.
    # Let's use a simpler form: encourage R_k to be positive. Penalty if negative.
    # Original reliability term was -abs(target_Prx - actual_Prx_at_ds)
    # target_Prx_at_ds = sr_dbm. actual_Prx_at_ds = p_tx_dbm - path_loss_at_ds_db.
    # So, -abs(sr_dbm - (p_tx_dbm - path_loss_at_ds_db)) = -abs(sr_dbm - p_tx_dbm + path_loss_at_ds_db)
    # This is equivalent to -abs(R_k) if R_k is defined as (p_tx - pl_ds) - sr
    # The user's code was: rel = -abs((sr + pl_ds) - p_tx). This is -abs(-(R_k)). So, -abs(R_k).
    term2_reliability = -OMEGA_P * abs(R_k)

    # 3) Data-rate penalty (Eq. 6)
    term3_datarate_penalty = -OMEGA_D * (data_rate_mbps ** OMEGA_E)

    total_reward = term1_congestion + term2_reliability + term3_datarate_penalty
    
    log_data(LOG_REWARD_PATH, json.dumps({
        'reward_total': total_reward,
        'term1_congestion (g_cbr)': term1_congestion,
        'g_cbr_raw': g_cbr,
        'calculated_cbr': calculated_cbr,
        'term2_reliability (R_k)': term2_reliability,
        'R_k_raw': R_k,
        'path_loss_at_ds_db': path_loss_at_ds_db,
        'term3_datarate_penalty': term3_datarate_penalty,
        'inputs': {'p_tx_dbm': p_tx_dbm, 'data_rate_mbps': data_rate_mbps, 'sr_dbm': sr_dbm}
    }, indent=2))
    return float(total_reward)

# --- Initialize PPO Agent ---
# State: [current_tx_power_dbm, current_data_rate_mbps, current_rho_density]
STATE_DIM = 3
ACTION_DIM = 2 # [delta_power_choice, delta_mcs_choice] or direct power/MCS choice

agent = PPOAgent(
    state_dim=STATE_DIM,
    action_dim=ACTION_DIM,
    lr_actor=LR_ACTOR,
    lr_critic=LR_CRITIC,
    gamma=GAMMA,
    clip_epsilon=CLIP_EPSILON,
    ppo_epochs=PPO_EPOCHS,
    batch_size=PPO_BATCH_SIZE,
    gae_lambda=GAE_LAMBDA,
    ent_coef=ENT_COEF,
    vf_coef=VF_COEF,
    hidden_dim=256
)

actor_model_path = os.path.join(MODEL_DIR, 'ppo_actor.pth')
critic_model_path = os.path.join(MODEL_DIR, 'ppo_critic.pth')
if os.path.exists(actor_model_path) and os.path.exists(critic_model_path):
    agent.load_models(actor_model_path, critic_model_path)
    logging.info('Loaded existing PPO models.')
else:
    logging.info('Initialized new PPO models.')

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Socket & Training Loop ---
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((SERVER_HOST, SERVER_PORT))
server.listen(1)
logging.info(f"PPO Training Server: Listening on port {SERVER_PORT}...")
conn, addr = server.accept()
conn.settimeout(SOCKET_TIMEOUT)
logging.info(f"PPO Training Server: Connected by {addr}")

batch_counter = 0
total_transitions_collected_this_rollout = 0
last_overall_next_state_for_gae = np.zeros(STATE_DIM) 

# Keep track of previous chosen TxPower (dBm) and MCS_idx for each vehicle for state construction
vehicle_previous_tx_power_dbm = {}
vehicle_previous_mcs_idx = {}

try:
    while True:
        try:
            data = conn.recv(SOCKET_BUFFER_SIZE)
        except socket.timeout:
            logging.warning(f"Socket timeout after {SOCKET_TIMEOUT}s. No data. Attempting to listen for new/re-connection.")
            conn.close() # Close old connection
            conn, addr = server.accept() # Wait for new connection
            conn.settimeout(SOCKET_TIMEOUT)
            logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue
        except ConnectionResetError:
            logging.warning("Connection reset by peer. Attempting to listen for new/re-connection.")
            conn.close()
            conn, addr = server.accept()
            conn.settimeout(SOCKET_TIMEOUT)
            logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue

        if not data:
            logging.info("Client disconnected gracefully. Attempting to listen for new/re-connection.")
            conn.close()
            conn, addr = server.accept()
            conn.settimeout(SOCKET_TIMEOUT)
            logging.info(f"PPO Training Server: Re-connected by {addr}")
            continue
        
        batch_counter += 1
        try:
            batch_vehicle_data = json.loads(data.decode())
            log_data(LOG_RECEIVED_PATH, f"Batch {batch_counter}: {json.dumps(batch_vehicle_data, indent=2)}")
            logging.info(f"Received data for PPO batch {batch_counter} ({len(batch_vehicle_data)} vehicles)")

            responses = {}

            for veh_id, vehicle_data in batch_vehicle_data.items():
                # --- 1. Extract/Construct Current State ---
                # State: [current_tx_power_dbm, current_data_rate_mbps, current_rho_density]
                # Use vehicle's reported current power/MCS if it's the first time we see it, or our last chosen values.
                prev_tx_power_dbm = vehicle_previous_tx_power_dbm.get(veh_id, float(vehicle_data.get('transmissionPower', MIN_TX_POWER_DBM)))
                prev_mcs_idx = vehicle_previous_mcs_idx.get(veh_id, int(vehicle_data.get('MCS', MIN_MCS_IDX)))
                prev_data_rate_mbps, _ = get_mcs_params(prev_mcs_idx) 
                
                # TODO: Client MUST send 'rho' (vehicle density) and 'observed_SNR'.
                # CBR is calculated, not directly taken from vehicle_data for state, but observed SNR is needed.
                current_rho = float(vehicle_data.get('rho', 0.001)) # Example default if not sent
                observed_snr_db = float(vehicle_data.get('SNR', 20.0)) # Example default
                # The paper's state might be slightly different, ensure this matches.
                # For example, if CBR is part of state instead of rho, it needs calculation first.
                # The user code states STATE_DIM = 3  # [p_tx, data_rate, rho]

                current_state = np.array([prev_tx_power_dbm, prev_data_rate_mbps, current_rho], dtype=np.float32)

                # --- 2. Agent Selects Action ---
                raw_action, log_prob_old, value_old = agent.select_action_for_rollout(current_state)
                
                # --- 3. Scale and Discretize Actions ---
                # Action 0: Choose new Transmission Power (dBm)
                chosen_tx_power_dbm = (raw_action[0] + 1) / 2 * (MAX_TX_POWER_DBM - MIN_TX_POWER_DBM) + MIN_TX_POWER_DBM
                chosen_tx_power_dbm = np.clip(chosen_tx_power_dbm, MIN_TX_POWER_DBM, MAX_TX_POWER_DBM)

                # Action 1: Choose new MCS Index
                # Scale continuous action to [MIN_MCS_IDX - 0.5, MAX_MCS_IDX + 0.5] then round and clip
                continuous_mcs_val = (raw_action[1] + 1) / 2 * (MAX_MCS_IDX + 1 - MIN_MCS_IDX) + MIN_MCS_IDX - 0.5
                chosen_mcs_idx = int(np.round(np.clip(continuous_mcs_val, MIN_MCS_IDX -0.49, MAX_MCS_IDX + 0.49)))
                chosen_mcs_idx = np.clip(chosen_mcs_idx, MIN_MCS_IDX, MAX_MCS_IDX)
                
                action_details_log = {
                    "veh_id": veh_id,
                    "current_state_approx": [float(x) for x in current_state],
                    "raw_action": [float(a) for a in raw_action],
                    "chosen_tx_power_dbm": float(chosen_tx_power_dbm),
                    "chosen_mcs_idx": int(chosen_mcs_idx)
                }
                log_data(LOG_ACTION_PATH, json.dumps(action_details_log, indent=2))

                # --- 4. Get Parameters for Chosen Action & Calculate CBR ---
                new_data_rate_mbps, new_sr_dbm, new_Cd_bits_per_symbol = get_mcs_params(chosen_mcs_idx)
                
                chosen_tx_power_watts = dbm_to_watts(chosen_tx_power_dbm)
                new_sr_watts = dbm_to_watts(new_sr_dbm)
                
                # TODO: Ensure 'current_rho' is the correct density for CBR calculation based on paper.
                calculated_cbr = compute_cbr(chosen_tx_power_watts, new_Cd_bits_per_symbol, current_rho, new_sr_watts)

                # --- 5. Calculate Reward ---
                # The reward uses parameters resulting from the chosen action.
                # SNR for reward: The paper might imply an SNR model or use observed SNR if it reflects post-action quality.
                # For now, using the SNR observed from client for this step for reward calculation, as post-action SNR is not available.
                # This is a simplification noted in previous TODOs.
                reward = calculate_reward_paper(
                    p_tx_dbm=chosen_tx_power_dbm,
                    data_rate_mbps=new_data_rate_mbps,
                    calculated_cbr=calculated_cbr,
                    sr_dbm=new_sr_dbm
                )
                
                write_performance_metrics(batch_counter, len(agent.rollout_buffer) +1, calculated_cbr, observed_snr_db, chosen_mcs_idx, new_data_rate_mbps, chosen_tx_power_dbm, reward)

                # --- 6. Define Next State for Buffer ---
                # State: [tx_power_dbm, data_rate_mbps, rho_density]
                # TODO: CRITICAL - For next_state, 'rho' should ideally be the density *after* this step.
                # Using 'current_rho' is a simplification if post-action rho isn't immediately available.
                next_rho_for_state = current_rho # Simplification
                next_state_for_buffer = np.array([chosen_tx_power_dbm, new_data_rate_mbps, next_rho_for_state], dtype=np.float32)
                
                last_overall_next_state_for_gae = next_state_for_buffer

                # --- 7. Store Transition ---
                # TODO: Implement logic for 'done' if episodes can terminate.
                done = False 
                agent.store_transition(current_state, raw_action, reward, next_state_for_buffer, done, log_prob_old, value_old)
                total_transitions_collected_this_rollout +=1

                # Update previous action trackers for this vehicle for the *next* step's state construction
                vehicle_previous_tx_power_dbm[veh_id] = chosen_tx_power_dbm
                vehicle_previous_mcs_idx[veh_id] = chosen_mcs_idx
                
                # --- 8. Prepare Response ---
                responses[veh_id] = {
                    "transmissionPower": float(chosen_tx_power_dbm),
                    "MCS": int(chosen_mcs_idx)
                }

            # --- 9. PPO Training Step (if buffer is full enough) ---
            if total_transitions_collected_this_rollout >= NUM_ROLLOUT_STEPS:
                logging.info(f"Collected {total_transitions_collected_this_rollout} transitions. Starting PPO training for {PPO_EPOCHS} epochs...")
                
                final_s_tensor = torch.FloatTensor(last_overall_next_state_for_gae).unsqueeze(0).to(agent.device)
                with torch.no_grad():
                    last_value_est = agent.value_net(final_s_tensor).cpu().item()
                
                # The PPO agent's GAE calculation internally handles if the last actual stored 'done' flag in buffer is True.
                agent.train(last_value_estimate_for_rollout=last_value_est)
                logging.info("PPO training finished for this rollout.")
                total_transitions_collected_this_rollout = 0 # Reset counter for next rollout
                
                agent.save_models(actor_model_path, critic_model_path)

            # Send responses for the batch
            if responses:
                response_data = json.dumps(responses).encode('utf-8')
                conn.sendall(response_data)
                # formatted_response = json.dumps(responses, indent=2) # Verbose for local log
                # log_data(LOG_SENT_PATH, f"Batch {batch_counter}: {formatted_response}")
                # logging.info(f"Sent PPO RL response to client for batch {batch_counter}")
            # else:
                # logging.info(f"No vehicles in batch {batch_counter} or no responses generated.")

        except json.JSONDecodeError as e:
            logging.error(f"JSON Decode Error: {e}. Data received (first 200 chars): {data[:200]}...")
        except Exception as e:
            logging.error(f"Error processing PPO batch {batch_counter}: {e}", exc_info=True)
            continue

finally:
    logging.info("Attempting to save PPO models on controlled exit...")
    agent.save_models(actor_model_path, critic_model_path)
    if conn:
        conn.close()
    if server:
        server.close()
    logging.info("PPO Training Server closed.") 