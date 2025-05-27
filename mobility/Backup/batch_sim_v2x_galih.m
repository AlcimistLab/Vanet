%% SNR Calculation



%% Calculate SNR from the power transmit of the neighbors nodes
% Define neighbors with P_tx (dBm) and distances (meters)
neighbors = [struct('P_tx', 20, 'distance', 50), ...
             struct('P_tx', 18, 'distance', 100), ...
             struct('P_tx', 15, 'distance', 150)];

% Parameters for SNR calculation
bandwidth = 20e6; % 20 MHz
noise_figure = 5; % Noise figure in dB
TX.P_tx             = 20;       % Transmit power in dBm
channel_model = 'rayleigh'; % Select channel model (can also be 'rician' or 'log-normal')

% Calculate SNR values based on neighbors and include TX.P_tx
SNR_values = calculate_snr_from_neighbors(neighbors, bandwidth, noise_figure, channel_model, TX.P_tx);

% Display calculated SNR values
disp('Calculated SNR values (dB):');
disp(SNR_values);

% Debugging output for the own node's SNR
fprintf('Own Node SNR (dB): %.2f\n', SNR_values(end));

% Use calculated SNR values for the simulation
SIM.snr = SNR_values(end); % Use own node's calculated SNR

%% Initialization
%clc
%clear all
%close all

addpath('./functions;./ext;./mex')

% Set random number generator to specific seed
rand_stream = RandStream('mt19937ar', 'Seed', 0);
RandStream.setGlobalStream(rand_stream);

%% Simulation Parameters
% General parameters
%SIM.mcs_vec         = 0:10;     % Scalar or vector containing MCS values (0...10)
SIM.mcs_vec          = 5;
%SIM.snr             = -5:5:35;    % Scalar or vector containing SNR values (dB)
%SIM.snr             = 40;
SIM.ovs             = 1;        % Oversampling factor
SIM.channel_model   = 1;        % Channel model (0: AWGN, 1-5: C2C models R-LOS, UA-LOS, C-NLOS, H-LOS and H-NLOS, 6-10: Enhanced C2C models R-LOS-ENH, UA-LOS-ENH, C-NLOS-ENH, H-LOS-ENH and H-NLOS-ENH)
SIM.use_mex         = false;    % Use MEX functions to accelerate simulation
SIM.n_iter          = 1;     	% Number of Monte-Carlo iterations
SIM.max_error       = 100;      % Number of packet errors before moving to next SNR point
SIM.min_error       = .005;     % Minimum PER target, beyond which, loop moves to next SNR point
SIM.check_sp        = false;    % Plot Tx spectrum and check for compliance
SIM.apply_cfo       = false;    % Apply CFO impairment on Tx and Rx

% Transmitter parameters
%TX.payload_len      = 350;     % MPDU_LENGTH / PHY payload length (bytes) (without A-MPDU header and without MAC padding)
TX.window_en        = false;    % Apply time-domain windowing
TX.w_beta           = 0;        % Kaiser window beta coefficient for spectral shaping (use "0" for disabling this filter)
TX.pa_enable        = false;    % Apply PA non-linearity model
TX.pn_en            = false;    % Model Tx phase noise
TX.bw_mhz           = 20;       % Bandwidth (10 or 20 MHz)



% Candidate NGV features
TX.ppdu_fmt         = 2;        % PPDU format (1: Legacy, 2: NGV)
TX.mid              = 4;        % NGV only: midamble period 4/8/16 symbols
TX.n_ss             = 1;        % NGV only: Number of MIMO spatial streams (1..2), 2 is not yet supported
TX.ltf_fmt_init     = 0;        % NGV only: LTF format for MCS 0-9. (0: NGV-LTF-2x (8 us, default), 1: compressed NGV-LTF-1x (4.8 us))
% Note: MCS 10 always uses LTF format 2, NGV-LTF-repeat (14.4 us), but this will be set automatically
TX.n_tx_ant         = 1;        % Number of transmit antennas

% Receiver parameters
RX.n_rx_ant         = 1;        % Number of receive antennas
RX.bw_mhz           = TX.bw_mhz; % Bandwidth of the received waveform
RX.pdet_thold_def   = 20;       % Packet detection threshold
RX.t_depth          = 2;        % Channel tracking time depth averaging (OFDM symbols)
RX.pn_en            = false;    % Model Rx phase noise
RX.ldpc_cfg.iter    = 50;       % Number of iterations
RX.ldpc_cfg.minsum  = 1;        % Only for external decoder 0: Sum-product (gives .5 - 1dB gain) 1: min-sum algorithm

% Initialize report structure
report = struct('P_tx', [], 'MCS', [], 'SNR', [], 'PER', [], 'BER', [], ...
                'Throughput', [], 'Latency', []);


error_char_vec = '.xo!uhnf!'; % For printing receiver message decoding results
% 0 => .   no error
% 1 => x   pdet error
% 2 => o   SIG error
% 3 => !   data decoding error
% 4 => u   undetected PHY parameter error (SIG/NGV-SIG)
% 5 => h   heuristic error
% 6 => n   NGV-SIG error
% 7 => f   format / autodetection error (NGV and legacy formats confused)
% 8 => !   A-MPDU header error (part of the data field)

ppdu_descriptor = {'p', 'bd'};

tic

%% Define The String To Be send
% Define Input String
input_string = 'Hello, this is a test message!'; % Example string to transmit

% Convert String to Bits
TX.bits = reshape(dec2bin(input_string, 8).' - '0', 1, []); % Convert string to bits

% Dynamically Set Payload Length
TX.payload_len = length(TX.bits) / 8; % Number of bytes in the payload

% Ensure payload length is valid
if mod(length(TX.bits), 8) ~= 0
    error('Input bits length must be a multiple of 8.');
end

% Debug Information
fprintf('Input String: %s\n', input_string);
fprintf('Transmitting %d bits.\n', length(TX.bits));

%% End of String Definition

if (TX.ppdu_fmt == 1 && TX.bw_mhz == 20)
    disp('Warning: non-usual 20 MHz transmissions with 802.11p (4 us symbols, 0.8 us GI, 312.5 kHz subcarrier spacing)')
    % Same PHY format as regular 802.11a 20 MHz. There is only a single 20 MHz wide (sub-)channel
    TX.n_chan = 1;
    RX.n_chan = 1;
else
    % Default for regular 10 MHz 802.11p and for 10/20 MHz 802.11bd, with 156 kHz subcarrier spacing
    % The number of adjacent subchannels (1 or 2) is determined by the bandwidth (10 or 20)
    % TX and RX may be different (Receiver in 20 MHz mode must also decode a 10 MHz PPDU in one subchannel)
    TX.n_chan = TX.bw_mhz/10;
    RX.n_chan = RX.bw_mhz/10;
end

% Check if midamble peridocity setting is valid
if (TX.ppdu_fmt == 1)
    TX.mid = 0;
else
    if ( ~any(TX.mid == [4 8 16]) )
        error('802.11bd Midamble periodicity (M) should be 4, 8, or 16')
    end
end

%% Loop for MCS values
avgPER = zeros(length(SIM.snr), length(SIM.mcs_vec)); % Initialize avgPER properly
latency_vs_per = zeros(length(SIM.snr), length(SIM.mcs_vec)); % Initialize latency_vs_per properly
avgTHR = zeros(length(SIM.snr), length(SIM.mcs_vec)); % Initialize avgTHR properly

for i_mcs = 1:length(SIM.mcs_vec)
    
    % Current MCS value
    TX.mcs = SIM.mcs_vec(i_mcs);
    if (TX.ppdu_fmt == 1  && TX.mcs > 7 ) ...
            || (TX.ppdu_fmt == 2  && TX.mcs == 9 && TX.bw_mhz == 10 )
        % MCS not supported
        continue
    end
    
    if (TX.ppdu_fmt == 2 && (TX.mcs == 0 || TX.mcs == 10))
        % MCS 0 (BPSK) and MCS 10 (BPSK-DCM) require a 3 dB power boost for STF and LTF
        % The packet detection does not yet implement gain control based on STF/LTF power and does not work properly
        % Workaround: Increase RX pdet threshold to compensate for power boost
        RX.pdet_thold = RX.pdet_thold_def*sqrt(2);
        % TODO: Implement more realistic pdet and remove this workaround
    else
        RX.pdet_thold = RX.pdet_thold_def;
    end
    
    % Initialize channel filter object
    [chan_obj, chan_name] = chan_mod_init(SIM.channel_model, RX.bw_mhz, SIM.ovs, TX.n_tx_ant, RX.n_rx_ant);
    
    % Debugging message
    PHY0 = tx_phy_params('TX', TX.mcs, TX.payload_len, TX.ppdu_fmt, TX.mid, TX.n_ss, TX.ltf_fmt_init, TX.n_chan, 0, 0);
    fprintf('PHY0 Debugging: n_sd=%d, n_dbps=%d, n_cbps=%d, n_bpscs=%d\n', ...
    PHY0.n_sd, PHY0.n_dbps, PHY0.n_cbps, PHY0.n_bpscs);

    fprintf('PHY0 Debugging: n_sd=%d, n_dbps=%d, n_cbps=%d, n_bpscs=%d\n', ...
    PHY0.n_sd, PHY0.n_dbps, PHY0.n_cbps, PHY0.n_bpscs);

    
    %% Loop for SNR values
    avgPER = zeros(length(SIM.snr), 1);
    for i_snr = 1:length(SIM.snr)
        % Total number of transmissions at the current SNR level
        sum_trials = 0;
        % Number of packer errors at the current SNR level
        sum_error = 0;
        
        % Debugging message
        fprintf('\nSNR: %4.1f dB ', SIM.snr(i_snr))
        
        %% Loop for Monte-Carlo iterations
        for i_iter = 1:SIM.n_iter
            % Fix random seed to allow reproduceability
            rand_seed = i_iter;
            reset(rand_stream, rand_seed);

            % Map Bits to Symbols
            q = 2; % Modulation order (QPSK as an example)
            % Define MCS-to-modulation-order mapping
            %mcs_to_q = [2, 4, 4, 16, 16, 64, 64, 256, 256, 1024]; % Example
            %q = mcs_to_q(SIM.mcs_vec);

            symbols = mapper_tx(TX.bits, q); % Map bits to symbols
            
            % Simulate channel effects (optional, if applicable)
            tx_wf = apply_channel_effects(symbols, SIM.channel_model); % Apply channel effects

                      
            % Add CFO error, assume [-5, 5] ppm per Tx/Rx device
            if SIM.apply_cfo
                cfo_err = sum(rand(2, 1) - .5)*10e-6;
                tx_wf = apply_cfo(tx_wf, cfo_err);
            else
                cfo_err = 0;
            end
            
            % Apply Tx phase noise
            tx_wf = add_tx_pn(tx_wf, TX.pn_en);
            
            % Mandatory 2x upsampling and frequency shift if transmitting a 10 MHz signal in 20 MHz receive mode
            % TODO: numerous adaptions at the receiver (in pdet, fine_sync, ...) to support this
            if (RX.bw_mhz == 20 && TX.bw_mhz == 10)
                [tx_wf, ovs_filt_len1] = upsample_tx(tx_wf, 2);
                % Shift into lower 10 MHz subchannel
                tx_wf = tx_wf .* exp(-1j*2*pi*(1:length(tx_wf))/128*32).';
            end
            
            % Optional oversampling of the tranmitted waveform
            [tx_wf, ovs_filt_len] = upsample_tx(tx_wf, SIM.ovs);
            
            % Apply a memoryless nonlinearity model of the power amplifier
            tx_wf = pa_model(tx_wf, TX.pa_enable);
            
            % Evaluate the PSD and check for compliance
            check_sp_mask(tx_wf, ovs_filt_len, PHY0.n_sym, SIM.ovs, SIM.check_sp);
            
            % Append silence samples at the beginning/end of useful waveform
            s0_len = randi([100 200]);
            tx_wf_full = [zeros(s0_len*SIM.ovs, TX.n_tx_ant); tx_wf; zeros((400 - s0_len)*SIM.ovs, TX.n_tx_ant)];
            
            % If channel model is defined, pass transmitted signal through channel filter
            if (SIM.channel_model == 0)
                % Enable multi-antenna modes for AWGN channel
                if TX.n_tx_ant == 2 && RX.n_rx_ant > 1
                    vec2 = [1 -1 1 1 ]; % entry i denotes the channel from TX antenna 2 to receive antenna i
                    rx_wf = repmat(tx_wf_full(:,1), 1, RX.n_rx_ant) + repmat(tx_wf_full(:,2), 1, RX.n_rx_ant).*vec2(1:RX.n_rx_ant);
                else
                    rx_wf = repmat(tx_wf_full, 1, RX.n_rx_ant);
                end
            else
                reset(chan_obj);
                rx_wf = step(chan_obj, tx_wf_full);
            end
            
            % Optional downsampling of the received waveform
            rx_wf = downsample_rx(rx_wf, SIM.ovs, ovs_filt_len);
            
            % Apply Rx phase noise
            rx_wf = add_rx_pn(rx_wf, RX.pn_en);
            
            % Add AWGN noise
            rx_wf = awgn(rx_wf, SIM.snr(i_snr));

            if ~exist('data_f_mtx', 'var') || isempty(data_f_mtx)
                data_f_mtx = []; % Placeholder if not provided
                warning('data_f_mtx is missing. Using a default empty array.');
            end

            
            % Receiver model (MEX or M)
            if SIM.use_mex
                err = sim_rx_mex(rx_wf, data_f_mtx, RX, TX, s0_len);
            else
                err = sim_rx(rx_wf, data_f_mtx, RX, TX, s0_len);
                % Receiver Side
                rx_symbols = tx_wf; % Assuming channel introduces no errors for simplicity
                received_bits = demodulate_rx(rx_symbols, q); % Demodulate received symbols
                
                % Convert Bits Back to String
                received_string = char(bin2dec(char(reshape(received_bits + '0', 8, []).'))).';
                
                % Compare Results
                bit_errors = sum(received_bits ~= TX.bits);
                BER = bit_errors / length(TX.bits);
                
                % Store BER and Display Results
                if ~strcmp(received_string, input_string)
                    warning('Received string does not match transmitted string!');
                else
                    disp('Received string matches transmitted string.');
                end
                fprintf('Received String: %s\n', received_string);

                fprintf('Bit Error Rate (BER): %.4f\n', BER);

            end
            
            % Receiver Side
            rx_symbols = tx_wf; % Assuming channel introduces no errors for simplicity
            received_bits = demodulate_rx(rx_symbols, q); % Demodulate received symbols
            
            % Convert Bits Back to String
            received_string = char(bin2dec(char(reshape(received_bits + '0', 8, []).'))).';
            
            % Compare Results
            if strcmp(received_string, input_string)
                err = 0; % No error if strings match
            else
                err = 1; % Error otherwise
            end
            
            % Update Trials and Errors
            sum_trials = sum_trials + 1;
            sum_error = sum_error + err;
            
            % Debugging Outputs
            fprintf('Debugging: i_iter=%d, err=%d, sum_error=%d, sum_trials=%d\n', ...
                    i_iter, err, sum_error, sum_trials);

            
            % Print error status character
            error_status = error_char_vec(err+1);
            fprintf(error_status);
            % Check if number of errors exceeds target
            if (sum_error >= SIM.max_error)
                break
            end
            
            pause(.001);
        end
        
        avgPER(i_snr, i_mcs) = sum_error / sum_trials; % Update avgPER with valid indexing
        % Calculate latency based on PHY configuration
        calc_latency = (TX.payload_len * 8) / (PHY0.n_sd * PHY0.n_dbps / PHY0.n_cbps * PHY0.n_bpscs / 8); % in microseconds
        % Calculate PER
        PER = sum_error / sum_trials; % Packet Error Rate
        fprintf('Debugging PER: sum_trials=%d, sum_error=%d, PER=%.4f\n', sum_trials, sum_error, PER);

        
        % Calculate symbol time, frame time, and throughput
        % Correct Throughput Calculation
        bits_per_symbol = PHY0.n_bpscs; % Bits per subcarrier
        total_symbols = TX.payload_len * 8 / PHY0.n_dbps; % Total OFDM symbols for the payload
        symbol_time = 4e-6; % OFDM symbol duration in seconds (20 MHz bandwidth)
        frame_time = total_symbols * symbol_time; % Total frame time in seconds
        throughput = (TX.payload_len * 8 * (1 - PER)) / frame_time / 1e6; % Throughput in Mbps
        
        % Debugging Outputs
        fprintf('Debugging: bits_per_symbol=%d, total_symbols=%.2f, frame_time=%.2e\n', ...
                bits_per_symbol, total_symbols, frame_time);
        fprintf('Calculated Throughput: %.4f Mbps\n', throughput);

       


        
        % Calculate BER (requires bit_errors and total_bits variables in the simulation)
        % BER = bit_errors / total_bits; % Ensure bit_errors and total_bits are updated elsewhere
        

        % Correct throughput calculation
        symbol_time = 8e-6; % OFDM symbol duration in seconds for 10 MHz bandwidth
        total_symbols = TX.payload_len * 8 / PHY0.n_dbps; % Total OFDM symbols for payload
        frame_time = total_symbols * symbol_time; % Total frame time in seconds
        throughput = (TX.payload_len * 8) / frame_time / 1e6; % Throughput in Mbps
      
        % Calculate BER
        BER = PER / (TX.payload_len * 8); % Approximation: PER spread over all bits
        
        % Store results in the report
        report.P_tx = [report.P_tx; TX.P_tx]; % Ideal Tx power usage
        %report.P_tx = [report.P_tx; TX.pa_enable * 10^(TX.w_beta / 10)];
        report.MCS = [report.MCS; TX.mcs];
        report.SNR = [report.SNR; SIM.snr(i_snr)];
        report.PER = [report.PER; PER];
        report.BER = [report.BER; BER];
        report.Throughput = [report.Throughput; throughput];
        report.Latency = [report.Latency; calc_latency]; % Add latency from your calculations


        % Store latency and PER values
        latency_vs_per(i_snr, i_mcs) = calc_latency; % Store latency for the current SNR and MCS


        % Calculate latency based on PHY configuration
        calc_latency = (TX.payload_len * 8) / (PHY0.n_sd * PHY0.n_dbps / PHY0.n_cbps * PHY0.n_bpscs / 8); % in microseconds
        if i_mcs <= size(latency_vs_per, 2)
            latency_vs_per(i_snr, i_mcs) = calc_latency; % Ensure index is valid before assigning
        end
        % If PER drops below min_error, break SNR loop
        if (sum_error/sum_trials < SIM.min_error)
            break
        end
    end
    fprintf('\n')
    
    % Find throughput efficiency factor (affected by midamble periodicity and duration of the midamble symbols, which is 8, 4.8 or 14.4 us)
    if (TX.mid == 0)
        eff = 1;
    else
        eff = (TX.mid*8)/(TX.mid*8 + PHY0.n_ss * PHY0.t_ngvltf);
    end
    % Goodput calculation
    drate = eff*PHY0.n_sd*PHY0.n_dbps./PHY0.n_cbps.*PHY0.n_bpscs/8e-6*1e-6;
    avgTHR(:, i_mcs) = (1 - avgPER).*repmat(drate, size(avgPER, 1), 1);
    
    figure(SIM.channel_model + 1);
    % Plot PER
    subplot(1, 2, 1)
    title(sprintf('\nChannel %i (%s), M=%d', SIM.channel_model, chan_name, TX.mid));
    legend_string = sprintf('%s-MCS-%i (%s %d/%d)', ppdu_descriptor{TX.ppdu_fmt}, TX.mcs, n_bpscs_to_string(PHY0.n_bpscs),PHY0.r_num,PHY0.r_denom);
    semilogy(SIM.snr, avgPER, 'DisplayName', legend_string, 'LineWidth', 1.5);
    drawnow; xlabel('SNR (dB)'); ylabel('PER');
    grid on; legend('Location', 'SouthWest'); hold on;
    
    % Plot throughput
    subplot(1, 2, 2)
    title(sprintf('\nChannel %i (%s), M=%d', SIM.channel_model, chan_name, TX.mid));
    plot(SIM.snr, avgTHR(:, i_mcs), 'DisplayName', legend_string, 'LineWidth', 1.5);
    drawnow; xlabel('SNR (dB)'); ylabel('Throughput (Mbps)');
    grid on; legend('Location', 'NorthWest'); hold on;

    % Plot Latency
    % Debugging output to check sizes
    disp(SIM.mcs_vec);
    disp(size(avgPER));
    disp(size(latency_vs_per));
    % Plot Latency vs PER
    % Plot Latency vs PER
    figure(SIM.channel_model + 2);
    for i_mcs = 1:size(latency_vs_per, 2)
        % Check if the current MCS index has valid data
        if all(latency_vs_per(:, i_mcs) == 0) || all(avgPER(:, i_mcs) == 0)
            continue;
        end
        plot(latency_vs_per(:, i_mcs), avgPER(:, i_mcs), ...
            'DisplayName', sprintf('MCS-%i', SIM.mcs_vec(i_mcs)), 'LineWidth', 1.5);
        hold on;
    end
    xlabel('Latency (\mus)');
    ylabel('PER');
    title(sprintf('Latency vs PER (Channel %i)', SIM.channel_model));
    grid on;
    legend('Location', 'NorthWest');
    hold off;



end

% Plot overall throughput
plot(SIM.snr, max(avgTHR, [], 2), 'k', 'DisplayName', 'Overall', 'LineWidth', 2);

disp('Simulation Report:');
table_report = table(report.P_tx, report.MCS, report.SNR, report.PER, ...
                     report.BER, report.Throughput, report.Latency, ...
                     'VariableNames', {'P_tx', 'MCS', 'SNR', 'PER', 'BER', 'Throughput', 'Latency'});
disp(table_report);


toc

% Supporting Functions
function symbols = mapper_tx(bits, q)
    % Map bits to symbols using QPSK (as an example)
    if mod(length(bits), q) ~= 0
        error('Number of bits must be a multiple of modulation order (q).');
    end
    reshaped_bits = reshape(bits, q, []);
    decimal_values = bi2de(reshaped_bits.', 'left-msb');
    mod_table = [-1-1j, -1+1j, 1-1j, 1+1j] / sqrt(2); % QPSK constellation
    symbols = mod_table(decimal_values + 1).';
end

function tx_wf = apply_channel_effects(symbols, channel_model)
    % Simulate channel effects (simplified, no noise here for example)
    if strcmp(channel_model, 'rayleigh')
        fading = (randn(size(symbols)) + 1j * randn(size(symbols))) / sqrt(2);
        tx_wf = symbols .* fading; % Apply Rayleigh fading
    else
        tx_wf = symbols; % No channel effect applied
    end
end

function received_bits = demodulate_rx(symbols, q)
    % Demodulate symbols back to bits (QPSK as an example)
    mod_table = [-1-1j, -1+1j, 1-1j, 1+1j] / sqrt(2); % QPSK constellation

    % Ensure symbols is a column vector
    symbols = symbols(:);

    % Debugging: Display size of symbols and mod_table
    disp('Debugging Information:');
    disp(['Size of symbols: ', mat2str(size(symbols))]);
    disp(['Size of mod_table: ', mat2str(size(mod_table))]);

    % Calculate distances to constellation points
    distances = abs(symbols - mod_table); 

    % Find the closest constellation points
    [~, min_indices] = min(distances, [], 2); 

    % Convert indices back to decimal
    decimal_values = min_indices - 1; 

    % Convert to bits
    received_bits = de2bi(decimal_values, q, 'left-msb');
    received_bits = reshape(received_bits.', 1, []); % Flatten into a single vector
end

