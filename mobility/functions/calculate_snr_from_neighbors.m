function SNR = calculate_snr_from_neighbors(neighbors, bandwidth, noise_figure, channel_model, TX_P_tx, varargin)
    % calculate_snr_from_neighbors: Calculates SNR values from neighbor nodes' transmit power and distance.
    %
    % Parameters:
    %   neighbors: Array of structs with fields:
    %       - P_tx: Transmit power in dBm
    %       - distance: Distance to the receiver in meters
    %   bandwidth: Receiver bandwidth in Hz (e.g., 10e6 for 10 MHz)
    %   noise_figure: Receiver noise figure in dB
    %   channel_model: 'rayleigh', 'rician', or 'log-normal'
    %   TX_P_tx: Transmit power of the current node in dBm (optional, default is 0 dBm)
    %   varargin: Additional parameters:
    %       - For 'rician', specify K-factor (e.g., varargin{1} = K-factor)
    %       - For 'log-normal', specify shadowing standard deviation (e.g., varargin{1} = std_dev)
    %
    % Returns:
    %   SNR: Array of SNR values in dB for each neighbor node.

    % Constants
    k = 1.38e-23; % Boltzmann constant in J/K
    T = 290; % Standard temperature in Kelvin
    P_thermal_noise_dBm = 10 * log10(k * T * bandwidth * 1e3); % Thermal noise in dBm
    P_noise_dBm = P_thermal_noise_dBm + noise_figure; % Total noise power in dBm
    
    % Antenna gains (can be parameterized)
    G_t = 0; % Transmitter antenna gain in dB
    G_r = 0; % Receiver antenna gain in dB

    % Initialize SNR array
    num_neighbors = length(neighbors);
    SNR = zeros(1, num_neighbors + 1); % Add 1 slot for our own TX.P_tx
    
    % Loop through each neighbor to calculate received power
    for i = 1:num_neighbors
        % Extract neighbor properties
        P_tx = neighbors(i).P_tx; % Transmit power in dBm
        distance = neighbors(i).distance; % Distance in meters
        
        % Calculate path loss and fading
        switch channel_model
            case 'rayleigh'
                % Rayleigh fading
                fading = 10 * log10(abs(raylrnd(1))); % Fading in dB
            case 'rician'
                % Rician fading (requires K-factor as varargin{1})
                if isempty(varargin)
                    error('K-factor required for Rician channel model');
                end
                k_factor = varargin{1};
                fading = 10 * log10(abs(ricernd(sqrt(k_factor / (1 + k_factor)), sqrt(1 / (1 + k_factor)))));
            case 'log-normal'
                % Log-normal shadowing (requires shadowing std. dev. as varargin{1})
                if isempty(varargin)
                    error('Shadowing standard deviation required for log-normal channel model');
                end
                shadowing_std_dev = varargin{1};
                fading = normrnd(0, shadowing_std_dev); % Shadowing in dB
            otherwise
                error('Unsupported channel model: %s', channel_model);
        end
        
        % Free-space path loss (FSPL)
        frequency_hz = 5.9e9; % Operating frequency in Hz (e.g., 5.9 GHz)
        path_loss = 20 * log10(distance) + 20 * log10(frequency_hz) - 147.55; % Path loss in dB

        % Calculate received power in dBm (updated with antenna gains)
        P_rx = P_tx + G_t + G_r - path_loss - fading;

        % Calculate SNR in dB
        SNR(i) = P_rx - P_noise_dBm;
    end

    % Calculate SNR for our own node's TX_P_tx (if provided)
    if nargin >= 5 && ~isempty(TX_P_tx)
        % Assume distance = 1 meter for self-SNR calculation
        own_distance = 1; % Replace with actual distance if needed
        path_loss_own = 20 * log10(own_distance) + 20 * log10(frequency_hz) - 147.55;
        P_rx_own = TX_P_tx + G_t + G_r - path_loss_own; % Received power for own node
        SNR(end) = P_rx_own - P_noise_dBm; % Add own SNR to the array

        % Debugging output
        fprintf('Debugging: TX_P_tx = %.2f dBm, P_rx_own = %.2f dBm, SNR_own = %.2f dB\n', ...
                TX_P_tx, P_rx_own, SNR(end));
    else
        SNR(end) = NaN; % Mark as NaN if TX_P_tx is not provided
    end
end
