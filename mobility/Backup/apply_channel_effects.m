function tx_wf = apply_channel_effects(symbols, channel_model)
    % APPLY_CHANNEL_EFFECTS Simulate channel effects on transmitted symbols.
    % 
    % Parameters:
    %   symbols: Input transmitted symbols (complex vector).
    %   channel_model: Type of channel (0: AWGN, 1: Rayleigh, 2: Rician).
    % 
    % Returns:
    %   tx_wf: Channel-affected transmitted symbols.

    % Default noise power (can be adjusted based on SNR requirements)
    noise_power = 0.01; 
    
    switch channel_model
        case 0 % AWGN Channel
            tx_wf = symbols + sqrt(noise_power/2) * (randn(size(symbols)) + 1j*randn(size(symbols)));
            
        case 1 % Rayleigh Fading Channel
            h = (randn(size(symbols)) + 1j*randn(size(symbols))) / sqrt(2); % Rayleigh fading coefficient
            tx_wf = h .* symbols + sqrt(noise_power/2) * (randn(size(symbols)) + 1j*randn(size(symbols)));
            
        case 2 % Rician Fading Channel
            K = 10; % Rician K-factor (ratio of direct path power to scattered power)
            s = sqrt(K/(K+1)); % Direct path scaling factor
            sigma = sqrt(1/(2*(K+1))); % Scattered components scaling factor
            h = s + sigma * (randn(size(symbols)) + 1j*randn(size(symbols))); % Rician fading coefficient
            tx_wf = h .* symbols + sqrt(noise_power/2) * (randn(size(symbols)) + 1j*randn(size(symbols)));
            
        otherwise
            error('Unsupported channel model: %d', channel_model);
    end
end
