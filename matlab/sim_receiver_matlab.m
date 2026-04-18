% DVB-S2X 64-APSK Receiver Simulation in MATLAB
clear; clc; close all;

%% 1. Parameters
Rs = 10e6; % Symbol rate 10 Msps
sps = 8;   % Samples per symbol
fs = Rs * sps; % Sampling frequency
rolloff = 0.2;
numSym = 10000; % Number of symbols to simulate

% Derived Parameters
T0 = 290;
T_ant = 150;
k_B = 1.380649e-23;
B_noise = Rs * (1 + rolloff); % Noise bandwidth
P_noise_in_W = k_B * T_ant * B_noise;
P_noise_in_dBm = 10*log10(P_noise_in_W) + 30;
SNR_in_dB = 20;
P_sig_in_dBm = P_noise_in_dBm + SNR_in_dB;
P_sig_in_W = 10^((P_sig_in_dBm-30)/10);

%% 2. Generate Symbols (DVB-S2 64-APSK: 4-12-20-28)
% Radii ratios for typical DVB-S2 64-APSK
n_points_per_ring = [4, 12, 20, 28];
ring_radii = [1.0, 2.73, 4.52, 6.15]; 

% Build the 64-APSK constellation array
constellation = zeros(64, 1);
idx = 1;
for i = 1:length(n_points_per_ring)
    N = n_points_per_ring(i);
    % Offset of pi/N aligns points diagonally like the standard 64-APSK
    phases = (0:N-1) * (2*pi / N) + (pi / N);
    constellation(idx : idx + N - 1) = ring_radii(i) * exp(1j * phases);
    idx = idx + N;
end

% Normalize constellation
constellation = constellation / sqrt(mean(abs(constellation).^2));

% Generate random symbol indices and map to constellation
tx_data = randi([1 64], numSym, 1);
tx_sym = constellation(tx_data);

%% 3. Pulse Shaping (RRC) Fallback
span = 20; % Symbol span for RRC
t = -span/2:1/sps:span/2;
rrcFilter = zeros(size(t));
for i = 1:length(t)
    if t(i) == 0.0
        rrcFilter(i) = 1.0 - rolloff + 4*rolloff/pi;
    elseif abs(abs(t(i)) - 1/(4*rolloff)) < 1e-6
        rrcFilter(i) = (rolloff/sqrt(2))*((1+2/pi)*sin(pi/(4*rolloff)) + (1-2/pi)*cos(pi/(4*rolloff)));
    else
        num = sin(pi*t(i)*(1-rolloff)) + 4*rolloff*t(i).*cos(pi*t(i)*(1+rolloff));
        den = pi*t(i).*(1-(4*rolloff*t(i)).^2);
        rrcFilter(i) = num / den;
    end
end
rrcFilter = rrcFilter / norm(rrcFilter);

tx_sig = zeros(length(tx_sym)*sps + length(rrcFilter) - 1, 1);
for i = 1:length(tx_sym)
    idx = (i-1)*sps + 1;
    tx_sig(idx:idx+length(rrcFilter)-1) = tx_sig(idx:idx+length(rrcFilter)-1) + tx_sym(i) * rrcFilter.';
end

% Set input power
tx_sig = tx_sig / sqrt(mean(abs(tx_sig).^2)); % Use RMS power
sig_in = tx_sig * sqrt(P_sig_in_W);

%% 4. Define Stages
% Format: {'Name', Gain_dB, NF_dB, P1dB_dBm, IP3_dBm, 'Ref_Port', PhaseNoise_dBcPerHz}
% Ref_Port: 'In' (IIP3/IP1dB) or 'Out' (OIP3/OP1dB)
stages = {
    'RF BPF', -0.4, 0.4,   Inf,  Inf, 'In',  -Inf;
    'LNA 1',   29.0, 1.8,   8.5, 17.5, 'Out', -Inf; % ADL8142S
    'IRF',    -0.6, 0.6,   Inf,  Inf, 'In',  -Inf;
    'MIX 1',  -9.0, 9.0,   2.0, 14.0, 'In',  -110;  % HMC264LC3B
    'IF 1 BPF',-0.6, 0.6,   Inf,  Inf, 'In',  -Inf;
    'LNA 2',   23.7, 0.27,  18.5, 27.8, 'Out', -Inf; % SAV-541-DG+
    'MIX 2 BB',-6.5, 6.5,  14.0, 30.0, 'In',  -143;  % SYM-25DHW+
    'BPF 2',  -3.0, 3.0,   Inf,  Inf, 'In',  -Inf;
    'LNA 3',   50.0, 3.5,  41.0, 43.0, 'Out', -Inf; % ZHL-20W-13SWX+
    'LIMITER', -0.5, 0.5,   Inf,  Inf, 'Out', -Inf  % Simulated strictly via voltage threshold below
};

numStages = size(stages, 1);
sig_current = sig_in;

noise_in = sqrt(P_noise_in_W / 2) * (randn(size(sig_in)) + 1j*randn(size(sig_in)));
rx_chain = sig_current + noise_in;

N_t0 = k_B * T0 * B_noise;
N_current = P_noise_in_W;
Gain_total_lin = 1.0;

fprintf('\n=== Receiver Chain Metrics ===\n');
fprintf('%-15s | %-10s | %-10s | %-10s | %-10s\n', 'Stage', 'SNR (dB)', 'EVM (%)', 'Gain (dB)', 'P_sig (dBm)');

results_sig = cell(numStages, 1);
results_snr = zeros(numStages, 1);
results_evm = zeros(numStages, 1);

for i = 1:numStages
    name = stages{i, 1};
    g_db = stages{i, 2};
    nf_db = stages{i, 3};
    p1db_spec = stages{i, 4};
    ip3_spec = stages{i, 5};
    ref_port = stages{i, 6};
    pn_dbc = stages{i, 7};
    
    g_lin = 10^(g_db/10);
    f_lin = 10^(nf_db/10);
    
    % Translate everything to Input Referred (IIP3 and IP1dB)
    if strcmp(ref_port, 'Out')
        iip3_dbm = ip3_spec - g_db;
        ip1db_dbm = p1db_spec - g_db;
    else
        iip3_dbm = ip3_spec;
        ip1db_dbm = p1db_spec;
    end
    
    % Keep the unamplified version for non-linear calculations
    rx_chain_in = rx_chain;
    
    % Signal Amplification
    rx_chain = rx_chain * sqrt(g_lin);
    Gain_total_lin = Gain_total_lin * g_lin;
    
    % Add Stage Noise
    P_added = N_t0 * g_lin * (f_lin - 1);
    local_noise = sqrt(P_added / 2) * (randn(size(rx_chain)) + 1j*randn(size(rx_chain)));
    rx_chain = rx_chain + local_noise;
    
    % Add Phase Noise from Local Oscillator (Mixer stages)
    if pn_dbc > -1000
        pn_var = 10^(pn_dbc/10) * B_noise;
        phase_jitter = sqrt(pn_var) * randn(size(rx_chain));
        rx_chain = rx_chain .* exp(1j * phase_jitter);
    end
    
    % Apply Soft/Hard Clipping (P1dB & IP3)
    if iip3_dbm < Inf || ip1db_dbm < Inf
        if iip3_dbm < Inf
            iip3_W = 10^((iip3_dbm - 30)/10);
            compression_factor = (1/3) * (abs(rx_chain_in).^2 / iip3_W);
            compression_factor(compression_factor > 0.9) = 0.9;
            rx_chain = rx_chain .* (1 - compression_factor);
        end
        if ip1db_dbm < Inf
            ip1db_W = 10^((ip1db_dbm - 30)/10);
            P_in_inst = abs(rx_chain_in).^2;
            clip_mask = P_in_inst > (ip1db_W * 0.794); % ~1dB before IP1dB
            if any(clip_mask)
                rx_chain(clip_mask) = rx_chain(clip_mask) .* sqrt(ip1db_W ./ P_in_inst(clip_mask));
            end
        end
    end
    if strcmp(name, 'LIMITER')
        % Typical RF Limiter threshold set here to limit amplitude exceeding +10dBm (output)
        lim_thresh_W = 10^((10.0 - 30)/10);
        P_out_inst = abs(rx_chain).^2;
        clip_mask = P_out_inst > lim_thresh_W;
        if any(clip_mask)
            rx_chain(clip_mask) = rx_chain(clip_mask) .* sqrt(lim_thresh_W ./ P_out_inst(clip_mask));
        end
    end
    
    % Update Noise Floor for SNR Tracking
    N_current = N_current * g_lin + P_added;
    P_sig_curr = P_sig_in_W * Gain_total_lin;
    snr_curr = 10*log10(P_sig_curr / N_current);
    
    % RX Filtering & EVM Eval
    rx_filt = conv(rx_chain, rrcFilter);
    rx_filt = rx_filt(1:sps:end);
    
    % Auto-align using cross-correlation
    [xc, lags] = xcorr(rx_filt(1:min(length(rx_filt), 2000)), tx_sym(1:500));
    [~, max_idx] = max(abs(xc));
    opt_delay = lags(max_idx);
    
    if opt_delay < 0
        opt_delay = span; % Fallback
    end
    
    eval_len = numSym - opt_delay;
    rx_sym = rx_filt(opt_delay + 1 : opt_delay + eval_len);
    tx_eval = tx_sym(1:eval_len);
    
    % Optimal phase & gain scaling
    alpha = (tx_eval' * rx_sym) / (rx_sym' * rx_sym);
    rx_sym_norm = rx_sym * alpha;
    
    evm = sqrt(mean(abs(rx_sym_norm - tx_eval).^2) / mean(abs(tx_eval).^2)) * 100;
    
    results_sig{i} = rx_sym_norm;
    results_snr(i) = snr_curr;
    results_evm(i) = evm;
    
    fprintf('%-15s | %10.2f | %10.2f | %10.2f | %10.2f\n', name, snr_curr, evm, g_db, 10*log10(P_sig_curr)+30);
end

% Check baseband output against 1Vpp targeted gain logic
BB_vpp = 2 * sqrt(2 * P_sig_in_W * Gain_total_lin * 50); % Approx Vpp on 50 ohms
fprintf('\nFinal Vpp on 50 Ohms: %.3f V\n', BB_vpp);

%% 6. Plot Constellations
f = figure('Name', 'Receiver Stage Constellations', 'Position', [100, 100, 1200, 800], 'Visible', 'off');
plot_idx = [1, 2, 4, 6, 8, 9]; % Select key stages to plot
for k = 1:length(plot_idx)
    idx = plot_idx(k);
    subplot(2, 3, k);
    plot(real(results_sig{idx}), imag(results_sig{idx}), '.', 'MarkerSize', 2);
    title(sprintf('%s\nSNR: %.1fdB, EVM: %.2f%%', stages{idx,1}, results_snr(idx), results_evm(idx)));
    grid on; axis square;
    xlim([-2 2]); ylim([-2 2]);
end
saveas(f, 'matlab_constellations.png');
disp('Saved constellations to matlab_constellations.png');

