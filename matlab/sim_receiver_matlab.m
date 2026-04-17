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
span = 6; % Symbol span for RRC
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
stages = {
    'RF BPF', -1.66, 0.6;
    'LNA 1',   15.0, 0.9;
    'IRF',    -0.4,  0.4;
    'MIX 1',  -3.0,  3.0;
    'IF 1 BPF',-0.4, 0.4;
    'LNA 2',   14.0, 0.9;
    'MIX 2 BB',-7.4, 7.4;
    'BPF 2',  -2.2,  2.2;
    'LNA 3',   24.0, 0.5
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
    
    g_lin = 10^(g_db/10);
    f_lin = 10^(nf_db/10);
    
    % Signal Amplification
    rx_chain = rx_chain * sqrt(g_lin);
    Gain_total_lin = Gain_total_lin * g_lin;
    
    % Add Stage Noise
    % Noise added by a component at its own output is k*T0*B * G_local * (F_local - 1)
    P_added = N_t0 * g_lin * (f_lin - 1);
    local_noise = sqrt(P_added / 2) * (randn(size(rx_chain)) + 1j*randn(size(rx_chain)));
    rx_chain = rx_chain + local_noise;
    
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

