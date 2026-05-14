function sim_receiver_matlab()
% DVB-S2X 64-APSK reference simulation (stage-by-stage, symbol-domain focus)
% This script is used as the MATLAB reference for constellation-domain behavior
% and stage metrics under the CascadeAnalyzer assumptions.
% clear; clc; close all; % Disabled for function mode

%% 1. Simulation Setup
Rs = 10e6; % Symbol rate [sym/s]
sps = 8;   % Samples per symbol
fs = Rs * sps; % MATLAB waveform sample rate [Hz]
rolloff = 0.2;
numSym = 10000; % Number of symbols

% Output directory for MATLAB-generated figures
out_dir = fullfile('..', 'out', 'matlab');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% Link-budget parameters (CascadeAnalyzer-compatible assumptions)
T0 = 290;
T_ant = 150;
k_B = 1.380649e-23;
B_noise = 200e6; % Equivalent receiver noise bandwidth [Hz]
R_load_ohm = 50; % Reference system impedance for power <-> voltage conversion
P_noise_in_W = k_B * T_ant * B_noise;
P_noise_in_dBm = 10*log10(P_noise_in_W) + 30;
SNR_target_dB = 20;
P_sig_budget_dBm = P_noise_in_dBm + SNR_target_dB;
P_sig_budget_W = 10^((P_sig_budget_dBm-30)/10);

% Force MATLAB input amplitude to the requested level (50 uV RMS).
input_target_vrms_uv = 50.0;
input_target_vrms = input_target_vrms_uv * 1e-6;
P_sig_in_W = (input_target_vrms^2) / R_load_ohm;
P_sig_in_dBm = 10*log10(P_sig_in_W) + 30;
SNR_in_dB = P_sig_in_dBm - P_noise_in_dBm;

% Final output target with linear RLM stage (no compression).
target_final_vpp = 1.0;
rlm_gain_db = -0.04;
lna3_target_vpp = target_final_vpp / (10^(rlm_gain_db/20));

%% 2. Symbol Generation (DVB-S2X 64-APSK)
% Ring populations and nominal radii
n_points_per_ring = [4, 12, 20, 28];
ring_radii = [1.0, 2.73, 4.52, 6.15]; 

% Build constellation points ring-by-ring
constellation = zeros(64, 1);
idx = 1;
for i = 1:length(n_points_per_ring)
    N = n_points_per_ring(i);
    % pi/N phase offset places points on the standard diagonal orientation
    phases = (0:N-1) * (2*pi / N) + (pi / N);
    constellation(idx : idx + N - 1) = ring_radii(i) * exp(1j * phases);
    idx = idx + N;
end

% Normalize to unit average power
constellation = constellation / sqrt(mean(abs(constellation).^2));

% Draw random symbols and map to complex points
tx_data = randi([1 64], numSym, 1);
tx_sym = constellation(tx_data);

%% 3. Pulse Shaping (RRC fallback implementation)
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

% Manual upsample + filter convolution (kept explicit for transparency)
tx_sig = zeros(length(tx_sym)*sps + length(rrcFilter) - 1, 1);
for i = 1:length(tx_sym)
    idx = (i-1)*sps + 1;
    tx_sig(idx:idx+length(rrcFilter)-1) = tx_sig(idx:idx+length(rrcFilter)-1) + tx_sym(i) * rrcFilter.';
end

% Set waveform power to match link-budget input signal power
tx_sig = tx_sig / sqrt(mean(abs(tx_sig).^2));
sig_in = tx_sig * sqrt(P_sig_in_W * R_load_ohm);

%% 4. Stage Model (aligned to CascadeAnalyzer)
% Load LO phase-noise assumptions from data catalog.
% Mapping used here:
%   LO_1_01 -> MIX 1
%   LO_2_01 -> MIX 2
pn_mix1_dbchz = -Inf;
pn_mix2_dbchz = -Inf;
catalog_path = fullfile('..', 'data_input', 'receiver_config.csv');

if exist(catalog_path, 'file')
    try
        catalog_tbl = readtable(catalog_path);
        var_names = lower(string(catalog_tbl.Properties.VariableNames));
        uid_idx = find(var_names == "component_uid", 1);
        pn_idx = find(var_names == "phase_noise_dbchz", 1);

        if ~isempty(uid_idx) && ~isempty(pn_idx)
            uid_col = string(catalog_tbl{:, uid_idx});
            pn_col_raw = catalog_tbl{:, pn_idx};

            if isnumeric(pn_col_raw)
                pn_vals = pn_col_raw;
            else
                pn_vals = str2double(string(pn_col_raw));
            end

            lo1_idx = find(uid_col == "LO_1_01", 1);
            lo2_idx = find(uid_col == "LO_2_01", 1);

            if ~isempty(lo1_idx) && isfinite(pn_vals(lo1_idx))
                pn_mix1_dbchz = pn_vals(lo1_idx);
            end
            if ~isempty(lo2_idx) && isfinite(pn_vals(lo2_idx))
                pn_mix2_dbchz = pn_vals(lo2_idx);
            end
        end
    catch ME
        warning('Phase-noise catalog load failed (%s). Continuing with phase noise disabled.', '%s', ME.message);
    end
end

fprintf('Phase noise from data: MIX 1 = %.1f dBc/Hz, MIX 2 = %.1f dBc/Hz\n', pn_mix1_dbchz, pn_mix2_dbchz);

% Row format:
% {'Name', Gain_dB, NF_dB, P1dB_dBm, IP3_dBm, 'Ref_Port', PhaseNoise_dBcPerHz}
% Ref_Port selects whether P1dB/IP3 are input-referred ('In') or output-referred ('Out').
stages = {
    'SWITCH',           -0.3, 0.3,  Inf,  Inf,   'In', -Inf;
    'PRESEL BPF',       -0.4, 0.4,  Inf,  Inf,   'In', -Inf;
    'LNA 1',            29.0, 1.8,  Inf, -11.5,  'In', -Inf;
    'IRF',              -3.5, 3.5,  Inf,  Inf,   'In', -Inf;
    'MIX 1',            -9.0, 9.0,  Inf, 14.0,   'In', pn_mix1_dbchz;
    'BPF 2',            -0.6, 0.6,  Inf,  Inf,   'In', -Inf;
    'LNA 2',            23.7, 0.27, Inf, 4.1,    'In', -Inf;
    'MIX 2',            -6.5, 6.5,  Inf, 30.0,   'In', pn_mix2_dbchz;
    'BPF 3',            -3.0, 3.0,  Inf,  Inf,   'In', -Inf;
    'LNA 3',            50.0, 3.5,  Inf, -7.0,   'In', -Inf;
    'RLM-43-5W+',       -0.04, 0.04, Inf, Inf,   'In', -Inf  % Linear limiter model (no compression)
};

numStages = size(stages, 1);
sig_current = sig_in;

% Add antenna/input noise at the chain entrance
noise_in = sqrt((P_noise_in_W * R_load_ohm) / 2) * (randn(size(sig_in)) + 1j*randn(size(sig_in)));
rx_chain = sig_current + noise_in;
input_trace = rx_chain;

% Build input constellation snapshot before the first stage.
rx_input_filt = conv(rx_chain, rrcFilter);
rx_input_filt = rx_input_filt(1:sps:end);
[xc_in, lags_in] = xcorr(rx_input_filt(1:min(length(rx_input_filt), 2000)), tx_sym(1:500));
[~, max_idx_in] = max(abs(xc_in));
opt_delay_in = lags_in(max_idx_in);
if opt_delay_in < 0
    opt_delay_in = span; % Fallback
end

eval_len_in = numSym - opt_delay_in;
rx_input_sym = rx_input_filt(opt_delay_in + 1 : opt_delay_in + eval_len_in);
tx_input_eval = tx_sym(1:eval_len_in);
alpha_in = (tx_input_eval' * rx_input_sym) / (rx_input_sym' * rx_input_sym);
input_sig_norm = rx_input_sym * alpha_in;

input_err_pow = mean(abs(input_sig_norm - tx_input_eval).^2);
input_sig_pow = mean(abs(tx_input_eval).^2);
if input_err_pow > 0
    input_snr_plot_db = 10 * log10(input_sig_pow / input_err_pow);
else
    input_snr_plot_db = Inf;
end
input_evm_plot = sqrt(input_err_pow / input_sig_pow) * 100;

N_t0 = k_B * T0 * B_noise;
N_current = P_noise_in_W;
Gain_total_lin = 1.0;

fprintf('Input equivalent @ %.0f ohm: Vrms = %.3f uV, sine Vpp = %.3f uV\n', ...
    R_load_ohm, sqrt(P_sig_in_W * R_load_ohm) * 1e6, 2*sqrt(2*P_sig_in_W*R_load_ohm) * 1e6);
fprintf('Input budget from equation: %.3f dBm (target SNR %.2f dB)\n', P_sig_budget_dBm, SNR_target_dB);
fprintf('Applied MATLAB input level: %.3f dBm (effective input SNR %.2f dB)\n', P_sig_in_dBm, SNR_in_dB);

fprintf('\n=== Receiver Chain Metrics ===\n');
fprintf('%-15s | %-10s | %-10s | %-10s | %-10s\n', 'Stage', 'SNR (dB)', 'EVM (%)', 'Gain (dB)', 'P_sig (dBm)');

results_sig = cell(numStages, 1);
results_snr = zeros(numStages, 1);
results_evm = zeros(numStages, 1);
results_trace = cell(numStages, 1); % Store per-stage waveform for trace plots
results_peak_v = zeros(numStages, 1);
results_vpp_i = zeros(numStages, 1);

for i = 1:numStages
    name = stages{i, 1};
    g_db = stages{i, 2};
    nf_db = stages{i, 3};
    p1db_spec = stages{i, 4};
    ip3_spec = stages{i, 5};
    ref_port = stages{i, 6};
    pn_dbc = stages{i, 7};
    post_gain_lin = 1.0;

    % MATLAB gain steering: make LNA3 deliver the level needed so the
    % following linear RLM stage lands close to 1 Vpp at output.
    if strcmp(name, 'LNA 3')
        curr_i_vpp = measure_i_vpp(rx_chain);
        if curr_i_vpp > 1e-15
            g_db = 20 * log10(lna3_target_vpp / curr_i_vpp);
        end
    end

    g_lin = 10^(g_db/10);
    f_lin = 10^(nf_db/10);
    
    % Convert stage nonlinearity specs to input-referred form
    if strcmp(ref_port, 'Out')
        iip3_dbm = ip3_spec - g_db;
        ip1db_dbm = p1db_spec - g_db;
    else
        iip3_dbm = ip3_spec;
        ip1db_dbm = p1db_spec;
    end
    
    % Preserve pre-gain signal for nonlinearity computations
    rx_chain_in = rx_chain;
    
    % Apply stage gain
    rx_chain = rx_chain * sqrt(g_lin);
    Gain_total_lin = Gain_total_lin * g_lin;
    
    % Add stage thermal noise contribution
    P_added = N_t0 * g_lin * (f_lin - 1);
    local_noise = sqrt((P_added * R_load_ohm) / 2) * (randn(size(rx_chain)) + 1j*randn(size(rx_chain)));
    rx_chain = rx_chain + local_noise;
    
    % Optional LO phase-noise injection (currently disabled with -Inf values)
    if pn_dbc > -1000
        pn_var = 10^(pn_dbc/10) * B_noise;
        phase_jitter = sqrt(pn_var) * randn(size(rx_chain));
        rx_chain = rx_chain .* exp(1j * phase_jitter);
    end
    
    % Apply nonlinear effects (IP3 soft compression + P1dB clipping)
    if iip3_dbm < Inf || ip1db_dbm < Inf
        if iip3_dbm < Inf
            iip3_W = 10^((iip3_dbm - 30)/10);
            iip3_V2 = iip3_W * R_load_ohm;
            compression_factor = (1/3) * (abs(rx_chain_in).^2 / iip3_V2);
            compression_factor(compression_factor > 0.9) = 0.9;
            rx_chain = rx_chain .* (1 - compression_factor);
        end
        if ip1db_dbm < Inf
            ip1db_W = 10^((ip1db_dbm - 30)/10);
            ip1db_V2 = ip1db_W * R_load_ohm;
            P_in_inst = abs(rx_chain_in).^2;
            clip_mask = P_in_inst > (ip1db_V2 * 0.794); % ~1dB before IP1dB
            if any(clip_mask)
                rx_chain(clip_mask) = rx_chain(clip_mask) .* sqrt(ip1db_V2 ./ P_in_inst(clip_mask));
            end
        end
    end

    % Update running SNR estimate
    N_current = (N_current * g_lin + P_added) * post_gain_lin;
    P_sig_curr = P_sig_in_W * Gain_total_lin;
    snr_curr = 10*log10(P_sig_curr / N_current);
    
    % Receiver filtering + symbol extraction for EVM computation
    rx_filt = conv(rx_chain, rrcFilter);
    rx_filt = rx_filt(1:sps:end);
    
    % Delay estimation via cross-correlation
    [xc, lags] = xcorr(rx_filt(1:min(length(rx_filt), 2000)), tx_sym(1:500));
    [~, max_idx] = max(abs(xc));
    opt_delay = lags(max_idx);
    
    if opt_delay < 0
        opt_delay = span; % Fallback
    end
    
    eval_len = numSym - opt_delay;
    rx_sym = rx_filt(opt_delay + 1 : opt_delay + eval_len);
    tx_eval = tx_sym(1:eval_len);
    
    % Best-fit complex scaling (phase + amplitude alignment)
    alpha = (tx_eval' * rx_sym) / (rx_sym' * rx_sym);
    rx_sym_norm = rx_sym * alpha;
    
    evm = sqrt(mean(abs(rx_sym_norm - tx_eval).^2) / mean(abs(tx_eval).^2)) * 100;
    
    results_sig{i} = rx_sym_norm;
    results_trace{i} = rx_chain;
    results_snr(i) = snr_curr;
    results_evm(i) = evm;
    results_peak_v(i) = max(abs(rx_chain));
    results_vpp_i(i) = max(real(rx_chain)) - min(real(rx_chain));
    
    fprintf('%-15s | %10.2f | %10.2f | %10.2f | %10.2f\n', name, snr_curr, evm, g_db, 10*log10(P_sig_curr)+30);
end

% Report measured final-stage output level from the actual waveform.
BB_vpp_measured = results_vpp_i(end);
fprintf('\nMeasured Final Stage I-Vpp: %.3f V\n', BB_vpp_measured);

fprintf('\n=== Stage Voltage Summary (raw waveform) ===\n');
fprintf('%-15s | %-14s | %-14s\n', 'Stage', 'Peak|x| (V)', 'I Vpp (V)');
for i = 1:numStages
    fprintf('%-15s | %12.6f | %12.6f\n', stages{i,1}, results_peak_v(i), results_vpp_i(i));
end

%% 6. Plot Key-Stage Constellations
f = figure('Name', 'Receiver Stage Constellations', 'Position', [100, 100, 1400, 800], 'Visible', 'off');
plot_idx = [0, 1, 3, 5, 7, 9, 11]; % Include input + representative stages
ncols_key = 4;
nrows_key = ceil(length(plot_idx) / ncols_key);
for k = 1:length(plot_idx)
    idx = plot_idx(k);
    subplot(nrows_key, ncols_key, k);
    if idx == 0
        plot(real(input_sig_norm), imag(input_sig_norm), '.', 'MarkerSize', 2);
        title(sprintf('INPUT\nSNR: %.1f dB, EVM: %.2f%%', SNR_in_dB, input_evm_plot));
    else
        plot(real(results_sig{idx}), imag(results_sig{idx}), '.', 'MarkerSize', 2);
        title(sprintf('%s\nSNR: %.1f dB, EVM: %.2f%%', stages{idx,1}, results_snr(idx), results_evm(idx)));
    end
    grid on; axis square;
    xlim([-2 2]); ylim([-2 2]);
end
export_figure_png_svg(f, out_dir, 'matlab_constellations');

%% 7. Plot C-Like Signal Traces (one file per stage)
% Match C style: per-stage file, I/Q curves, time axis in us, amplitude in volts.
plot_trace_like_c(input_trace, [], 'Input Signal Trace', out_dir, 'matlab_input_trace', fs, Rs, false);
for idx = 1:numStages
    sig_stage = results_trace{idx};
    prev_stage = [];
    if idx == 1
        prev_stage = input_trace;
    elseif idx > 1
        prev_stage = results_trace{idx - 1};
    end

    stage_slug = regexprep(lower(stages{idx,1}), '[^a-z0-9]+', '_');
    stage_slug = regexprep(stage_slug, '^_+|_+$', '');
    base_name = sprintf('matlab_stage_%02d_%s_trace', idx, stage_slug);
    title_txt = sprintf('Stage %02d - %s Trace', idx, stages{idx,1});

    % Keep limiter chart uncluttered (as in C limiter exception).
    is_limiter = strcmpi(stages{idx,1}, 'LIMITER');
    plot_trace_like_c(sig_stage, prev_stage, title_txt, out_dir, base_name, fs, Rs, ~is_limiter);
end

%% 8. Plot Constellations For All Stages
f_all = figure('Name', 'MATLAB All Stage Constellations', 'Position', [100, 100, 1500, 900], 'Visible', 'off');
ncols_all = 4;
nrows_all = ceil((numStages + 1) / ncols_all);

for idx = 1:(numStages + 1)
    subplot(nrows_all, ncols_all, idx);
    if idx == 1
        plot(real(input_sig_norm), imag(input_sig_norm), '.', 'MarkerSize', 1.8);
        title(sprintf('INPUT\nSNR %.1f dB, EVM %.2f%%', SNR_in_dB, input_evm_plot), 'FontSize', 8);
    else
        stage_idx = idx - 1;
        plot(real(results_sig{stage_idx}), imag(results_sig{stage_idx}), '.', 'MarkerSize', 1.8);
        title(sprintf('%s\nSNR %.1f dB, EVM %.2f%%', stages{stage_idx,1}, results_snr(stage_idx), results_evm(stage_idx)), 'FontSize', 8);
    end
    grid on;
    axis square;
    xlim([-2 2]);
    ylim([-2 2]);
end

export_figure_png_svg(f_all, out_dir, 'matlab_constellations_all_stages');


end
