% MATLAB Script: Earth-to-GEO Path Loss Frequency Sweep (1 GHz to 60 GHz)
% Required Toolboxes: Communications Toolbox, Phased Array System Toolbox
clear; clc;

% Parameters 
dist_m = 35786000;           % GEO Altitude (m) - Approx 35,786 km
freq = (1:0.1:60) * 1e9;     % Frequency Sweep (Hz) - 1 GHz to 60 GHz
rainRate = 45;               % Rain Rate (mm/h) - Heavy rain
temperature = 25;            % Temperature (Celsius)
waterDensity = 7.5;          % Water vapor density (g/m^3)

% Calculate Dry Air Pressure for gaspl
total_pressure = 101325;     % Air pressure at sea level (Pa)
e_vapor = (waterDensity * (temperature + 273.15)) / 2.167; 
dry_pressure = total_pressure - e_vapor; 

c = physconst('LightSpeed');

eff_atm_dist = 10000;  % Effective atmospheric thickness for gas (~10 km)
eff_rain_dist = 5000;  % Effective rain height (~5 km)

% Free Space Path Loss (FSPL)
fspl_loss = fspl(dist_m, c./freq);

% Gas Attenuation 
gas_loss = gaspl(eff_atm_dist, freq, temperature, dry_pressure, waterDensity);

% Rain Attenuation 
rain_loss = rainpl(eff_rain_dist, freq, rainRate);

% Total Loss 
fspl_loss_col = fspl_loss(:);
gas_loss_col  = gas_loss(:);
rain_loss_col = rain_loss(:);
freq_ghz_col  = freq(:) / 1e9;

total_loss_col = fspl_loss_col + gas_loss_col + rain_loss_col;

% Plot Results 
plot(freq_ghz_col, fspl_loss_col, 'Color', '#ff7f0e', 'LineWidth', 2); 
hold on;

% 2. FSPL + Gas 
plot(freq_ghz_col, fspl_loss_col + gas_loss_col, '--', 'Color', [0 0.7 0.9], 'LineWidth', 2);

% 3. Total Loss 
plot(freq_ghz_col, total_loss_col, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2);

grid on;
xlabel('Frequency (GHz)');
ylabel('Loss (dB)');
legend('FSPL', 'FSPL + Gas', 'Total Loss (FSPL+Gas+Rain)', 'Location', 'northwest');
title('Earth-to-GEO Propagation Loss (1 - 60 GHz)');

%% Comparison Script

% System parameters
N = 100000; % Number of Symbols
BW_24GHz = 200e6; % 200 MHz
BW_30GHz = 400e6; % 400 MHz

% Channel Noise 
loss_24GHz = total_loss_col(freq_ghz_col == 24.0);
loss_30GHz = total_loss_col(freq_ghz_col == 30.0);
delta_loss = loss_30GHz - loss_24GHz;

% Thermal Noise Penalty 
noise_penalty = 10 * log10(BW_30GHz / BW_24GHz); % ~3 dB (B'=2B)

total_penalty_30GHz = delta_loss + noise_penalty;

fprintf('Επιπλέον Απώλειες Καναλιού στα 30GHz: %.2f dB\n', delta_loss);
fprintf('Ποινή Θορύβου λόγω 400MHz vs 200MHz : %.2f dB\n', noise_penalty);
fprintf('Συνολικό Μειονέκτημα SNR στα 30GHz  : %.2f dB\n\n', total_penalty_30GHz);

% Input SNR
SNR_base = 20; % Design link budget in 20 dB for 24GHz

SNR_64QAM = SNR_base; 
SNR_8PSK = SNR_base - total_penalty_30GHz;

% Signal Modulation
data8 = double(randi([0 7], N, 1));
data64 = double(randi([0 63], N, 1));

% Use qammod for 64 psk
mod8 = pskmod(data8, 8);
mod64 = qammod(data64, 64, 'UnitAveragePower', true); % Normalize power

% Add noise
rx8 = awgn(mod8, SNR_8PSK, 'measured');
rx64 = awgn(mod64, SNR_64QAM, 'measured');

% Demodulation
demod8 = pskdemod(rx8, 8);
demod64 = qamdemod(rx64, 64, 'UnitAveragePower', true);

% Error calculation
[~, ser8] = symerr(data8, demod8);
[~, ser64] = symerr(data64, demod64);

% Results
fprintf('--- ΑΠΟΤΕΛΕΣΜΑΤΑ ---\n');
fprintf('64-QAM (24GHz, 200MHz) | Διαθέσιμο SNR: %.2f dB | SER: %f\n', SNR_64QAM, ser64);
fprintf('8-PSK  (30GHz, 400MHz) | Διαθέσιμο SNR: %.2f dB | SER: %f\n', SNR_8PSK, ser8);

% Visualisation 
figure;

subplot(1,2,1);
plot(real(rx8), imag(rx8), '.'); 
title(sprintf('8-PSK (30GHz)\nSNR: %.1fdB', SNR_8PSK));
axis square; 

subplot(1,2,2);
plot(real(rx64), imag(rx64), '.');
title(sprintf('64-QAM (24GHz)\nSNR: %.1fdB', SNR_64QAM));
axis square;