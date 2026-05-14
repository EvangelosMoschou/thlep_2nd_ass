% RF Receiver Front-End Cascade Analysis & Simulation
% Modulation: 64-PSK | Frequency: 24 GHz | Bandwidth: 200 MHz
clear; clc; close all;

%% 1. SYSTEM PARAMETERS & SPECIFICATIONS
k = 1.38e-23;           % Boltzmann constant [J/K]
T_A = 150;              % Antenna equivalent noise temperature [K]
T_0 = 290;              % Reference temperature [K]
B = 200e6;              % Bandwidth [Hz]
SNR_in_target = 20;     % Desired input SNR [dB]
R_load = 50;            % Load resistance [Ohms]
Vpp_out = 1.0;          % Target output voltage peak-to-peak [V]

%% 2. SIGNAL & NOISE POWER CALCULATIONS (Input & Output)
% Input Noise Power (Ni)
P_n_watts = k * T_A * B;
Ni_dBm = 10 * log10(P_n_watts / 1e-3);

% Input Signal Power (Si) based on required SNR
Si_dBm = Ni_dBm + SNR_in_target;

% Output Target Power (Pout) for 1 Vpp on 50 Ohms
Vrms = Vpp_out / (2 * sqrt(2));
Pout_watts = (Vrms^2) / R_load;
Pout_dBm = 10 * log10(Pout_watts / 1e-3);

% Total Required Gain
Total_Required_Gain = Pout_dBm - Si_dBm;

fprintf('--- SYSTEM REQUIREMENTS ---\n');
fprintf('Input Noise Power (Ni): %.2f dBm\n', Ni_dBm);
fprintf('Input Signal Power (Si): %.2f dBm\n', Si_dBm);
fprintf('Target Output Power: %.2f dBm\n', Pout_dBm);
fprintf('Total Required Gain: %.2f dB\n\n', Total_Required_Gain);

%% 3. CASCADE ANALYSIS (Gain, NF, IP3)
Names = {'switch','Pre-selector BPF', 'LNA1', 'Image Rejection Filter', 'Mixer1', 'BPF2','LNA2', 'Mixer2', 'BPF3', 'LNA3', 'Limiter'}; 
Gain_dB = [-0.3, -0.4, 29, -3.5, -9.0, -0.6, 23.7, -6.5, -3, 50, -0.25]; 
NF_dB   = [0.3, 0.4,  1.8, 3.5, 9.0, 0.6, 0.27, 6.5, 3, 3.5, 0.25]; 
IIP3_dBm= [100, 100,  -11.5, 100.0, 14.0, 100, 4.1, 30, 100.0, -7, 100]; 
N_stages = length(Gain_dB);

% Pre-allocate cumulative arrays
Cum_Gain_dB = zeros(1, N_stages);
Cum_NF_dB = zeros(1, N_stages);
Cum_IIP3_dBm = zeros(1, N_stages);

% Linear Values for calculations
g = 10.^(Gain_dB / 10);
f = 10.^(NF_dB / 10);
iip3_mw = 10.^(IIP3_dBm / 10);

% Initialize Cascade
Cum_Gain_dB(1) = Gain_dB(1);
Cum_NF_dB(1) = NF_dB(1);
Cum_IIP3_dBm(1) = IIP3_dBm(1);
F_total = f(1);
IIP3_total_inv = 1 / iip3_mw(1);
G_total_lin = g(1);

for i = 2:N_stages
    % Cumulative Gain
    Cum_Gain_dB(i) = Cum_Gain_dB(i-1) + Gain_dB(i);
    
    % Cumulative Noise Factor (Friis Equation)
    F_total = F_total + (f(i) - 1) / G_total_lin;
    Cum_NF_dB(i) = 10 * log10(F_total);
    
    % Cumulative IIP3
    IIP3_total_inv = IIP3_total_inv + (G_total_lin / iip3_mw(i));
    Cum_IIP3_dBm(i) = 10 * log10(1 / IIP3_total_inv);
    
    % Update rolling gain
    G_total_lin = G_total_lin * g(i);
end

fprintf('--- SYSTEM CACULATED PARAMETERS ---\n');
fprintf('Total Gain: %.2f dB\n', Cum_Gain_dB(end));
fprintf('Total Noise Figure: %.2f dB\n', Cum_NF_dB(end));
fprintf('Total IIP3: %.2f dBm\n\n', Cum_IIP3_dBm(end));

%% 4. DYNAMIC RANGE CALCULATIONS
% Output Noise Power CAlculation (No) based on temperatures
F_linear = 10^(Cum_NF_dB(end)/10);          
Te = (F_linear - 1) * T_0;                  
G_linear = 10^(Cum_Gain_dB(end)/10);        
No_watts = k * (T_A + Te) * B * G_linear;   
N_out_dBm = 10 * log10(No_watts / 1e-3);    

% P1dB and OIP3 calculation
OIP3_end = Cum_IIP3_dBm(end) + Cum_Gain_dB(end); 
Input_P1dB = Cum_IIP3_dBm(end) - 9.6;            
Output_P1dB = Input_P1dB + Cum_Gain_dB(end) - 1; 

% Up-bound calculation (Pout) for SFDR
Pout_SFDR_max = N_out_dBm + (2/3) * (OIP3_end - N_out_dBm); % Pout when IM3 = Noise Floor
SNR_min_level = N_out_dBm + SNR_in_target;      

LDR = Output_P1dB - N_out_dBm; 
SFDR = Pout_SFDR_max - N_out_dBm; 

fprintf('ΑΚΡΙΒΗΣ ΔΥΝΑΜΙΚΗ ΠΕΡΙΟΧΗ (Βάσει Θερμοκρασιών)\n');
fprintf('Ισοδύναμη Θερμοκρασία (Te): %.2f K\n', Te);
fprintf('Ακριβές Output Noise Level (No): %.2f dBm\n', N_out_dBm);
fprintf('Linear Dynamic Range (LDR): %.2f dB\n', LDR);
fprintf('Spurious Free Dynamic Range (SFDR): %.2f dB\n\n', SFDR);

%% 5. Final Dynamic Range Diagram
figure('Name', 'Master Receiver Dynamic Range', 'Color', 'w', 'Position', [100, 100, 900, 650]);
hold on; grid on;

% Χ axis definition (Pin)
Pin_range = linspace(Ni_dBm - 10, Cum_IIP3_dBm(end) + 15, 300);

% Curves Calculation
Pout_ideal_linear = Pin_range + Cum_Gain_dB(end);
Pout_comp = Pout_ideal_linear - 10*log10(1 + 10.^((Pin_range - Input_P1dB)/10));
Pout_IM3 = 3*Pin_range - 2*Cum_IIP3_dBm(end) + Cum_Gain_dB(end);

% Common Axis
x_fill = [Pin_range(1) Pin_range(end) Pin_range(end) Pin_range(1)];

% 1. REGIONS
% LDR Region 
y_ldr = [N_out_dBm N_out_dBm Output_P1dB Output_P1dB];
fill(x_fill, y_ldr, [0.8 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'DisplayName', 'LDR Region'); 

% SFDR Region 
y_sfdr = [N_out_dBm N_out_dBm Pout_SFDR_max Pout_SFDR_max];
fill(x_fill, y_sfdr, [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'SFDR Region'); 

% 2. Curves (Signal, Ideal, IM3)
plot(Pin_range, Pout_ideal_linear, 'k--', 'LineWidth', 1, 'DisplayName', 'Ideal Linear'); 
plot(Pin_range, Pout_comp, 'k', 'LineWidth', 2.5, 'DisplayName', 'P_{out} (Fundamental, n=1)');           
plot(Pin_range, Pout_IM3, 'k', 'LineWidth', 2, 'DisplayName', 'IM3 (n=3)');            

% 3. Reference lines
yline(N_out_dBm, 'k', 'LineWidth', 1.5, 'DisplayName', 'Noise Floor (No)');                     
yline(SNR_min_level, 'r', 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Min Target SNR');                 

% 4.  Intersection Point (IP3)
plot(Cum_IIP3_dBm(end), OIP3_end, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7, 'DisplayName', 'IP3 Point');
text(Cum_IIP3_dBm(end)-3.5, OIP3_end+4, 'IP_3', 'FontSize', 12, 'FontWeight', 'bold');

% 5. Labels
% SFDR Arrow
x_sfdr_arrow = Pin_range(40);
line([x_sfdr_arrow x_sfdr_arrow], [N_out_dBm Pout_SFDR_max], 'Color', 'r', 'LineWidth', 2, 'HandleVisibility','off');
text(x_sfdr_arrow + 1, (N_out_dBm + Pout_SFDR_max)/2, 'SFDR', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 11);

% LDR Arrow
x_ldr_arrow = Pin_range(110); % Πιο δεξιά για καθαρότητα
line([x_ldr_arrow x_ldr_arrow], [N_out_dBm Output_P1dB], 'Color', 'b', 'LineWidth', 2, 'HandleVisibility','off');
text(x_ldr_arrow + 1, (N_out_dBm + Output_P1dB)/2, 'LDR', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 11);

% Labels
text(Pin_range(10), SNR_min_level + 2.5, 'Target SNR', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
text(Pin_range(10), N_out_dBm - 3.5, 'Noise Floor', 'FontSize', 10);
text(Input_P1dB - 15, Output_P1dB - 7, 'n = 1', 'Rotation', 45, 'FontSize', 11);
text(Input_P1dB - 5, Pout_IM3(find(Pin_range >= Input_P1dB-5, 1)) + 5, 'n = 3', 'Rotation', 70, 'FontSize', 11);

% 6. Final Graph
xlabel('Input Power - P_{in} (dBm)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Output Power - P_{out} (dBm)', 'FontSize', 12, 'FontWeight', 'bold');
title('Master Receiver Dynamic Range Analysis', 'FontSize', 14);
legend('Location', 'northwest', 'FontSize', 10); 
xlim([Pin_range(1) Pin_range(end)]);
ylim([N_out_dBm - 15, OIP3_end + 10]);

hold off;