% =========================================================================
% ΑΥΤΟΜΑΤΗ ΔΗΜΙΟΥΡΓΙΑ SIMULINK MODEL ΓΙΑ ΤΟΝ 64-APSK ΔΕΚΤΗ
% ΠΛΗΡΩΣ ΕΝΑΡΜΟΝΙΣΜΕΝΟ ΜΕ ΤΗΝ C-ΤΟΠΟΛΟΓΙΑ (Dual Conversion Superheterodyne)
% =========================================================================
modelName = 'DVB_S2X_64APSK_Receiver_Sim';

if bdIsLoaded(modelName)
    close_system(modelName, 0); % Close without saving
end
new_system(modelName);
open_system(modelName);

% ======= ΔΗΜΙΟΥΡΓΙΑ BLOCKS ΣΥΜΦΩΝΑ ΜΕ ΤΟ runtime_stage_models_target16.csv =======

% 1. Transmitter & AWGN (Προσομοίωση Εκπομπής και Καναλιού)
add_block('simulink/Sources/In1', [modelName '/RF Input (Real)']);
add_block('simulink/Sources/Band-Limited White Noise', [modelName '/AWGN (Channel)'], 'Cov', '0.01');
add_block('simulink/Math Operations/Add', [modelName '/Channel Adder']);

% 2. RF Frontend - Stage 1: RF_BPF (-1.66dB)
add_block('simulink/Discrete/Discrete Filter', [modelName '/RF_BPF']);
add_block('simulink/Math Operations/Gain', [modelName '/RF_BPF_Gain (-1.66 dB)'], 'Gain', '10^(-1.66/20)');

% 3. RF Frontend - Stage 2: LNA1 (15.0dB)
add_block('simulink/Math Operations/Gain', [modelName '/LNA1 (15.0 dB)'], 'Gain', '10^(15.0/20)');

% 4. RF Frontend - Stage 3: IRF Filter (-0.4dB)
add_block('simulink/Discrete/Discrete Filter', [modelName '/IRF_Filter']);
add_block('simulink/Math Operations/Gain', [modelName '/IRF_Gain (-0.4 dB)'], 'Gain', '10^(-0.4/20)');

% 5. RF Frontend - Stage 4: MIX1 Downconversion to IF (-3.0dB)
add_block('simulink/Sources/Sine Wave', [modelName '/LO1 (IF Mix)'], 'Amplitude', '2', 'Frequency', '2*pi*22.5e9', 'Phase', '0');
add_block('simulink/Math Operations/Product', [modelName '/Mixer1_IF']);
add_block('simulink/Math Operations/Gain', [modelName '/MIX1_Gain (-3.0 dB)'], 'Gain', '10^(-3.0/20)');

% 6. RF Frontend - Stage 5: IF1_BPF (-0.4dB)
add_block('simulink/Discrete/Discrete Filter', [modelName '/IF1_BPF']);
add_block('simulink/Math Operations/Gain', [modelName '/IF1_BPF_Gain (-0.4 dB)'], 'Gain', '10^(-0.4/20)');

% 7. RF Frontend - Stage 6: LNA2 (14.0dB)
add_block('simulink/Math Operations/Gain', [modelName '/LNA2 (14.0 dB)'], 'Gain', '10^(14.0/20)');

% 8. MIX2 Downconversion to Baseband (Split IF to I/Q Baseband)
add_block('simulink/Sources/Sine Wave', [modelName '/LO2 Cosine (I-Branch)'], 'Amplitude', '2', 'Frequency', '2*pi*1.5e9', 'Phase', '0');
add_block('simulink/Sources/Sine Wave', [modelName '/LO2 Sine (Q-Branch)'], 'Amplitude', '-2', 'Frequency', '2*pi*1.5e9', 'Phase', 'pi/2');
add_block('simulink/Math Operations/Product', [modelName '/Mixer2_I']);
add_block('simulink/Math Operations/Product', [modelName '/Mixer2_Q']);

% 9. Baseband - Stage 7: BPF2 (-2.2dB)
add_block('simulink/Discrete/Discrete Filter', [modelName '/BPF2_I']);
add_block('simulink/Discrete/Discrete Filter', [modelName '/BPF2_Q']);
add_block('simulink/Math Operations/Gain', [modelName '/BPF2_Gain_I (-2.2 dB)'], 'Gain', '10^(-2.2/20)');
add_block('simulink/Math Operations/Gain', [modelName '/BPF2_Gain_Q (-2.2 dB)'], 'Gain', '10^(-2.2/20)');

% 10. Baseband - Stage 8: LNA3 (24.0dB - 1Vpp Target)
add_block('simulink/Math Operations/Gain', [modelName '/LNA3_I (24.0 dB)'], 'Gain', '10^(24.0/20)');
add_block('simulink/Math Operations/Gain', [modelName '/LNA3_Q (24.0 dB)'], 'Gain', '10^(24.0/20)');

% 11. Εξοδος
add_block('simulink/Math Operations/Real-Imag to Complex', [modelName '/Complex BB Builder']);
add_block('simulink/Sinks/Out1', [modelName '/Baseband Symbols (Complex)']);

% ======= ΣΥΝΔΕΣΙΜΟΤΗΤΑ (WIRING) =======
% Input -> Channel -> RF_BPF
add_line(modelName, 'RF Input (Real)/1', 'Channel Adder/1');
add_line(modelName, 'AWGN (Channel)/1', 'Channel Adder/2');
add_line(modelName, 'Channel Adder/1', 'RF_BPF/1');
add_line(modelName, 'RF_BPF/1', 'RF_BPF_Gain (-1.66 dB)/1');

% RF_BPF -> LNA1 -> IRF -> Mixer1
add_line(modelName, 'RF_BPF_Gain (-1.66 dB)/1', 'LNA1 (15.0 dB)/1');
add_line(modelName, 'LNA1 (15.0 dB)/1', 'IRF_Filter/1');
add_line(modelName, 'IRF_Filter/1', 'IRF_Gain (-0.4 dB)/1');
add_line(modelName, 'IRF_Gain (-0.4 dB)/1', 'Mixer1_IF/1');
add_line(modelName, 'LO1 (IF Mix)/1', 'Mixer1_IF/2');
add_line(modelName, 'Mixer1_IF/1', 'MIX1_Gain (-3.0 dB)/1');

% Mixer1 -> IF1_BPF -> LNA2
add_line(modelName, 'MIX1_Gain (-3.0 dB)/1', 'IF1_BPF/1');
add_line(modelName, 'IF1_BPF/1', 'IF1_BPF_Gain (-0.4 dB)/1');
add_line(modelName, 'IF1_BPF_Gain (-0.4 dB)/1', 'LNA2 (14.0 dB)/1');

% LNA2 -> Mixer2 (I/Q Split to BB)
add_line(modelName, 'LNA2 (14.0 dB)/1', 'Mixer2_I/1');
add_line(modelName, 'LNA2 (14.0 dB)/1', 'Mixer2_Q/1');
add_line(modelName, 'LO2 Cosine (I-Branch)/1', 'Mixer2_I/2');
add_line(modelName, 'LO2 Sine (Q-Branch)/1', 'Mixer2_Q/2');

% Mixer2 -> BPF2 (I/Q)
add_line(modelName, 'Mixer2_I/1', 'BPF2_I/1');
add_line(modelName, 'Mixer2_Q/1', 'BPF2_Q/1');
add_line(modelName, 'BPF2_I/1', 'BPF2_Gain_I (-2.2 dB)/1');
add_line(modelName, 'BPF2_Q/1', 'BPF2_Gain_Q (-2.2 dB)/1');

% BPF2 -> LNA3 (I/Q)
add_line(modelName, 'BPF2_Gain_I (-2.2 dB)/1', 'LNA3_I (24.0 dB)/1');
add_line(modelName, 'BPF2_Gain_Q (-2.2 dB)/1', 'LNA3_Q (24.0 dB)/1');

% LNA3 -> Complex Builder -> Out
add_line(modelName, 'LNA3_I (24.0 dB)/1', 'Complex BB Builder/1');
add_line(modelName, 'LNA3_Q (24.0 dB)/1', 'Complex BB Builder/2');
add_line(modelName, 'Complex BB Builder/1', 'Baseband Symbols (Complex)/1');

try
    Simulink.BlockDiagram.arrangeSystem(modelName);
catch
end

out_path = fullfile(pwd, [modelName '.slx']);
save_system(modelName, out_path);

disp('================================================================');
disp('ΕΠΙΤΥΧΙΑ! Το μοντέλο Simulink διορθώθηκε ώστε να έχει ακριβώς τη δομή της Superheterodyne C-τοπολογίας.');
disp('================================================================');
