# receiver_dual_sim - Πληρης Επεξηγηση Κωδικα

## 1. Τι ειναι αυτο το project

Το `receiver_dual_sim` ειναι ενας dual-path προσομοιωτης δεκτη για DVB-S2 64-APSK.
Μοντελοποιει την ιδια αλυσιδα δεκτη με δυο διαφορετικους τροπους:

- **Complex baseband path** (`baseband_rx`): γρηγορη προσεγγιση στο πεδιο συμβολων/μιγαδικου σηματος.
- **Brute-force RF path** (`rf_frontend` + `rf_postmix_bb`): ρητη RF προσομοιωση κυματομορφης με upconversion και downconversion.

Το project ειναι οργανωμενο ωστε να μπορεις να:

- αλλαζεις σταδια δεκτη απο CSV χωρις recompilation,
- τρεχεις C simulations και να παραγεις per-stage CSV/SVG artifacts,
- τρεχεις MATLAB reference μοντελο για constellation-focused validation,
- κανεις προαιρετικα αυτοματοποιημενα component sweeps απο catalog εξαρτηματων.


## 2. Δομη repository και ρολος καθε μερους

- `src/`
  - Βασικη C μηχανη (`main.c`) και υποστηρικτικα modules (`stage_models.c`, `stage_artifacts.c`, `prng.c`).
- `include/`
  - Public headers που μοιραζονται τα C source files.
- `stage_models/`
  - Runtime stage-chain CSVs και topology-design CSV που χρησιμοποιει το sweep.
- `data/`
  - Component catalog για sweep και MATLAB phase-noise lookup.
- `matlab/`
  - MATLAB reference simulation και script δημιουργιας Simulink μοντελου.
- `scripts/`
  - Script batch sweep automation (`run_component_sweep.py`).
- `bin/`
  - Built executables, κυριως το `dual_receiver_sim`.
- `out/`
  - Παραγομενα artifacts απο C και MATLAB runs.


## 3. End-to-end runtime ροη (C simulator)

Το βασικο binary χτιζεται απο:

- `src/main.c`
- `src/prng.c`
- `src/stage_models.c`
- `src/stage_artifacts.c`

Στο run-time, η ροη ειναι:

1. Parse των CLI arguments (seed, symbols, SNR, stage CSV path κ.λπ.).
2. Build του DVB-S2 64-APSK constellation και random symbol stream.
3. Υπολογισμος input budget βασισμενου σε thermal noise.
4. Κλιμακωση κυματομορφης στο φυσικο input level (περιλαμβανει conversion για 50 ohm).
5. Φορτωση των stage chains απο canonical CSV.
6. Εκτελεση complex baseband simulation (`baseband_rx`).
7. Εκτελεση brute-force RF simulation (`rf_frontend` -> downconvert -> `rf_postmix_bb`).
8. Εγγραφη per-stage artifacts (constellations/traces σε SVG + CSV).
9. Εκτυπωση stage metrics summary στο terminal.


## 4. Βασικοι τυποι δεδομενων (`include/sim_types.h`)

- `Complex`
  - Δυο doubles (`re`, `im`) για I/Q αναπαρασταση.

- `StageMetric`
  - Stage label, domain string, signal power, noise power, SNR (dB), EVM (%).

- `SimConfig`
  - Carrier, symbol rate, number of symbols, rolloff, input SNR, temperatures, RF sample rate, PRNG seed.

### 4.1 Βασικοι υπολογισμοι

Αυτα ειναι τα βασικα μαθηματικα που χρησιμοποιει ο κωδικας σε καθε stage:

- **Signal power**
  - Υπολογιζεται ως ο μεσος ορος του τετραγωνου του σηματος.
  - Για complex σηματα: `P_signal = mean(|x|^2)`.
  - Για real σηματα: `P_signal = mean(x^2)`.

- **Noise power**
  - Υπολογιζεται ως ο μεσος ορος του τετραγωνου της διαφορας μεταξυ received και reference σηματος.
  - Για complex σηματα: `P_noise = mean(|sig - ref|^2)`.
  - Για real σηματα: `P_noise = mean((sig - ref)^2)`.

- **SNR**
  - `SNR_dB = 10 * log10(P_signal / P_noise)`.
  - Αν το noise power γινει μηδεν, το SNR θεωρειται `+inf`.

- **EVM**
  - `EVM_% = sqrt(P_noise / P_signal) * 100`.
  - Δηλαδη, δειχνει ποσο μεγαλο ειναι το error σε σχεση με το useful signal.

- **Voltage ανα βαθμιδα**
  - Οταν μια βαθμιδα εχει gain σε dB, το amplitude scale ειναι `10^(gain_db/20)`.
  - Το power scale αντιστοιχει σε `10^(gain_db/10)`.
  - Στον κωδικα, το reference και το received signal κλιμακωνονται με το ιδιο voltage gain, ωστε να μενει συνεπης η σχεση σηματος και noise.
  - Για μετατροπη ισχυος σε voltage σε 50 ohm χρησιμοποιειται το `V_rms = sqrt(P * R)`.
  - Οταν υπαρχει `auto_gain_to_vpp`, το stage υπολογιζει gain ωστε το I-component peak-to-peak να φτασει στο `target_vpp`.


## 5. Stage model API (`include/stage_models.h` + `src/stage_models.c`)

### 5.1 Μοντελο stage και chain

- `StageModel`
  - `name`, `gain_db`, `nf_db`, `filter_len`, `auto_gain_to_vpp`, `target_vpp`.
- `StageChainId`
  - `STAGE_CHAIN_BASEBAND_RX`
  - `STAGE_CHAIN_RF_FRONTEND`
  - `STAGE_CHAIN_RF_POSTMIX_BB`
- `StageModelsConfig`
  - Περιεχει arrays και για τα 3 chains.

### 5.2 Συμπεριφορα loader

`stage_models_load_csv()`:

- Κανει auto-detect το schema απο το header:
  - **Canonical**: περιλαμβανει `chain` column.
  - **Legacy**: δεν εχει `chain`, χρησιμοποιει component-name mapping.
- Κανει parse required fields (`name/component`, `gain`, `nf`) και optional fields (`filter_len`, `auto_gain_to_vpp`, `target_vpp`, `enabled`).
- Υποστηριζει ευελικτη ονομασια header με normalization.
- Παραλειπει disabled rows (`enabled=0`).
- Ελεγχει οτι και τα 3 chains εχουν πληθυσμο.
- Χρησιμοποιει atomic update semantics: αντικαθιστα το output config μονο μετα απο επιτυχημενο full parse.

### 5.3 Legacy mapping

Το `map_legacy_component()` χαρτογραφει παλια ονοματα οπως `Filter 1`, `LNA 1`, `Mixer 1` κ.λπ. σε canonical chain entries.

### 5.4 Ιδιοκτησια μνημης

- Τα stage names δεσμευονται στο heap.
- Το `stage_models_free()` απελευθερωνει ολα τα chain arrays και names.
- Το `stage_models_get()` επιστρεφει εσωτερικους pointers (δεν γινεται free εξωτερικα).


## 6. Artifact API (`include/stage_artifacts.h` + `src/stage_artifacts.c`)

Αυτο το module χειριζεται formatting και παραγωγη αρχειων.

### 6.1 Naming και labels

- `humanize_stage_name()` μετατρεπει εσωτερικα stage names σε αναγνωσιμα labels.
- `slugify_text()` δημιουργει file-safe slugs.
- Τα metric tags ενσωματωνονται στα filenames οπου χρειαζεται.

### 6.2 CSV writers

- `write_constellation_csv()`
  - Αποθηκευει `idx, ref_i, ref_q, sig_i, sig_q`.
- `write_real_trace_csv()`
  - Αποθηκευει `idx, ref, sig`, με προαιρετικο truncation σε `max_points`.
- `write_metrics_csv()`
  - Γραφει stage metric tables.
- `write_input_budget_csv()`
  - Γραφει run-level budget παραμετρους.

### 6.3 SVG writers

- `write_constellation_svg()`
  - Dots/clouds constellation rendering με fixed MATLAB-like axis range `[-2, 2] x [-2, 2]`.
- `write_trace_svg()`
  - Real-signal trace plot με adaptive time window και interpolation sampling.
- `write_complex_trace_svg()`
  - I/Q trace plotting για complex streams, με δυνατοτητα overlay του προηγουμενου stage.
- `write_metrics_svg()`, `write_budget_svg()`
  - Συνοπτικα table/card style outputs.

### 6.4 Composite high-level calls που χρησιμοποιει το main

- `write_constellation_stage_artifacts()`
  - Χτιζει names και γραφει per-stage constellation CSV+SVG.
- `write_trace_stage_artifacts()`
  - Γραφει per-stage real trace CSV+SVG.
- `write_complex_trace_stage_artifacts()`
  - Γραφει complex I/Q trace SVG για postmix/baseband style signals.

### 6.5 Mermaid architecture export

- Το `write_chain_architecture_mermaid()` υπαρχει και μπορει να παραγει Mermaid διαγραμμα απο τα φορτωμενα stage chains.

### 6.6 Σημειωση για API που προς το παρον δεν χρησιμοποιειται

Οι συναρτησεις `write_metrics_csv()`, `write_metrics_svg()`, `write_input_budget_csv()`, `write_budget_svg()` και `write_chain_architecture_mermaid()` υπαρχουν και ειναι λειτουργικες, αλλα δεν καλουνται αυτη τη στιγμη απο το `main.c`.


## 7. PRNG module (`include/prng.h` + `src/prng.c`)

### 7.1 Πυρηνας generator

- Χρησιμοποιει **xoshiro256** με 256-bit state.
- Το seed expansion γινεται με **SplitMix64**.

### 7.2 Public functions

- `prng_seed(uint32_t seed)`
- `prng_uniform()` -> uniform `[0, 1)` double
- `prng_gauss()` -> standard normal, Box-Muller με cached spare sample
- `prng_uint32()` -> random 32-bit integer

### 7.3 Σχεδιαστικος σκοπος

- Deterministic reproducibility για symbol generation και AWGN.
- Single global state (οχι thread-safe by design για αυτο το project).


## 8. Εσωτερικα του βασικου simulator (`src/main.c`)

Το `main.c` περιεχει το orchestration και πολλες signal-processing βοηθητικες συναρτησεις.

### 8.1 Σημαντικες σταθερες

- `K_BOLTZMANN` για thermal noise.
- `SYSTEM_IMPEDANCE_OHM = 50.0` για power-voltage conversion.
- `OUTPUT_ROOT_DIR = "out"`.
- `MAX_METRICS = 32` χωρητικοτητα stage metrics ανα path.

### 8.2 Ομαδες βοηθητικων

- Filesystem/output housekeeping:
  - `ensure_dir_exists`, `ensure_output_dirs`, `clear_directory_contents`.
- Numeric helpers:
  - `db_to_lin`, `lin_to_db`.
- Constellation generation:
  - `dvbs2x_quadrant_angle`, `build_64apsk_constellation_dvbs2`, `normalize_constellation`.
- Signal primitives:
  - copy/scale functions, moving-average filters, BPF-like real filtering.
- Noise και metrics:
  - `add_awgn_complex`, `add_awgn_real`, `compute_metric_complex`, `compute_metric_real`.

### 8.3 MATLAB parity tracker

Το `MatlabParityMetricTracker` παρεχει MATLAB-aligned metric recursion:

- διατηρει cumulative gain/noise,
- ενημερωνει SNR/EVM ανα stage,
- εφαρμοζει matched-filter gain assumption στη μετατροπη EVM.

Χρησιμοποιειται για reported parity-style metrics, ενω τα trace plots μενουν βασισμενα στο raw φυσικο σημα.

### 8.4 Συμπεριφορα limiter

- Υπαρχουν soft-limiter helpers (`soft_limiter_envelope`, `apply_soft_limiter_*`).
- Ο limiter εφαρμοζεται μονο αν το stage name περιεχει `"limiter"`.
- Αν το τελικο stage λεγεται `rlm43_5w` (οπως στο τρεχον CSV), το limiter block δεν ενεργοποιειται, αρα η συμπεριφορα ειναι linear.

### 8.5 Λογικη εφαρμογης stage

- Τα `apply_stage_complex()` και `apply_stage_real()` υλοποιουν ανα stage:
  1. προαιρετικο filter,
  2. gain,
  3. NF-based local noise injection.

Ο χειρισμος noise χρησιμοποιει T0-referenced tracker, ωστε η προσθεση stage noise να ακολουθει Friis-like behavior.

### 8.6 RF conversion και synchronization

- `env_to_rf_real()`:
  - IQ upconversion σε real RF με oscillator recursion.
- `mix_down_and_lowpass()`:
  - RF downconversion πισω σε complex baseband.
- `synchronize_and_downsample()`:
  - timing offset scan + phase alignment + scale fit πριν την εξαγωγη συμβολων.

### 8.7 Βασικες μηχανες προσομοιωσης

- `simulate_complex_baseband()`:
  - Τρεχει το `baseband_rx` απευθειας σε complex symbols.
  - Εξαγει per-stage constellation και complex traces.

- `simulate_bruteforce_rf()`:
  - Κανει upsample, upconvert σε RF, προσθηκη RF noise και εφαρμογη RF stages.
  - Κανει downconvert και επεξεργασια postmix baseband stages σε oversampled streams.
  - Εξαγει RF traces και per-stage constellations.

### 8.8 CLI surface

Υποστηριζονται τα options:

- `--seed`
- `--symbols`
- `--symbol-rate`
- `--rf-fs`
- `--carrier`
- `--snr`
- `--stage-csv`
- `--topology-sim` / `--stage-sim` (legacy compatibility)

Default stage CSV:

- `stage_models/runtime_stage_models_target16.csv`


## 9. Stage CSV αρχεια (`stage_models/`)

### 9.1 Ενεργο runtime αρχειο

- `stage_models/runtime_stage_models_target16.csv`

Canonical στηλες:

- `chain`
- `name`
- `gain_db`
- `nf_db`
- `filter_len`
- `auto_gain_to_vpp`
- `target_vpp`
- `enabled`

Το τρεχον active profile περιλαμβανει:

- πολυ-σταδιακη baseband αλυσιδα,
- RF frontend chain,
- RF postmix chain,
- τελικο linear stage `rlm43_5w` με περιπου `-0.04 dB` gain και `0.04 dB` NF,
- ενεργο auto-gain στα LNA3 stages με `target_vpp = 1.6` ωστε μετα το linear τελικο stage να φτανει κοντα σε 1 Vpp.

### 9.2 Topology-design αρχειο

- `stage_models/stage_models.csv`

Αυτο ειναι design/topology περιγραφη που χρησιμοποιει το sweep script για να δημιουργει canonical runtime chain CSV υποψηφιους.


## 10. Component catalog (`data/component_catalog.csv`)

Περιεχει υποψηφια devices και χαρακτηριστικα οπως:

- στοιχεια ταυτοτητας (`component_uid`, `component_name`, `part_number`),
- gain/loss,
- NF,
- phase noise,
- πεδια σχετικα με P1dB και IP3.

Χρησιμοποιειται απο:

- `scripts/run_component_sweep.py` για combinatorial search,
- το MATLAB script για mixer LO phase-noise lookup.


## 11. MATLAB reference μοντελο (`matlab/sim_receiver_matlab.m`)

Αυτο το script δινει symbol-domain validation και plotting με CascadeAnalyzer-style assumptions.

### 11.1 Τι κανει

1. Οριζει simulation και link-budget σταθερες.
2. Χτιζει και κανει normalize το DVB-S2X 64-APSK constellation.
3. Δημιουργει pulse-shaped transmit waveform με ρητη RRC κατασκευη.
4. Επιβαλλει το ζητουμενο input level (προς το παρον 50 uV RMS σε 50 ohm).
5. Τρεχει stage-by-stage το chain model, συμπεριλαμβανοντας:
   - gain/NF noise additions,
   - προαιρετικο phase-noise injection απο catalog,
   - nonlinearity terms (IP3/P1dB) οπου υπαρχουν,
   - LNA3 steering για final linear RLM output target.
6. Υπολογιζει ανα stage SNR/EVM και waveform voltage στατιστικα.
7. Εξαγει:
   - key-stage constellation plots,
   - all-stage constellation plots,
   - C-like per-stage I/Q trace figures.

### 11.2 Helper functions στο τελος του αρχειου

- `export_figure_png_svg(...)`
- `measure_i_vpp(...)`
- `plot_trace_like_c(...)`


## 12. Simulink builder (`matlab/build_simulink_model.m`)

Δημιουργει προγραμματιστικα το `DVB_S2X_64APSK_Receiver_Sim.slx` με σταδιακη RF -> IF -> BB αλυσιδα blocks και τις διασυνδεσεις.

Σκοπος:

- reproducible δημιουργια Simulink topology,
- ευθυγραμμιση της block-level ροης με τη C αρχιτεκτονικη του δεκτη.


## 13. Sweep automation (`scripts/run_component_sweep.py`)

Αυτο το script εκτελει design-space exploration σε συνδυασμους εξαρτηματων.

### 13.1 Κυρια βηματα

1. Φορτωνει component επιλογες ανα role απο `data/component_catalog.csv`.
2. Φορτωνει design topologies απο `stage_models/stage_models.csv`.
3. Χτιζει Cartesian combinations ανα design.
4. Δημιουργει canonical stage CSV ανα combination.
5. Τρεχει το simulator και συλλεγει quality metrics.
6. Κανει ranking και export των top candidates.
7. Προαιρετικα ξανατρεχει τα top candidates με μεγαλυτερο symbol count.

### 13.2 Scoring

Υλοποιει:

- signal quality score (βασισμενο σε SNR/EVM),
- NF/IP3 score (Friis NF + cascaded IIP3 estimate),
- objective επιλογη: `hybrid`, `snr-evm`, `nf-ip3`.

### 13.3 Παραλληλισμος

- Χρησιμοποιει thread pool και slot queue (`topology_sim` slots) για να αποφευγονται συγκρουσεις.

### 13.4 Πρακτικη σημειωση συμβατοτητας

Το script αναμενει old-style outputs του simulator (`out/topology_sim_*/csv/stage_metrics_*.csv`).
Το τρεχον `main.c` γραφει per-stage artifacts σε `out/baseband` και `out/rf` και δεν παραγει by-default αυτα τα παλια summary CSV paths.
Αρα, για αξιοπιστη χρηση του sweep, ειτε:

- προσαρμοζεις το script στο τρεχον output contract, ειτε
- επαναφερεις στο C το legacy output layout/summary files.


## 14. Build και run (`Makefile`)

Targets:

- `make` / `make all`
  - Χτιζει το `bin/dual_receiver_sim`.
- `make run`
  - Τρεχει το binary με default ρυθμισεις.
- `make sweep`
  - Τρεχει το Python sweep script.
- `make clean`
  - Σβηνει το `bin/` και ξαναδημιουργει τους βασικους output φακελους.


## 15. Output artifacts (τρεχουσα συμπεριφορα)

### 15.1 C output

- `out/baseband/`
  - per-stage constellation CSV/SVG,
  - per-stage complex trace SVG.
- `out/rf/`
  - per-stage real trace CSV/SVG για RF-domain σημεια,
  - per-stage constellation CSV/SVG και postmix complex traces.

### 15.2 MATLAB output

- `out/matlab/`
  - constellation και trace figures (`.png` και `.svg`).


## 16. Σημαντικες φυσικες παραδοχες που ειναι encoded

1. Το input thermal noise υπολογιζεται με:

   - `P_noise = k * T_ant * B_noise`, με `B_noise = 200 MHz`.

2. Το voltage/power conversion γινεται με αναφορα 50 ohm:

   - `V_rms = sqrt(P * R)`.

3. Η signal waveform scaling συνδεεται με το link-budget power πριν απο stage processing.

4. Ο χειρισμος stage NF ειναι local ανα stage και propagated με T0 tracker.

5. Το τελικο RLM stage μοντελοποιειται ως linear attenuation/noise stage, εκτος αν οριστει ρητα/ονομαστικα ως limiter.


## 17. Πως να επεκτεινεις το project με ασφαλη τροπο

### 17.1 Προσθηκη η ρυθμιση receiver stages

- Κανε edit το `stage_models/runtime_stage_models_target16.csv`.
- Κρατα εγκυρα chain names (`baseband_rx`, `rf_frontend`, `rf_postmix_bb`).
- Βεβαιωσου οτι και τα 3 chains εχουν δεδομενα, αλλιως ο loader αποτυγχανει.

### 17.2 Προσθηκη νεου artifact type

- Υλοποιησε writer στο `stage_artifacts.c`.
- Προσθεσε declaration στο `include/stage_artifacts.h`.
- Καλεσε τον απο τις simulation engines στο `main.c` στα stage points που θες.

### 17.3 Προσθηκη νεου simulation metric

- Επεκτεινε το `StageMetric` στο `include/sim_types.h`.
- Ενημερωσε metric calculators και πινακες CSV/SVG writers.

### 17.4 Προσθηκη νεου component role για sweep

- Ενημερωσε τα role mapping dictionaries στο `scripts/run_component_sweep.py`.
- Ελεγξε οτι τα generated canonical stage rows παραμενουν συμβατα με τις απαιτησεις του loader.


## 18. Γρηγορος χαρτης troubleshooting

- **"CSV must define all chains"**
  - Το runtime stage CSV δεν εχει ολα τα chains η ενα chain εχει ολα τα rows disabled.

- **"Unknown chain" / malformed row**
  - Λαθος ονομα chain η ασυμβατοτητα header/μορφης στο CSV.

- **Πολυ λαθος output amplitude**
  - Ελεγξε τις παραδοχες input-level (uV RMS vs dBm equivalence και 50 ohm conversion).

- **Sweep returns no valid results**
  - Πιθανοτατα ασυμβατοτητα output-path contract μεταξυ sweep script και τρεχουσας C output δομης.


## 19. Ελαχιστες εντολες

Build και run C:

```bash
make
./bin/dual_receiver_sim --stage-csv stage_models/runtime_stage_models_target16.csv
```

Run MATLAB reference:

```bash
cd matlab
matlab -batch "sim_receiver_matlab"

**Συγκεντρωτικη εκθεση**:
- **Αρχη**: Καθαρο σημα (50 µV RMS).
- **Διαδρομη**: 11 stages με φιλτρα, ενισχυτες, και μειξερ.
- **Τελος**: Σημα με επαρκη amplitude (1-2 V, ανεξαρτητα απο input) και συσσωρευμενο θορυβο (κυριως απο την πρωτη LNA).
- **Constellation εξαγωγη**: Καθε stage CSV/SVG δειχνει την "στιγμη" του σηματος σε εκεινο το σημειο της αλυσιδας.
- **Metrics**: SNR και EVM ειδοποιουν για την υποβαθμιση της ποιοτητας καθε stage, αρκετα χρησιμα για να δουμε που χανεται το SNR.

```


## 20. Πρακτικη ανα-stage walkthrough: τι αλλαζει σε καθε σταδιο

Ας δουμε τι συμβαινει *στην πραγματικοτητα* με το σημα σε καθε stage του τρεχοντος receiver. Θα ακολουθησουμε το **baseband_rx chain** δια μεσου:

### 20.1 Input αναφορα

Η προσομοιωση αρχιζει με ενα καθαρο εξοδο του δεκτη RF (frontendα αποδωσης) στο level που περιγραφηκε:
- **Input level**: ~ 50 µV RMS (ανταποκρινει σε πολυ ασθενες δορυφορικο σημα).
- **Bandwidth**: 200 MHz (thermal noise floor).
- **Constellation**: DVB-S2X 64-APSK με 64 σημεια.
- **I/Q trace**: Clean, zero-error constellation αν δεν προσθεσουμε θορυβο.

Για σκοπους visualization, ολα τα stages του baseband_rx chain α γυναικες εξοδες σχεδιαζονται στο εσωτερικο του `out/baseband/` directory.

### 20.2 Εναρξη: bb_00_switch (αποκλιση -0.30 dB, NF 0.30 dB)

**Τι ειναι**: RF switch με πολυ μικρη απωλεια.

**Τι συμβαινει**:
- Το σημα μειωνεται κατα **-0.30 dB** (πολυ λιγη απωλεια).
- Προστιθεται **minimal noise**: εχει NF=0.30 dB αλλα μικρη ισχυς θορυβου σε αυτο το σταδιο.
- **Στο output**: Η constellation παραμενει επι των ιδιων σημειων (δεν υπαρχει ακομα σημαντικος θορυβος), αλλα η amplitude εχει κοπει λιγο.

**Artifacts**: `constellation_baseband_rx_bb_00_switch.csv` και `.svg` θα δειξουν τα 64 σημεια ακομα σε καθαρη διαταξη, απλα λιγο πιο μικρα.

### 20.3 Προενισχυση: bb_01_preselector_bpf (απωλεια -0.40 dB, NF 0.40 dB)

**Τι ειναι**: Bandpass filter που απομακρυνει εξω-ζωνης θορυβο και αλληλοπαρεμβολη.

**Τι συμβαινει**:
- Το σημα μειωνεται και παλι κατα **-0.40 dB**.
- Η pulse shape αρχιζει να "ομαλυνεται" λιγο απο το BPF.
- **Στα traces I/Q**: Η κυματομορφη χασει κομματι της υψηλης συχνοτητας τμηματος κατα την δειγματοληψια.

### 20.4 Πρωτη ενισχυση: bb_02_lna1 (ενισχυση +29 dB, NF 1.8 dB)

**Τι ειναι**: Low-Noise Amplifier με μεγαλη ενισχυση και χαμηλο θορυβο.

**Τι συμβαινει**:
- Το σημα μεγαλωνει κατα **+29 dB** (σχεδον 30x σε ισχυ).
- Θορυβος εισαγεται αλλα **ειναι ακριβως εδω που πρεπει**: πριν απο κανα αλλο μεγαλο ενισχυτη.
- **Στη constellation**: Τα σημεια μεγαλωνουν διαδραματικα, αλλα αν το SNR ειναι ακομα υψηλο, παραμενουν καθαρα διακριτα.
- **Στο voltage trace**: Η amplitude πολλαπλασιαζεται με `10^(29/20) ≈ 28x`.

**Στο CSV**: `stage_metrics_bb_02_lna1_* .csv` θα δειξει:
- SNR = αρχικο SNR (λογω της χαμηλης NF),
- Waveform amplitude = πολυ μεγαλυτερη απο τα προηγουμενα.

### 20.5 Μεσοσταδιακη φιλτραρισμα: bb_03_image_rejection_filter (απωλεια -3.5 dB, NF 3.5 dB)

**Τι ειναι**: Image-rejection filter για απομακρυνση του συζυγους frequency-mirrored σηματος.

**Τι συμβαινει**:
- Απωλεια **-3.5 dB** (αρκετα ισχυρη).
- Το σημα σμικρυνεται αλλα απομακρυνονται σημαντικες συχνοτητες εκτος ζωνης.
- **Στη constellation**: Ηδη ειδατε θορυβο/scatter απο την LNA1. Τωρα, η φιλτραρισμα αποφευγει περαιτερω αυξηση του εξω-ζωνης θορυβου.

### 20.6 Μειξη προς baseband: bb_04_mixer1 (απωλεια -9.0 dB, NF 9.0 dB)

**Τι ειναι**: Mixer που μεταφερει το RF σημα προς πολυ χαμηλες συχνοτητες (σχεδον baseband).

**Τι συμβαινει**:
- Απωλεια μειξης **-9.0 dB** (σημαντικη, αλλα αναμεναι).
- Το σημα γυρizes απο "RF" σε "complex baseband" αναπαρασταση.
- **Στη constellation**: Το scatter αρχιζει να γινεται περισσοτερο προφανες. Ενω η LNA προσθεσε θορυβο, ο mixer τον κανει ορατο με τη μειωση λαμδα.

### 20.7 Τσισματισμα: bb_05_bpf2 (απωλεια -0.6 dB, NF 0.6 dB)

**Τι ειναι**: Δευτερος BPF να εξα απο εξω-ζωνης παραγωγικοι του mixer (spur noise).

**Τι συμβαινει**:
- Μικρη απωλεια **-0.6 dB**.
- Κανει τριγωνο και εξομαλυνει τους spur.

### 20.8 Δευτερη ενισχυση: bb_06_lna2 (ενισχυση +23.7 dB, NF 0.27 dB)

**Τι ειναι**: Δευτερος LNA, αυτη τη φορα με πολυ χαμηλο θορυβο.

**Τι συμβαινει**:
- Ενισχυση **+23.7 dB** (σχεδον 24x σε ισχυ).
- **NF = 0.27 dB**: Πολυ καλο! Και αυτο ειναι το σημειο: απο εδω και περα, ο συνολικος θορυβος του δεκτη ειναι κυριως "προσθηκη" της πρωτης LNA.
- **Στη constellation**: Αν υπαρχει ηδη scatter, τωρα θα ειναι παρασταθει καλυτερα απο τη μεγαλη ενισχυση.

### 20.9 Τριτη μειξη: bb_07_mixer2 (απωλεια -6.5 dB, NF 6.5 dB)

**Τι ειναι**: Δευτερος mixer για περαιτερω κατε-μεσοσταδιακη.

**Τι συμβαινει**:
- Απωλεια **-6.5 dB**.
- Κι εδω ξανα, ο θορυβος προσθημενος απο την LNA2 γινεται προφανης με τη μειωση.

### 20.10 Τριτη φιλτραρισμα: bb_08_bpf3 (απωλεια -3.0 dB, NF 3.0 dB)

**Τι ειναι**: Τριτος BPF, συνηθως στενοτερος για τελικη αποκοπη θορυβου.

**Τι συμβαινει**:
- Απωλεια **-3.0 dB**.
- Αυτο ειναι ο τελευταιος "φιλτρο σταθμευσης" πριν απο τον τελικο ενισχυτη απο.

### 20.11 Τελικη ενισχυση με αυτοματη ρυθμιση: bb_09_lna3 (ενισχυση +50 dB, NF 3.5 dB, **auto_gain_to_vpp=1**)

**Τι ειναι**: Variable-gain amplifier που στοχευει στο να εξοδο του σηματος σε ενα καθορισμενο voltage.

**Τι συμβαινει**:
- Ενισχυση **+50 dB** (μεγαλη!).
- Αλλα μολονοτι, το `auto_gain_to_vpp=1` σημαινει οτι στο stage αυτο:
  - Μετριεται η amplitude του σηματος I/Q,
  - Υπολογιζεται ο απαραιτητος gain για να φτασει `target_vpp = 1.6` (voltage peak-to-peak),
  - Το actual gain που εφαρμοζεται ειναι αυτο που υπολογιστηκε, οχι σταθερα +50 dB.
- **Στη constellation**: Τα σημεια εχουν κυριως σταθερη amplitude (λογω της auto-gain steering).

**Σημαντικο**: Εδω ειναι που ο δεκτης αρχιζει να *προσαρμοζεται* στο ασυνθετο input level. Αν το SNR του εισερχ. σηματος δυσμενησ, η auto-gain θα "ενταθει" για να παρει καθαροτερη εικονα, αλλα θα μεγαλωσει και τον θορυβο.

### 20.12 Τελικο stage: bb_10_rlm43_5w (ενισχυση -0.04 dB, NF 0.04 dB)

**Τι ειναι**: Integrated linear RF module με πολυ μικρη απωλεια.

**Τι συμβαινει**:
- Ενισχυση **-0.04 dB** (σχεδον καμια απωλεια).
- Minimal noise (0.04 dB NF).
- **Στην εξοδο**: Το constellation σε αυτο το σταδιο ειναι ουσιαστικα το ιδιο οπως το input στο stage αυτο, γιατι το RLM ειναι *σχεδον διαφανες*.

---


---

## 21. Συνοψη

Αυτο το repository ειναι δομημενο γυρω απο τρια ευδιακριτα επιπεδα:

- **C path** για deterministic stage-by-stage runtime simulation και artifacts,
- **MATLAB path** για reference-level constellation validation και visualization,
- **CSV model layer** για γρηγορη αρχιτεκτονικη/component iteration χωρις recompilation.

Αν αυτα τα τρια layers μενουν ευθυγραμμισμενα (stage CSV, C assumptions, MATLAB assumptions), το project παραμενει ευκολο σε calibration και συγκριση μεταξυ εργαλειων.

Ναι, και απο το stage 6 και μετα ειναι φυσιολογικο να βλεπουμε I/Q αντι για μια μονο real κυματομορφη, γιατι εκει το σημα αντιμετωπιζεται ως complex baseband. Ετσι δεν κραταμε μονο το πλατος, αλλα και τη φαση του σηματος, που ειναι αναγκαια για να περιγραφει σωστα η πληροφορια μετα το mixing και τα υπολοιπα baseband stages. Με απλα λογια, το I και το Q ειναι οι δυο ορθογωνιες συνιστωσες του ιδιου σηματος, αρα μαζι μεταφερουν ολοκληρη την πληροφορια χωρις να χανεται τιποτα.

## 22. Γιατι βαζουμε το καθε component

Κανενα component δεν υπαρχει τυχαια. Καθε stage μπαινει για να λυσει ενα συγκεκριμενο πρακτικο προβλημα του receiver: απορριψη ανεπιθυμητων συχνοτητων, ενισχυση αδυναμου σηματος, μεταφορα σε χαμηλοτερη συχνοτητα, και τελικη σταθεροποιηση της amplitude.

### 22.1 Switch

Το switch μπαινει στην αρχη για να μοντελοποιησει την πρωτη επαφη του σηματος με το frontend. Στην πραξη εχει μικρη insertion loss, αλλα ειναι αναγκαιο για να δωσει ρεαλισμο στο input path και να δειξει οτι οχι ολα τα στοιχεια ειναι ιδανικα.

### 22.2 Preselector BPF

Το preselector bandpass filter μπαινει για να κοψει ο,τι βρισκεται εκτος της επιθυμητης ζωνης πριν το σημα ενισχυθει. Ετσι μειωνεται ο out-of-band θορυβος και προστατευονται τα επομενα stages απο ανεπιθυμητες παρεμβολες.

### 22.3 LNA1

Ο πρωτος low-noise amplifier μπαινει για να σηκωσει το πολυ αδυναμο εισερχομενο σημα πανω απο το noise floor τοσο γρηγορα οσο γινεται. Αυτο ειναι κρισιμο, γιατι ο πρωτος ενισχυτης καθοριζει σε μεγαλο βαθμο το συνολικο SNR της αλυσιδας.

### 22.4 Image rejection filter

Αυτο το filter μπαινει για να απομακρυνει το image του mixer και τα ανεπιθυμητα κατοπτρικα συνιστωσα που δημιουργουνται απο τη μετατροπη συχνοτητας. Χωρις αυτο, το mixer θα μετεφερε και λαθος τμηματα του φασματος μεσα στη ζητη που μας ενδιαφερει.

### 22.5 Mixer1

Ο πρωτος mixer μπαινει για να κατεβασει το σημα απο RF σε χαμηλοτερη συχνοτητα, ωστε να γινει πιο ευκολη η περαιτερω επεξεργασια του. Στο σημειο αυτο αρχιζει να φαινεται πιο καθαρα η complex baseband συμπεριφορα του σηματος.

### 22.6 BPF2

Το δευτερο bandpass filter μπαινει μετα το mixer για να καθαρισει τα mixer products και τα spur που εμφανιζονται στην μετατροπη. Ειναι ουσιαστικα ενα σταδιο “καθαρισμου” πριν απο την επομενη ενισχυση.

### 22.7 LNA2

Ο δευτερος LNA μπαινει για να ανακτησει το επιπεδο του σηματος μετα τις προηγουμενες απωλειες, αλλα με οσο το δυνατον χαμηλο noise figure. Ετσι το σήμα παραμενει χρησιμο για τα επομενα stages χωρις να χαθει η πληροφορια του.

### 22.8 Mixer2

Ο δευτερος mixer μπαινει για περαιτερω frequency translation, οταν χρειαζεται το σημα να μεταφερθει ακομα πιο κοντα στη ζητη τελικης επεξεργασιας. Σε αυτη τη φαση η αλυσιδα δουλευει πλεον πιο πολυ ως baseband-oriented chain παρα ως καθαρο RF path.

### 22.9 BPF3

Το τριτο filter μπαινει για τελικη αποκοπη residual θορυβου και ανεπιθυμητων συχνοτητων πριν απο το τελικο gain staging. Ειναι το τελευταιο “cleanup” σταδιο πριν την εξοδο.

### 22.10 LNA3 με auto-gain

Ο τριτος ενισχυτης μπαινει για να φερει το σημα στο επιθυμητο level εξοδου, χωρις να βασιζεται παντα σε σταθερο gain. Το auto-gain βοηθα να πετυχουμε συγκεκριμενο target Vpp, ωστε το output να ειναι συνεπες και ευκολο να συγκριθει με το MATLAB reference.

### 22.11 RLM43-5W

Το τελικο RLM stage μπαινει για να μοντελοποιησει ενα πραγματικο τελευταιο linear block με σχεδον μηδενικη επιπλεον υποβαθμιση. Με αυτο το stage το chain δινει μια ρεαλιστικη τελικη εξοδο που δειχνει οτι το συστημα δεν ειναι ιδανικο, αλλα παραμενει σχεδον διαφανο στο τελος της αλυσιδας.

## 23. Διαδικαστικες ρυθμισεις του simulation

Για να βγαζει νοημα το output, το simulator δουλευει με συγκεκριμενες διαδικαστικες ρυθμισεις που οριζουν ποσο «γρηγορα» δειγματοληπτει το σημα και με ποια συνθηκη το τρεχει.

### 23.1 Symbol rate

Το symbol rate καθοριζει ποσα συμβολα περνανε ανα δευτερολεπτο. Οσο μεγαλυτερο ειναι, τοσο πιο πυκνα ειναι τα changes του σηματος και τοσο πιο απαιτητικη γινεται η δειγματοληψια.

### 23.2 Sampling rate

Το RF sampling rate καθοριζει ποσα samples παιρνουμε ανα δευτερολεπτο στον RF κλαδο. Πρεπει να ειναι αρκετα μεγαλυτερο απο το symbol rate, ωστε να αποτυπωνονται σωστα οι μετατροπες συχνοτητας, το filtering και η shape της κυματομορφης χωρις aliasing.

### 23.3 Carrier frequency

Η carrier συχνοτητα οριζει σε ποιο RF σημειο «καθεται» το σημα πριν τη μετατροπη προς baseband. Χρησιμοποιειται κυριως για να γινει ρεαλιστικη η upconversion/downconversion ροη και να φανει καθαρα η διαφορα μεταξυ RF και baseband domain.

### 23.4 Input SNR

Το input SNR λεει ποσο δυνατο ειναι το σημα σε σχεση με τον thermal noise floor στην εισοδο. Αυτη η τιμη καθοριζει ποσο καθαρο η ποσο θορυβωδες θα ειναι το αρχικο constellation πριν απο τα stages της αλυσιδας.

### 23.5 PRNG seed

Το seed του PRNG καθοριζει ποιο συγκεκριμενο τυχαιο sequence θα παραχθει για symbols και noise. Με το ιδιο seed παιρνεις reproducible αποτελεσματα, ενω με αλλο seed βλεπεις μικρες φυσικες διακυμανσεις στο output.

### 23.6 Stage CSV

Το stage CSV καθοριζει τη σειρα των components, τα gains, τα noise figures και το αν καποιο stage θα κανει auto-gain. Αυτο ειναι το αρχειο που μετατρεπει το simulation απο γενικη ιδεα σε συγκεκριμενη αλυσιδα δεκτη.

### 23.7 Τι εχει μεσα ο κωδικας για αυτα

Στο `main.c` οι βασικες τιμες μπαινουν ως defaults και μετα μπορουν να αλλαξουν απο τα CLI flags:

- `cfg.symbol_rate_hz = 10.0e6` και δινει το `--symbol-rate`, που στο code χρησιμοποιειται για το symbol timing, το noise bandwidth handling και το downconversion/filter cutoff logic.
- `cfg.rf_sample_rate_hz = 192.0e9` και δινει το `--rf-fs`, που στον RF κλαδο καθοριζει τα samples per symbol ως `sps = round(rf_sample_rate_hz / symbol_rate_hz)`.
- `cfg.carrier_hz = 24.0e9` και δινει το `--carrier`, που χρησιμοποιειται στο `env_to_rf_real()` και στο `mix_down_and_lowpass()` για upconversion και downconversion.
- `cfg.input_snr_db = 20.0` και δινει το `--snr`, που μπαινει στον link budget υπολογισμο για να βγει η input noise power και το αντιστοιχο signal power.
- `cfg.seed = (unsigned int)time(NULL)` και δινει το `--seed`, που περναει στο `prng_seed(cfg.seed)` για να γινει το run reproducible οταν θες.
- `stage_csv_path = "stage_models/runtime_stage_models_target16.csv"` και δινει το `--stage-csv`, που μετα λυνεται με `resolve_stage_csv_path()` και φορτωνεται με `stage_models_load_csv()`.

Στην πραξη, το code κανει τα εξης βηματα:

1. Διαβαζει τα CLI arguments και αντικαθιστα τα defaults αν δωσεις δικιες σου τιμες.
2. Αρχικοποιει το PRNG με το seed.
3. Υπολογιζει το thermal noise με `k*T*B`, οπου το `B` ειναι fixed στα 200 MHz.
4. Υπολογιζει το input signal power απο το ζητουμενο SNR.
5. Κλιμακωνει τα tx symbols στην αντιστοιχη voltage αμplitude για 50 ohm.
6. Βγαζει το `sps` απο το ratio RF sample rate προς symbol rate και απο αυτο φτιαχνει το RF waveform.
7. Χρησιμοποιει το stage CSV για να ξερει ποια components θα περασουν απο την αλυσιδα και με ποια σειρα.

Αρα, στον κωδικα αυτα δεν υπαρχουν σαν αφηρημενες ιδεες: υπαρχουν σαν συγκεκριμενα πεδια του `SimConfig`, σαν CLI options, και σαν τιμες που επηρεαζουν αμεσα το PRNG, το budget, το oversampling και το stage processing.
