# receiver_dual_sim - Πλήρης Επεξήγηση Κώδικα

## 1. Τι είναι αυτό το project

Το `receiver_dual_sim` είναι ένας προσομοιωτής δέκτη για δορυφορικό downlink
στα 24 GHz (K-band) με διαμόρφωση DVB-S2X 64-APSK. Υλοποιεί:

- **Μέρος Δ (Propagation Analysis)** — Απώλειες διάδοσης με πρότυπα ITU-R
- **Μέρος Ε (Cascade Analysis)** — Ανάλυση αλυσίδας δέκτη (Friis, IP3, DR)
- **Μέρος ΣΤ (Simulation)** — Προσομοίωση σήματος stage-by-stage με RF impairments

Ο κώδικας είναι γραμμένος σε C11 με OpenMP για παραλληλοποίηση. Παράγει
μετρήσεις SNR/EVM, διαγράμματα αστερισμού (constellation) και κυματομορφής
(traces) σε μορφή SVG/CSV για κάθε στάδιο του δέκτη.

## 2. Σενάριο δορυφορικού link

### 2.1 Παράμετροι σεναρίου

Οι τρέχουσες παράμετροι ορίζονται στο `main.c` (συνάρτηση `main()`)
και στο `propagation.h` (struct `PropagationScenario`):

| Παράμετρος | Τιμή | Επεξήγηση |
|-----------|------|-----------|
| Τροχιά | **GEO** | Γεωστατική, 36.000 km απόσταση |
| Συχνότητα | **24.0 GHz** | K-band (εξόδου δέκτη) |
| Απόσταση | **36.000 km** | Τυπικό slant range για γεωστατικό |
| Γωνία ανύψωσης | **30°** | Τυπική για Ευρώπη |
| Πόλωση | **45° (κυκλική)** | Συνηθισμένη σε SATCOM — προς επιβεβαίωση από ομάδα |
| EIRP | **85 dBm** | Από εκφώνηση για δορυφορικό link |
| Κεραία δορυφόρου | **Beyond Gravity 0.6m** | All-metal reflector, ESA space-qualified |
| Κέρδος κεραίας δορυφόρου | **~40 dBi** @ 25.5 GHz (41.0 dBi) | P_tx ≈ 45 dBm (32 W) για EIRP 85 dBm |
| Κεραία εδάφους | **ASC Signal 8.1m** | Δύο ανακλαστήρες, Gregorian |
| Κέρδος κεραίας εδάφους | **62.0 dBi** | Υπολογισμένο @ 24 GHz (βλ. §2.4) |
| Θερμοκρασία θορύβου κεραίας | **93 K** | Σε γωνία ανύψωσης 30° (από datasheet) |
| Πόλωση | **Κυκλική** | RHCP/LHCP, axial ratio 0.5 dB |
| Ευαισθησία δέκτη | **−61.88 dBm** | Υπολογισμένη από cascade analysis (Μέρος Ε) |
| Ρυθμός βροχής | **10 mm/h** | 0.01% exceedance, generic ITU reference |
| Πίεση επιφάνειας | **1013.25 hPa** | Στάθμη θαλάσσης |
| Θερμοκρασία | **288.15 K (15 °C)** | Standard |
| Υδρατμοί | **7.5 g/m³** | Reference |
| Ομίχλη | **0.05 g/m³** | Μέτρια ομίχλη (300 m ορατότητα) |

### 2.2 Πού να αλλάξετε παραμέτρους

Για διαφορετικό σενάριο, αλλάξτε τα πεδία στο block "Part D: Propagation
Analysis & Link Budget" στο `main.c`:

```c
prop_scenario.distance_km       = 36000.0;      // LEO: 500–2000, MEO: 20000
prop_scenario.elevation_deg     = 30.0;          // 5°–90°
prop_scenario.polarization_deg  = 45.0;          // 0°=horizontal, 90°=vertical
prop_scenario.rain_rate_mmh     = 10.0;          // ITU rain zone: K=42, M=63, N=95
prop_scenario.liquid_water_gm3  = 0.05;          // 0.5 = thick fog
```

ITU rain zones (0.01% exceedance):
- Zone K (Ν. Ευρώπη): 42 mm/h
- Zone M (Ν. Αφρική): 63 mm/h
- Zone N (Ισημερινός): 95 mm/h

### 2.3 GEO station-keeping και μετατόπιση

Οι γεωστατικοί δορυφόροι διατηρούνται εντός "κουτιού" ±0.05° έως ±0.1°
σε longitude/latitude, που αντιστοιχεί σε ~±75 km από τη θέση αναφοράς.

**Επίδραση στο link budget: αμελητέα.**
- Μεταβολή απόστασης ±75 km σε 36.000 km → FSPL μεταβάλλεται ±0.004 dB
- Γωνία ανύψωσης δεν επηρεάζεται πρακτικά
- Doppler στο GEO είναι σχεδόν μηδέν

Για LEO δορυφόρο (π.χ. Starlink 550 km), η μετατόπιση και το Doppler
θα ήταν σημαντικά και θα απαιτούσαν δυναμικό μοντέλο.

### 2.4 Υπολογισμός κέρδους κεραίας στα 24 GHz

**Δορυφορική κεραία (Tx):** Beyond Gravity 0.6m, all-metal reflector
- Band: 25.5–27.0 GHz
- Boresight gain: 41.0 dBi @ 25.5 GHz, 41.3 dBi @ 27.0 GHz
- Στα 24 GHz: ~40 dBi (εκτός ζώνης σχεδιασμού, εκτίμηση)
- Πόλωση: RHCP/LHCP
- Ισχύς πομπού: 45 dBm (32 W) για EIRP 85 dBm

**Κεραία εδάφους (Rx):** ASC Signal 8.1m, dual-reflector Gregorian, κυκλική πόλωση

**Υπολογισμός:**

```
Στα 24 GHz: λ = 0.0125 m, D = 8.1 m
Μέγιστο θεωρητικό κέρδος (100% απόδοση):
  G_max = 10·log₁₀((π·D/λ)²) = 66.2 dBi

Απόδοση διαφράγματος (aperture efficiency):
  Στα 20.7 GHz: 62.5 dBi → η = 57.7%
  Στα 30.5 GHz: 65.4 dBi → η = 51.8%
  Στα 24 GHz (παρεμβολή): η ≈ 55% → 63.6 dBi

Επιπλέον απώλειες εκτός ζώνης σχεδιασμού (~1.5 dB):
  Τροφοδότης βελτιστοποιημένος για 20.2-21.2 GHz και 30-31 GHz
  Στα 24 GHz: απώλειες προσαρμογής, spillover, phase center shift

Τελική εκτίμηση: 63.6 - 1.5 ≈ 62.0 dBi
```

**Επίδραση στο link budget:**
- Παλιά (40 dBi): Rx power = 85 − 222.83 + 40 = −97.83 dBm → Margin = −35.94 dB
- Νέα (62 dBi):  Rx power = 85 − 222.83 + 62 = −75.83 dBm → Margin = −13.95 dB

## 3. Δομή repository

```
receiver_dual_sim/
├── data_input/
│   └── receiver_config.csv        ← Ενιαίο αρχείο: topology + component specs
├── src/
│   ├── main.c                     ← Κύρια μηχανή (CLI, cascade, propagation, simulation)
│   ├── propagation.c              ← Μέρος Δ: FSPL, rain (P.838-3), fog (P.840-9), gas (P.676-13)
│   ├── cascade.c                  ← Μέρος Ε: Friis NF, IIP3, dynamic range, sensitivity
│   ├── component_catalog.c        ← Φόρτωση datasheet τιμών από το receiver_config.csv
│   ├── signal_chain.c             ← Επεξεργασία σήματος ανά στάδιο (gain, noise, nonlinearity)
│   ├── sim_baseband.c             ← Complex baseband analytical path
│   ├── stage_models.c             ← CSV-driven stage chain loader
│   ├── stage_artifacts.c          ← Δημιουργία SVG/CSV artifacts
│   ├── phase_noise.c              ← Phase noise (IIR 3-region model)
│   ├── iq_imbalance.c             ← I/Q gain/phase error
│   ├── flicker_noise.c            ← 1/f noise (Voss-McCartney)
│   ├── adc_model.c                ← ADC quantization + jitter
│   ├── biquad_filter.c            ← Butterworth IIR filter
│   ├── constellation.c            ← DVB-S2X 64-APSK constellation
│   ├── metrics.c                  ← SNR, EVM, mean power computations
│   ├── cli_args.c                 ← CLI flag parsing
│   ├── output_mgr.c               ← Directory creation/cleanup
│   └── prng.c                     ← xoshiro256** PRNG + ziggurat Gaussian
├── include/                       ← Header files (ένα .h ανά module)
├── matlab/
│   ├── sim_receiver_matlab.m      ← MATLAB reference simulation
│   ├── CascadeAnalyzer.m          ← Cascade analysis (από συνεργάτη)
│   ├── propagation_losses.m       ← Frequency sweep propagation (από συνεργάτη)
│   ├── build_simulink_model.m     ← Simulink model generator
│   ├── measure_i_vpp.m            ← I-component peak-to-peak (extracted)
│   ├── export_figure_png_svg.m    ← Figure export (extracted)
│   └── plot_trace_like_c.m        ← Time-domain trace plot (extracted)
├── docs/
│   ├── part_d_propagation.md      ← Τεκμηρίωση Μέρους Δ
│   └── part_e_cascade.md          ← Τεκμηρίωση Μέρους Ε
├── scripts/
│   └── run_component_sweep.py     ← batch sweep automation
├── Makefile & CMakeLists.txt      ← Build systems
└── run.sh                         ← Launcher (λειτουργεί από παντού)
```

## 4. Βασική ροή εκτέλεσης (C simulator)

```
main()
├── resolve_project_root()         ← Εύρεση απόλυτου μονοπατιού (για double-click)
├── link budget (Μέρος Δ)          ← FSPL + rain + fog + gas + link margin
├── stage_models_load_csv()        ← Φόρτωση receiver_config.csv
├── component_catalog_load()       ← Φόρτωση datasheet IIP3/OIP3/P1dB
├── component_catalog_override()   ← Διόρθωση stage model με catalog values
├── cascade analysis (Μέρος Ε)     ← Friis NF, IIP3 cascade, dynamic range
├── simulate_baseband()            ← Complex baseband path (προαιρετικό)
├── simulate_bruteforce_rf()       ← RF path (upconvert → stages → downconvert)
│   ├─ RRC pulse shaping
│   ├─ IQ upconversion @ 24 GHz
│   ├─ RF frontend stages (real)
│   ├─ Downconversion to baseband
│   └─ Post-mixer baseband stages
├── simulate_realistic_rf()        ← RF path + impairments
│   ├─ TX LO phase noise
│   ├─ RX LO phase noise
│   ├─ I/Q imbalance
│   ├─ Flicker noise
│   ├─ Butterworth filters
│   ├─ ADC quantization + jitter
│   └─ LO leakage DC offset
└── stage_artifacts                ← SVG/CSV output
```

## 5. Μέρος Δ — Propagation Analysis

Υλοποιεί 5 μοντέλα διάδοσης:

1. **FSPL**: `92.45 + 20·log₁₀(f_GHz) + 20·log₁₀(d_km)` dB
2. **Rain (ITU-R P.838-3)**: γ_R = k·R^α με curve-fit 4/5-όρων από Tables 1-4
3. **Fog (ITU-R P.840-9)**: Double-Debye μοντέλο διηλεκτρικής σταθεράς νερού
4. **Gas (ITU-R P.676-13)**: Reference values + pressure/temperature scaling
5. **Link margin**: EIRP − Σαπώλειες + G_rx − sensitivity

Default σενάριο: GEO (36.000 km), 30° elevation, 10 mm/h rain rate.

## 6. Μέρος Ε — Cascade Analysis

Υπολογίζει:

- **Friis NF**: F_total = F₁ + (F₂−1)/G₁ + (F₃−1)/(G₁·G₂) + …
- **IIP3 cascade**: 1/IIP3 = 1/IIP3₁ + G₁/IIP3₂ + G₁·G₂/IIP3₃ + …
- **Dynamic range**: LDR, SFDR, noise floor
- **Sensitivity**: kT₀B + NF_total + SNR_required

Οι τιμές IIP3 και P1dB προέρχονται από τα **datasheets** μέσω του
component_catalog module (LNA1 ADL8142S: OIP3=17.5 dBm, LNA2 SAV-541-DG+:
OIP3=27.8 dBm, Mixer1 HMC264LC3B: IIP3=14 dBm, κ.λπ.)

## 7. Δομή receiver_config.csv

Το ενιαίο αρχείο εισόδου `data_input/receiver_config.csv` περιέχει:

| Column | Περιγραφή |
|--------|-----------|
| `chain` | baseband_rx / rf_frontend / rf_postmix_bb |
| `name` | Όνομα σταδίου (π.χ. bb_02_lna1) |
| `gain_db` | Κέρδος σε dB |
| `nf_db` | Noise figure σε dB |
| `filter_len` | Μήκος φίλτρου (moving average taps) |
| `is_limiter` | 1 αν το στάδιο είναι limiter |
| `p1db_dbm` | Output P1dB (catalog value, output-referred) |
| `ip3_dbm` | Output IP3 (catalog OIP3, output-referred) |
| `ref` | out/in (reference convention για p1db/ip3) |
| `enabled` | 1 για ενεργό στάδιο |
| `component_uid` | Αναγνωριστικό component (π.χ. LNA_1_01) |
| `part_number` | Part number (π.χ. ADL8142S) |
| `oip3_db` | Output IP3 από datasheet |
| `iip3_db` | Input IP3 από datasheet (για mixers) |

## 8. Βασικοί τύποι δεδομένων

- `Complex` — Δύο doubles (`re`, `im`) για I/Q αναπαράσταση
- `StageMetric` — Stage label, signal/noise power, SNR (dB), EVM (%)
- `SimConfig` — Carrier, symbol rate, symbols, rolloff, input SNR, temperatures
- `StageModel` — name, gain_db, nf_db, filter_len, p1db_dbm, ip3_dbm, ref
- `PropagationScenario` — Frequency, distance, elevation, rain, temperature, κ.λπ.
- `CascadeResult` — Total gain/NF/IIP3, LDR, SFDR, sensitivity
- `CatalogEntry` — UID, part number, OIP3, IIP3, P1dB

## 9. Σήματα εξόδου

```
out/
├── rf_baseline/
│   ├── csv/              ← Αριθμητικές μετρήσεις ανά στάδιο
│   ├── constellations/   ← Διαγράμματα αστερισμού (SVG)
│   └── traces/           ← Κυματομορφές χρόνου (SVG)
└── realistic/
    ├── csv/
    ├── constellations/
    └── traces/
```

Κάθε αρχείο ονομάζεται με βάση το στάδιο και περιέχει SNR/EVM στο filename
για γρήγορη αναγνώριση (π.χ. `stage_03_bb_02_lna1_snr_16_83db_evm_5_05pct.svg`).

## 10. Build και Run

```bash
make                    # Build
./run.sh                # Run με default παραμέτρους
./run.sh --symbols 100 --disable-rf --disable-realistic  # Μόνο link budget + cascade
```

Το `run.sh` χρησιμοποιεί `dirname $0` για να λειτουργεί από οποιονδήποτε
φάκελο (απαραίτητο για double-click από file manager).

## 11. Κώδικας MATLAB

Το MATLAB reference βρίσκεται στο `matlab/`:
- `sim_receiver_matlab.m` — Κύρια αναφορά, ίδια αλυσίδα 11 σταδίων
- Τα helper functions (`measure_i_vpp`, `export_figure_png_svg`,
  `plot_trace_like_c`) βρίσκονται σε ξεχωριστά .m αρχεία για Octave compatibility
- `CascadeAnalyzer.m` και `propagation_losses.m` — από συνεργάτη
