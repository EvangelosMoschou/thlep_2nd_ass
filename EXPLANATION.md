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

Υλοποιεί 5 μοντέλα διάδοσης με τις εξής θεωρητικές σχέσεις και παραμέτρους:

### 5.1 Ελεύθερος Χώρος (Free-Space Path Loss - FSPL)
$$FSPL = \left(\frac{4\pi d f}{c}\right)^2$$
Σε λογαριθμική κλίμακα (dB):
$$\text{FSPL (dB)} = 92.45 + 20\log_{10}(f_{\text{GHz}}) + 20\log_{10}(d_{\text{km}})$$
* **Συχνότητα ($f$):** $24.0 \text{ GHz}$ (K-band).
* **Απόσταση ($d$ - `distance_km`):** $36000.0 \text{ km}$ (GEO τροχιά).

### 5.2 Εξασθένηση Βροχής (Rain Attenuation - ITU-R P.838-3)
Η ειδική εξασθένηση βροχής $\gamma_R$ [dB/km] υπολογίζεται από τη σχέση:
$$\gamma_R = k \cdot R^\alpha$$
Η συνολική εξασθένηση στη λοξή διαδρομή (slant path) είναι:
$$A_{\text{rain}} = \gamma_R \cdot \frac{h_R}{\sin(\theta)}$$
* **Ρυθμός Βροχής ($R$ - `rain_rate_mmh`):** **$10.0\text{ mm/h}$** (Generic 0.01% exceedance). *(Στο MATLAB χρησιμοποιείται $45.0\text{ mm/h}$ για heavy rain).*
* **Γωνία Ανύψωσης ($\theta$ - `elevation_deg`):** **$44.0^\circ$** στο `main.c` (ζεύξη Αθήνα $\to$ SES-17) ή $30.0^\circ$ στο generic σενάριο.
* **Ύψος Βροχής ($h_R$):** Σταθερό στα **$4.0\text{ km}$** (ITU-R P.839).
* **Συντελεστές $k, \alpha$:** Υπολογίζονται από curve-fits 4 και 5 όρων με βάση το standard για κυκλική πόλωση ($\tau = 45^\circ$).

### 5.3 Εξασθένηση Νεφών/Ομίχλης (Fog/Cloud Attenuation - ITU-R P.840-9)
Η ειδική εξασθένηση $\gamma_c$ [dB/km] ορίζεται ως:
$$\gamma_c = K_l \cdot M$$
$$A_{\text{fog}} = \frac{\gamma_c}{\sin(\theta)}$$
* **Πυκνότητα Υγρού Νερού ($M$ - `liquid_water_gm3`):** **$0.05\text{ g/m}^3$** (μέτρια ομίχλη / 300 m ορατότητα).
* **Θερμοκρασία ($T$ - `surface_temp_k`):** **$288.15\text{ K}$** ($15^\circ\text{C}$ standard surface temperature). Εισέρχεται στο μοντέλο double-Debye για τον υπολογισμό της διηλεκτρικής σταθεράς του νερού $\epsilon'(f), \epsilon''(f)$, από την οποία προκύπτει ο συντελεστής $K_l$ [(dB/km)/(g/m³)].

### 5.4 Εξασθένηση Ατμοσφαιρικών Αερίων (ITU-R P.676-13)
$$\gamma_o = \gamma_{o,\text{ref}} \cdot \left(\frac{P}{1013.25}\right)^2 \cdot \left(\frac{288.15}{T}\right)^3 \quad \text{[dB/km]}$$
$$\gamma_w = \gamma_{w,\text{ref}} \cdot \left(\frac{\rho}{7.5}\right) \cdot \left(\frac{288.15}{T}\right)^{1.5} \quad \text{[dB/km]}$$
$$A_{\text{gas}} = \frac{\gamma_o \cdot h_o + \gamma_w \cdot h_w}{\sin(\theta)}$$
* **Πίεση Επιφάνειας ($P$ - `surface_pressure_hpa`):** **$1013.25\text{ hPa}$** (Sea level standard).
* **Πυκνότητα Υδρατμών ($\rho$ - `water_vapor_gm3`):** **$7.5\text{ g/m}^3$** (Reference).
* **Ισοδύναμα Ύψη ($h_o, h_w$):** $h_o = 6.0\text{ km}$ (οξυγόνο), $h_w = 2.0\text{ km}$ (υδρατμοί) στα 24 GHz.
* **Τιμές Αναφοράς:** $\gamma_{o,\text{ref}} = 0.0080\text{ dB/km}$, $\gamma_{w,\text{ref}} = 0.0850\text{ dB/km}$.

### 5.5 Κέρδος Κεραίας και Link Budget
$$P_{\text{rx}} = \text{EIRP} - A_{\text{total}} + G_{\text{rx}} \quad \text{[dBm]}$$
$$\text{Margin} = P_{\text{rx}} - \text{Sensitivity} \quad \text{[dB]}$$
* **EIRP:** $85.0\text{ dBm}$.
* **Κέρδος Κεραίας Λήψης ($G_{\text{rx}}$ - `rx_gain_dbi`):**
  * **$68.70\text{ dBi}$** στο `main.c` (Viasat 13.5m).
  * **$62.0\text{ dBi}$** (ASC Signal 8.1m με απόδοση $\eta \approx 55\%$).
    * *Υπολογισμός Κέρδους & Απόδοσης:*
      $$G_{\text{max}} = 10\log_{10}\left(\left(\frac{\pi D}{\lambda}\right)^2\right) = 66.2\text{ dBi} \quad (\text{για } D=8.1\text{m}, \lambda=0.0125\text{m})$$
      Με απόδοση διαφράγματος $\eta \approx 55\%$:
      $$G_{\text{eff}} = 66.2 + 10\log_{10}(0.55) = 63.6\text{ dBi}$$
      Αφαιρώντας $\sim 1.5\text{ dB}$ απώλειες εκτός ζώνης (spillover, matching): $G_{\text{rx}} = 62.0\text{ dBi}$.

---

## 6. Μέρος Ε — Cascade Analysis

Υπολογίζει τα χαρακτηριστικά της RF αλυσίδας του δέκτη:

### 6.1 Friis Noise Cascade (Θόρυβος)
$$F_{\text{total}} = F_1 + \frac{F_2 - 1}{G_1} + \frac{F_3 - 1}{G_1 G_2} + \dots + \frac{F_n - 1}{\prod_{i=1}^{n-1} G_i}$$
$$\text{NF}_{\text{total}} = 10\log_{10}(F_{\text{total}}) \quad \text{[dB]}$$
*(όπου $F_i = 10^{\text{NF}_i/10}$ και $G_i = 10^{\text{Gain}_i/10}$ σε γραμμική κλίμακα).*

### 6.2 IIP3 Cascade (Μη-γραμμικότητα)
$$\frac{1}{\text{IIP3}_{\text{total}}} = \frac{1}{\text{IIP3}_1} + \frac{G_1}{\text{IIP3}_2} + \frac{G_1 G_2}{\text{IIP3}_3} + \dots + \frac{\prod_{i=1}^{n-1} G_i}{\text{IIP3}_n}$$
*(όπου όλες οι τιμές ισχύος και κέρδους είναι σε γραμμική κλίμακα Watts και ratio. Αν δίνεται OIP3, μετατρέπεται σε $\text{IIP3} = \text{OIP3}/G$).*

### 6.3 Receiver Sensitivity (Ευαισθησία)
$$\text{Sensitivity (dBm)} = \text{kTB (dBm)} + \text{NF}_{\text{total}} + \text{SNR}_{\text{required}}$$
$$\text{kTB (dBm)} = 10\log_{10}(k \cdot T_0 \cdot B \cdot 1000) \approx -90.96 \text{ dBm}$$
* **Θερμοκρασία Αναφοράς ($T_0$):** $290.0\text{ K}$.
* **Εύρος Ζώνης Θορύβου ($B$):** $200.0\text{ MHz}$.
* **Απαιτούμενο SNR (DVB-S2X 64-APSK):** **$26.5\text{ dB}$**.
* **Υπολογισμένη Ευαισθησία:** **$-61.88\text{ dBm}$** (για $\text{NF}_{\text{total}} = 2.58\text{ dB}$).

### 6.4 Δυναμικό Εύρος (Dynamic Range)
* **Linear Dynamic Range (LDR):**
  $$\text{LDR} = P_{\text{1dB,out}} - N_0 \quad \text{[dB]}$$
* **Spurious-Free Dynamic Range (SFDR):**
  $$\text{SFDR} = \frac{2}{3} ( \text{OIP3}_{\text{total}} - N_0 ) \quad \text{[dB]}$$
  *(όπου $N_0$ είναι η ισχύς θορύβου εξόδου: $N_0 = k T_{\text{ant}} B G_{\text{total}} + \text{εσωτερικός θόρυβος} \approx -10.37\text{ dBm}$).*

Οι τιμές IIP3 και P1dB προέρχονται από τα **datasheets** μέσω του
`component_catalog` module (LNA1 ADL8142S: OIP3=17.5 dBm, LNA2 SAV-541-DG+:
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
