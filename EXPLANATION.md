# receiver_dual_sim - Πλήρης Επεξήγηση Κώδικα

## 1. Τι είναι αυτό το project

Το `receiver_dual_sim` είναι ένας προσομοιωτής δέκτη για δορυφορικό downlink
στα 20 GHz (K-band) με διαμόρφωση DVB-S2X 64-APSK. *(Το project σχεδιάστηκε
αρχικά στα 24 GHz· η βελτιστοποιημένη αλυσίδα λειτουργεί στα 20 GHz —
default carrier στο `src/main.c`, δεδομένα στο `data_input/20ghz/`.)* Υλοποιεί:

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
| Συχνότητα | **20.0 GHz** | K-band (εξόδου δέκτη) |
| Απόσταση | **36.000 km** | Τυπικό slant range για γεωστατικό |
| Γωνία ανύψωσης | **44°** | Τυπική για Αθήνα |
| Πόλωση | **45° (κυκλική)** | Συνηθισμένη σε SATCOM — προς επιβεβαίωση από ομάδα |
| EIRP | **85 dBm** | Από εκφώνηση για δορυφορικό link |
| Κεραία δορυφόρου | **Beyond Gravity 0.6m** | All-metal reflector, ESA space-qualified |
| Κέρδος κεραίας δορυφόρου | **~40 dBi** @ 25.5 GHz (41.0 dBi) | P_tx ≈ 45 dBm (32 W) για EIRP 85 dBm |
| Κεραία εδάφους | **Viasat 13.5m** | Full-motion ground station |
| Κέρδος κεραίας εδάφους | **68.70 dBi** | Operative τιμή στο `main.c` — Viasat 13.5m, απόδοση διαφράγματος ~65% (βλ. §2.4) |
| Θερμοκρασία θορύβου κεραίας | **91 K** | Σε γωνία ανύψωσης 44° (Viasat 13.5m, Athens→SES-17) |
| Πόλωση | **Κυκλική** | RHCP/LHCP, axial ratio 0.5 dB |
| Ευαισθησία δέκτη | **−61.80 dBm** | Υπολογισμένη από cascade analysis (Μέρος Ε) |
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
prop_scenario.elevation_deg     = 44.0;          // Athens→SES-17
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

### 2.4 Υπολογισμός κέρδους κεραίας εδάφους

**Δορυφορική κεραία (Tx):** Beyond Gravity 0.6m, all-metal reflector
- Band: 25.5–27.0 GHz
- Boresight gain: 41.0 dBi @ 25.5 GHz, 41.3 dBi @ 27.0 GHz
- Πόλωση: RHCP/LHCP
- Ισχύς πομπού: 45 dBm (32 W) για EIRP 85 dBm

**Κεραία εδάφους (Rx):** Viasat 13.5m, full-motion ground station, κυκλική πόλωση

**Υπολογισμός:**

```
Στην κύρια ζώνη σχεδιασμού του σταθμού (K-band, λ = 0.0125 m), D = 13.5 m:
Μέγιστο θεωρητικό κέρδος (100% απόδοση):
  G_max = 10·log₁₀((π·D/λ)²) = 10·log₁₀(3392.9²) ≈ 70.6 dBi

Απόδοση διαφράγματος η ≈ 65%:
  G_eff = 70.6 + 10·log₁₀(0.65) = 70.6 − 1.87 ≈ 68.7 dBi

Operative τιμή στο main.c: G_rx = 68.70 dBi
```

**Επίδραση στο link budget (τρέχουσα αλυσίδα @ 20 GHz):**
- P_rx = 85.00 − 215.59 + 68.70 = **−61.89 dBm**
- Margin = −61.89 − (−61.80) = **−0.09 dB ✗ ανεπάρκες**

*(Ιστορικό: στην αρχική μελέτη στα 24 GHz είχαν εξεταστεί τιμές ~40 dBi και
~62 dBi με margin −35.94 dB και −13.95 dB αντίστοιχα — πλέον χρησιμοποιείται
η βαθμονομημένη τιμή 68.70 dBi.)*

## 3. Δομή repository

```
receiver_dual_sim/
├── data_input/
│   ├── 20ghz/
│   │   ├── receiver.csv           ← Receiver chain: topology + datasheet specs
│   │   └── transmitter.csv        ← Transmitter chain configuration
│   ├── component_cache/           ← Digi-Key API cache
│   └── component_cache.db         ← Component database
├── src/
│   ├── main.c                     ← Κύρια μηχανή (CLI, cascade, propagation, simulation)
│   ├── propagation.c              ← Μέρος Δ: FSPL, rain (P.838-3), fog (P.840-9), gas (P.676-13)
│   ├── cascade.c                  ← Μέρος Ε: Friis NF, IIP3, dynamic range, sensitivity
│   ├── component_catalog.c        ← Φόρτωση datasheet τιμών από το receiver.csv
│   ├── signal_chain.c             ← Επεξεργασία σήματος ανά στάδιο (gain, noise, nonlinearity)
│   ├── sim_baseband.c             ← Complex baseband analytical path
│   ├── stage_models.c             ← CSV-driven stage chain loader
│   ├── stage_artifacts.c          ← Δημιουργία SVG/CSV artifacts
│   ├── phase_noise.c              ← Phase noise (IIR 3-region model, ανεξάρτητα Gaussian draws ανά κλάδο)
│   ├── iq_imbalance.c             ← I/Q gain/phase error
│   ├── flicker_noise.c            ← 1/f noise (Voss-McCartney)
│   ├── adc_model.c                ← ADC quantization + jitter
│   ├── biquad_filter.c            ← Butterworth IIR filter
│   ├── constellation.c            ← DVB-S2X 64-APSK constellation
│   ├── metrics.c                  ← SNR, EVM, mean power computations
│   ├── cli_args.c                 ← CLI flag parsing
│   ├── output_mgr.c               ← Directory creation/cleanup
│   └── prng.c                     ← xoshiro256** PRNG + exact Box-Muller Gaussian (τα παλιά ziggurat tables ήταν στατιστικά άκυρα και αντικαταστάθηκαν)
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
├── stage_models_load_csv()        ← Φόρτωση receiver.csv (π.χ. data_input/20ghz/receiver.csv)
├── component_catalog_load()       ← Φόρτωση datasheet IIP3/OIP3/P1dB
├── component_catalog_override()   ← Διόρθωση stage model με catalog values
├── cascade analysis (Μέρος Ε)     ← Friis NF, IIP3 cascade, dynamic range
├── simulate_baseband()            ← Complex baseband path (προαιρετικό)
├── simulate_bruteforce_rf()       ← RF path (upconvert → stages → downconvert)
│   ├─ RRC pulse shaping
│   ├─ IQ upconversion @ 20 GHz
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
* **Συχνότητα ($f$):** $20.0 \text{ GHz}$ (K-band).
* **Απόσταση ($d$ - `distance_km`):** $36000.0 \text{ km}$ (GEO τροχιά).

### 5.2 Εξασθένηση Βροχής (Rain Attenuation - ITU-R P.838-3)
Η ειδική εξασθένηση βροχής $\gamma_R$ [dB/km] υπολογίζεται από τη σχέση:
$$\gamma_R = k \cdot R^\alpha$$
Η συνολική εξασθένηση στη λοξή διαδρομή (slant path) είναι:
$$A_{\text{rain}} = \gamma_R \cdot \frac{h_R}{\sin(\theta)}$$
* **Ρυθμός Βροχής ($R$ - `rain_rate_mmh`):** **$10.0\text{ mm/h}$** (Generic 0.01% exceedance). *(Στο MATLAB χρησιμοποιείται $45.0\text{ mm/h}$ για heavy rain).*
* **Γωνία Ανύψωσης ($\theta$ - `elevation_deg`):** **$44.0^\circ$** (ζεύξη Αθήνα $\to$ SES-17).
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
* **Ισοδύναμα Ύψη ($h_o, h_w$):** $h_o = 6.0\text{ km}$ (οξυγόνο), $h_w = 2.0\text{ km}$ (υδρατμοί) στα 20 GHz.
* **Τιμές Αναφοράς:** $\gamma_{o,\text{ref}} = 0.0080\text{ dB/km}$, $\gamma_{w,\text{ref}} = 0.0850\text{ dB/km}$.

### 5.5 Κέρδος Κεραίας και Link Budget
$$P_{\text{rx}} = \text{EIRP} - A_{\text{total}} + G_{\text{rx}} \quad \text{[dBm]}$$
$$\text{Margin} = P_{\text{rx}} - \text{Sensitivity} \quad \text{[dB]}$$
* **EIRP:** $85.0\text{ dBm}$.
* **Κέρδος Κεραίας Λήψης ($G_{\text{rx}}$ - `rx_gain_dbi`):**
  * **$68.70\text{ dBi}$** στο `main.c` (Viasat 13.5m).
    * *Υπολογισμός Κέρδους & Απόδοσης:*
      $$G_{\text{max}} = 10\log_{10}\left(\left(\frac{\pi D}{\lambda}\right)^2\right) = 70.6\text{ dBi} \quad (\text{για } D=13.5\text{m}, \lambda=0.0125\text{m})$$
      Με απόδοση διαφράγματος $\eta \approx 65\%$:
      $$G_{\text{eff}} = 70.6 + 10\log_{10}(0.65) = 68.7\text{ dBi}$$

**Αριθμητική αλυσίδα (τρέχουσα εκτέλεση @ 20 GHz):**

```
A_total = FSPL + Rain + Fog + Gas = 209.60 + 5.66 + 0.02 + 0.31 = 215.59 dB
P_rx    = 85.00 − 215.59 + 68.70 = −61.89 dBm
Margin  = −61.89 − (−61.80) = −0.09 dB  ✗ INSUFFICIENT
```

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
* **Υπολογισμένη Ευαισθησία:** **$-61.80\text{ dBm}$** (για $\text{NF}_{\text{total}} = 2.67\text{ dB}$).

### 6.4 Δυναμικό Εύρος (Dynamic Range)
* **Linear Dynamic Range (LDR):**
  $$\text{LDR} = P_{\text{1dB,out}} - N_0 \quad \text{[dB]}$$
* **Spurious-Free Dynamic Range (SFDR):**
  $$\text{SFDR} = \frac{2}{3} ( \text{OIP3}_{\text{total}} - N_0 ) \quad \text{[dB]}$$
  *(όπου $N_0$ είναι η ισχύς θορύβου εξόδου: $N_0 = k T_{\text{ant}} B G_{\text{total}} + \text{εσωτερικός θόρυβος} \approx 61.26\text{ dBm}$ — output-referred, διογκωμένη από τη σύμβαση του σταδίου-κεραίας 67.20 dB ως τελευταίου κρίκου της αλυσίδας).*

**Τρέχουσες τιμές cascade (Part E):**

| Μεγέθος | Τιμή |
|---------|------|
| Total Gain | **151.57 dB** (περιλαμβάνει τη σειρά κεραίας 67.20 dB· μόνο ηλεκτρονικά ≈ 84.37 dB) |
| Total NF | **2.67 dB** |
| Total IIP3 | **−41.39 dBm** |
| OIP3_total | **110.18 dBm** (= −41.39 + 151.57) |
| Output P1dB | **99.58 dBm** |
| Output noise $N_0$ | **61.26 dBm** |
| LDR | **38.32 dB** (= 99.58 − 61.26) |
| SFDR | **32.61 dB** (= $\frac{2}{3}$(110.18 − 61.26)) |

Οι τιμές IIP3 και P1dB προέρχονται από τα **datasheets** μέσω του
`component_catalog` module (LNA1 ADL8142S: OIP3=17.5 dBm, LNA2 SAV-541-DG+:
OIP3=27.8 dBm, Mixer1 HMC264LC3B: IIP3=14 dBm, κ.λπ.)

## 7. Δομή receiver.csv

Το αρχείο εισόδου `data_input/<freq>/receiver.csv` (π.χ. `data_input/20ghz/receiver.csv`) περιέχει:

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

## 12. Τύποι Υπολογισμών (C Implementation)

Όλοι οι παρακάτω τύποι υλοποιούνται στο C source (`src/cascade.c`,
`src/propagation.c`, `src/main.c`) και αντιστοιχούν ακριβώς στις
παραμέτρους του receiver.

### 12.1 Φυσικές Σταθερές

| Σταθερά | Τιμή | Περιγραφή |
|---------|------|-----------|
| $k$ | $1.380649 \times 10^{-23}$ J/K | Σταθερά Boltzmann (K_BOLTZMANN) |
| $T_0$ | 290 K | Θερμοκρασία αναφοράς (t0_k) |
| $T_{\text{ant}}$ | 91 K | Ισοδύναμη θερμοκρασία θορύβου κεραίας (antenna_temp_k) |
| $B$ | 200 MHz | Ισοδύναμο εύρος ζώνης θορύβου (bw_hz, B_NOISE_HZ) |
| $R_s$ | 10 MHz | Symbol rate (symbol_rate_hz) |
| $\alpha$ | 0.2 | Roll-off factor RRC φίλτρου |
| $V_{\text{pp,out}}$ | 1.0 V | Επιθυμητή τάση εξόδου peak-to-peak |
| $\text{SNR}_{\text{req}}$ | 26.5 dB | Απαιτούμενο SNR για 64-APSK (DVB-S2X) |

### 12.2 Ισχύς Θορύβου Εισόδου (Antenna Noise)

Η ισχύς θερμικού θορύβου στην είσοδο του δέκτη από την κεραία:

$$N_i = k \cdot T_{\text{ant}} \cdot B \quad [\text{W}]$$

$$N_i(\text{dBm}) = 10 \log_{10}(k \cdot T_{\text{ant}} \cdot B) + 30$$

Για $T_{\text{ant}} = 91$ K, $B = 200$ MHz:

$$N_i = 1.38 \times 10^{-23} \cdot 91 \cdot 2 \times 10^8 = 2.51 \times 10^{-13} \, \text{W}$$

$$N_i(\text{dBm}) = 10 \log_{10}(2.51 \times 10^{-13}) + 30 = -96.00 \, \text{dBm}$$

> **Σημείωση:** Το Python optimizer και το C cascade analysis
> χρησιμοποιούν πλέον το ίδιο bandwidth $B = 200$ MHz
> (B_NOISE_HZ, physics.h). Είναι πλήρως συνεπή.

### 12.3 Θόρυβος Αναφοράς (kT₀B)

Θερμικό noise floor στη θερμοκρασία αναφοράς $T_0 = 290$ K:

$$\text{kT₀B}(\text{dBm}) = 10 \log_{10}(k \cdot T_0 \cdot B) + 30$$

$$\text{kT₀B}(\text{dBm}) = 10 \log_{10}(1.38 \times 10^{-23} \cdot 290 \cdot 2 \times 10^8) + 30 = -90.96 \, \text{dBm}$$

### 12.4 Friis Cascade — Gain

Ολικό κέρδος της αλυσίδας $N$ σταδίων:

$$G_{\text{total}} = \sum_{i=1}^{N} G_i \quad [\text{dB}]$$

όπου $G_i$ το κέρδος του $i$-οστού σταδίου σε dB.

Σε γραμμική κλίμακα (απαραίτητη για NF και IIP3 cascade):

$$g_i = 10^{G_i/10}$$

### 12.5 Friis Cascade — Noise Figure

Συντελεστής θορύβου $F_i$ κάθε σταδίου σε γραμμική κλίμακα:

$$F_i = 10^{\text{NF}_i/10}$$

Cascade noise figure κατά Friis:

$$F_{\text{total}} = F_1 + \frac{F_2 - 1}{g_1} + \frac{F_3 - 1}{g_1 g_2} + \cdots + \frac{F_N - 1}{g_1 g_2 \cdots g_{N-1}}$$

$$\text{NF}_{\text{total}}(\text{dB}) = 10 \log_{10}(F_{\text{total}})$$

**Ισοδύναμη θερμοκρασία θορύβου:**

$$T_e = T_0 \cdot (F_{\text{total}} - 1) \quad [\text{K}]$$

**Ισχύς θορύβου εξόδου:**

$$N_{\text{out}} = k \cdot (T_{\text{ant}} + T_e) \cdot B \cdot G_{\text{total}} \quad [\text{W}]$$

$$N_{\text{out}}(\text{dBm}) = 10 \log_{10}\left( k \cdot (T_{\text{ant}} + T_e) \cdot B \cdot G_{\text{total}} \right) + 30$$

### 12.6 IIP3 Cascade

Το IIP3 cascade υπολογίζεται σε γραμμική κλίμακα (mW):

$$\text{iip3}_i = 10^{\text{IIP3}_i/10} \quad [\text{mW}]$$

$$\frac{1}{\text{iip3}_{\text{total}}} = \frac{1}{\text{iip3}_1} + \frac{g_1}{\text{iip3}_2} + \frac{g_1 g_2}{\text{iip3}_3} + \cdots + \frac{g_1 g_2 \cdots g_{N-1}}{\text{iip3}_N}$$

$$\text{IIP3}_{\text{total}}(\text{dBm}) = 10 \log_{10}(\text{iip3}_{\text{total}})$$

$$\text{OIP3}_{\text{total}} = \text{IIP3}_{\text{total}} + G_{\text{total}} \quad [\text{dBm}]$$

> **Σημείωση:** Για στάδια χωρίς προδιαγραφή IIP3 (όπως φίλτρα,
> limiters), χρησιμοποιείται $\text{IIP3} = 100$ dBm (INFTY_DBM),
> ώστε να μην επηρεάζεται το cascade.

### 12.7 Receiver Sensitivity

$$S_{\text{min}}(\text{dBm}) = \text{kT₀B}(\text{dBm}) + \text{NF}_{\text{total}}(\text{dB}) + \text{SNR}_{\text{req}}(\text{dB})$$

Για το default configuration (NF = 2.67 dB):

$$S_{\text{min}} = -90.96 + 2.67 + 26.50 = -61.80 \, \text{dBm}$$

### 12.8 IIP3 → P1dB Approximation

Όταν δεν υπάρχει προδιαγραφή P1dB από datasheet, το C code εκτιμά:

$$\text{P1dB}_{\text{in}} \approx \text{IIP3}_{\text{total}} - 9.6 \quad [\text{dBm}]$$

$$\text{P1dB}_{\text{out}} = \text{P1dB}_{\text{in}} + G_{\text{total}} - 1.0 \quad [\text{dBm}]$$

Η προσέγγιση των 9.6 dB είναι η τυπική σχέση για ενισχυτές
(το P1dB είναι συνήθως 8-11 dB κάτω από το IIP3). Η αφαίρεση
1.0 dB στο output αντιστοιχεί στο compression του κέρδους.

> **Σημείωση (compression στην προσομοίωση):** Στο time-domain σήμα η
> πεπερασμένη πλάκας δεν γίνεται πλέον με hard-threshold clamp αλλά με το
> ομαλό μοντέλο **Rapp** ($p = 3$, $2p = 6$), στο πεδίο τάσης:
> $$\text{comp}(r) = \left(1 + \left(\frac{r}{r_s}\right)^{6}\right)^{-1/6}$$
> με $r_s$ βαθμονομημένο ώστε το compression στην τάση που αντιστοιχεί στο
> input P1dB να ισούται ακριβώς με $-1$ dB.

### 12.9 Dynamic Range

**Linear Dynamic Range (LDR):**

$$\text{LDR}(\text{dB}) = \text{P1dB}_{\text{out}}(\text{dBm}) - N_{\text{out}}(\text{dBm})$$

**Spurious-Free Dynamic Range (SFDR):**

$$\text{SFDR}(\text{dB}) = \frac{2}{3} \big( \text{OIP3}_{\text{total}}(\text{dBm}) - N_{\text{out}}(\text{dBm}) \big)$$

όπου η ισχύς εξόδου στο SFDR είναι:

$$P_{\text{out,SFDR}} = N_{\text{out}} + \frac{2}{3}(\text{OIP3} - N_{\text{out}}) \quad [\text{dBm}]$$

### 12.10 System Power Requirements

Ισχύς σήματος εισόδου για επιθυμητό input SNR:

$$S_i(\text{dBm}) = N_i(\text{dBm}) + \text{SNR}_{\text{target}}(\text{dB})$$

όπου $\text{SNR}_{\text{target}} = 20$ dB (default snr_target_db).

Ισχύς εξόδου για στόχο $V_{\text{pp}}$:

$$P_{\text{out}} = \frac{V_{\text{pp}}^2}{8 \cdot R_{\text{load}}} \quad [\text{W}]$$

$$P_{\text{out}}(\text{dBm}) = 10 \log_{10}\left( \frac{V_{\text{pp}}^2}{8 \cdot R_{\text{load}}} \right) + 30$$

με $R_{\text{load}} = 50 \, \Omega$.

### 12.11 Free Space Path Loss (FSPL)

$$L_{\text{fspl}} = \left( \frac{4 \pi d f}{c} \right)^2 \quad [\text{linear}]$$

$$L_{\text{fspl}}(\text{dB}) = 20 \log_{10}(4\pi) + 20 \log_{10}(d) + 20 \log_{10}(f) - 20 \log_{10}(c)$$

$$L_{\text{fspl}}(\text{dB}) = 92.45 + 20 \log_{10}(f_{\text{GHz}}) + 20 \log_{10}(d_{\text{km}})$$

Για $f = 20$ GHz, $d = 36000$ km (GEO):

$$L_{\text{fspl}} = 92.45 + 20 \log_{10}(20) + 20 \log_{10}(36000) = 92.45 + 26.02 + 91.13 = 209.60 \, \text{dB}$$

### 12.12 Rain Attenuation (ITU-R P.838-3)

Ειδική εξασθένηση βροχής:

$$\gamma_R = k \cdot R^\alpha \quad [\text{dB/km}]$$

όπου $R$ η ένταση βροχής σε mm/h, και $k, \alpha$ συντελεστές
που εξαρτώνται από τη συχνότητα και την πόλωση:

$$k = \frac{k_H + k_V + (k_H - k_V) \cos^2\theta \cos 2\tau}{2}$$

$$\alpha = \frac{k_H \alpha_H + k_V \alpha_V + (k_H \alpha_H - k_V \alpha_V) \cos^2\theta \cos 2\tau}{2k}$$

με $\theta$ την ανύψωση και $\tau$ τη γωνία πόλωσης ($\tau = 45^\circ$
για circular polarization).

Εξασθένηση slant-path:

$$A_{\text{rain}} = \frac{\gamma_R \cdot h_R}{\sin\theta} \quad [\text{dB}]$$

όπου $h_R \approx 4.0$ km το effective rain height (ITU-R P.839).

Default: $R = 10$ mm/h (0.01% exceedance), $\theta = 44^\circ$.

### 12.13 Fog/Cloud Attenuation (ITU-R P.840-9)

Συντελεστής εξασθένησης $K_l$:

$$K_l = \frac{0.819}{f''(1+\eta^2)} \quad [(\text{dB/km})/(\text{g/m}^3)]$$

όπου $\eta = \frac{2+\varepsilon'}{\varepsilon''}$, με $\varepsilon', \varepsilon''$
το διηλεκτρική σταθερά του νερού (double-Debye model).

Ειδική εξασθένηση:

$$\gamma_c = K_l \cdot M \quad [\text{dB/km}]$$

όπου $M$ η πυκνότητα υγρού νερού σε g/m³.

Εξασθένηση slant-path:

$$A_{\text{fog}} = \frac{\gamma_c}{\sin\theta} \quad [\text{dB}]$$

Default: $M = 0.05$ g/m³ (medium fog, ~300m ορατότητα), $\theta = 44^\circ$.

### 12.14 Atmospheric Gas Attenuation (ITU-R P.676-13)

Ειδική εξασθένηση οξυγόνου:

$$\gamma_{O_2} = f^2 \cdot 10^{-4} \left[ 1.08 \times 10^{-2} \cdot \frac{r_t^{3.5}}{r_p^2} + \frac{7.19 \times 10^{-3} \cdot r_t^{0.6}}{1 + 0.36 \cdot r_t^{0.6} \cdot (f/57)^2} \right] \quad [\text{dB/km}]$$

Ειδική εξασθένηση υδρατμών:

$$\gamma_{H_2O} = f^2 \cdot 10^{-4} \left[ 3.57 \times 10^{-3} \cdot r_t^{7.5} \cdot r_p^{-1} + 1.74 \times 10^{-3} \cdot r_t^{1.5} \right] \quad [\text{dB/km}]$$

όπου $r_t = 288/T$, $r_p = 1013/p$ με $T$ θερμοκρασία [K] και $p$ πίεση [hPa].

Ισοδύναμα ύψη:

$$h_{O_2} = 6 \, \text{km}, \quad h_{H_2O} = 2 \, \text{km}$$

Συνολική εξασθένηση:

$$A_{\text{gas}} = \frac{\gamma_{O_2} \cdot h_{O_2} + \gamma_{H_2O} \cdot h_{H_2O}}{\sin\theta} \quad [\text{dB}]$$

### 12.15 Received Power and Link Margin

Λαμβανόμενη ισχύς:

$$P_{rx}(\text{dBm}) = \text{EIRP}(\text{dBm}) - L_{\text{fspl}} - A_{\text{rain}} - A_{\text{fog}} - A_{\text{gas}} + G_{rx,\text{ant}}$$

όπου $\text{EIRP} = 85.00$ dBm (satellite), $G_{rx,\text{ant}} = 68.70$ dBi
(Viasat 13.5m — βλ. §2.4).

SNR εισόδου:

$$\text{SNR}_{\text{in}}(\text{dB}) = P_{rx}(\text{dBm}) - N_i(\text{dBm})$$

Link Margin (όπως υπολογίζεται στο `propagation.c`):

$$M(\text{dB}) = P_{rx}(\text{dBm}) - \text{Sensitivity}(\text{dBm})$$

**Παράδειγμα @ 20 GHz, $d = 36000$ km, $\theta = 44^\circ$:**

```
P_rx   = 85.00 − 209.60 − 5.66 − 0.02 − 0.31 + 68.70 = −61.89 dBm
Margin = −61.89 − (−61.80) = −0.09 dB  ✗ INSUFFICIENT
```

### 12.16 Noise Injection in Time-Domain Simulation (main.c)

Στο time-domain simulation, ο θόρυβος κάθε σταδίου προσομοιώνεται ως:

$$N_{\text{stage},i} = k \cdot T_0 \cdot B_{\text{noise}} \cdot g_i \cdot (F_i - 1) \cdot \frac{f_s}{2 \cdot B_{\text{noise}}} \quad [\text{W}]$$

όπου:
- $f_s = R_s \cdot \text{SPS}$ η actual sampling frequency
- $\text{SPS} = \text{round}(f_{rf}/R_s)$ samples per symbol ($= 9600$ για $f_{rf} = 96$ GHz, $R_s = 10$ MHz)
- $B_{\text{noise}} = 200$ MHz το equivalent noise bandwidth
- Ο όρος $f_s/(2 \cdot B_{\text{noise}})$ διορθώνει για το sampling rate

Ο θόρυβος εισάγεται ως complex Gaussian noise στο envelope,
με την ισχύ να κατανέμεται ισομερώς στα I και Q κλάδια.

**Συμβάσεις κλίμακας πυκνότητας θορύβου (συνεπείς σε όλη την αλυσίδα):**

| Domain | Κλιμάκωση διακύμανσης ανά δείγμα | Bandwidth αναφοράς |
|--------|----------------------------------|--------------------|
| Real passband | $\sigma^2 \propto f_s/(2B)$ | πραγματικό σήμα εκτείνεται έως $f_s/2$ |
| Complex baseband (μετά την decimation) | $\sigma^2 \propto f_{s,bb}/B$, με $f_{s,bb} = 80$ MHz | two-sided εύρος $f_{s,bb}$ |
| Complex envelope | $\sigma^2 \propto f_s/B$ | two-sided εύρος $f_s$ |

Με αυτές τις συμβάσεις η in-band ισχύς του θορύβου ισούται με $kT \cdot W$
για οποιοδήποτε εύρος $W$, ανεξάρτητα από το sampling rate του εκάστοτε
κλάδου.

### 12.17 EVM και SNR μέτρησης

Μετά από κάθε στάδιο, το EVM μετριέται ως:

$$\text{EVM}_{\%} = \sqrt{\frac{\frac{1}{N} \sum_{n=1}^{N} |s_n - \hat{s}_n|^2}{\frac{1}{N} \sum_{n=1}^{N} |\hat{s}_n|^2}} \times 100$$

όπου $s_n$ το λαμβανόμενο σύμβολο και $\hat{s}_n$ το ιδανικό σύμβολο.

$$\text{SNR}_{\text{dB}} = -20 \log_{10}\left(\frac{\text{EVM}_{\%}}{100}\right)$$

### 12.18 Παράμετροι Προσομοίωσης (Default Values)

| Παράμετρος | Τιμή | Μονάδα | Ορισμός |
|------------|------|--------|---------|
| $f_c$ | 20.0 | GHz | carrier_hz |
| $R_s$ | 10.0 | MHz | symbol_rate_hz |
| $\alpha$ | 0.2 | — | rolloff |
| $T_{\text{ant}}$ | 91 | K | antenna_temp_k |
| $T_0$ | 290 | K | t0_k |
| $B$ | 200 | MHz | bw_hz / B_NOISE_HZ |
| $f_{rf}$ | 96.0 | GHz | rf_sample_rate_hz |
| SPS | 9600 | — | round($f_{rf} / R_s$) |
| $d$ | 36000 | km | GEO distance |
| $\theta$ | 44 | deg | elevation |
| Rain $R$ | 10 | mm/h | 0.01% exceedance |
| Fog $M$ | 0.05 | g/m³ | medium fog |
| EIRP | 85 | dBm | satellite EIRP |
| SNR$_{\text{target}}$ | 20 | dB | input SNR για cascade analysis |
| SNR$_{\text{req}}$ (64-APSK) | 26.5 | dB | DVB-S2X threshold |
| $V_{\text{pp,out}}$ | 1.0 | V | target output level |
