# Σημειώσεις Παρουσίασης — Receiver Dual SIM (C Implementation)

## 1. Impairments που έχουμε υλοποιήσει

Το σύστημα έχει **τρία μονοπάτια προσομοίωσης**:

| Μονοπάτι | Σκοπός |
|---|---|
| **Baseband** (`simulate_baseband`) | Γρήγορη αναλυτική προσομοίωση στο complex baseband — χωρίς RF impairments |
| **Brute-force RF** (`simulate_bruteforce_rf`) | Πλήρης RF upconversion/downconversion με θερμικό θόρυβο — βασικό RF, χωρίς extra impairments |
| **Realistic RF** (`simulate_realistic_rf`) | Ίδιο με το brute-force **συν 8 μοντέλα impairment** |

### Τα 8 impairments του realistic path

| # | Impairment | Υλοποίηση |
|---|---|---|
| 1 | **TX LO Phase Noise** | Περιστροφή φάσης I/Q πριν την upconversion. Παράμετροι: white floor −155 dBc/Hz, corner 100 kHz/10 kHz, κλίσεις −100/−80 dBc/Hz |
| 2 | **RX LO Phase Noise** | Περιστροφή φάσης στο baseband μετά τη downconversion. Παράμετροι: white floor −150 dBc/Hz, corner 50 kHz/5 kHz, κλίσεις −95/−75 dBc/Hz |
| 3 | **I/Q Imbalance** | Gain error 0.3 dB + phase error 1.5° μετά τον downconverter |
| 4 | **Flicker Noise (1/f)** | 1/f θόρυβος σε κάθε baseband stage μετά το φιλτράρισμα. Corner frequency 1 kHz |
| 5 | **Biquad Butterworth Filters** | Αντικαθιστά το moving-average filter με 4ης τάξης Butterworth όπου `filter_type == 1` |
| 6 | **AM-to-PM Conversion** | Phase shift ανάλογο της ισχύος εισόδου πάνω από το P1dB |
| 7 | **ADC Quantization + Jitter** | 12-bit ADC, full-scale 1 Vpp, jitter 1 ps. Εφαρμόζεται στα downsampled symbols |
| 8 | **LO Leakage DC Offset** | DC offset 1 mV στα I/Q symbols |

Επιπλέον, υπάρχουν **toggles** (`RealisticPathConfig`) για enable/disable των: LO phase noise, I/Q gain error, I/Q phase error, AM-to-PM, Butterworth.

---

## 2. Πώς εισάγεται ο θόρυβος σε κάθε στάδιο

### Στάδιο εισόδου (Antenna)
- **Baseband path**: AWGN στο complex envelope με βάση το `input_snr_db`
- **RF paths**: AWGN στο I/Q envelope μέσω `add_awgn_soa` με `P_noise_v2 = k·T_ant·B·R`

### Friis noise cascade (κάθε RF/baseband stage)
Η βασική συνάρτηση `apply_stage_real_fused` κάνει:
```
pn_add = N_t0 × g_lin × (F - 1) × (fs / (2 × B_noise))
```
όπου `F = 10^(NF/10)` (linear noise figure). Αυτό είναι **τύπος Friis**: κάθε στάδιο προσθέτει θόρυβο ανάλογο με το NF του, πολλαπλασιασμένο με το gain των προηγούμενων. Ο θόρυβος προστίθεται ως `sigma * prng_gauss()` στο σήμα.

### RF frontend stages (real domain)
Στα RF stages, εφαρμόζεται:
1. **Mixer** (αν `lo_hz > 0`): πολλαπλασιασμός RF με `cos(2π·f_lo/fs · i)`
2. **IP3 compression**: `s_scale *= (1 - 1/3 · p_in / IIP3)`
3. **P1dB compression**: `s_scale *= sqrt(P1dB_v2 / p_in)`
4. **AM-PM**: phase shift `coeff · (p_in_dBm - P1dB_dBm)` μοίρες
5. **Gain**: πολλαπλασιασμός με `sqrt(g_lin)`
6. **Θόρυβος Friis**: `s += sigma · prng_gauss()`
7. **Limiter**: αν `is_limiter`, πλάτος στο `max_amp`

### Post-mixer BB stages (realistic path)
Εδώ μπαίνουν τα επιπλέον impairments:
1. **Biquad φίλτρο** (αν `filter_type == 1`): Butterworth 4ης τάξης
2. **Flicker noise**: `fn = flicker_noise_generate()` προστίθεται σε I και Q
3. **RX Phase Noise** (μετά downconversion): phase rotation σε όλο το baseband
4. **I/Q Imbalance** (μετά downconversion): εφαρμογή gain/phase error
5. **ADC**: quantization + jitter στα downsampled symbols
6. **LO Leakage**: DC offset 1 mV

---

## 3. Τι έχουμε κάνει για ρεαλισμό

### Α. Φυσικά μοντέλα διάδοσης (ITU-R Standards)
Το `propagation.c` υλοποιεί τρία διεθνή πρότυπα:
- **ITU-R P.838-3**: Εξασθένηση βροχής με συντελεστές k, α (Gaussian-exponent curve-fit)
- **ITU-R P.840-9**: Εξασθένηση ομίχλης/νέφωσης
- **ITU-R P.676-13**: Εξασθένηση ατμοσφαιρικών αερίων (Ο₂, Η₂Ο)

### Β. Πραγματικά εξαρτήματα (Component Catalog)
Το `cascade.c` χρησιμοποιεί **πραγματικά datasheets**:
- LNA1: ADL8142S (NF 1.8 dB, IIP3 17.5 dBm)
- Mixer1: CMD179C3 / HMC264LC3B
- LNA2: SAV-541-DG+ (NF 0.27 dB!)
- Mixer2: SYM-25DHW+ (IIP3 30 dBm)
- LNA3: ZHL-20W-13SWX+ (gain 50 dB, IIP3 43 dBm)
- Κεραία: Viasat 13.5m (gain 67.2 dBi)

### Γ. Τρεις διακριτές διαδρομές προσομοίωσης
Το realistic path συγκρίνεται πάντα με το brute-force RF (ίδιο upconversion/downconversion αλλά χωρίς impairments) και το baseband (γρήγορο analytical). Αυτό επιτρέπει **απομόνωση της επίδρασης κάθε impairment**.

### Δ. Παράλληλη επεξεργασία (OpenMP)
Όλα τα computational-hot loops (stage processing, noise injection) χρησιμοποιούν `#pragma omp parallel for` για ρεαλιστικά μεγέθη προσομοίωσης (π.χ. 2.4M RF samples για 256 symbols × 9600 SPS).

### Ε. Structure-of-Arrays (SoA)
Τα buffers I/Q είναι ξεχωριστά `double*` arrays (όχι interleaved Complex) για καλύτερη cache locality — απαραίτητο για τα RF μεγέθη δεδομένων.

### ΣΤ. Πλήρης μετρική αλυσίδας
Κάθε στάδιο παράγει μετρήσεις **SNR (dB)**, **EVM (%)**, Constellation διαγράμματα (SVG), Trace κυματομορφών, και Φάσμα (SVG) — επιτρέποντας οπτική σύγκριση της υποβάθμισης σε κάθε βήμα.

### Ζ. Γεωστατικό σενάριο Heraklion
Οι προσομοιώσεις βασίζονται σε πραγματικά γεωγραφικά δεδομένα: Heraklion Crete (35.34°N, 25.14°E), slant range 37.138 km, elevation 48.9°, Ka-band 20 GHz, DVB-S2X 64-APSK.
