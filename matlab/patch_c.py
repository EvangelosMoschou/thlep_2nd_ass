import sys

path = "c_port/src/simulate_apsk.c"
with open(path, "r") as f:
    content = f.read()

old_func_start = "static int build_64apsk_constellation_dvbs2x("
old_func_end = "    return normalize_constellation(out, 64);\n}"

if old_func_start in content and old_func_end in content:
    start_idx = content.find(old_func_start)
    end_idx = content.find(old_func_end) + len(old_func_end)
    
    new_func = """static int build_64apsk_constellation_dvbs2(Complex* out, unsigned short* labels) {
    if (!out || !labels) return -1;

    /* DVB-S2 64-APSK: 4 rings of 4, 12, 20, 28 points */
    const int n_points[4] = {4, 12, 20, 28};
    const double ring_radii[4] = {1.0, 2.73, 4.52, 6.15};

    int idx = 0;
    for (int i = 0; i < 4; ++i) {
        int N = n_points[i];
        for (int k = 0; k < N; ++k) {
            double phase = (double)k * (2.0 * M_PI / N) + (M_PI / N);
            out[idx].re = ring_radii[i] * cos(phase);
            out[idx].im = ring_radii[i] * sin(phase);
            labels[idx] = (unsigned short)idx;
            idx++;
        }
    }

    return normalize_constellation(out, 64);
}"""
    content = content[:start_idx] + new_func + content[end_idx:]
    content = content.replace("build_64apsk_constellation_dvbs2x(c64, labels64)", "build_64apsk_constellation_dvbs2(c64, labels64)")

    with open(path, "w") as f:
        f.write(content)
    print("Patched successfully")
else:
    print("Could not find the function to patch")
