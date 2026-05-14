#include "metrics.h"
#include <stdio.h>
#include <string.h>

int write_metrics_csv(const char* path, const StageMetric* metrics, size_t count) {
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    fprintf(f, "Stage,SNR(dB),EVM(%%),P_sig(dBm),P_noise(dBm),V_peak(V),V_rms(V)\n");
    for (size_t i = 0; i < count; i++) {
        fprintf(f, "%s,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f\n",
                metrics[i].stage, metrics[i].snr_db, metrics[i].evm_pct,
                metrics[i].p_sig_dbm, metrics[i].p_noise_dbm,
                metrics[i].v_peak, metrics[i].v_rms);
    }
    fclose(f);
    return 0;
}

void print_metrics(const char* title, const StageMetric* metrics, size_t count) {
    printf("\n--- %s ---\n", title);
    printf("%-20s | %10s | %10s | %10s | %10s\n", "Stage", "SNR(dB)", "EVM(%)", "P_sig(dBm)", "P_noise(dBm)");
    for (size_t i = 0; i < count; i++) {
        printf("%-20s | %10.2f | %10.2f | %10.2f | %10.2f\n",
               metrics[i].stage, metrics[i].snr_db, metrics[i].evm_pct,
               metrics[i].p_sig_dbm, metrics[i].p_noise_dbm);
    }
}
