#ifndef PROPAGATION_H
#define PROPAGATION_H

#include <stddef.h>

typedef struct PropagationScenario {
    double frequency_hz;
    double distance_km;
    double elevation_deg;
    double polarization_deg;
    double rain_rate_mmh;
    double surface_temp_k;
    double surface_pressure_hpa;
    double water_vapor_gm3;
    double liquid_water_gm3;
    double eirp_dbm;
    double rx_gain_dbi;
    double rx_sensitivity_dbm;
} PropagationScenario;

typedef struct LinkBudgetResult {
    double fspl_db;
    double rain_specific_dbkm;
    double rain_slant_km;
    double rain_atten_db;
    double fog_specific_dbkm;
    double fog_atten_db;
    double gas_o2_dbkm;
    double gas_h2o_dbkm;
    double gas_o2_km;
    double gas_h2o_km;
    double gas_atten_db;
    double total_atten_db;
    double eirp_dbm;
    double rx_power_dbm;
    double rx_sensitivity_dbm;
    double link_margin_db;
} LinkBudgetResult;

void compute_link_budget(const PropagationScenario *scenario, LinkBudgetResult *budget);
void print_link_budget(const LinkBudgetResult *b);

#endif
