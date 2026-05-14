#ifndef COMPONENT_CATALOG_H
#define COMPONENT_CATALOG_H

/*
 * component_catalog.h — Datasheet component catalog loader
 *
 * Loads data/component_catalog.csv which contains the actual RF component
 * parameters (gain, NF, P1dB, OIP3, IIP3) from manufacturer datasheets.
 *
 * Used by the cascade analysis to override P1dB/IIP3 values with correct
 * catalog-specified values instead of the (potentially wrong) runtime CSV.
 */

#include "stage_models.h"

#define CATALOG_MAX_ENTRIES 32

typedef struct CatalogEntry {
    char  uid[64];           /**< component_uid  (e.g. "LNA_1_01") */
    char  name[64];          /**< component_name (e.g. "LNA 1") */
    char  part[64];          /**< part_number    (e.g. "ADL8142S") */
    double gain_db;          /**< gain_loss_db */
    double nf_db;            /**< noise_figure_db (NaN if unspecified) */
    double p1db_dbm;         /**< output P1dB [dBm] (NaN if unspecified) */
    double oip3_dbm;         /**< output IP3  [dBm] (NaN if unspecified) */
    double iip3_dbm;         /**< input IP3   [dBm] (NaN if unspecified) */
} CatalogEntry;

typedef struct ComponentCatalog {
    CatalogEntry entries[CATALOG_MAX_ENTRIES];
    int          count;       /**< Number of valid entries loaded */
    char         filepath[256]; /**< Path to the loaded CSV */
} ComponentCatalog;

/**
 * Load component catalog from CSV file.
 * @param path   Path to component_catalog.csv
 * @param cat    [out] Filled catalog struct
 * @return 0 on success, -1 on error
 */
int component_catalog_load(const char *path, ComponentCatalog *cat);

/**
 * Look up a catalog entry by component UID.
 * @param cat   Loaded catalog
 * @param uid   UID string (e.g. "LNA_1_01")
 * @return Pointer to entry, or NULL if not found
 */
const CatalogEntry *component_catalog_find(const ComponentCatalog *cat,
                                           const char *uid);

/**
 * Override a stage model's P1dB and IIP3 with datasheet-correct values
 * from the component catalog.  Matching is done by substring search on
 * the stage name (e.g. a stage named "bb_02_lna1" matches catalog entry
 * "LNA_1_01" via the substring "lna1").
 *
 * The stage's gain_db is used to convert output-referred catalog values
 * (OIP3, P1dB) to input-referred values for the stage model.
 *
 * @param stg   [in/out] Stage model to override (ip3_dbm, p1db_dbm modified)
 * @param cat   Loaded component catalog (ignored if NULL)
 */
void component_catalog_override_stage(StageModel *stg,
                                      const ComponentCatalog *cat);

#endif /* COMPONENT_CATALOG_H */
