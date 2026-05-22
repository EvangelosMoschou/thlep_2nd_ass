#!/usr/bin/env python3
"""Initialize SQLite component cache DB and migrate JSON data.

Usage:
    python3 scripts/init_db.py
"""

import json
import sqlite3
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DB_PATH = PROJECT_ROOT / "data_input" / "component_cache.db"
JSON_PATH = PROJECT_ROOT / "data_input" / "component_cache" / "component_cache.json"

SCHEMA = """
CREATE TABLE IF NOT EXISTS components (
    part_number TEXT PRIMARY KEY,
    manufacturer TEXT,
    description TEXT,
    category TEXT,
    datasheet_url TEXT,
    product_url TEXT,
    gain_db REAL,
    nf_db REAL,
    insertion_loss_db REAL,
    oip3_dbm REAL,
    iip3_dbm REAL,
    p1db_dbm REAL,
    freq_min_hz REAL,
    freq_max_hz REAL,
    source_api TEXT,
    last_updated TEXT,
    raw_json TEXT
);

CREATE INDEX IF NOT EXISTS idx_category ON components(category);
CREATE INDEX IF NOT EXISTS idx_freq ON components(freq_min_hz, freq_max_hz);
"""

DB_COLUMNS = [
    "part_number",
    "manufacturer",
    "description",
    "category",
    "datasheet_url",
    "product_url",
    "gain_db",
    "nf_db",
    "insertion_loss_db",
    "oip3_dbm",
    "iip3_dbm",
    "p1db_dbm",
    "freq_min_hz",
    "freq_max_hz",
    "source_api",
    "last_updated",
]


def create_db(conn: sqlite3.Connection) -> None:
    conn.executescript(SCHEMA)
    print(f"Schema created at {DB_PATH}")


def migrate_json(conn: sqlite3.Connection) -> int:
    if not JSON_PATH.exists():
        print(f"JSON cache not found at {JSON_PATH}, skipping migration")
        return 0

    with open(JSON_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not data:
        print("JSON cache is empty, nothing to migrate")
        return 0

    placeholders = ", ".join(["?"] * (len(DB_COLUMNS) + 1))
    cols = ", ".join(DB_COLUMNS + ["raw_json"])
    sql = f"INSERT OR REPLACE INTO components ({cols}) VALUES ({placeholders})"

    rows = []
    for pn, entry in data.items():
        values = [entry.get(col, None) for col in DB_COLUMNS]
        values.append(json.dumps(entry))
        rows.append(values)

    conn.executemany(sql, rows)
    conn.commit()
    print(f"Migrated {len(rows)} components from JSON to SQLite")
    return len(rows)


def main():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(DB_PATH))
    try:
        create_db(conn)
        migrate_json(conn)
    finally:
        conn.close()
    print("Done.")


if __name__ == "__main__":
    main()
