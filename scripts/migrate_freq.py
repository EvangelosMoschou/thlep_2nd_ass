#!/usr/bin/env python3
"""Extract frequency ranges from cached component descriptions.

The Digi-Key API does not populate freq_min_hz/freq_max_hz in its
Product Information response for RF components.  However, the product
*description* almost always contains the operating frequency range
(e.g. "RF LNA 24GHz ~ 30GHz 3dB NF").

This script scans all cached components, extracts the frequency range
from the description field via regex, and writes it into the DB.

Usage:
    python3 scripts/migrate_freq.py

After migration, run ``SELECT COUNT(*) FROM components
WHERE freq_min_hz > 0`` to verify coverage.
"""

import re
import sqlite3
import sys
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPTS_DIR.parent

DB_PATH = _PROJECT_ROOT / "data_input" / "component_cache.db"
sys.path.insert(0, str(_SCRIPTS_DIR))
from component_db import _parse_freq_range, _parse_freq

# Pattern: explicit range like "10MHz ~ 6GHz", "2GHz-6GHz", "902MHz ~ 928MHz"
RE_RANGE = re.compile(
    r'(\d+\.?\d*\s*[MGk]?Hz\s*[~\-–—]+\s*\d+\.?\d*\s*[MGk]?Hz)',
    re.IGNORECASE,
)

# Also catch "DC-6GHz", "DC ~ 6GHz"
RE_DC_RANGE = re.compile(
    r'(DC\s*[~\-–]+\s*\d+\.?\d*\s*[MGk]?Hz)',
    re.IGNORECASE,
)


def update_freq_for_component(
    cur: sqlite3.Cursor, pn: str, desc: str,
) -> tuple[str, float, float] | None:
    if not desc:
        return None

    # 1. Try explicit range (most reliable)
    m = RE_RANGE.search(desc)
    if m:
        fmin, fmax = _parse_freq_range(m.group(1))
        if fmin > 0:
            cur.execute(
                "UPDATE components SET freq_min_hz=?, freq_max_hz=? WHERE part_number=?",
                (fmin, fmax, pn),
            )
            return (pn, fmin, fmax)

    # 2. Try DC-coupled range: "DC ~ 6GHz"
    m = RE_DC_RANGE.search(desc)
    if m:
        raw = m.group(1)
        raw_clean = re.sub(r'DC\s*[~\-–]+\s*', '', raw, flags=re.IGNORECASE)
        fmax = _parse_freq(raw_clean)
        if fmax > 0:
            # DC means from very low freq — set min to 0 to accept any target
            # We keep freq_min=0 so the component passes the filter
            # (a DC-coupled amp works at any RF frequency up to fmax)
            cur.execute(
                "UPDATE components SET freq_max_hz=? WHERE part_number=? AND (freq_max_hz=0 OR freq_max_hz>?)",
                (fmax, pn, fmax),
            )
            return (pn, 0.0, fmax)

    return None


def main():
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()

    cur.execute("SELECT part_number, description FROM components")
    rows = cur.fetchall()
    total = len(rows)

    updated = 0
    already = 0
    skipped = 0

    for pn, desc in rows:
        # Check if already populated
        cur2 = conn.execute(
            "SELECT freq_min_hz, freq_max_hz FROM components WHERE part_number=?",
            (pn,),
        )
        existing = cur2.fetchone()
        if existing and existing[0] > 0 and existing[1] > 0:
            already += 1
            continue

        result = update_freq_for_component(cur, pn, desc)
        if result:
            updated += 1
        else:
            skipped += 1

    conn.commit()

    print(f"Components processed: {total}")
    print(f"  Updated (range from desc): {updated}")
    print(f"  Already had freq data:     {already}")
    print(f"  No freq in description:    {skipped}")

    # Verify
    cur.execute("SELECT COUNT(*) FROM components WHERE freq_min_hz>0 AND freq_max_hz>0")
    with_freq = cur.fetchone()[0]
    print(f"\nComponents with freq data after migration: {with_freq}/{total}")

    conn.close()


if __name__ == "__main__":
    main()
