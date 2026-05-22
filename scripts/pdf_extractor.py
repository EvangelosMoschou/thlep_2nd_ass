#!/usr/bin/env python3
"""
pdf_extractor.py — RF Component Datasheet PDF Extractor

Extracts key parameters from RF component datasheets using regex pattern
matching on both free text and tables extracted from PDF files.

Target parameters:
    - Part number, Manufacturer
    - Frequency range (Hz)
    - Gain (dB), Noise Figure (dB)
    - P1dB (dBm), OIP3 (dBm), IIP3 (dBm)

Usage:
    from pdf_extractor import extract_pdf, extract_all_pdfs

    # Single PDF
    result = extract_pdf("pdfs/PMA5-83-2W+.pdf")
    print(result["part_number"], result["oip3_dbm"])

    # All PDFs in a directory
    results = extract_all_pdfs("pdfs/")
    for r in results:
        print(r["part_number"], r["gain_db"])
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path


# ──────────────────────────────────────────────────────────────────────
# REGEX PATTERNS
# ──────────────────────────────────────────────────────────────────────

# --- Part number & manufacturer ---
RE_PART_NUMBER = re.compile(
    r"(?:(?:Part\s*(?:Number|#|No)\s*[=:])|(?:Model\s*[=:]))\s*([A-Za-z0-9][A-Za-z0-9\-+/.]+)",
    re.I,
)
RE_MANUFACTURER = re.compile(
    r"(?:Manufacturer|Company|Vendor|Brand|Mfr)\s*[=:]\s*([A-Za-z0-9\s.]+)",
    re.I,
)
# Mini-Circuits / Analog Devices / etc often in header
RE_MFR_KNOWN = re.compile(
    r"(Mini[- ]?Circuits|Analog\s*Devices|Qorvo|Skyworks|Maxim|\bTI\b|\bNXP\b|Infineon|Cree|Wolfspeed|MACOM|Microchip|Renesas|\bADI\b|Hittite)",
    re.I,
)

# --- Frequency range ---
# "0.01 to 10 GHz", "DC-6 GHz", "20-40 MHz", "0.01 10 GHz"
RE_FREQ_RANGE = re.compile(
    r"(?:Frequency\s*(?:Range|Operation|Coverage))\s*[=:≈]?\s*"
    r"([0-9.]+)\s*(?:\s*to\s*|-|–|–|\s+)\s*([0-9.]+)\s*(GHz|MHz|kHz)",
    re.I,
)
RE_FREQ_RANGE_SHORT = re.compile(
    r"([0-9.]+)\s*(?:\s*to\s*|-|–|–|\s+)\s*([0-9.]+)\s*(GHz|MHz|kHz)",
)

# --- OIP3 (Output IP3) ---
RE_OIP3 = re.compile(
    r"(?:OIP3?|Output\s*(?:Third-?Order\s*)?IP3?|Output\s*(?:Third-?Order\s*)?Intercept)"
    r"[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)
RE_OIP3_FEATURE = re.compile(
    r"(?:High\s+)?OIP3?[\s,;:=≈]+(?:Typ\s*\.?\s*)?[=:≈]?\s*([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)

# --- IIP3 (Input IP3) ---
RE_IIP3 = re.compile(
    r"(?:IIP3?|Input\s*(?:Third-?Order\s*)?IP3?|Input\s*(?:Third-?Order\s*)?Intercept)"
    r"[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)
RE_IIP3_FEATURE = re.compile(
    r"(?:High\s+)?IIP3?[\s,;:=≈]+(?:Typ\s*\.?\s*)?[=:≈]?\s*([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)

# --- IP3 (generic) ---
RE_IP3_GENERIC = re.compile(
    r"(?:IP3|TOI|Third-?Order\s*Intercept)"
    r"[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)

# --- P1dB ---
RE_P1DB = re.compile(
    r"(?:P1dB|P-?1\s*dB|1\s*dB\s*Compression\s*(?:Point)?)"
    r"[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dBm",
    re.I,
)
RE_P1DB_FEATURE = re.compile(
    r"([+-]?\d+\.?\d*)\s*dBm\s*(?:\w+\s+){0,3}P1dB",
    re.I,
)

# --- Gain ---
RE_GAIN = re.compile(
    r"(?:Gain|Small\s*Signal\s*Gain)"
    r"[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dB\b(?!m)",
    re.I,
)

# --- Noise Figure ---
RE_NF = re.compile(
    r"(?:Noise\s*Figure|NF)[\s,;:=≈]*(?:Typ\s*\.?\s*[=:≈]?\s*)?([+-]?\d+\.?\d*)\s*dB\b(?!m)",
    re.I,
)

# --- Supply voltage ---
RE_SUPPLY_V = re.compile(
    r"Supply\s*(?:Voltage|V\w*)\s*[=:≈]?\s*([+-]?\d+\.?\d*)\s*V\b",
    re.I,
)

# --- Supply current ---
RE_SUPPLY_I = re.compile(
    r"(?:Supply\s*)?(?:Current|I\w*)\s*[=:≈]?\s*(\d+\.?\d*)\s*(mA|A)\b",
    re.I,
)

# ──────────────────────────────────────────────────────────────────────
# TABLE-AWARE PATTERNS (for columnar data in pdfplumber tables)
# ──────────────────────────────────────────────────────────────────────

# When we find a row with these keywords, extract numeric values
TABLE_KEYWORDS = {
    "gain": re.compile(r"^(?:gain|small\s*signal\s*gain)", re.I),
    "p1db": re.compile(r"^(?:p1db|output\s*power.*1\s*dB|1\s*dB\s*comp)", re.I),
    "psat": re.compile(r"^(?:psat|output\s*power.*sat)", re.I),
    "nf": re.compile(r"^(?:noise\s*figure|nf)\b", re.I),
    "oip3": re.compile(r"^(?:oip3|output\s*ip3)", re.I),
    "iip3": re.compile(r"^(?:iip3|input\s*ip3)", re.I),
    "frequency": re.compile(r"^(?:frequency|freq)", re.I),
}

# ──────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────


def _freq_to_hz(value: float, unit: str) -> float:
    """Convert frequency value to Hz."""
    unit = unit.lower().strip()
    if unit == "ghz":
        return value * 1e9
    elif unit == "mhz":
        return value * 1e6
    elif unit == "khz":
        return value * 1e3
    return value


def _first_match(pattern: re.Pattern, text: str, group: int = 1) -> str | None:
    """Return first regex match group, or None."""
    m = pattern.search(text)
    return m.group(group) if m else None


def _all_matches(pattern: re.Pattern, text: str, group: int = 1) -> list[str]:
    """Return all regex match groups."""
    return [m.group(group) for m in pattern.finditer(text)]


def _clean_part_number(raw: str) -> str:
    """Clean up extracted part number."""
    return raw.strip().rstrip(".,;:").upper()


def _parse_table_for_param(rows: list[list[str]], param_re: re.Pattern) -> float | None:
    """Search table rows for a parameter and return the typical value."""
    TYP_COL = 3  # Standard: [Param, Cond, Min, Typ, Max, Units]
    for row in rows:
        if not row or not row[0]:
            continue
        if param_re.search(row[0].strip()):
            if len(row) > TYP_COL:
                cell = row[TYP_COL].strip()
                # Handle multi-line cells ("17.9\n12.3\n...") — take first value
                first_val = cell.split("\n")[0].strip()
                try:
                    return float(first_val)
                except ValueError:
                    pass
            # Fallback: scan all cells for any numeric
            for cell in row[1:]:
                for part in cell.split("\n"):
                    part = part.strip()
                    try:
                        return float(part)
                    except ValueError:
                        continue
    return None


# ──────────────────────────────────────────────────────────────────────
# MAIN EXTRACTION
# ──────────────────────────────────────────────────────────────────────


def extract_pdf(pdf_path: str | Path) -> dict:
    """
    Extract component parameters from a single datasheet PDF.

    Args:
        pdf_path: Path to PDF file.

    Returns:
        dict with keys:
            pdf_path, part_number, manufacturer,
            freq_min_hz, freq_max_hz,
            gain_db, nf_db, p1db_dbm, oip3_dbm, iip3_dbm,
            supply_voltage, supply_current,
            confidence (per-field: "exact" | "table" | "estimated" | "not_found"),
            raw_text (first 2000 chars for debugging)
    """
    pdf_path = Path(pdf_path)

    # Default result
    result: dict = {
        "pdf_path": str(pdf_path),
        "part_number": pdf_path.stem.upper(),
        "manufacturer": None,
        "freq_min_hz": None,
        "freq_max_hz": None,
        "gain_db": None,
        "nf_db": None,
        "p1db_dbm": None,
        "oip3_dbm": None,
        "iip3_dbm": None,
        "supply_voltage": None,
        "supply_current": None,
        "confidence": {
            "part_number": "filename",
            "manufacturer": "not_found",
            "freq_range": "not_found",
            "gain_db": "not_found",
            "nf_db": "not_found",
            "p1db_dbm": "not_found",
            "oip3_dbm": "not_found",
            "iip3_dbm": "not_found",
        },
        "raw_text_preview": "",
    }

    if not pdf_path.exists():
        result["error"] = f"File not found: {pdf_path}"
        return result

    try:
        import pdfplumber
    except ImportError:
        result["error"] = "pdfplumber not installed (pip install pdfplumber)"
        return result

    try:
        all_text_parts = []
        all_tables = []

        with pdfplumber.open(pdf_path) as pdf:
            for page in pdf.pages:
                # Extract text
                text = page.extract_text() or ""
                all_text_parts.append(text)

                # Extract tables
                tables = page.extract_tables()
                if tables:
                    all_tables.extend(tables)

        full_text = "\n".join(all_text_parts)
        result["raw_text_preview"] = full_text[:2000]

        # ── Part number (from text, override filename if found) ──
        pn = _first_match(RE_PART_NUMBER, full_text)
        if pn:
            result["part_number"] = _clean_part_number(pn)
            result["confidence"]["part_number"] = "exact"

        # ── Manufacturer ──
        mfr = _first_match(RE_MANUFACTURER, full_text)
        if mfr:
            result["manufacturer"] = mfr.strip().title()
            result["confidence"]["manufacturer"] = "exact"
        else:
            mfr = _first_match(RE_MFR_KNOWN, full_text)
            if mfr:
                raw = mfr.strip()
                # Fix common casing
                casing = {"minicircuits": "Mini-Circuits", "adi": "Analog Devices",
                          "ti": "Texas Instruments", "nxp": "NXP"}
                result["manufacturer"] = casing.get(raw.lower(), raw.title())
                result["confidence"]["manufacturer"] = "exact"

        # ── Frequency range ──
        fm = _first_match(RE_FREQ_RANGE, full_text)
        if fm:
            # Won't work because _first_match returns only one group
            pass
        freq_m = RE_FREQ_RANGE.search(full_text)
        if freq_m:
            fmin = float(freq_m.group(1))
            fmax = float(freq_m.group(2))
            unit = freq_m.group(3)
            result["freq_min_hz"] = _freq_to_hz(fmin, unit)
            result["freq_max_hz"] = _freq_to_hz(fmax, unit)
            result["confidence"]["freq_range"] = "exact"

        # ── OIP3 ──
        oip3 = _first_match(RE_OIP3, full_text) or _first_match(RE_OIP3_FEATURE, full_text)
        if oip3:
            result["oip3_dbm"] = float(oip3)
            result["confidence"]["oip3_dbm"] = "exact"

        # ── IIP3 ──
        iip3 = _first_match(RE_IIP3, full_text) or _first_match(RE_IIP3_FEATURE, full_text)
        if iip3:
            result["iip3_dbm"] = float(iip3)
            result["confidence"]["iip3_dbm"] = "exact"

        # ── Generic IP3 (if OIP3/IIP3 not found) ──
        if result["oip3_dbm"] is None and result["iip3_dbm"] is None:
            ip3 = _first_match(RE_IP3_GENERIC, full_text)
            if ip3:
                # Default to OIP3 if generic
                result["oip3_dbm"] = float(ip3)
                result["confidence"]["oip3_dbm"] = "exact"

        # ── P1dB ──
        p1 = _first_match(RE_P1DB, full_text) or _first_match(RE_P1DB_FEATURE, full_text)
        if p1:
            result["p1db_dbm"] = float(p1)
            result["confidence"]["p1db_dbm"] = "exact"

        # ── Gain ──
        gain = _first_match(RE_GAIN, full_text)
        if gain:
            result["gain_db"] = float(gain)
            result["confidence"]["gain_db"] = "exact"

        # ── Noise Figure ──
        nf = _first_match(RE_NF, full_text)
        if nf:
            result["nf_db"] = float(nf)
            result["confidence"]["nf_db"] = "exact"

        # ── Supply ──
        sv = _first_match(RE_SUPPLY_V, full_text)
        if sv:
            result["supply_voltage"] = sv + " V"

        si_m = RE_SUPPLY_I.search(full_text)
        if si_m:
            val = si_m.group(1)
            unit = si_m.group(2)
            result["supply_current"] = f"{val} {unit}"

        # ── TABLE-BASED EXTRACTION ──
        for table in all_tables:
            rows = []
            for row in table:
                if row:
                    rows.append([(c or "").strip() for c in row])

            if not rows:
                continue

            # Freq range from table
            if result["freq_min_hz"] is None:
                for row in rows:
                    if row and row[0] and re.search(r"frequency\s*range", row[0], re.I):
                        if len(row) >= 5:
                            try:
                                fmin = float(row[2])
                                fmax = float(row[4])
                                unit = row[5] if len(row) > 5 else "GHz"
                                result["freq_min_hz"] = _freq_to_hz(fmin, unit)
                                result["freq_max_hz"] = _freq_to_hz(fmax, unit)
                                result["confidence"]["freq_range"] = "table"
                            except ValueError:
                                pass

            # Gain from table
            if result["gain_db"] is None:
                g = _parse_table_for_param(rows, TABLE_KEYWORDS["gain"])
                if g is not None:
                    result["gain_db"] = g
                    result["confidence"]["gain_db"] = "table"

            # P1dB from table
            if result["p1db_dbm"] is None:
                p = _parse_table_for_param(rows, TABLE_KEYWORDS["p1db"])
                if p is not None:
                    result["p1db_dbm"] = p
                    result["confidence"]["p1db_dbm"] = "table"

            # NF from table
            if result["nf_db"] is None:
                n = _parse_table_for_param(rows, TABLE_KEYWORDS["nf"])
                if n is not None:
                    result["nf_db"] = n
                    result["confidence"]["nf_db"] = "table"

        # ── ESTIMATE missing values ──
        # OIP3 from P1dB + 10 (rule of thumb)
        if result["oip3_dbm"] is None and result["p1db_dbm"] is not None:
            result["oip3_dbm"] = result["p1db_dbm"] + 10.0
            result["confidence"]["oip3_dbm"] = "estimated"

        # IIP3 from OIP3 - Gain
        if result["iip3_dbm"] is None and result["oip3_dbm"] is not None and result["gain_db"] is not None:
            result["iip3_dbm"] = result["oip3_dbm"] - result["gain_db"]
            result["confidence"]["iip3_dbm"] = "estimated"

        return result

    except Exception as e:
        result["error"] = str(e)
        return result


def extract_all_pdfs(pdf_dir: str | Path = "pdfs") -> list[dict]:
    """
    Extract parameters from all PDFs in a directory.
    Skips already-cached files (json sidecar).

    Args:
        pdf_dir: Directory containing PDF files.

    Returns:
        List of extracted component dicts.
    """
    pdf_dir = Path(pdf_dir)
    if not pdf_dir.exists():
        print(f"[pdf_extractor] Directory not found: {pdf_dir}")
        return []

    results = []
    pdf_files = sorted(pdf_dir.glob("*.pdf"))

    if not pdf_files:
        print(f"[pdf_extractor] No PDF files found in {pdf_dir}")
        return []

    print(f"[pdf_extractor] Found {len(pdf_files)} PDF(s) in {pdf_dir}")

    for pdf_path in pdf_files:
        json_path = pdf_path.with_suffix(".json")

        # Skip if already cached
        if json_path.exists():
            try:
                with open(json_path) as f:
                    data = json.load(f)
                results.append(data)
                print(f"  [CACHED] {pdf_path.name}")
                continue
            except (json.JSONDecodeError, IOError):
                pass  # Re-extract if cache corrupt

        print(f"  [EXTRACT] {pdf_path.name}...", end=" ")
        data = extract_pdf(pdf_path)

        # Save cache
        try:
            cache_data = {k: v for k, v in data.items() if k != "raw_text_preview"}
            with open(json_path, "w") as f:
                json.dump(cache_data, f, indent=2)
        except IOError as e:
            print(f"cache write failed: {e}")

        results.append(data)
        print("done")

    return results


def pretty_print(result: dict) -> None:
    """Print extracted data in human-readable format."""
    pn = result.get("part_number", "?")
    print(f"\n{'='*50}")
    print(f"  Part: {pn}")
    print(f"{'='*50}")

    fields = [
        ("Manufacturer", "manufacturer"),
        ("Freq Range", "freq_range_human"),
        ("Gain", "gain_db", " dB"),
        ("Noise Figure", "nf_db", " dB"),
        ("P1dB", "p1db_dbm", " dBm"),
        ("OIP3", "oip3_dbm", " dBm"),
        ("IIP3", "iip3_dbm", " dBm"),
        ("Supply", "supply_voltage"),
        ("Current", "supply_current"),
    ]

    # Build freq range string
    if result.get("freq_min_hz") is not None and result.get("freq_max_hz") is not None:
        fmin = result["freq_min_hz"] / 1e9
        fmax = result["freq_max_hz"] / 1e9
        result["freq_range_human"] = f"{fmin:.2f} – {fmax:.2f} GHz"
    else:
        result["freq_range_human"] = None

    for field_info in fields:
        label = field_info[1]
        key = field_info[1]
        suffix = field_info[2] if len(field_info) > 2 else ""

        value = result.get(key)
        conf = result.get("confidence", {}).get(key, "")

        if value is not None:
            conf_mark = {"exact": "✓", "table": "≈", "estimated": "~", "filename": "?"}.get(conf, "?")
            print(f"  {conf_mark} {field_info[0]:20s}: {value}{suffix}")
        else:
            print(f"  ✗ {field_info[0]:20s}: (not found)")

    if "error" in result:
        print(f"\n  ERROR: {result['error']}")
    print()


# ──────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────


def main():
    """CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Extract RF component parameters from datasheet PDFs")
    parser.add_argument("pdfs", nargs="*", help="PDF file(s) to extract (default: all in pdfs/)")
    parser.add_argument("--dir", default="pdfs", help="PDF directory (default: pdfs/)")
    parser.add_argument("--json", action="store_true", help="Output JSON")
    parser.add_argument("--cache", action="store_true", help="Re-extract even if cached")

    args = parser.parse_args()

    if args.pdfs:
        results = [extract_pdf(p) for p in args.pdfs]
    else:
        results = extract_all_pdfs(args.dir)
        if args.cache:
            # Force re-extract
            pdf_dir = Path(args.dir)
            for json_path in pdf_dir.glob("*.json"):
                json_path.unlink()
            results = extract_all_pdfs(args.dir)

    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            pretty_print(r)

    return 0


if __name__ == "__main__":
    sys.exit(main())
