#!/usr/bin/env python3
"""Unified API client for RF component distributor APIs.

Supports Mouser (API key) and Digi-Key (OAuth2) with local JSON caching.

Usage:
    db = ComponentDB()
    parts = db.search_mouser("ADL8142S")
    parts = db.search_digikey("ADL8142S", record_count=5)
    part  = db.get_part("SAV-541-DG+")       # checks both + cache
    all   = db.search_by_category("LNA", 24e9)  # future: sweep

Requires:
    pip install requests  (for Mouser API)
    pip install digikey-api  (for Digi-Key API, optional)
"""

from __future__ import annotations

import json
import os
import sqlite3
import sys
import time
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

# Ensure scripts/ is in path for sibling imports
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from component_config import config

# SQLite DB path (same as init_db.py uses)
DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data_input', 'component_cache.db')

try:
    import requests
except ImportError:
    requests = None  # type: ignore

try:
    import digikey
    _HAS_DIGIKEY_LIB = True
except ImportError:
    _HAS_DIGIKEY_LIB = False


# ── Data model ───────────────────────────────────────────────────────

@dataclass
class ComponentSpec:
    """Normalized record for one RF component."""
    # Identifiers
    part_number: str = ""
    manufacturer: str = ""
    description: str = ""
    category: str = ""  # e.g. "LNA", "Mixer", "BPF", "Switch", "Limiter"

    # RF specs (what cascade analysis needs)
    freq_min_hz: float = 0.0
    freq_max_hz: float = 0.0
    gain_db: float = 0.0
    nf_db: float = 0.0
    oip3_dbm: float = 0.0          # from pdf_extractor or API
    iip3_dbm: float = 0.0          # = oip3 - gain  (if only OIP3 known)
    p1db_dbm: float = 0.0

    # Passive-specific (filters, switches)
    insertion_loss_db: float = 0.0  # positive number
    filter_order: int = 0

    # Source tracking
    datasheet_url: str = ""
    product_url: str = ""           # distributor product page
    mouser_url: str = ""
    digikey_sku: str = ""
    source_api: str = ""            # "mouser", "digikey", "local"
    last_updated: str = ""

    def iip3_from_oip3(self) -> float:
        """Estimate IIP3 = OIP3 - Gain  when only OIP3 is available."""
        if self.iip3_dbm == 0.0 and self.oip3_dbm != 0.0 and self.gain_db != 0.0:
            return self.oip3_dbm - self.gain_db
        return self.iip3_dbm

    def to_cache_entry(self) -> dict:
        d = asdict(self)
        d["_cache_version"] = 1
        return d

    @classmethod
    def from_cache(cls, data: dict) -> ComponentSpec:
        data.pop("_cache_version", None)
        return cls(**data)

    @classmethod
    def from_mouser(cls, part: dict) -> ComponentSpec:
        spec = cls(
            part_number=part.get("ManufacturerPartNumber", ""),
            manufacturer=part.get("Manufacturer", ""),
            description=part.get("Description", ""),
            category=part.get("Category", ""),
            datasheet_url=part.get("DataSheetUrl", ""),
            mouser_url=part.get("ProductDetailUrl", ""),
            source_api="mouser",
            last_updated=time.strftime("%Y-%m-%d"),
        )
        return spec

    @classmethod
    def from_digikey(cls, part: dict) -> ComponentSpec:
        spec = cls(
            part_number=part.get("ManufacturerProductNumber", ""),
            manufacturer=part.get("Manufacturer", {}).get("Name", ""),
            description=part.get("Description", {}).get("DetailedDescription", "")
                       or part.get("ProductDescription", ""),
            datasheet_url=part.get("DatasheetUrl", ""),
            product_url=part.get("ProductUrl", ""),
            digikey_sku=part.get("DigiKeyPartNumber", ""),
            source_api="digikey",
            last_updated=time.strftime("%Y-%m-%d"),
        )
        # Parse structured Parameters from Digi-Key V4 API
        for param in part.get("Parameters", []):
            pname = param.get("ParameterText", "")
            raw = param.get("ValueText", "")
            pval = raw.strip() if raw else ""
            if pval in ("", "-", "None", "none", "N/A"):
                continue
            if pname == "Gain":
                spec.gain_db = _parse_db(pval)
            elif pname == "Noise Figure":
                spec.nf_db = _parse_db(pval)
            elif pname == "P1dB":
                spec.p1db_dbm = _parse_db(pval)
            elif pname == "OIP3":
                spec.oip3_dbm = _parse_db(pval)
            elif pname == "IIP3":
                spec.iip3_dbm = _parse_db(pval)
            elif pname == "Insertion Loss":
                spec.insertion_loss_db = _parse_db(pval)
            elif pname in ("Frequency", "Operating Frequency"):
                parsed = _parse_freq_range(pval)
                if parsed and parsed[0] > 0:
                    spec.freq_min_hz = parsed[0]
                    spec.freq_max_hz = parsed[1]
            elif pname in ("Frequency - Min", "Frequency Min"):
                f = _parse_freq(pval)
                if f > 0:
                    spec.freq_min_hz = f
            elif pname in ("Frequency - Max", "Frequency Max"):
                f = _parse_freq(pval)
                if f > 0:
                    spec.freq_max_hz = f
        # Estimate IIP3 from OIP3 - Gain if not directly provided
        if spec.iip3_dbm == 0.0 and spec.oip3_dbm != 0.0 and spec.gain_db != 0.0:
            spec.iip3_dbm = spec.oip3_dbm - spec.gain_db
        return spec


# ── Helpers ──────────────────────────────────────────────────────────

def _parse_freq(val: str) -> float:
    val = val.strip().lower()
    try:
        if "ghz" in val:
            return float(val.replace("ghz", "").strip()) * 1e9
        if "mhz" in val:
            return float(val.replace("mhz", "").strip()) * 1e6
        if "khz" in val:
            return float(val.replace("khz", "").strip()) * 1e3
        if "hz" in val:
            return float(val.replace("hz", "").strip())
        return float(val)
    except (ValueError, TypeError):
        return 0.0


def _parse_freq_range(val: str) -> tuple[float, float]:
    """Parse frequency range strings into (freq_min_hz, freq_max_hz).

    Handles:
      "24GHz"          → (24e9, 24e9)           single operating freq
      "2GHz ~ 6GHz"    → (2e9, 6e9)             tilde-delimited
      "2~6GHz"         → (2e9, 6e9)             compact range
      "2 to 6 GHz"     → (2e9, 6e9)             English range
      "2 - 6 GHz"      → (2e9, 6e9)             dash-delimited
      "0Hz ~ 0Hz"      → (0.0, 0.0)             zero means unknown
      "" / "-" / "N/A" → (0.0, 0.0)             unparseable

    For a single frequency value the range is exact (freq, freq) —
    the filter accepts any component whose range covers the target,
    rather than guessing a bandwidth.  Components with a single
    specified freq pass only when the target matches it exactly.

    Returns (0.0, 0.0) when val cannot be meaningfully parsed.
    """
    val = val.strip()
    if not val or val.lower() in ("-", "", "none", "n/a", "0", "0hz", "0 hz"):
        return (0.0, 0.0)

    # Try range separators first — parse each part independently with _parse_freq
    # so mixed units like "600MHz ~ 2.5GHz" work correctly.
    for sep in [" ~ ", "~", " to ", " – ", " - ", "—"]:
        if sep in val:
            parts = val.split(sep, 1)
            if len(parts) == 2:
                fa = _parse_freq(parts[0].strip())
                fb = _parse_freq(parts[1].strip())
                if fa > 0 and fb > 0:
                    # Handle mismatched units: if one value is tiny (Hz)
                    # and the other is clearly MHz/GHz, scale the tiny one
                    if fa < 1000 and fb >= 1e6:
                        fa *= 1e9 if fb >= 1e9 else 1e6
                    elif fb < 1000 and fa >= 1e6:
                        fb *= 1e9 if fa >= 1e9 else 1e6
                    return (min(fa, fb), max(fa, fb))

    # Single frequency value
    freq = _parse_freq(val)
    if freq > 0:
        return (freq, freq)

    return (0.0, 0.0)


def _parse_db(val: str) -> float:
    v = val.strip().lower()
    if v in ("-", "", "none", "n/a"):
        return 0.0
    try:
        return float(v.replace("dbm", "").replace("db", "").strip())
    except (ValueError, TypeError):
        return 0.0


# ── API Clients ──────────────────────────────────────────────────────

class MouserClient:
    """Thin wrapper around Mouser Search API v2."""

    BASE = "https://api.mouser.com/api/v2/search"

    def __init__(self, api_key: str = ""):
        self.api_key = api_key or config.mouser_api_key
        if not self.api_key:
            raise ValueError(
                "Mouser API key not set. "
                "Set MOUSER_API_KEY env var or add to component_config_local.py"
            )

    def search_keyword(self, keyword: str) -> list[dict]:
        """Search Mouser by keyword (no manufacturer required)."""
        if requests is None:
            raise ImportError("pip install requests")
        body = {
            "SearchByKeywordRequest": {
                "keyword": keyword,
            }
        }
        resp = requests.post(
            f"{self.BASE}/keyword",
            params={"apiKey": self.api_key},
            json=body,
            timeout=30,
        )
        resp.raise_for_status()
        data = resp.json()
        results = data.get("SearchResults", {}) or {}
        return results.get("Parts", [])

    def search_part_number(self, part_number: str) -> list[dict]:
        if requests is None:
            raise ImportError("pip install requests")
        body = {
            "SearchByPartMfrNameRequest": {
                "partNumber": part_number,
                "manufacturerName": "",
            }
        }
        resp = requests.post(
            f"{self.BASE}/partnumberandmanufacturer",
            params={"apiKey": self.api_key},
            json=body,
            timeout=30,
        )
        resp.raise_for_status()
        data = resp.json()
        results = data.get("SearchResults", {})
        return results.get("Parts", [])

    def get_manufacturers(self) -> list[str]:
        if requests is None:
            raise ImportError("pip install requests")
        resp = requests.get(
            f"{self.BASE}/manufacturerlist",
            params={"apiKey": self.api_key},
            timeout=30,
        )
        resp.raise_for_status()
        data = resp.json()
        return [m["ManufacturerName"] for m in data.get("MouserManufacturersName", [])]


class DigikeyClient:
    """REST client for Digi-Key Product Information V4 API.

    Uses the digikey-api library's TokenHandler for OAuth2 (handles HTTPS
    redirect server, certificate generation, browser login, token refresh).
    Makes API calls to V4 endpoints directly.

    Usage:
        dk = DigikeyClient()
        parts = dk.search_keyword("ADL8142S")
    """

    BASE = "https://api.digikey.com/products/v4"

    def __init__(self):
        self.client_id = config.digikey_client_id
        self.client_secret = config.digikey_client_secret
        if not self.client_id:
            raise ValueError("DIGIKEY_CLIENT_ID not set")
        if not self.client_secret:
            raise ValueError("DIGIKEY_CLIENT_SECRET not set")
        # Use the library's TokenHandler for OAuth
        os.environ.setdefault("DIGIKEY_CLIENT_ID", self.client_id)
        os.environ.setdefault("DIGIKEY_CLIENT_SECRET", self.client_secret)
        os.environ.setdefault("DIGIKEY_STORAGE_PATH", str(config.digikey_storage_path))
        from digikey.oauth.oauth2 import TokenHandler
        self._token_handler = TokenHandler(
            version=3,  # OAuth URLs are same for V3/V4
            sandbox=config.digikey_sandbox,
        )
        self._access_token: str | None = None

    def _ensure_token(self):
        if not self._access_token:
            token = self._token_handler.get_access_token()
            self._access_token = token.access_token

    def set_token_manual(self, access_token: str):
        self._access_token = access_token

    def _get(self, path: str, params: dict | None = None) -> dict:
        self._ensure_token()
        headers = {
            "Authorization": f"Bearer {self._access_token}",
            "X-Digikey-Client-Id": self.client_id,
            "Accept": "application/json",
        }
        url = f"{self.BASE}/{path.lstrip('/')}"
        resp = requests.get(url, headers=headers, params=params, timeout=30)
        if resp.status_code == 401:
            self._access_token = None
            self._ensure_token()
            headers["Authorization"] = f"Bearer {self._access_token}"
            resp = requests.get(url, headers=headers, params=params, timeout=30)
        resp.raise_for_status()
        return resp.json()

    def _post(self, path: str, body: dict) -> dict:
        self._ensure_token()
        headers = {
            "Authorization": f"Bearer {self._access_token}",
            "X-Digikey-Client-Id": self.client_id,
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        url = f"{self.BASE}/{path.lstrip('/')}"
        resp = requests.post(url, headers=headers, json=body, timeout=30)
        if resp.status_code == 401:
            self._access_token = None
            self._ensure_token()
            headers["Authorization"] = f"Bearer {self._access_token}"
            resp = requests.post(url, headers=headers, json=body, timeout=30)
        resp.raise_for_status()
        return resp.json()

    def search_keyword(self, keyword: str, record_count: int = 50,
                       start_position: int = 0,
                       search_options: list[str] | None = None,
                       taxonomy_ids: list[int] | None = None,
                       parameter_filters: list[dict] | None = None,
                       category_filter_id: str | None = None) -> dict:
        """Full keyword search with optional filters.

        Returns the raw API response dict (contains Products, ProductsCount,
        FilterOptions with available parametric filters, etc.).

        Args:
            keyword: Search keywords.
            record_count: Number of results (1–50, capped at 50).
            start_position: Pagination offset.
            search_options: e.g. ["CollapsePackingTypes"].
            taxonomy_ids: Optional taxonomy ID filter.
            parameter_filters: List of parametric filter dicts, each:
                {"ParameterId": int, "FilterValues": [{"Id": "str"}]}
            category_filter_id: Optional category ID string to scope search.

        NOTE: V4 API field names are 'Limit' and 'Offset' (not
        'RecordCount' / 'RecordStartPosition').
        """
        body: dict[str, Any] = {
            "Keywords": keyword,
            "Limit": min(record_count, 50),
            "Offset": start_position,
        }
        if search_options:
            body["SearchOptions"] = search_options
        if taxonomy_ids:
            body.setdefault("Filters", {})["TaxonomyIds"] = taxonomy_ids
        if parameter_filters or category_filter_id:
            pfr: dict[str, Any] = {}
            if category_filter_id:
                pfr["CategoryFilter"] = {"Id": category_filter_id}
            if parameter_filters:
                pfr["ParameterFilters"] = parameter_filters
            body.setdefault("FilterOptionsRequest", {})["ParameterFilterRequest"] = pfr
        return self._post("search/keyword", body)

    def search_keyword_products(self, keyword: str, record_count: int = 50,
                                start_position: int = 0,
                                search_options: list[str] | None = None,
                                taxonomy_ids: list[int] | None = None) -> list[dict]:
        """Convenience: return just the Products list from search_keyword()."""
        data = self.search_keyword(keyword, record_count, start_position,
                                   search_options, taxonomy_ids)
        return data.get("Products", [])

    def search_keyword_deep(self, keyword: str,
                            max_results: int = 100,
                            search_options: list[str] | None = None,
                            taxonomy_ids: list[int] | None = None,
                            parameter_filters: list[dict] | None = None,
                            category_filter_id: str | None = None,
                            start_offset: int = 0) -> list[dict]:
        """Paginated keyword search. Supports parametric filters and offset.

        Calls search_keyword() repeatedly (Offset = start_offset, +50, +100…)
        until we collect max_results or a page comes back with < 50 products.

        Args:
            start_offset: First page offset. Use >0 for incremental search
                          (e.g., start_offset=100 skips first 2 pages already cached).
        """
        all_products: list[dict] = []
        page_size = 50
        pos = start_offset
        while len(all_products) < max_results:
            data = self.search_keyword(keyword, record_count=page_size,
                                       start_position=pos,
                                       search_options=search_options,
                                       taxonomy_ids=taxonomy_ids,
                                       parameter_filters=parameter_filters,
                                       category_filter_id=category_filter_id)
            products = data.get("Products", [])
            if not products:
                break
            all_products.extend(products)
            if len(products) < page_size:
                break
            pos += page_size
        return all_products

    def discover_parametric_filters(
        self,
        keyword: str,
        category_filter_id: str | None = None,
        search_options: list[str] | None = None,
    ) -> list[dict]:
        """Discover available parametric filters for a keyword/category.

        Makes a lightweight search (Limit=1) and extracts the
        FilterOptions.ParametricFilters[] from the response.

        Returns list of dicts, each:
          {
            "Category": {"Id": int, "Value": str, "ProductCount": int},
            "ParameterType": str,          # e.g. "RangeUnitOfMeasure"
            "ParameterId": int,            # use this in subsequent param filters
            "ParameterName": str,          # e.g. "Frequency - RF"
            "FilterValues": [
              {
                "ProductCount": int,
                "ValueId": str,            # use this Id in FilterValues
                "ValueName": str,          # e.g. "18GHz~26.5GHz"
                "RangeFilterType": str     # "Min", "Max", or "Range"
              }
            ]
          }
        """
        data = self.search_keyword(
            keyword, record_count=1,
            search_options=search_options,
            category_filter_id=category_filter_id,
        )
        return (data.get("FilterOptions") or {}).get("ParametricFilters") or []

    def search_keyword_parametric(
        self,
        keyword: str,
        parameter_filters: list[dict],
        category_filter_id: str | None = None,
        max_results: int = 100,
        search_options: list[str] | None = None,
    ) -> list[dict]:
        """Lightweight: call search_keyword_deep() with parametric filters.

        Args:
            keyword: Search keywords.
            parameter_filters: e.g. [{"ParameterId": 1154,
                                      "FilterValues": [{"Id": "12345"}]}]
            category_filter_id: Optional Digi-Key category ID.
            max_results: Max products to fetch across pages.
            search_options: e.g. ["CollapsePackingTypes"].

        Returns list of product dicts.
        """
        return self.search_keyword_deep(
            keyword=keyword,
            max_results=max_results,
            search_options=search_options,
            parameter_filters=parameter_filters,
            category_filter_id=category_filter_id,
        )

    def product_details(self, part_number: str) -> dict:
        return self._get(f"search/{part_number}/productdetails")

    def product_pricing(self, part_number: str) -> dict:
        return self._get(f"search/{part_number}/pricing")

    def get_manufacturers(self) -> list[dict]:
        return self._get("search/manufacturers").get("manufacturers", [])


# ── Local Cache ──────────────────────────────────────────────────────

SCHEMA_SQL = """
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
    "part_number", "manufacturer", "description", "category",
    "datasheet_url", "product_url", "gain_db", "nf_db",
    "insertion_loss_db", "oip3_dbm", "iip3_dbm", "p1db_dbm",
    "freq_min_hz", "freq_max_hz", "source_api", "last_updated",
]


class ComponentCache:
    """Persistent SQLite cache for fetched component specs."""

    def __init__(self, db_path: str | Path = ""):
        self.db_path = str(db_path or DB_PATH)
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._conn = sqlite3.connect(self.db_path)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(SCHEMA_SQL)

    def close(self):
        if self._conn:
            self._conn.close()
            self._conn = None

    def __del__(self):
        self.close()

    def get(self, part_number: str) -> ComponentSpec | None:
        row = self._conn.execute(
            "SELECT * FROM components WHERE part_number=?",
            (part_number.upper(),)
        ).fetchone()
        if row:
            data = dict(row)
            raw = data.pop("raw_json", None)
            if raw:
                try:
                    return ComponentSpec.from_cache(json.loads(raw))
                except (json.JSONDecodeError, TypeError):
                    pass
            data.pop("_cache_version", None)
            return ComponentSpec(**{k: v for k, v in data.items() if k in ComponentSpec.__dataclass_fields__})
        return None

    def put(self, spec: ComponentSpec):
        entry = spec.to_cache_entry()
        values = [entry.get(col, None) for col in DB_COLUMNS]
        values.append(json.dumps(entry))
        placeholders = ", ".join(["?"] * (len(DB_COLUMNS) + 1))
        cols = ", ".join(DB_COLUMNS + ["raw_json"])
        self._conn.execute(
            f"INSERT OR REPLACE INTO components ({cols}) VALUES ({placeholders})",
            values
        )
        self._conn.commit()

    def search(self, query: str) -> list[ComponentSpec]:
        q = f"%{query.upper()}%"
        rows = self._conn.execute(
            "SELECT * FROM components WHERE UPPER(part_number) LIKE ? OR UPPER(manufacturer) LIKE ?",
            (q, q)
        ).fetchall()
        results = []
        for row in rows:
            data = dict(row)
            raw = data.pop("raw_json", None)
            if raw:
                try:
                    results.append(ComponentSpec.from_cache(json.loads(raw)))
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass
            data.pop("_cache_version", None)
            results.append(ComponentSpec(**{k: v for k, v in data.items() if k in ComponentSpec.__dataclass_fields__}))
        return results

    def search_by_category(self, category: str, freq_min: float = 0,
                           freq_max: float = 0) -> list[ComponentSpec]:
        q = f"%{category.upper()}%"
        sql = "SELECT * FROM components WHERE UPPER(category) LIKE ?"
        params: list = [q]
        if freq_min:
            sql += " AND freq_min_hz >= ?"
            params.append(freq_min)
        if freq_max:
            sql += " AND freq_max_hz <= ?"
            params.append(freq_max)
        rows = self._conn.execute(sql, params).fetchall()
        results = []
        for row in rows:
            data = dict(row)
            raw = data.pop("raw_json", None)
            if raw:
                try:
                    results.append(ComponentSpec.from_cache(json.loads(raw)))
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass
            data.pop("_cache_version", None)
            results.append(ComponentSpec(**{k: v for k, v in data.items() if k in ComponentSpec.__dataclass_fields__}))
        return results

    def search_by_keyword(self, keyword: str) -> list[ComponentSpec]:
        q = f"%{keyword.upper()}%"
        rows = self._conn.execute(
            "SELECT * FROM components WHERE UPPER(part_number) LIKE ? "
            "OR UPPER(manufacturer) LIKE ? OR UPPER(description) LIKE ?",
            (q, q, q)
        ).fetchall()
        results = []
        for row in rows:
            data = dict(row)
            raw = data.pop("raw_json", None)
            if raw:
                try:
                    results.append(ComponentSpec.from_cache(json.loads(raw)))
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass
            data.pop("_cache_version", None)
            results.append(ComponentSpec(**{k: v for k, v in data.items() if k in ComponentSpec.__dataclass_fields__}))
        return results

    def search_by_freq(self, category: str, freq_ghz: float) -> list[ComponentSpec]:
        """Find cached components of a category that operate at freq_ghz.
        
        Includes components with unknown freq (freq_min=0) for backward
        compatibility with legacy cache entries.
        
        Uses the idx_freq index for the frequency-range portion.
        """
        like = f"%{category.upper()}%"
        f_hz = freq_ghz * 1e9
        rows = self._conn.execute(
            """SELECT * FROM components
               WHERE UPPER(category) LIKE ?
                 AND (freq_min_hz = 0  -- unknown freq → pass through
                      OR (freq_min_hz <= ? AND freq_max_hz >= ?))
               ORDER BY gain_db DESC""",
            (like, f_hz, f_hz),
        ).fetchall()
        results = []
        for row in rows:
            data = dict(row)
            raw = data.pop("raw_json", None)
            if raw:
                try:
                    results.append(ComponentSpec.from_cache(json.loads(raw)))
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass
            data.pop("_cache_version", None)
            results.append(ComponentSpec(**{k: v for k, v in data.items() if k in ComponentSpec.__dataclass_fields__}))
        return results


# ── Orchestrator ─────────────────────────────────────────────────────

class ComponentDB:
    """High-level interface: search both APIs, cache results, extract specs.

    Usage:
        db = ComponentDB()

        # Search by part number
        parts = db.search("ADL8142S")
        for p in parts:
            print(p.part_number, p.gain_db, p.datasheet_url)

        # Get single part (cached or fresh)
        spec = db.get_part("ADL8142S")

        # Search by category + frequency band
        candidates = db.search_by_category("LNA", freq_ghz=24)
    """

    def __init__(self):
        self.cache = ComponentCache()
        self._mouser: MouserClient | None = None
        self._digikey: DigikeyClient | None = None

    def close(self):
        if self.cache:
            self.cache.close()

    def __del__(self):
        self.close()

    def _get_mouser(self) -> MouserClient:
        if self._mouser is None:
            self._mouser = MouserClient()
        return self._mouser

    def _get_digikey(self) -> DigikeyClient:
        if self._digikey is None:
            self._digikey = DigikeyClient()
        return self._digikey

    def search(self, query: str, prefer_digikey: bool = False) -> list[ComponentSpec]:
        """Search by part number or keyword across available APIs."""
        cached = self.cache.search(query)
        if cached:
            return cached

        results: list[ComponentSpec] = []

        if config.mouser_api_key:
            try:
                mouser = self._get_mouser()
                for raw in mouser.search_part_number(query):
                    results.append(ComponentSpec.from_mouser(raw))
            except Exception as e:
                print(f"[warn] Mouser API error: {e}")

        if config.digikey_client_id:
            try:
                dk = self._get_digikey()
                for raw in dk.search_keyword_products(query):
                    spec = ComponentSpec.from_digikey(raw)
                    if not any(s.part_number == spec.part_number for s in results):
                        results.append(spec)
            except Exception as e:
                print(f"[warn] Digi-Key API error: {e}")

        for spec in results:
            self.cache.put(spec)

        return results

    def get_part(self, part_number: str) -> ComponentSpec | None:
        cached = self.cache.get(part_number)
        if cached:
            return cached

        results = self.search(part_number)
        for r in results:
            if r.part_number.upper() == part_number.upper():
                return r
        return None

    def fetch_datasheet(self, spec: ComponentSpec, output_dir: str | Path = "") -> Path | None:
        """Download the datasheet PDF for a given component spec."""
        url = ""
        if spec.datasheet_url:
            url = spec.datasheet_url
        elif spec.mouser_url:
            url = spec.mouser_url

        if not url:
            print(f"[warn] No datasheet URL for {spec.part_number}")
            return None

        out_dir = Path(output_dir or config.pdfs_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        # Check both sanitized and original part-number filenames
        part_slug = spec.part_number.replace("/", "_")
        out_paths = [out_dir / f"{part_slug}.pdf"]
        if part_slug != spec.part_number:
            out_paths.append(out_dir / f"{spec.part_number}.pdf")
        for p in out_paths:
            if p.exists():
                return p
        out_path = out_paths[0]

        try:
            r = requests.get(url, timeout=60, allow_redirects=True)
            r.raise_for_status()
            # Detect actual PDF URL from redirect
            if "text/html" in r.headers.get("Content-Type", "") and not url.endswith(".pdf"):
                print(f"[warn] URL returned HTML, not PDF: {url[:80]}")
            with open(out_path, "wb") as f:
                f.write(r.content)
            print(f"[ok] Downloaded {out_path.name}  ({len(r.content)} bytes)")
            return out_path
        except Exception as e:
            print(f"[error] Failed to fetch {url[:80]}: {e}")
            return None

    def search_by_category(self, category: str, freq_ghz: float = 0,
                           record_count: int = 10) -> list[ComponentSpec]:
        """Search by component category (e.g. 'LNA', 'Mixer').

        Appends frequency to query if given, e.g. "LNA 24 GHz".
        """
        keyword = category
        if freq_ghz:
            keyword = f"{category} {freq_ghz} GHz"
        return self.search(keyword)

    def search_by_freq(self, category: str, freq_ghz: float) -> list[ComponentSpec]:
        """Find cached components by category and operating frequency.
        
        Only searches the local SQLite cache — does NOT hit external APIs.
        Includes components with unknown freq (legacy cache entries).
        """
        return self.cache.search_by_freq(category, freq_ghz)


# ── CLI entry point ──────────────────────────────────────────────────

def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: component_db.py <part_number> [--mouser-only]")
        sys.exit(1)

    query = sys.argv[1]
    prefer_dk = "--mouser-only" not in sys.argv

    db = ComponentDB()
    parts = db.search(query)
    if not parts:
        print(f"No results found for '{query}'")
        sys.exit(1)

    for i, p in enumerate(parts, 1):
        print(f"\n{'='*60}")
        print(f"  [{i}] {p.part_number:30s}  ({p.source_api})")
        print(f"      Mfr:      {p.manufacturer}")
        print(f"      Desc:     {p.description[:80]}")
        print(f"      Freq:     {p.freq_min_hz/1e9:.2f}-{p.freq_max_hz/1e9:.2f} GHz"
              if p.freq_max_hz else f"      Freq:     n/a")
        print(f"      Gain:     {p.gain_db} dB" if p.gain_db else f"      Gain:     n/a")
        print(f"      NF:       {p.nf_db} dB" if p.nf_db else f"      NF:       n/a")
        print(f"      OIP3:     {p.oip3_dbm} dBm" if p.oip3_dbm else f"      OIP3:    n/a")
        print(f"      IIP3:     {p.iip3_dbm} dBm ({p.iip3_from_oip3():.1f} from OIP3)"
              if p.iip3_dbm or p.oip3_dbm else f"      IIP3:     n/a")
        print(f"      P1dB:     {p.p1db_dbm} dBm" if p.p1db_dbm else f"      P1dB:    n/a")
        print(f"      Datasheet: {p.datasheet_url[:60]}" if p.datasheet_url else "")
        print(f"      Mouser:   {p.mouser_url[:60]}" if p.mouser_url else "")
        print(f"      DK SKU:   {p.digikey_sku}" if p.digikey_sku else "")


if __name__ == "__main__":
    main()
