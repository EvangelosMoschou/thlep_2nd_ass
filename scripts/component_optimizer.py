#!/usr/bin/env python3
"""Cascade optimization engine — finds optimal component combinations via DP.

The receiver chain is a sequence of stages (Switch, BPF, LNA, Mixer, ...).
Each stage has candidate components with different RF specs (gain, NF, OIP3).

Cascade formulas are Markovian, so we use Dynamic Programming:
  state = (cum_gain_linear, cum_nf_linear, cum_iip3_inv, cum_p1db_inv)
  transition for stage k with component c:
    cum_gain' = cum_gain * c.gain_linear
    cum_nf'  = cum_nf + (c.nf_linear - 1) / cum_gain
    cum_iip3_inv' = cum_iip3_inv + cum_gain^2 / c.iip3_linear^2
    cum_p1db_inv' = cum_p1db_inv + cum_gain / c.p1db_linear

Pareto-optimal states are kept at each stage (not dominated on SNR + IIP3).

Usage:
    from component_optimizer import optimize_chain, search_stage_candidates

    stages = [
        ("Switch", [comp_a, comp_b]),
        ("BPF",    [comp_c, comp_d]),
        ("LNA",    [comp_e, comp_f]),
        ...
    ]
    results = optimize_chain(stages)
    for score, state in results[:5]:
        print(score, [c.part_number for c in state.choices])
"""

from __future__ import annotations

import subprocess
import sys
import os
import time
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from component_db import ComponentDB, ComponentSpec, DigikeyClient

# ── Type alias for per-stage frequency specification ─────────────────
# Templates can mix plain strings (use global --freq) with tuples (explicit freq).
StageDef = str | tuple[str, float | None | str]  # "LNA" or ("LNA", 24.0) or ("BPF", None) or ("BPF", "IF")

# ── Parametric filter cache ──────────────────────────────────────────
# Digi-Key API V4 supports parametric filtering via ParameterFilterRequest.
# We discover available ParameterId / ValueId values via a lightweight
# search (Limit=1) and cache them here to avoid repeated discovery calls.

PARAMETRIC_FILTER_CACHE_PATH = Path(__file__).parent / ".." / "data_input" / "component_cache" / "parametric_filters.json"
_parametric_cache = None  # type: dict | None

SEARCH_DEPTH_CACHE_PATH = Path(__file__).parent / ".." / "data_input" / "component_cache" / "search_depth.json"
_search_depth_cache = None  # type: dict | None

def _load_parametric_cache() -> dict:
    global _parametric_cache
    if _parametric_cache is not None:
        return _parametric_cache
    path = PARAMETRIC_FILTER_CACHE_PATH
    if path.exists():
        import json
        _parametric_cache = json.loads(path.read_text())
    else:
        _parametric_cache = {}
    return _parametric_cache  # pyright: ignore[reportReturnType]

def _save_parametric_cache():
    if _parametric_cache is not None:
        import json
        PARAMETRIC_FILTER_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
        PARAMETRIC_FILTER_CACHE_PATH.write_text(json.dumps(_parametric_cache, indent=2))


def _load_search_depth() -> dict:
    global _search_depth_cache
    if _search_depth_cache is not None:
        return _search_depth_cache
    path = SEARCH_DEPTH_CACHE_PATH
    if path.exists():
        import json
        _search_depth_cache = json.loads(path.read_text())
    else:
        _search_depth_cache = {}
    return _search_depth_cache  # pyright: ignore


def _save_search_depth():
    if _search_depth_cache is not None:
        import json
        SEARCH_DEPTH_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
        SEARCH_DEPTH_CACHE_PATH.write_text(json.dumps(_search_depth_cache, indent=2))


def _get_cached_search_depth(stage_name: str, freq_ghz: float) -> int:
    """Return cached max_pages for this stage+freq, or 0 if never searched."""
    cache = _load_search_depth()
    key = f"{stage_name}@{freq_ghz}"
    entry = cache.get(key)
    return entry.get("max_pages", 0) if entry else 0


def _set_cached_search_depth(stage_name: str, freq_ghz: float, max_pages: int,
                              use_parametric_filter: bool):
    import time
    cache = _load_search_depth()
    key = f"{stage_name}@{freq_ghz}"
    cache[key] = {
        "max_pages": max_pages,
        "use_parametric_filter": use_parametric_filter,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    _save_search_depth()


def _get_parametric_freq_filter(
    keyword: str,
    freq_ghz: float,
    category_filter_id: str | None = None,
    search_options: list | None = None,
) -> list | None:
    """Discover and cache the Frequency - RF parametric filter for a keyword.

    Returns a list suitable for ParameterFilters in the request body:
        [{"ParameterId": <int>, "FilterValues": [{"Id": "<value_id>"}]}]
    or None if no frequency filter found.
    """
    cache = _load_parametric_cache()
    # Use keyword + category as cache key
    cache_key = f"{keyword}|{category_filter_id or ''}"
    cached = cache.get(cache_key)

    if cached and freq_ghz:
        # Find a cached ValueId whose range covers our target frequency
        for entry in cached.get("freq_filters", []):
            min_hz = entry.get("min_hz", 0)
            max_hz = entry.get("max_hz", 0)
            if min_hz <= freq_ghz * 1e9 <= max_hz:
                return [{
                    "ParameterId": cached["parameter_id"],
                    "FilterValues": [{"Id": entry["value_id"]}],
                }]

    if cached and cached.get("_no_freq_param"):
        return None

    try:
        from component_db import DigikeyClient
        dk = DigikeyClient()
        filters = dk.discover_parametric_filters(
            keyword=keyword,
            category_filter_id=category_filter_id,
            search_options=search_options,
        )
    except Exception as exc:
        print(f"    [warn] Parametric filter discovery failed: {exc}")
        cache[cache_key] = {"_discovery_failed": str(exc)[:100]}
        _save_parametric_cache()
        return None

    freq_param = None
    for pattern in ("RF", ""):
        if freq_param:
            break
        for pf in filters:
            name = pf.get("ParameterName", "")
            if "Frequency" not in name:
                continue
            if pattern and pattern not in name:
                continue
            freq_param = pf
            break
    if not freq_param:
        cache[cache_key] = {"_no_freq_param": True}
        _save_parametric_cache()
        return None

    param_id = freq_param["ParameterId"]
    freq_filters: list[dict] = []
    for fv in freq_param.get("FilterValues", []):
        value_id = fv["ValueId"]
        value_name = fv.get("ValueName", "")
        min_hz, max_hz = _parse_freq_range_hz(value_name)
        freq_filters.append({
            "value_id": value_id,
            "value_name": value_name,
            "range_filter_type": fv.get("RangeFilterType", ""),
            "min_hz": min_hz,
            "max_hz": max_hz,
        })

    cache[cache_key] = {
        "parameter_id": param_id,
        "parameter_name": freq_param.get("ParameterName", ""),
        "freq_filters": freq_filters,
    }
    _save_parametric_cache()

    # Retry match with newly discovered data
    if freq_ghz:
        for entry in freq_filters:
            if entry["min_hz"] <= freq_ghz * 1e9 <= entry["max_hz"]:
                return [{
                    "ParameterId": param_id,
                    "FilterValues": [{"Id": entry["value_id"]}],
                }]

    return None

def _parse_freq_range_hz(value_name: str) -> tuple[float, float]:
    """Parse a Digi-Key frequency value like '18GHz~26.5GHz' or '24GHz'.

    Returns (min_hz, max_hz). For a single frequency, both are equal.
    Returns (0, 0) if unparseable.
    """
    def _to_hz(s: str) -> float:
        s = s.strip()
        if s.endswith("GHz"):
            return float(s.replace("GHz", "")) * 1e9
        if s.endswith("MHz"):
            return float(s.replace("MHz", "")) * 1e6
        if s.endswith("kHz"):
            return float(s.replace("kHz", "")) * 1e3
        if s.endswith("Hz"):
            return float(s.replace("Hz", ""))
        return 0.0

    if "~" in value_name:
        parts = value_name.split("~", 1)
        return _to_hz(parts[0]), _to_hz(parts[1])
    if value_name.startswith("~"):
        return 0.0, _to_hz(value_name[1:])
    single_freq = _to_hz(value_name)
    if single_freq > 0:
        return single_freq, single_freq
    return 0.0, 0.0


# ── Data structures ──────────────────────────────────────────────────

@dataclass
class SystemConfig:
    """System-level parameters for SNR calculation (matches cascade.c physics).
    
    SNR_at_receiver formula (from cascade.c line 264):
      No_W = k * (T_antenna + (F_lin-1)*T0) * BW * G_lin
      SNR_out_dB = (input_signal_dbm + Gain_dB) - (10*log10(No_W) + 30)
    """
    antenna_temp_k: float = 91.0   # Viasat 13.5m @ 44° elev, Athens→SES-17
    t0_k: float = 290.0
    bw_hz: float = 200.0e6  # matches C code B_NOISE_HZ (cascade.h, physics.h)
    input_signal_dbm: float = -65.8
    k_boltz: float = 1.380649e-23
    
    def input_noise_dbm(self) -> float:
        return 10.0 * math.log10(self.k_boltz * self.antenna_temp_k * self.bw_hz) + 30.0
    
    def input_snr_db(self) -> float:
        return self.input_signal_dbm - self.input_noise_dbm()


@dataclass
class ChainState:
    """Cumulative cascade state at a given stage number."""
    stage_idx: int = 0
    gain_lin: float = 1.0       # cumulative linear gain
    nf_lin: float = 1.0         # cumulative noise factor (linear)
    iip3_inv: float = 0.0       # cumulative 1/IIP3^2 (linear)
    p1db_inv: float = 0.0       # cumulative 1/P1dB (linear) ... actually G/P1dB
    choices: list = None        # list of ComponentSpec chosen so far

    def __post_init__(self):
        if self.choices is None:
            self.choices = []

    def nf_db(self) -> float:
        return 10 * log10(self.nf_lin)

    def gain_db(self) -> float:
        return 10 * log10(self.gain_lin)

    def iip3_dbm(self) -> float:
        if self.iip3_inv <= 0:
            return 999.0
        return -10 * log10(self.iip3_inv)

    def p1db_dbm(self) -> float:
        if self.p1db_inv <= 0:
            return 999.0
        # P1dB_total = 1 / p1db_inv_cum
        # But the formula is: 1/P1dB = 1/P1dB₁ + G₁/P1dB₂ + G₁G₂/P1dB₃ + ...
        return 10 * log10(1.0 / self.p1db_inv)

    def snr_at_receiver(self, config: SystemConfig | None = None) -> float:
        """SNR at receiver output in dB, matching cascade.c physics."""
        if config is None:
            config = SystemConfig()
        k = config.k_boltz
        Te = (self.nf_lin - 1.0) * config.t0_k
        # Total output noise power: k * (T_ant + Te) * BW * G
        No_W = k * max(config.antenna_temp_k + Te, 1e-12) * config.bw_hz * max(self.gain_lin, 1e-12)
        No_dBm = 10.0 * math.log10(No_W) + 30.0
        # Signal at output: Si * G
        So_dBm = config.input_signal_dbm + 10.0 * math.log10(max(self.gain_lin, 1e-12))
        return So_dBm - No_dBm

    def score(self, config: SystemConfig | None = None) -> float:
        """Score = -SNR_at_receiver (lower=better), with IIP3 floor penalty.
        
        Primary objective: maximize SNR at receiver.
        If IIP3 is below -10 dBm, add a small penalty to avoid unrealistic chains.
        """
        snr = self.snr_at_receiver(config)
        iip3 = self.iip3_dbm()
        penalty = 0.0
        if iip3 < -10.0:
            penalty = (-10.0 - iip3) * 0.05  # 0.05 dB per dB below -10
        return -snr + penalty

    def dominates(self, other: ChainState) -> bool:
        """True if this state is strictly better on SNR or IIP3 and not worse on either."""
        better_snr = self.snr_at_receiver() > other.snr_at_receiver()
        better_iip3 = self.iip3_dbm() > other.iip3_dbm()
        worse_snr = self.snr_at_receiver() < other.snr_at_receiver()
        worse_iip3 = self.iip3_dbm() < other.iip3_dbm()
        if better_snr and not worse_iip3:
            return True
        if better_iip3 and not worse_snr:
            return True
        return False

    def tx_score(self) -> float:
        """Tx score = -P1dB_out (lower=better), with OIP3 penalty.

        Primary objective: maximize output P1dB.
        Penalize when OIP3 - P1dB < 10 dB (poor linearity margin).
        """
        p1db = self.p1db_dbm()
        oip3 = self.iip3_dbm()  # iip3_inv field holds cumulative OIP3
        penalty = 0.0
        margin = oip3 - p1db
        if margin < 10.0:
            penalty = (10.0 - margin) * 0.1
        return -p1db + penalty

    def tx_dominates(self, other: ChainState) -> bool:
        """True if better on P1dB or OIP3 and not worse on either."""
        better_p1db = self.p1db_dbm() > other.p1db_dbm()
        better_oip3 = self.iip3_dbm() > other.iip3_dbm()
        worse_p1db = self.p1db_dbm() < other.p1db_dbm()
        worse_oip3 = self.iip3_dbm() < other.iip3_dbm()
        if better_p1db and not worse_oip3:
            return True
        if better_oip3 and not worse_p1db:
            return True
        return False


def log10(x):
    from math import log10 as _log
    return _log(x) if x > 0 else -999.0


# ── DP Optimizer ─────────────────────────────────────────────────────

def estimate_iip3(c: ComponentSpec, stage_name: str = "") -> float:
    """Estimate IIP3 (dBm) for the cascade calculation.

    Preference order:
      1. OIP3 − Gain directly (from PDF or API) — when the component
         has freq_min/max populated, confirming it operates in our
         target band (frequency filter passed).
      2. P1dB-based heuristic — for components without frequency
         data (legacy cache with freq=0) or when OIP3 is unavailable.
      3. Unverified OIP3 − Gain (conservative fallback).
      4. Category-based default.

    Returns IIP3 in dBm, or None if truly unknown (will default to very linear).
    """
    cat = (c.category or stage_name).lower()
    gain_db_val = c.gain_db if c.gain_db is not None else 0.0
    il_val = c.insertion_loss_db if c.insertion_loss_db is not None else 0.0
    gain = gain_db_val if gain_db_val != 0.0 else (-il_val if il_val != 0.0 else 0.0)
    p1db = c.p1db_dbm if c.p1db_dbm else 0.0

    # ── 1. Frequency-verified: OIP3−Gain directly from datasheet ──
    # Component passed search_stage_candidates() frequency filter,
    # so any OIP3/IIP3 in the DB is valid at our operating frequency.
    fmin = c.freq_min_hz if c.freq_min_hz is not None else 0.0
    fmax = c.freq_max_hz if c.freq_max_hz is not None else 0.0
    freq_populated = fmin > 0 and fmax > 0
    if freq_populated:
        api_iip3 = c.iip3_from_oip3()
        if api_iip3 and abs(api_iip3) > 0.01:
            return api_iip3
        if c.iip3_dbm and abs(c.iip3_dbm) > 0.01:
            return c.iip3_dbm

    # ── 2. P1dB-based heuristic (conservative for unknown freq) ──
    if p1db and abs(p1db) > 0.01:
        if cat in ("switch", "bpf", "limiter"):
            return max(p1db + 20, 60.0) if p1db else 100.0
        if cat == "lna":
            if gain and gain > 0:
                return (p1db + 12.0) - gain
            return p1db + 12.0
        if cat == "mixer":
            return p1db + 3.0
        return p1db + 10.0

    # ── 3. Unverified OIP3 − Gain fallback ──────────────────────
    if not freq_populated:
        api_iip3 = c.iip3_from_oip3()
        if api_iip3 and abs(api_iip3) > 0.01:
            return api_iip3

    # ── 4. Category defaults ──────────────────────────────────
    if cat in ("switch", "bpf", "limiter"):
        return 100.0
    if cat == "lna":
        return 5.0
    if cat == "mixer":
        return 5.0
    return None


def optimize_chain(
    stage_candidates: list[tuple[str, list[ComponentSpec]]],
    max_states: int = 500,
    system_config: SystemConfig | None = None,
) -> list[tuple[float, ChainState]]:
    """Dynamic Programming cascade optimization.

    Args:
        stage_candidates: list of (stage_name, [ComponentSpec, ...])
        max_states: max Pareto states to keep per stage (for performance)
        system_config: system parameters for SNR-based scoring (optional)

    Returns:
        Sorted list of (score, final_ChainState) tuples.
    """
    states = [ChainState()]

    for stage_idx, (stage_name, candidates) in enumerate(stage_candidates):
        if not candidates:
            print(f"  [skip] {stage_name}: no candidates")
            continue

        new_states: list[ChainState] = []

        for s in states:
            for c in candidates:
                if not _compatible(c):
                    continue

                g_lin = db_to_lin_gain(c.gain_db if c.gain_db else 0.0)
                gain_db_val = c.gain_db if c.gain_db is not None else 0.0
                if gain_db_val <= 0 and c.insertion_loss_db:
                    g_lin = db_to_lin_gain(-c.insertion_loss_db)

                # NF: if component has insertion loss (passive), NF ≈ IL in dB
                nf_val = 0.0
                if c.nf_db:
                    nf_val = c.nf_db
                elif c.insertion_loss_db and gain_db_val <= 0:
                    nf_val = abs(c.insertion_loss_db)  # IL stored as negative, NF is positive
                elif c.gain_db:
                    nf_val = abs(c.gain_db)
                nf_lin = db_to_lin_nf(nf_val)

                iip3_val = estimate_iip3(c, stage_name)
                iip3_lin = dbm_to_mw(iip3_val) if iip3_val else 1e9  # fallback: very linear

                p1db_val = c.p1db_dbm if c.p1db_dbm else 999.0
                p1db_lin = dbm_to_mw(p1db_val) if p1db_val else 1e9

                g_cum = s.gain_lin

                new = ChainState(
                    stage_idx=stage_idx + 1,
                    gain_lin=s.gain_lin * g_lin,
                    nf_lin=s.nf_lin + (nf_lin - 1.0) / g_cum,
                    iip3_inv=s.iip3_inv + g_cum / iip3_lin,
                    p1db_inv=s.p1db_inv + g_cum / p1db_lin,
                    choices=s.choices + [c],
                )
                new_states.append(new)

        # Pareto pruning
        states = _pareto_prune(new_states, max_states)
        print(f"  {stage_name:20s} {len(candidates):3d} candidates → {len(states):4d} Pareto states")

    # Score and sort
    scored = [(s.score(system_config), s) for s in states]
    scored.sort(key=lambda x: x[0])
    return scored


def _compatible(c: ComponentSpec) -> bool:
    """Component must have a part number and at least one useful spec."""
    if not c.part_number:
        return False
    gat = c.gain_db or 0.0
    il = c.insertion_loss_db or 0.0
    nf = c.nf_db or 0.0
    has_gain = gat != 0.0 or il != 0.0
    has_nf = nf != 0.0
    return has_gain or has_nf


def _has_required_stages(state: ChainState, required: list[str]) -> bool:
    """Check if chain contains all required stage types (e.g. Mixer, LNA).
    
    Uses ComponentSpec.category set by search_stage_candidates().
    """
    if not required:
        return True
    categories = {c.category for c in state.choices}
    return all(req in categories for req in required)


def _pareto_prune(states: list[ChainState], max_states: int) -> list[ChainState]:
    """Keep only Pareto-optimal states."""
    kept = []
    for s in states:
        dominated = False
        for k in kept:
            if k.dominates(s):
                dominated = True
                break
        if not dominated:
            # Remove states dominated by this new one
            kept = [k for k in kept if not s.dominates(k)]
            kept.append(s)

    # Sort by SNR (via score), then by IIP3
    kept.sort(key=lambda s: (s.score(), -s.iip3_dbm()))
    if len(kept) > max_states:
        # Keep evenly spread along the Pareto frontier
        step = len(kept) // max_states
        kept = kept[::step][:max_states]
    return kept


# ── Unit conversion ──────────────────────────────────────────────────

def db_to_lin_gain(db: float) -> float:
    return 10.0 ** (db / 10.0)


def db_to_lin_nf(db: float) -> float:
    return 10.0 ** (db / 10.0)


def dbm_to_mw(dbm: float) -> float:
    return 10.0 ** (dbm / 10.0)


import re

_FREQ_DESC_RE = re.compile(
    r'(?P<min>\d+\.?\d*)\s*(?:GHz|GHz|Ghz|ghz)\s*(?:[–\-to~]*)\s*'
    r'(?P<max>\d+\.?\d*)\s*(?:GHz|GHz|Ghz|ghz)'
)

_FREQ_SINGLE_RE = re.compile(
    r'(?P<val>\d+\.?\d*)\s*(?:GHz|GHz|Ghz|ghz)'
)


def _extract_freq_from_desc(spec: ComponentSpec) -> None:
    """Parse frequency range from component description as fallback.

    Digi-Key often omits parametric freq_min/max for RF components,
    but the product description includes them (e.g. "18-26 GHz Bandpass
    Filter"). This fills in freq_min_hz/freq_max_hz from the description.
    """
    if not spec.description:
        return

    m = _FREQ_DESC_RE.search(spec.description)
    if m:
        fmin = float(m.group("min")) * 1e9
        fmax = float(m.group("max")) * 1e9
        if fmin > 0 and fmax > 0 and fmin < fmax:
            spec.freq_min_hz = fmin
            spec.freq_max_hz = fmax
            return

    m = _FREQ_SINGLE_RE.search(spec.description)
    if m:
        fc = float(m.group("val")) * 1e9
        if fc > 0:
            bw_pct = 0.05
            spec.freq_min_hz = fc * (1 - bw_pct)
            spec.freq_max_hz = fc * (1 + bw_pct)


# ── API-based candidate search ───────────────────────────────────────

STAGE_KEYWORDS = {
    "Switch": ["RF Switch", "SPDT Switch"],
    "BPF": ["RF Filter", "Band Pass Filter", "Bandpass Filter", "RF Bandpass Filter",
            "Waveguide Bandpass Filter", "K Band Filter", "Ka Band Filter",
            "Cavity Bandpass Filter", "Ceramic Bandpass Filter"],
    "LNA": ["LNA", "Low Noise Amplifier", "RF Amplifier", "Ultra Low Noise Amplifier"],
    "Mixer": ["RF Mixer", "Frequency Mixer"],
    "Limiter": ["RF Limiter", "Limiter"],
    "LO": ["VCO", "Voltage Controlled Oscillator", "Local Oscillator"],
    "VCO": ["VCO", "Voltage Controlled Oscillator"],
    # Tx stages
    "PA": ["Power Amplifier", "RF Power Amplifier", "MMIC Power Amplifier"],
    "Driver": ["RF Driver Amplifier", "Driver Amplifier", "Gain Block"],
    "Upconverter": ["RF Upconverter", "Frequency Mixer"],
    "TxBPF": ["Band Pass Filter", "Bandpass Filter"],
    "HarmonicFilter": ["Low Pass Filter", "Harmonic Filter", "Low Pass Filter"],
    "TxLO": ["VCO", "Voltage Controlled Oscillator", "Local Oscillator"],
}


def _is_spec_plausible(spec: ComponentSpec, stage_name: str) -> bool:
    """Reject components with physically impossible RF specs from API."""
    # NF < 0 dB is physically impossible (None means unknown — allow)
    if spec.nf_db is not None and spec.nf_db < 0:
        return False
    # Insertion loss > 0 is physically correct (positive loss)
    # but API may store as negative — that's fine if abs(IL) >= NF
    # The real check: active stages (LNA, ULNA, Mixer) should have non-zero gain
    if stage_name in ("LNA", "ULNA", "Mixer") and (spec.gain_db is None or spec.gain_db <= 0) and (spec.insertion_loss_db is None or spec.insertion_loss_db == 0):
        return False
    return True


def search_stage_candidates(
    stage_name: str,
    freq_ghz: float,
    max_per_search: int = 50,
    use_cache: bool = True,
    extract_pdfs: bool = False,
    freq_guard_band_pct: float = 10.0,
    use_deep_search: bool = True,        # Phase 1+2: pagination + filters
    collapse_packaging: bool = True,      # Phase 1: CollapsePackingTypes
    max_pages: int = 2,                   # Phase 2: how many pages (50/page)
    use_parametric_filter: bool = False,
    require_freq_data: bool = True,       # Reject components without freq range at RF
) -> list[ComponentSpec]:
    """Search Digi-Key for candidate components matching a stage type.

    Args:
        stage_name: "LNA", "Mixer", "BPF", etc.
        freq_ghz: Target frequency band
        max_per_search: Max results per keyword search
        use_cache: Return cached results if available
        extract_pdfs: Download PDFs and extract RF specs (gain, NF, OIP3)
        freq_guard_band_pct: Reject components that don't cover
            [freq*(1-pct/100), freq*(1+pct/100)].  Default 10% means
            for 24 GHz we require coverage of [21.6, 26.4] GHz.
            0 disables the guard band (matches old exact behaviour).
        use_deep_search: Enable paginated search with SearchOptions.
        collapse_packaging: CollapsePackingTypes SearchOption.
        max_pages: Max API pages to fetch (50 products/page).
        use_parametric_filter: Discover and apply Digi-Key parametric
            frequency-range filters to narrow results at the API level.

    Returns deduplicated ComponentSpec list.
    """
    from component_db import DigikeyClient

    db = ComponentDB()
    candidates: list[ComponentSpec] = []
    seen: set[str] = set()

    keywords = STAGE_KEYWORDS.get(stage_name, [stage_name])

    # Build SearchOptions
    search_options: list[str] = []
    if collapse_packaging:
        search_options.append("CollapsePackingTypes")

    for kw in keywords:
        query = f"{kw} {freq_ghz} GHz" if freq_ghz else kw

        # Check cache first — skip if already searched at ≥ current depth
        if use_cache:
            cached = db.cache.search_by_freq(stage_name, freq_ghz)
            prev = _get_cached_search_depth(stage_name, freq_ghz)
            if cached and prev >= max_pages:
                prev_used_pf = (_load_search_depth()
                                .get(f"{stage_name}@{freq_ghz}", {})
                                .get("use_parametric_filter", False))
                if prev_used_pf == bool(use_parametric_filter):
                    for spec in cached:
                        if spec.part_number and spec.part_number not in seen:
                            if _is_spec_plausible(spec, stage_name):
                                seen.add(spec.part_number)
                                candidates.append(spec)
                    continue

        param_filters = None
        if use_parametric_filter and freq_ghz:
            param_filters = _get_parametric_freq_filter(
                keyword=kw,
                freq_ghz=freq_ghz,
                search_options=search_options,
            )

        # Incremental search: start from cached depth instead of page 0
        search_start_offset = 0
        prev = _get_cached_search_depth(stage_name, freq_ghz)
        if use_cache and prev > 0:
            prev_used_pf = (_load_search_depth()
                            .get(f"{stage_name}@{freq_ghz}", {})
                            .get("use_parametric_filter", False))
            if prev_used_pf == bool(use_parametric_filter):
                search_start_offset = prev * 50

        if use_deep_search:
            # Directly call Digi-Key API with pagination + SearchOptions,
            # bypassing the potentially stale cache.
            try:
                dk = DigikeyClient()
                max_results = max_per_search if max_per_search <= 50 else max_per_search
                products = dk.search_keyword_deep(
                    keyword=query,
                    max_results=max(max_per_search, max_pages * 50),
                    search_options=search_options,
                    parameter_filters=param_filters,
                    start_offset=search_start_offset,
                )
                for raw in products:
                    spec = ComponentSpec.from_digikey(raw)
                    if spec.part_number and spec.part_number not in seen:
                        # Reject components with impossible RF specs
                        if _is_spec_plausible(spec, stage_name):
                            seen.add(spec.part_number)
                            spec.category = stage_name
                            candidates.append(spec)
                            # Cache it
                            db.cache.put(spec)
                _set_cached_search_depth(stage_name, freq_ghz, max_pages,
                                         bool(use_parametric_filter))
            except Exception as e:
                print(f"    [warn] Deep search failed: {e}, falling back to cache")
                parts = db.search(query)
                for spec in parts:
                    if spec.part_number and spec.part_number not in seen:
                        if _is_spec_plausible(spec, stage_name):
                            seen.add(spec.part_number)
                            spec.category = stage_name
                            candidates.append(spec)
        else:
            # Cache-only path: search by category (not text match on part_number)
            # The cache now has category tags from previous deep-search runs.
            parts = db.cache.search_by_category(stage_name)
            for spec in parts:
                if spec.part_number and spec.part_number not in seen:
                    if _is_spec_plausible(spec, stage_name):
                        seen.add(spec.part_number)
                        spec.category = stage_name
                        candidates.append(spec)

    # Try to extract frequency range from component description if parametric
    # data unavailable (Digi-Key often has freq in text but not in structured fields).
    for spec in candidates:
        if (not (freq_ghz and spec.freq_min_hz > 0 and spec.freq_max_hz > 0)
                and spec.description and freq_ghz):
            _extract_freq_from_desc(spec)

    # Frequency guard-band filter: exact match by default.
    # Set --freq-guard-band N for ±N% margin around target.
    # When require_freq_data is True (default) and freq > 1 GHz, components
    # missing frequency range data are rejected to avoid polluting results
    # with parts not designed for the target band (e.g. GPS LNA found for 20 GHz).
    filtered: list[ComponentSpec] = []
    rf_freq_strict = freq_ghz > 5.0
    for spec in candidates:
        has_freq_data = freq_ghz and spec.freq_min_hz > 0 and spec.freq_max_hz > 0
        if has_freq_data:
            f_hz = freq_ghz * 1e9
            if freq_guard_band_pct > 0:
                margin = freq_ghz * (freq_guard_band_pct / 100.0) * 1e9
                lo_req = f_hz - margin
                hi_req = f_hz + margin
                ok = spec.freq_min_hz <= lo_req and hi_req <= spec.freq_max_hz
            else:
                ok = spec.freq_min_hz <= f_hz <= spec.freq_max_hz
            if not ok:
                print(f"    [skip] {spec.part_number}: freq {spec.freq_min_hz/1e9:.1f}-{spec.freq_max_hz/1e9:.1f} GHz "
                      f"does not cover {freq_ghz} GHz"
                      + (f" (guard ±{freq_guard_band_pct:.0f}% @ {freq_ghz} GHz)" if freq_guard_band_pct > 0 else ""))
                continue
        elif require_freq_data and rf_freq_strict:
            print(f"    [skip] {spec.part_number}: no frequency range data, "
                  f"cannot verify {freq_ghz} GHz (use --freq-guard-band 0 to allow)")
            continue
        if extract_pdfs:
            spec = fetch_and_extract_specs(spec, db)
        filtered.append(spec)

    filtered.sort(key=lambda s: (s.manufacturer, s.part_number))
    return filtered


def fetch_and_extract_specs(
    spec: ComponentSpec,
    db: ComponentDB,
    cache_dir: str | Path = "",
) -> ComponentSpec:
    """Download datasheet PDF and extract RF specs via pdf_extractor.

    Overwrites the ComponentSpec fields *only* for non-linear specs
    (OIP3, IIP3, P1dB, freq_min/max) where the regex-based extraction
    is most valuable.  **Gain and NF are kept from the API** because
    PDF extraction frequently picks up table values at wrong frequencies
    and would corrupt the cascade calculation.

    Frequency range (freq_min/max) is also extracted from the PDF as a
    fallback — the API is preferred but often returns zero for these
    fields.  Once populated, subsequent calls to ``estimate_iip3()``
    will use PDF-extracted OIP3−Gain instead of P1dB heuristics.

    Returns the (possibly updated) ComponentSpec.
    """
    from pdf_extractor import extract_pdf
    pdf_path = db.fetch_datasheet(spec)
    if pdf_path and pdf_path.exists():
        try:
            extracted = extract_pdf(str(pdf_path))
            # OIP3/P1dB from PDF — these are what we need for nonlinear
            # cascade calculation at the component's operating frequency.
            if extracted.get("oip3_dbm") is not None and extracted["oip3_dbm"] != "":
                spec.oip3_dbm = float(extracted["oip3_dbm"])
            if extracted.get("p1db_dbm") is not None and extracted["p1db_dbm"] != "":
                spec.p1db_dbm = float(extracted["p1db_dbm"])
            # Frequency range from PDF — only when API returned zero
            pdf_fmin = extracted.get("freq_min_hz")
            pdf_fmax = extracted.get("freq_max_hz")
            if pdf_fmin is not None and pdf_fmax is not None:
                try:
                    fmin = float(pdf_fmin)
                    fmax = float(pdf_fmax)
                    if fmin > 0 and spec.freq_min_hz == 0.0:
                        spec.freq_min_hz = fmin
                    if fmax > 0 and spec.freq_max_hz == 0.0:
                        spec.freq_max_hz = fmax
                except (ValueError, TypeError):
                    pass
            # Insertion loss for passives — only if API had 0
            extracted_il = extracted.get("insertion_loss_db", "")
            if extracted_il not in (None, "") and (not spec.insertion_loss_db or spec.insertion_loss_db == 0.0):
                spec.insertion_loss_db = float(extracted_il)
        except Exception as e:
            print(f"    [warn] pdf_extract failed for {spec.part_number}: {e}")
    return spec


# ── Stage sequence templates ─────────────────────────────────────────

_DEFAULT_BB_FREQ = 0.1  # GHz (100 MHz baseband)


def _resolve_freq(stage_def: StageDef, global_freq: float, if_freq: float, bb_freq: float) -> tuple[str, float]:
    """Resolve (stage_name, search_freq_ghz) from a template entry.

    Accepts:
      "LNA"        → ("LNA", global_freq)
      ("BPF", 2.5) → ("BPF", 2.5)
      ("BPF", None) → ("BPF", global_freq)
      ("BPF", "IF") → ("BPF", if_freq)
      ("BPF", "BB") → ("BPF", bb_freq)
    """
    if isinstance(stage_def, str):
        return stage_def, global_freq
    name, freq_val = stage_def
    if freq_val is None:
        return name, global_freq
    if isinstance(freq_val, str):
        if freq_val.upper() == "IF":
            return name, if_freq
        elif freq_val.upper() == "BB":
            return name, bb_freq
    return name, float(freq_val)


TEMPLATES: dict[str, list[StageDef]] = {
    "7stage":     ["Switch", "BPF", "LNA", "Mixer", "BPF", "LNA"],
    "lna_first":  ["Switch", "LNA", "BPF", "Mixer", "BPF", "LNA"],
    "compact":    ["BPF", "LNA", "Mixer", "LNA"],
    "no_switch":  ["BPF", "LNA", "Mixer", "BPF", "LNA"],
    "full":       ["Switch", "BPF", "LNA", "BPF", "Mixer", "BPF", "LNA", "BPF"],
    # Minimal downconv: no RF BPF (for bands where filter data is unavailable)
    "no_bpf_rf": [
        "Switch",              # RF
        "LNA",                 # RF LNA
        ("Mixer", None),       # RF → IF
        ("BPF", "IF"),         # IF filter
        ("LNA", "IF"),         # IF amplifier
    ],
    # Zero-IF / direct conversion: RF → BB (single mixer, no IF strip)
    "zero_if": [
        "Switch",              # RF
        "BPF",                 # RF preselector
        "LNA",                 # RF LNA
        ("Mixer", None),       # RF → BB direct
        ("BPF", "BB"),         # BB filter
        ("LNA", "BB"),         # BB amplifier
    ],
    # Single IF: RF → IF (single downconversion, ideal for SDR/ADC)
    "single_if": [
        "Switch",              # RF
        "BPF",                 # RF preselector
        "LNA",                 # RF LNA
        ("Mixer", None),       # RF → IF
        ("BPF", "IF"),         # IF filter
        ("LNA", "IF"),         # IF amplifier
    ],
    # Double conversion: RF → IF → BB (full superheterodyne)
    "full_receiver": [
        "Switch",              # RF
        "BPF",                 # RF preselector
        "LNA",                 # RF LNA
        ("BPF", None),         # RF image rejection
        ("Mixer", None),       # RF → IF1
        ("BPF", "IF"),         # IF1 filter
        ("LNA", "IF"),         # IF1 amplifier
        ("Mixer", "IF"),       # IF1 → IF2/BB
        ("BPF", "BB"),         # BB filter
        ("LNA", "BB"),         # BB amplifier
    ],
}

# ── Tx templates ─────────────────────────────────────────────────────
# Digi-Key structured data includes P1dB/OIP3 for PAs but NOT PAE.

TX_TEMPLATES: dict[str, list[StageDef]] = {
    # Single upconversion: IF → RF (direct upconvert)
    "single_upconv": [
        ("BPF", "IF"),         # IF filter
        ("LNA", "IF"),         # IF amplifier
        ("Upconverter", None), # IF → RF upconvert
        ("TxBPF", None),       # RF image rejection
        "Driver",              # RF driver amplifier
        "PA",                  # Power amplifier
        "HarmonicFilter",      # Output harmonic rejection
    ],
    # Double upconversion: IF → IF2 → RF (two-stage upconversion)
    "double_upconv": [
        ("BPF", "IF"),         # IF filter
        ("LNA", "IF"),         # IF amplifier
        ("Upconverter", None), # IF → IF2
        ("TxBPF", None),       # IF2 filter
        "Driver",              # IF2 driver
        ("Upconverter", None), # IF2 → RF upconvert
        ("TxBPF", None),       # RF image rejection
        "Driver",              # RF driver amplifier
        "PA",                  # Power amplifier
        "HarmonicFilter",      # Output harmonic rejection
    ],
}


def optimize_all_templates(
    templates: dict[str, list[StageDef]],
    freq_ghz: float,
    max_per_stage: int = 10,
    extract_pdfs: bool = False,
    freq_guard_band_pct: float = 10.0,
    use_deep_search: bool = True,
    collapse_packaging: bool = True,
    max_pages: int = 2,
    force_full: bool = False,  # use only the full template length
    max_states: int = 500,
    system_config: SystemConfig = None,
    min_stages: int = 3,
    required_stages: list[str] = None,
    if_freq_ghz: float = 2.5,
    bb_freq_ghz: float = 0.1,
    require_freq_data: bool = True,
) -> list[tuple]:
    """Try all templates at all valid stage lengths, return sorted results.
    
    Each result tuple: (score, ChainState, template_name, stage_count)
    """
    all_results: list[tuple] = []
    
    for tmpl_name, stage_types in templates.items():
        print(f"\n  ── Template: {tmpl_name} ({len(stage_types)} stages) ──")
        
        # Search candidates once per (stage_type, freq) pair
        candidate_cache: dict[tuple[str, float], list[ComponentSpec]] = {}
        for sd in stage_types:
            name, sfreq = _resolve_freq(sd, freq_ghz, if_freq_ghz, bb_freq_ghz)
            cache_key = (name, sfreq)
            if cache_key not in candidate_cache:
                print(f"    Searching {name} at ~{sfreq} GHz...")
                candidate_cache[cache_key] = search_stage_candidates(
                    name, sfreq,
                    max_per_search=max_per_stage,
                    extract_pdfs=extract_pdfs,
                    freq_guard_band_pct=freq_guard_band_pct,
                    use_deep_search=use_deep_search,
                    collapse_packaging=collapse_packaging,
                    max_pages=max_pages,
                    require_freq_data=require_freq_data,
                )
                print(f"    → {len(candidate_cache[cache_key])} candidates")
        
        # Try variable lengths
        max_len = len(stage_types)
        lengths = [max_len] if force_full else range(min(min_stages, max_len), max_len + 1)
        
        # NEW: Ensure IF stages after Mixer are included (unless force_full bypasses this)
        # Find the first Mixer stage; if there are stages after it, the minimum length
        # must include the Mixer + at least one post-Mixer (IF/BB) stage.
        if not force_full:
            for i, sd in enumerate(stage_types):
                name, _ = _resolve_freq(sd, freq_ghz, if_freq_ghz, bb_freq_ghz)
                if name == "Mixer" and i + 1 < len(stage_types):
                    min_with_if = max(min_stages, i + 2)
                    lengths = [l for l in lengths if l >= min_with_if]
                    break
        
        for length in lengths:
            sub_defs = stage_types[:length]
            stage_candidates = []
            for sd in sub_defs:
                name, sfreq = _resolve_freq(sd, freq_ghz, if_freq_ghz, bb_freq_ghz)
                cache_key = (name, sfreq)
                stage_candidates.append((name, candidate_cache[cache_key]))
            
            if not any(c for _, c in stage_candidates):
                continue
            
            print(f"\n    [{tmpl_name}] Trying {length} stages: {' → '.join(s[0] for s in stage_candidates)}")
            results = optimize_chain(stage_candidates, max_states=max_states, system_config=system_config)
            
            for score, state in results:
                all_results.append((score, state, tmpl_name, length))
    
    # Filter out chains missing required stage types (e.g. no Mixer → invalid receiver)
    if required_stages:
        before = len(all_results)
        all_results = [r for r in all_results if _has_required_stages(r[1], required_stages)]
        if len(all_results) < before:
            print(f"\n  [filter] Removed {before - len(all_results)} chains missing required stages: {required_stages}")
    
    # Sort by score (ascending = better SNR)
    all_results.sort(key=lambda x: x[0])
    return all_results


# ── Output ───────────────────────────────────────────────────────────

def format_chain_result(results: list[tuple], top_n: int = 5, system_config: SystemConfig | None = None):
    """Pretty-print the top-N optimal chains.
    
    Handles both old format (score, ChainState) and new format
    (score, ChainState, template_name, stage_count).
    """
    print(f"\n{'='*70}")
    print(f"  TOP {min(top_n, len(results))} OPTIMAL CHAINS")
    print(f"{'='*70}")
    
    for rank, item in enumerate(results[:top_n], 1):
        # Support both old and new formats
        if len(item) == 2:
            score, state = item
            tmpl_name = "custom"
            n_stages = len(state.choices)
        else:
            score, state, tmpl_name, n_stages = item
        
        gain = state.gain_db()
        nf = state.nf_db()
        iip3 = state.iip3_dbm()
        snr = state.snr_at_receiver(system_config)
        
        print(f"\n  [{rank}] Template={tmpl_name}  Stages={n_stages}  "
              f"SNR={snr:.2f} dB  Score={score:.2f}")
        print(f"  Gain={gain:.1f} dB  NF={nf:.2f} dB  IIP3={iip3:.1f} dBm")
        print(f"  {'─'*66}")
        for i, c in enumerate(state.choices):
            eff_gain = c.gain_db if c.gain_db else (-c.insertion_loss_db if c.insertion_loss_db else 0.0)
            eff_nf = c.nf_db if c.nf_db else (abs(c.insertion_loss_db) if c.insertion_loss_db else abs(c.gain_db or 0))
            print(f"    Stage {i+1}: {c.part_number:25s}  G={eff_gain:+.1f} dB  NF={eff_nf:.1f} dB  "
                  f"Mfr={c.manufacturer[:20]}")
        print()


def format_chain_result_to_config(results: list[tuple], top_n: int = 1) -> list[str]:
    """Generate receiver.csv lines from the best chain.

    Handles both old format (score, ChainState) and new (score, ChainState, ...).

    Writes three chain sections so the C simulator can run with RF enabled:
      - ``baseband_rx``   — all stages, used for analytical cascade metrics
      - ``rf_frontend``   — stages up to and including the first Mixer
      - ``rf_postmix_bb`` — stages after the first Mixer (IF / baseband)

    Columns (14+ extra): chain,name,gain_db,nf_db,filter_len,is_limiter,
    p1db_dbm,oip3_dbm,ref,enabled,component_uid,part_number,oip3_db,iip3_db,comment,
    stage_type,product_url
    """
    if not results:
        return []
    item = results[0]
    if len(item) >= 2:
        state = item[1]
    else:
        state = item

    def _row(chain_name: str, idx: int, c: ComponentSpec) -> str:
        stage_name = f"{chain_name[:2]}_{idx:02d}_{c.part_number[:8].lower()}"
        eff_gain = c.gain_db if c.gain_db else (-c.insertion_loss_db if c.insertion_loss_db else 0.0)
        eff_nf = c.nf_db if c.nf_db else (c.insertion_loss_db if c.insertion_loss_db else 0.0)
        oip3 = c.oip3_dbm if c.oip3_dbm else ""
        iip3 = c.iip3_dbm if c.iip3_dbm else ""
        p1db = c.p1db_dbm if c.p1db_dbm else ""
        part = c.part_number
        gain_str = f"{eff_gain:.1f}" if eff_gain not in (None, "") else "0.0"
        nf_str = f"{eff_nf:.1f}" if eff_nf not in (None, "") else "0.0"
        stage_type = c.category or ""
        url = c.product_url or c.datasheet_url or ""
        return (f"{chain_name},{stage_name},{gain_str},{nf_str},1,,{p1db},{oip3},"
                f"out,1,,{part},{oip3},{iip3},auto,{stage_type},{url}")

    header = ("chain,name,gain_db,nf_db,filter_len,is_limiter,p1db_dbm,oip3_dbm,"
              "ref,enabled,component_uid,part_number,oip3_db,iip3_db,comment,"
              "stage_type,product_url")

    lines = [header]

    # ── baseband_rx: all stages ─────────────────────────────────────
    for i, c in enumerate(state.choices):
        lines.append(_row("baseband_rx", i, c))

    # ── rf_frontend / rf_postmix_bb split ───────────────────────────
    # Split at the first Mixer: everything up to & including the first
    # Mixer goes into rf_frontend; everything after goes into rf_postmix_bb.
    mixer_idx: int | None = None
    for i, c in enumerate(state.choices):
        cat = (c.category or "").lower()
        if "mixer" in cat:
            mixer_idx = i
            break

    if mixer_idx is not None:
        for i, c in enumerate(state.choices[:mixer_idx + 1]):
            lines.append(_row("rf_frontend", i, c))
        post_mixer = state.choices[mixer_idx + 1:]
        if post_mixer:
            for i, c in enumerate(post_mixer, 1):
                lines.append(_row("rf_postmix_bb", i, c))
        else:
            # C simulator needs rf_postmix_bb defined; insert dummy passthrough
            lines.append("rf_postmix_bb,dummy_bypass,0.0,0.0,1,0,999.0,999.0,out,1,,,,,,,")
    else:
        # No mixer — write everything as rf_frontend + dummy rf_postmix_bb
        for i, c in enumerate(state.choices):
            lines.append(_row("rf_frontend", i, c))
        lines.append("rf_postmix_bb,dummy_bypass,0.0,0.0,1,0,999.0,999.0,out,1,,,,,,,")

    return lines


def format_tx_chain_result_to_config(results: list[tuple]) -> list[str]:
    """Generate Tx chain CSV lines (same column format as Rx for C simulator).

    Tx chain has a single 'tx_chain' section (no rf_frontend/postmix split)
    since there is no downconversion to split on.
    """
    if not results:
        return []
    item = results[0]
    state = item[1] if len(item) >= 2 else item

    def _row(chain_name: str, idx: int, c: ComponentSpec) -> str:
        stage_name = f"{chain_name[:2]}_{idx:02d}_{c.part_number[:8].lower()}"
        eff_gain = c.gain_db if c.gain_db else (-c.insertion_loss_db if c.insertion_loss_db else 0.0)
        eff_nf = c.nf_db if c.nf_db else (c.insertion_loss_db if c.insertion_loss_db else 0.0)
        oip3 = c.oip3_dbm if c.oip3_dbm else ""
        iip3 = c.iip3_dbm if c.iip3_dbm else ""
        p1db = c.p1db_dbm if c.p1db_dbm else ""
        part = c.part_number
        gain_str = f"{eff_gain:.1f}" if eff_gain not in (None, "") else "0.0"
        nf_str = f"{eff_nf:.1f}" if eff_nf not in (None, "") else "0.0"
        stage_type = c.category or ""
        url = c.product_url or c.datasheet_url or ""
        return (f"{chain_name},{stage_name},{gain_str},{nf_str},1,,{p1db},{oip3},"
                f"out,1,,{part},{oip3},{iip3},auto,{stage_type},{url}")

    header = ("chain,name,gain_db,nf_db,filter_len,is_limiter,p1db_dbm,oip3_dbm,"
              "ref,enabled,component_uid,part_number,oip3_db,iip3_db,comment,"
              "stage_type,product_url")
    lines = [header]
    for i, c in enumerate(state.choices):
        lines.append(_row("tx_chain", i, c))
    return lines


# ── Post-optimization PDF verification ────────────────────────────────

_PROJECT_ROOT = os.path.dirname(_SCRIPTS_DIR)


def recalculate_chain(choices: list[ComponentSpec]) -> ChainState:
    """Compute full cascade metrics from a concrete list of components.

    Re-applies the same Friis + IIP3 cascade formulas that
    optimize_chain() uses, but with only one candidate per stage.
    """
    state = ChainState()
    for c in choices:
        g_lin = db_to_lin_gain(c.gain_db if c.gain_db else 0.0)
        if c.gain_db is not None and c.gain_db <= 0 and c.insertion_loss_db:
            g_lin = db_to_lin_gain(-c.insertion_loss_db)

        if c.nf_db:
            nf_val = c.nf_db
        elif c.insertion_loss_db and (c.gain_db is None or c.gain_db <= 0):
            nf_val = c.insertion_loss_db
        else:
            nf_val = abs(c.gain_db) if c.gain_db else 0.0
        nf_lin = db_to_lin_nf(nf_val)

        iip3_val = estimate_iip3(c, c.category or "")
        iip3_lin = dbm_to_mw(iip3_val) if iip3_val else 1e9
        p1db_val = c.p1db_dbm if c.p1db_dbm else 999.0
        p1db_lin = dbm_to_mw(p1db_val) if p1db_val else 1e9

        g_cum = state.gain_lin
        state = ChainState(
            stage_idx=state.stage_idx + 1,
            gain_lin=state.gain_lin * g_lin,
            nf_lin=state.nf_lin + (nf_lin - 1.0) / g_cum,
            iip3_inv=state.iip3_inv + g_cum / iip3_lin,
            p1db_inv=state.p1db_inv + g_cum / p1db_lin,
            choices=state.choices + [c],
        )
    return state


def refine_best_chain(
    results: list[tuple],
    system_config: SystemConfig | None = None,
) -> list[tuple]:
    """Post-optimisation PDF extraction for the best chain *only*.

    Downloads datasheet PDFs for each component in the #1 chain,
    extracts real RF specs (OIP3, P1dB, gain, NF) via pdf_extractor.py,
    recalculates cascade metrics with the verified values, and returns
    the updated results list.

    Step-by-step:
      1. For each ComponentSpec in the best chain, call
         ``fetch_and_extract_specs()`` which downloads the PDF and
         runs regex extraction.
      2. Rebuild a fresh ``ChainState`` using ``recalculate_chain()`` so
         that cumulative NF, IIP3, P1dB reflect the real datasheet values.
      3. Print a before/after comparison table.
      4. Replace the #1 entry in *results* with the updated state.
    """
    if not results:
        return results

    item = results[0]
    if len(item) >= 2:
        score = item[0]
        state = item[1]
        tmpl_name: str = item[2] if len(item) > 2 else ""
        n_stages: int = item[3] if len(item) > 3 else len(state.choices)
    else:
        return results  # unparseable format, skip

    print(f"\n{'='*70}")
    print(f"  DATASHEET PDF VERIFICATION — Best Chain")
    print(f"{'='*70}")

    old_snr = state.snr_at_receiver(system_config)
    old_iip3 = state.iip3_dbm()
    old_gain = state.gain_db()
    old_nf = state.nf_db()

    db = ComponentDB()

    # Download + extract for every component in the best chain
    updated_choices: list[ComponentSpec] = []
    for i, c in enumerate(state.choices):
        stage_cat = c.category or ""
        print(f"  [{i+1}/{len(state.choices)}] {c.part_number:25s} ({stage_cat}) ... ",
              end="", flush=True)
        updated = fetch_and_extract_specs(c, db)
        updated_choices.append(updated if updated is not None else c)
        print("done")

    # Recalculate with extracted values
    new_state = recalculate_chain(updated_choices)
    new_score = new_state.score(system_config)
    new_snr = new_state.snr_at_receiver(system_config)
    new_iip3 = new_state.iip3_dbm()
    new_nf = new_state.nf_db()
    new_gain = new_state.gain_db()

    if len(item) == 2:
        results[0] = (new_score, new_state)
    else:
        results[0] = (new_score, new_state, tmpl_name, n_stages)

    # Before / after comparison
    print(f"\n  {'─'*66}")
    print(f"  {'Metric':20s}  {'Before (est.)':>14s}  {'After (extracted)':>18s}  {'Δ':>6s}")
    print(f"  {'─'*66}")
    print(f"  {'SNR  (dB)':20s}  {old_snr:>14.2f}  {new_snr:>18.2f}  {new_snr-old_snr:>+6.2f}")
    print(f"  {'NF   (dB)':20s}  {old_nf:>14.2f}  {new_nf:>18.2f}  {new_nf-old_nf:>+6.2f}")
    print(f"  {'Gain (dB)':20s}  {old_gain:>14.1f}  {new_gain:>18.1f}  {new_gain-old_gain:>+6.1f}")
    print(f"  {'IIP3 (dBm)':20s}  {old_iip3:>14.1f}  {new_iip3:>18.1f}  {new_iip3-old_iip3:>+6.1f}")
    print(f"  {'─'*66}")
    print()

    return results


def run_c_simulation(csv_path: str) -> int:
    """Invoke the C simulator with RF + realistic impairments enabled.

    Calls::

        bin/dual_receiver_sim --stage-csv <csv_path>

    with the project root as working directory.  Both RF and realistic
    paths are enabled by default.  If the CSV does not contain post-mixer
    stages (``rf_postmix_bb`` chain is empty), RF mode is automatically
    disabled since the simulator cannot upconvert/downconvert without both
    chain sections.

    Returns the simulator's exit code.
    """
    sim_bin = os.path.join(_PROJECT_ROOT, "bin", "dual_receiver_sim")
    if not os.path.isfile(sim_bin) or not os.access(sim_bin, os.X_OK):
        print(f"\n  [warn] C simulator not found or not executable: {sim_bin}")
        return -1

    csv_abs = os.path.abspath(csv_path)

    # Check if rf_postmix_bb has real stages (scan CSV)
    has_postmix = False
    try:
        with open(csv_abs) as f:
            for line in f:
                if line.startswith("rf_postmix_bb,") and line.count(",") > 2:
                    has_postmix = True
                    break
    except OSError:
        pass

    cmd = [sim_bin, "--stage-csv", csv_abs]
    mode_label = "RF + Realistic Impairments"
    if not has_postmix:
        cmd.append("--disable-rf")
        mode_label = "Baseband-only (no RF — no post-mixer stages in chain)"

    print(f"\n{'='*70}")
    print(f"  C SIMULATION — {mode_label}")
    print(f"{'='*70}")
    print(f"  Binary : {sim_bin}")
    print(f"  Config : {csv_abs}")
    print()

    try:
        result = subprocess.run(
            cmd,
            cwd=_PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=600,  # 10 minutes max
        )
        # Print last ~50 lines of output (SNR summary etc.)
        out_lines = result.stdout.splitlines()
        tail = out_lines[-60:] if len(out_lines) > 60 else out_lines
        print("\n".join(tail))

        if result.returncode != 0:
            print(f"\n  [error] C sim exited with code {result.returncode}")
            if result.stderr:
                print(f"  stderr: {result.stderr[-500:]}")
        else:
            print(f"\n  [ok] C simulation finished (exit code 0)")

        return result.returncode
    except FileNotFoundError:
        print(f"\n  [error] binary not found: {sim_bin}")
        return -2
    except subprocess.TimeoutExpired:
        print(f"\n  [error] C sim timed out after 600 s")
        return -3


# ── CLI ──────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="End-to-end: search Digi-Key → extract specs → DP optimize → output best chain"
    )
    parser.add_argument("--freq", type=float, default=24.0,
                        help="RF frequency band in GHz (default: 24)")
    parser.add_argument("--if-freq", type=float, default=2.5,
                        help="IF frequency in GHz for full receiver (default: 2.5)")
    parser.add_argument("--bb-freq", type=float, default=0.1,
                        help="Baseband frequency in GHz (default: 0.1)")
    parser.add_argument("--if-sweep", type=str, default="",
                        help="Comma-separated IF frequencies to sweep (GHz), e.g. '1.0,2.5,5.0'")
    parser.add_argument("--bb-sweep", type=str, default="",
                        help="Comma-separated BB frequencies to sweep (GHz), e.g. '0.05,0.1,0.5'")
    parser.add_argument("--top", type=int, default=5,
                        help="Number of top chains to show (default: 5)")
    parser.add_argument("--stages", type=str, default="",
                        help="Fixed comma-separated stage types (overrides --templates)")
    parser.add_argument("--extract", action="store_true",
                        help="Download PDFs for ALL candidates during search (slow)")
    parser.add_argument("--verify", action="store_true",
                        help="Post-optimisation: download PDFs only for the best chain, "
                             "extract real OIP3/P1dB/Gain/NF from datasheets, update CSV")
    parser.add_argument("--run-sim", action="store_true",
                        help="After writing CSV, auto-run C simulation with "
                             "full RF + realistic impairments enabled")
    parser.add_argument("--output", type=str, default="",
                        help="Write best chain as CSV to this path")
    parser.add_argument("--max-per-stage", type=int, default=50,
                        help="Max candidates per stage (default: 50)")
    parser.add_argument("--no-deep-search", action="store_true",
                        help="Disable paginated search with SearchOptions (Phase 1+2)")
    parser.add_argument("--no-collapse-packaging", action="store_true",
                        help="Disable CollapsePackingTypes SearchOption")
    parser.add_argument("--parametric-filter", action="store_true", default=False,
                        help="Use Digi-Key parametric frequency filter (more precise search)")
    parser.add_argument("--deep-pages", type=int, default=2,
                        help="Number of API pages to fetch (50/page, default: 2)")
    parser.add_argument("--require-freq-data", action="store_true", default=True,
                        help="Reject components without frequency range data at RF (default: True). "
                             "Set --no-require-freq-data to allow parts with unknown freq range.")
    parser.add_argument("--no-require-freq-data", action="store_false", dest="require_freq_data",
                        help="Allow components without frequency range data")
    
    # SNR optimization
    parser.add_argument("--snr-optimize", action="store_true", default=True,
                        help="Use SNR-at-receiver as objective (default: True)")
    parser.add_argument("--antenna-temp", type=float, default=91.0,
                        help="Antenna temperature in K (default: 91)")
    parser.add_argument("--bw", type=float, default=200.0e6,
                        help="System bandwidth in Hz (default: 200.0e6 = B_NOISE_HZ)")
    parser.add_argument("--freq-guard-band", type=float, default=0.0,
                        help="Frequency guard band in %% — reject components that "
                             "don't cover [f*(1-pct), f*(1+pct)]. 0=exact match "
                             "(default: 0 = exact match, manual chain passes)")
    parser.add_argument("--input-signal", type=float, default=-65.8,
                        help="Input signal power in dBm (default: -65.8)")
    parser.add_argument("--min-stages", type=int, default=3,
                        help="Minimum number of stages (default: 3)")
    parser.add_argument("--force-full", action="store_true",
                        help="Use only the FULL length of each template (no shorter variants). "
                             "Required for realistic downconversion chains.")
    parser.add_argument("--required-stages", type=str, default="Mixer,LNA",
                        help="Comma-separated mandatory stage types (default: Mixer,LNA)")
    
    # Templates
    parser.add_argument("--templates", type=str,
                        default="lna_first,full,zero_if,single_if,full_receiver",
                        help="Comma-separated template names, or 'all' for all defined")
    
    # Tx mode
    parser.add_argument("--tx", action="store_true",
                        help="Transmitter mode: use Tx templates, P1dB objective, Tx output")
    
    args = parser.parse_args()

    # Tx mode: switch templates, objective, and output
    if args.tx:
        ChainState.score = ChainState.tx_score
        ChainState.dominates = ChainState.tx_dominates

    # Build system config
    sys_cfg = SystemConfig(
        antenna_temp_k=args.antenna_temp,
        t0_k=290.0,
        bw_hz=args.bw,
        input_signal_dbm=args.input_signal,
    )

    # Determine stage sequences to try
    _all_templates = TX_TEMPLATES if args.tx else TEMPLATES
    if args.stages:
        # Fixed stages mode (backward compat)
        templates = {"custom": [s.strip() for s in args.stages.split(",")]}
    else:
        # Template mode
        if args.templates == "all":
            templates = dict(_all_templates)
        else:
            tmpl_names = [t.strip() for t in args.templates.split(",")]
            templates = {}
            for name in tmpl_names:
                if name in _all_templates:
                    templates[name] = _all_templates[name]
                else:
                    print(f"  [warn] Unknown template '{name}', skipping")
            if not templates:
                print("No valid templates. Using default.")
                default_tmpl = "single_upconv" if args.tx else "7stage"
                templates = {default_tmpl: _all_templates[default_tmpl]}

    mode_str = "TRANSMITTER" if args.tx else "RECEIVER"
    
    mode_str = "TRANSMITTER" if args.tx else "RECEIVER"

    if args.tx and not args.output:
        args.output = "data_input/optimized/transmitter.csv"

    print(f"\n{'='*65}")
    print(f"  {mode_str} CHAIN OPTIMIZER")
    print(f"  {args.freq} GHz  |  extract={'yes' if args.extract else 'no'}")
    print(f"  Templates: {', '.join(templates.keys())}")
    print(f"  Objective: {'SNR at receiver' if args.snr_optimize else 'weighted NF+IIP3'}")
    if any(isinstance(sd, tuple) and not isinstance(sd, str) for tmpl in templates.values() for sd in tmpl):
        print(f"  Freq bands: RF={args.freq} GHz, IF={args.if_freq} GHz, BB={args.bb_freq} GHz")
    required = [s.strip() for s in args.required_stages.split(",") if s.strip()]
    if required:
        print(f"  Required stages: {', '.join(required)}")
    print(f"{'='*65}")
    
    if args.snr_optimize:
        deep_status = f"deep={not args.no_deep_search}({args.deep_pages}pg)"
        freq_req = "req_freq" if args.require_freq_data else "allow_no_freq"
        print(f"  System: T_ant={args.antenna_temp} K, BW={args.bw/1e6:.1f} MHz, "
              f"Si={args.input_signal:.1f} dBm (SNR_in={sys_cfg.input_snr_db():.1f} dB), "
              f"guard_band=±{args.freq_guard_band:.0f}%, {deep_status}, {freq_req}")
    
    # ── Frequency sweep ────────────────────────────────────────────────
    if_freqs = [float(f.strip()) for f in args.if_sweep.split(",") if f.strip()] if args.if_sweep else [args.if_freq]
    bb_freqs = [float(f.strip()) for f in args.bb_sweep.split(",") if f.strip()] if args.bb_sweep else [args.bb_freq]
    n_combos = len(if_freqs) * len(bb_freqs)
    if n_combos > 1:
        print(f"  Frequency sweep: {n_combos} combos (IF={if_freqs} x BB={bb_freqs})")
    
    all_results: list = []
    best_results: list = []
    for if_f in if_freqs:
        for bb_f in bb_freqs:
            label = f"IF={if_f}GHz, BB={bb_f}GHz"
            if n_combos > 1:
                print(f"\n  ── Sweep: {label} ──")
            results = optimize_all_templates(
                templates=templates,
                freq_ghz=args.freq,
                max_per_stage=args.max_per_stage,
                extract_pdfs=args.extract,
                freq_guard_band_pct=args.freq_guard_band,
                use_deep_search=not args.no_deep_search,
                collapse_packaging=not args.no_collapse_packaging,
                max_pages=args.deep_pages,
                force_full=args.force_full,
                system_config=sys_cfg if args.snr_optimize else None,
                min_stages=args.min_stages,
                required_stages=required,
                if_freq_ghz=if_f,
                bb_freq_ghz=bb_f,
                require_freq_data=args.require_freq_data,
            )
            if results:
                if not best_results or results[0][0] < best_results[0][0]:
                    best_results = results
                    print(f"  ✓ New best: {label}  SNR={results[0][1].score():.2f}")
    
    results = best_results
    if n_combos > 1 and results:
        print(f"\n  ✓ Best across sweep: IF={results[0][2] if len(results[0])>2 else if_freqs[0]} GHz")
    
    if results:
        format_chain_result(results, top_n=args.top, system_config=sys_cfg if args.snr_optimize else None)

        if args.verify:
            results = refine_best_chain(results, sys_cfg if args.snr_optimize else None)
            format_chain_result(results, top_n=1, system_config=sys_cfg if args.snr_optimize else None)

        if args.output:
            out_path = Path(args.output)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            if args.tx:
                config_lines = format_tx_chain_result_to_config(results)
            else:
                config_lines = format_chain_result_to_config(results)
            with open(out_path, "w") as f:
                for line in config_lines:
                    f.write(line + "\n")
            print(f"\n  Best chain written to: {args.output}")

        if args.run_sim and args.output:
            run_c_simulation(args.output)
    else:
        print("\nNo valid chain found.")


if __name__ == "__main__":
    main()
