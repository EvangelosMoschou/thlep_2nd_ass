#!/usr/bin/env python3
"""Exhaustive component sweep runner for receiver_dual_sim.

This script:
1) Reads component options from data/component_catalog.csv
2) Reads topology models from stage_models/stage_models.csv
3) Generates canonical stage-model CSVs for every component combination
4) Runs the simulator and collects final SNR/EVM metrics
5) Ranks combinations and saves top configurations
6) Optionally re-runs top candidates with higher symbol count
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import itertools
import math
import os
import queue
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


NUM_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


ROLE_FROM_COMPONENT_NAME = {
    "filter 1": "FILTER_1",
    "lna 1": "LNA_1",
    "mixer 1": "MIXER_1",
    "filter 2": "FILTER_2",
    "lna 2": "LNA_2",
    "mixer 2": "MIXER_2",
    "filter 3": "FILTER_3",
    "lna 3": "LNA_3",
    "limiter": "LIMITER",
}


FILTER_LEN_BY_ROLE = {
    "FILTER_1": 3,
    "FILTER_2": 5,
    "FILTER_3": 5,
    "LNA_1": 1,
    "LNA_2": 1,
    "LNA_3": 1,
    "MIXER_1": 1,
    "MIXER_2": 1,
    "LIMITER": 1,
}


@dataclass(frozen=True)
class Component:
    uid: str
    component_name: str
    part_number: str
    role: str
    gain_db: float
    nf_db: float
    nf_inferred: bool
    iip3_dbm: float
    ip3_inferred: bool


@dataclass(frozen=True)
class RoleInstance:
    role: str
    block_label: str
    ordinal: int


@dataclass
class SweepResult:
    design_id: int
    combo_index: int
    quality_score: float
    signal_quality_score: float
    nf_ip3_score: float
    final_bb_snr_db: float
    final_bb_evm_pct: float
    final_rf_snr_db: float
    final_rf_evm_pct: float
    effective_bb_gain_db: float
    estimated_bb_nf_db: float
    estimated_bb_iip3_dbm: float
    selected_uid_chain: str
    selected_part_chain: str
    stage_csv_path: str


def parse_float_maybe(text: str) -> Optional[float]:
    if text is None:
        return None
    s = text.strip()
    if not s:
        return None
    m = NUM_RE.search(s)
    if not m:
        return None
    try:
        return float(m.group(0))
    except ValueError:
        return None


def infer_nf_db(component_name: str, gain_db: float) -> float:
    name = component_name.lower()
    if "filter" in name or "mixer" in name:
        return abs(gain_db) if gain_db < 0.0 else max(0.5, gain_db)
    if "lna" in name:
        return 2.0
    return 3.0


def infer_iip3_dbm(component_name: str, gain_db: float, p1db_db: Optional[float], oip3_db: Optional[float], iip3_db: Optional[float]) -> Tuple[float, bool]:
    if iip3_db is not None:
        return iip3_db, False

    if oip3_db is not None:
        if p1db_db is not None:
            # When the catalog provides OIP3 and P1dB, treat P1dB as output-referred.
            return p1db_db - gain_db + 10.0, True
        # OIP3 ~= IIP3 + Gain (dB)
        return oip3_db - gain_db, True

    if p1db_db is not None:
        # Rule-of-thumb for many active RF devices
        return p1db_db + 10.0, True

    name = component_name.lower()
    if "filter" in name:
        return 45.0, True
    if "mixer" in name:
        return 20.0, True
    if "lna" in name:
        return 25.0, True
    return 20.0, True


def normalize_block(text: str) -> str:
    s = text.lower().strip()
    s = s.replace("(", " ").replace(")", " ")
    s = re.sub(r"\s+", " ", s)
    return s


def block_to_role(block_label: str, mixers_seen: int) -> Optional[str]:
    b = normalize_block(block_label)

    if b == "rf bpf":
        return "FILTER_1"
    if b == "lna":
        if mixers_seen == 0:
            return "LNA_1"
        if mixers_seen == 1:
            return "LNA_2"
        return "LNA_3"
    if b in {"irf", "if1 bpf", "if bpf"}:
        return "FILTER_2"
    if "image reject mixer" in b or "mixer simple" in b:
        return "MIXER_1"
    if b == "mixer":
        return "MIXER_2" if mixers_seen >= 1 else "MIXER_1"
    if b == "bpf":
        if mixers_seen >= 2:
            return "FILTER_3"
        if mixers_seen >= 1:
            return "FILTER_2"
        return "FILTER_1"
    if "limiter" in b:
        return "LIMITER"
    return None


def load_component_catalog(path: Path, allow_inferred_nf: bool, allow_inferred_ip3: bool) -> Dict[str, List[Component]]:
    by_role: Dict[str, List[Component]] = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            component_name = (row.get("component_name") or "").strip()
            role = ROLE_FROM_COMPONENT_NAME.get(component_name.lower())
            if not role:
                continue

            uid = (row.get("component_uid") or "").strip()
            part_number = (row.get("part_number") or "").strip()
            gain_db = parse_float_maybe(row.get("gain_loss_db") or "")
            nf_db = parse_float_maybe(row.get("noise_figure_db") or "")
            p1db_db = parse_float_maybe(row.get("p1db_db") or "")
            oip3_db = parse_float_maybe(row.get("oip3_db") or "")
            iip3_db = parse_float_maybe(row.get("iip3_db") or "")

            if gain_db is None:
                continue

            inferred = False
            if nf_db is None:
                if not allow_inferred_nf:
                    continue
                nf_db = infer_nf_db(component_name, gain_db)
                inferred = True

            iip3_val, ip3_inferred = infer_iip3_dbm(component_name, gain_db, p1db_db, oip3_db, iip3_db)
            if ip3_inferred and not allow_inferred_ip3 and iip3_db is None and oip3_db is None and p1db_db is None:
                continue

            comp = Component(
                uid=uid,
                component_name=component_name,
                part_number=part_number,
                role=role,
                gain_db=gain_db,
                nf_db=nf_db,
                nf_inferred=inferred,
                iip3_dbm=iip3_val,
                ip3_inferred=ip3_inferred,
            )
            by_role.setdefault(role, []).append(comp)

    for role_items in by_role.values():
        role_items.sort(key=lambda c: c.uid)
    return by_role


def load_design_roles(path: Path) -> Dict[int, List[RoleInstance]]:
    designs: Dict[int, List[RoleInstance]] = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            model_raw = (row.get("Model") or "").strip()
            if not model_raw:
                continue
            try:
                design_id = int(model_raw)
            except ValueError:
                continue

            blocks: List[str] = []
            for i in range(1, 11):
                block = (row.get(f"Block {i}") or "").strip()
                if block:
                    blocks.append(block)

            roles: List[RoleInstance] = []
            mixers_seen = 0
            role_ordinals: Dict[str, int] = {}
            for block in blocks:
                role = block_to_role(block, mixers_seen)
                if role is None:
                    continue
                if role.startswith("MIXER"):
                    mixers_seen += 1

                role_ordinals[role] = role_ordinals.get(role, 0) + 1
                roles.append(RoleInstance(role=role, block_label=block, ordinal=role_ordinals[role]))

            designs[design_id] = roles

    return designs


def role_instance_label(role_inst: RoleInstance) -> str:
    return f"{role_inst.role}_{role_inst.ordinal}"


def slugify_text(text: str) -> str:
    s = (text or "").strip().lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "stage"


def stage_name(chain_prefix: str, idx: int, role_inst: RoleInstance, comp: Component) -> str:
    role_slug = slugify_text(role_inst.block_label)
    component_slug = slugify_text(comp.part_number or comp.component_name or comp.uid)
    return f"{chain_prefix}_{idx:02d}_{role_slug}_{component_slug}"


def split_roles_for_chains(roles: Sequence[RoleInstance]) -> Tuple[List[int], List[int], List[int]]:
    active_indexes = [i for i, r in enumerate(roles) if r.role != "LIMITER"]
    if not active_indexes:
        return [], [], []

    first_mixer_pos = None
    for i in active_indexes:
        if roles[i].role == "MIXER_1":
            first_mixer_pos = i
            break

    if first_mixer_pos is None:
        front = active_indexes[: max(1, min(2, len(active_indexes)))]
        post = active_indexes[max(1, len(active_indexes) - 2) :]
        return active_indexes, front, post

    front = [i for i in active_indexes if i < first_mixer_pos]
    post = [i for i in active_indexes if i > first_mixer_pos]

    if not front:
        front = [active_indexes[0]]
    if not post:
        post = [active_indexes[-1]]

    return active_indexes, front, post


def write_stage_csv(
    path: Path,
    roles: Sequence[RoleInstance],
    selected_by_role_index: Sequence[Optional[Component]],
) -> None:
    selected_by_idx = {idx: comp for idx, comp in enumerate(selected_by_role_index)}

    baseband_idxs, rf_front_idxs, rf_post_idxs = split_roles_for_chains(roles)

    rows: List[List[object]] = []

    def append_chain_rows(chain: str, indexes: Sequence[int], chain_prefix: str, auto_gain_last: bool) -> None:
        if not indexes:
            return
        for out_idx, role_idx in enumerate(indexes, start=1):
            role_inst = roles[role_idx]
            comp = selected_by_idx.get(role_idx)
            if comp is None:
                continue
            is_last = out_idx == len(indexes)
            auto_gain = 1 if (auto_gain_last and is_last) else 0
            target_vpp = 1.0 if auto_gain else 0.0

            rows.append(
                [
                    chain,
                    stage_name(chain_prefix, out_idx, role_inst, comp),
                    f"{comp.gain_db:.6f}",
                    f"{comp.nf_db:.6f}",
                    FILTER_LEN_BY_ROLE.get(role_inst.role, 1),
                    auto_gain,
                    f"{target_vpp:.1f}",
                    1,
                ]
            )

    append_chain_rows("baseband_rx", baseband_idxs, "bb", auto_gain_last=True)
    append_chain_rows("rf_frontend", rf_front_idxs, "rf", auto_gain_last=False)
    append_chain_rows("rf_postmix_bb", rf_post_idxs, "post", auto_gain_last=True)

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["chain", "name", "gain_db", "nf_db", "filter_len", "auto_gain_to_vpp", "target_vpp", "enabled"])
        writer.writerows(rows)


def parse_metrics_file(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def parse_metric_float(value: str) -> float:
    v = float(value)
    if math.isnan(v):
        return float("nan")
    return v


def safe_db_ratio(num: float, den: float) -> float:
    if num <= 0.0 or den <= 0.0:
        return float("nan")
    return 10.0 * math.log10(num / den)


def estimate_friis_nf_db(comps: Sequence[Component]) -> float:
    if not comps:
        return float("nan")

    f_total = None
    g_prod = 1.0
    for comp in comps:
        f_lin = 10.0 ** (comp.nf_db / 10.0)
        g_lin = 10.0 ** (comp.gain_db / 10.0)
        if f_total is None:
            f_total = f_lin
        else:
            f_total += (f_lin - 1.0) / g_prod
        g_prod *= g_lin

    if f_total is None or f_total <= 0.0:
        return float("nan")
    return 10.0 * math.log10(f_total)


def estimate_cascaded_iip3_dbm(comps: Sequence[Component]) -> float:
    if not comps:
        return float("nan")

    reciprocal = 0.0
    g_before = 1.0

    for comp in comps:
        iip3_mw = 10.0 ** (comp.iip3_dbm / 10.0)
        if iip3_mw <= 0.0:
            return float("nan")
        reciprocal += g_before / iip3_mw
        g_before *= 10.0 ** (comp.gain_db / 10.0)

    if reciprocal <= 0.0:
        return float("nan")

    iip3_total_mw = 1.0 / reciprocal
    return 10.0 * math.log10(iip3_total_mw)


def quality_score_signal(bb_snr: float, bb_evm: float, rf_snr: float, rf_evm: float) -> float:
    rf_evm_term = rf_evm if not math.isnan(rf_evm) else 999.0
    return (bb_snr + rf_snr) - 0.5 * (bb_evm + rf_evm_term)


def quality_score_nf_ip3(est_nf_db: float, est_iip3_dbm: float, nf_weight: float, ip3_weight: float) -> float:
    if math.isnan(est_nf_db) or math.isnan(est_iip3_dbm):
        return -1.0e9
    return (-nf_weight * est_nf_db) + (ip3_weight * est_iip3_dbm)


def quality_score_total(
    objective: str,
    signal_score: float,
    nf_ip3_score: float,
) -> float:
    if objective == "snr-evm":
        return signal_score
    if objective == "nf-ip3":
        return nf_ip3_score
    # hybrid
    return signal_score + nf_ip3_score


def evaluate_combination(
    root: Path,
    bin_path: Path,
    slot_queue: "queue.Queue[int]",
    stage_csv: Path,
    combo_index: int,
    design_id: int,
    selected_by_role_index: Sequence[Optional[Component]],
    active_role_indexes: Sequence[int],
    seed: int,
    symbols: int,
    snr: float,
    quiet: bool,
    objective: str,
    nf_weight: float,
    ip3_weight: float,
) -> Optional[SweepResult]:
    slot = slot_queue.get()
    try:
        cmd = [
            str(bin_path),
            "--seed",
            str(seed),
            "--symbols",
            str(symbols),
            "--snr",
            str(snr),
            "--stage-csv",
            str(stage_csv),
            "--topology-sim",
            str(slot),
        ]
        rc = run_cmd(cmd, cwd=root, quiet=quiet)
        if rc != 0:
            return None

        csv_dir, _ = topology_output_paths(root, slot)
        bb_metrics = parse_metrics_file(csv_dir / "stage_metrics_baseband.csv")
        rf_metrics = parse_metrics_file(csv_dir / "stage_metrics_rf.csv")
        if not bb_metrics or not rf_metrics:
            return None

        bb_first = bb_metrics[0]
        bb_last = bb_metrics[-1]
        rf_last = rf_metrics[-1]

        bb_snr = parse_metric_float(bb_last["snr_db"])
        bb_evm = parse_metric_float(bb_last["evm_pct"])
        rf_snr = parse_metric_float(rf_last["snr_db"])
        rf_evm = parse_metric_float(rf_last["evm_pct"])

        bb_input_sig = parse_metric_float(bb_first["signal_power"])
        bb_output_sig = parse_metric_float(bb_last["signal_power"])
        eff_gain_db = safe_db_ratio(bb_output_sig, bb_input_sig)

        selected_active = [selected_by_role_index[i] for i in active_role_indexes]
        selected_active = [c for c in selected_active if c is not None]
        est_nf = estimate_friis_nf_db(selected_active)
        est_iip3 = estimate_cascaded_iip3_dbm(selected_active)
        signal_score = quality_score_signal(bb_snr, bb_evm, rf_snr, rf_evm)
        nf_ip3_score = quality_score_nf_ip3(est_nf, est_iip3, nf_weight, ip3_weight)
        score = quality_score_total(objective, signal_score, nf_ip3_score)

        uid_chain = "|".join(c.uid for c in selected_active)
        part_chain = "|".join(c.part_number for c in selected_active)

        return SweepResult(
            design_id=design_id,
            combo_index=combo_index,
            quality_score=score,
            signal_quality_score=signal_score,
            nf_ip3_score=nf_ip3_score,
            final_bb_snr_db=bb_snr,
            final_bb_evm_pct=bb_evm,
            final_rf_snr_db=rf_snr,
            final_rf_evm_pct=rf_evm,
            effective_bb_gain_db=eff_gain_db,
            estimated_bb_nf_db=est_nf,
            estimated_bb_iip3_dbm=est_iip3,
            selected_uid_chain=uid_chain,
            selected_part_chain=part_chain,
            stage_csv_path=str(stage_csv),
        )
    finally:
        slot_queue.put(slot)


def run_cmd(cmd: Sequence[str], cwd: Path, quiet: bool = True) -> int:
    if quiet:
        proc = subprocess.run(cmd, cwd=str(cwd), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    else:
        proc = subprocess.run(cmd, cwd=str(cwd), check=False)
    return proc.returncode


def topology_output_paths(root: Path, topology_sim: int) -> Tuple[Path, Path]:
    csv_dir = root / "out" / f"topology_sim_{topology_sim}" / "csv"
    svg_dir = root / "out" / f"topology_sim_{topology_sim}" / "svg"
    return csv_dir, svg_dir


def write_results_csv(path: Path, rows: Sequence[SweepResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "design_id",
                "combo_index",
                "quality_score",
                "signal_quality_score",
                "nf_ip3_score",
                "final_bb_snr_db",
                "final_bb_evm_pct",
                "final_rf_snr_db",
                "final_rf_evm_pct",
                "effective_bb_gain_db",
                "estimated_bb_nf_db",
                "estimated_bb_iip3_dbm",
                "selected_uid_chain",
                "selected_part_chain",
                "stage_csv_path",
            ]
        )
        for r in rows:
            writer.writerow(
                [
                    r.design_id,
                    r.combo_index,
                    f"{r.quality_score:.6f}",
                    f"{r.signal_quality_score:.6f}",
                    f"{r.nf_ip3_score:.6f}",
                    f"{r.final_bb_snr_db:.6f}",
                    f"{r.final_bb_evm_pct:.6f}",
                    f"{r.final_rf_snr_db:.6f}",
                    f"{r.final_rf_evm_pct:.6f}",
                    f"{r.effective_bb_gain_db:.6f}",
                    f"{r.estimated_bb_nf_db:.6f}",
                    f"{r.estimated_bb_iip3_dbm:.6f}",
                    r.selected_uid_chain,
                    r.selected_part_chain,
                    r.stage_csv_path,
                ]
            )


def summarize_to_markdown(path: Path, best_overall: Sequence[SweepResult], best_per_design: Dict[int, SweepResult]) -> None:
    lines: List[str] = []
    lines.append("# Component Sweep Summary")
    lines.append("")
    lines.append("## Best Overall")
    lines.append("")
    lines.append("| Rank | Design | Score | BB SNR | BB EVM | RF SNR | RF EVM | Est. NF | Est. IIP3 |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for rank, res in enumerate(best_overall, start=1):
        lines.append(
            "| {} | {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |".format(
                rank,
                res.design_id,
                res.quality_score,
                res.final_bb_snr_db,
                res.final_bb_evm_pct,
                res.final_rf_snr_db,
                res.final_rf_evm_pct,
                res.estimated_bb_nf_db,
                res.estimated_bb_iip3_dbm,
            )
        )

    lines.append("")
    lines.append("## Best Per Design")
    lines.append("")
    lines.append("| Design | Score | NF/IP3 Score | BB SNR | BB EVM | RF SNR | RF EVM | Est. NF | Est. IIP3 | Selected Components |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |")
    for design_id in sorted(best_per_design):
        res = best_per_design[design_id]
        lines.append(
            "| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {} |".format(
                design_id,
                res.quality_score,
                res.nf_ip3_score,
                res.final_bb_snr_db,
                res.final_bb_evm_pct,
                res.final_rf_snr_db,
                res.final_rf_evm_pct,
                res.estimated_bb_nf_db,
                res.estimated_bb_iip3_dbm,
                res.selected_uid_chain,
            )
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_design_filter(raw: str, available: Iterable[int]) -> List[int]:
    if not raw:
        return sorted(set(available))
    out: List[int] = []
    available_set = set(available)
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        v = int(item)
        if v in available_set:
            out.append(v)
    return sorted(set(out))


def main() -> int:
    parser = argparse.ArgumentParser(description="Run exhaustive component combinations against receiver_dual_sim")
    parser.add_argument("--root", default=".", help="Project root path (receiver_dual_sim)")
    parser.add_argument("--catalog", default="data/component_catalog.csv", help="Component catalog CSV path")
    parser.add_argument("--designs-csv", default="stage_models/stage_models.csv", help="Topology designs CSV path")
    parser.add_argument("--out-dir", default="out/benchmark", help="Output folder for sweep reports")
    parser.add_argument("--symbols", type=int, default=64, help="Symbol count for exhaustive sweep")
    parser.add_argument("--full-symbols", type=int, default=256, help="Symbol count for rerun of top candidates")
    parser.add_argument("--snr", type=float, default=20.0, help="Input SNR in dB")
    parser.add_argument("--seed", type=int, default=20260415, help="RNG seed")
    parser.add_argument("--top-k", type=int, default=15, help="Top entries to keep in summary")
    parser.add_argument("--rerun-top", type=int, default=4, help="Rerun top N candidates at full-symbols")
    parser.add_argument("--max-combos-per-design", type=int, default=0, help="Limit combinations per design (0 = all)")
    parser.add_argument("--design-filter", default="", help="Comma-separated design IDs to run, e.g. 1,2,4")
    parser.add_argument("--no-infer-nf", action="store_true", help="Skip components that have missing NF")
    parser.add_argument("--no-infer-ip3", action="store_true", help="Skip components that have missing IP3/OIP3/P1dB")
    parser.add_argument("--objective", choices=["hybrid", "snr-evm", "nf-ip3"], default="hybrid", help="Ranking objective")
    parser.add_argument("--nf-weight", type=float, default=1.0, help="NF penalty weight for NF/IP3 score")
    parser.add_argument("--ip3-weight", type=float, default=0.1, help="IIP3 reward weight for NF/IP3 score")
    parser.add_argument("--jobs", type=int, default=0, help="Parallel workers (0 = auto, capped by stage slots)")
    parser.add_argument("--build", action="store_true", help="Run make before sweep")
    parser.add_argument("--quiet", action="store_true", help="Silence simulator command output")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    catalog_path = (root / args.catalog).resolve()
    designs_path = (root / args.designs_csv).resolve()
    out_dir = (root / args.out_dir).resolve()
    generated_dir = out_dir / "generated_stage_csv"
    best_dir = out_dir / "best_stage_csv"
    stage_slot_count = 4

    requested_jobs = args.jobs if args.jobs > 0 else (os.cpu_count() or 1)
    jobs = min(max(1, requested_jobs), stage_slot_count)

    if not catalog_path.exists():
        print(f"[ERROR] Missing catalog: {catalog_path}")
        return 2
    if not designs_path.exists():
        print(f"[ERROR] Missing designs CSV: {designs_path}")
        return 2

    bin_path = root / "bin" / "dual_receiver_sim"
    if args.build or not bin_path.exists():
        print("[INFO] Building simulator...")
        rc = run_cmd(["make", "clean"], cwd=root, quiet=False)
        if rc != 0:
            print("[ERROR] make clean failed")
            return 3

        rc = run_cmd(["make"], cwd=root, quiet=False)
        if rc != 0:
            print("[ERROR] Build failed")
            return 3

    if not bin_path.exists():
        print(f"[ERROR] Missing simulator binary: {bin_path}")
        return 3

    by_role = load_component_catalog(
        catalog_path,
        allow_inferred_nf=(not args.no_infer_nf),
        allow_inferred_ip3=(not args.no_infer_ip3),
    )
    designs = load_design_roles(designs_path)
    selected_designs = parse_design_filter(args.design_filter, designs.keys())

    if not selected_designs:
        print("[ERROR] No valid designs selected")
        return 4

    print(f"[INFO] Objective: {args.objective} (nf_weight={args.nf_weight}, ip3_weight={args.ip3_weight})")
    print(f"[INFO] Parallel workers: {jobs} (stage slots available: {stage_slot_count})")
    print("[INFO] Component options per role:")
    for role in sorted(by_role):
        inferred_count = sum(1 for c in by_role[role] if c.nf_inferred)
        inferred_ip3_count = sum(1 for c in by_role[role] if c.ip3_inferred)
        print(f"  - {role}: {len(by_role[role])} options ({inferred_count} inferred NF, {inferred_ip3_count} inferred IP3)")

    all_results: List[SweepResult] = []
    sweep_start = time.time()

    slot_queue: "queue.Queue[int]" = queue.Queue()
    for slot in range(1, stage_slot_count + 1):
        slot_queue.put(slot)

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as executor:
        for design_id in selected_designs:
            roles = designs.get(design_id, [])
            if not roles:
                print(f"[WARN] Design {design_id}: empty role list, skipping")
                continue

            active_role_indexes = [i for i, r in enumerate(roles) if r.role != "LIMITER"]
            missing_roles = [r.role for i, r in enumerate(roles) if i in active_role_indexes and r.role not in by_role]
            if missing_roles:
                print(f"[WARN] Design {design_id}: missing components for roles {sorted(set(missing_roles))}, skipping")
                continue

            option_lists: List[List[Component]] = [by_role[roles[i].role] for i in active_role_indexes]
            total_combos = 1
            for opts in option_lists:
                total_combos *= len(opts)
            if args.max_combos_per_design > 0:
                total_combos = min(total_combos, args.max_combos_per_design)

            print(f"[INFO] Design {design_id}: running up to {total_combos} combinations")

            combo_counter = 0
            fail_counter = 0
            design_start = time.time()
            futures: List[concurrent.futures.Future[Optional[SweepResult]]] = []

            cartesian_iter = itertools.product(*option_lists)
            for combo in cartesian_iter:
                if args.max_combos_per_design > 0 and combo_counter >= args.max_combos_per_design:
                    break
                combo_counter += 1

                selected_by_role_index: List[Optional[Component]] = [None] * len(roles)
                for role_idx, comp in zip(active_role_indexes, combo):
                    selected_by_role_index[role_idx] = comp

                stage_csv = generated_dir / f"design_{design_id:02d}_combo_{combo_counter:05d}.csv"
                write_stage_csv(stage_csv, roles, selected_by_role_index)

                futures.append(
                    executor.submit(
                        evaluate_combination,
                        root,
                        bin_path,
                        slot_queue,
                        stage_csv,
                        combo_counter,
                        design_id,
                        tuple(selected_by_role_index),
                        tuple(active_role_indexes),
                        args.seed,
                        args.symbols,
                        args.snr,
                        args.quiet,
                        args.objective,
                        args.nf_weight,
                        args.ip3_weight,
                    )
                )

            completed = 0
            for future in concurrent.futures.as_completed(futures):
                completed += 1
                result = future.result()
                if result is None:
                    fail_counter += 1
                else:
                    all_results.append(result)

                if completed % 50 == 0:
                    elapsed = time.time() - design_start
                    print(
                        f"[INFO] Design {design_id}: {completed}/{combo_counter} combos, "
                        f"ok={completed - fail_counter}, fail={fail_counter}, elapsed={elapsed:.1f}s"
                    )

            design_elapsed = time.time() - design_start
            print(
                f"[INFO] Design {design_id}: completed combos={combo_counter}, "
                f"ok={combo_counter - fail_counter}, fail={fail_counter}, elapsed={design_elapsed:.1f}s"
            )

    if not all_results:
        print("[ERROR] Sweep produced no valid results")
        return 5

    all_results_sorted = sorted(all_results, key=lambda r: r.quality_score, reverse=True)
    top_k = max(1, args.top_k)
    top_overall = all_results_sorted[:top_k]

    best_per_design: Dict[int, SweepResult] = {}
    for res in all_results_sorted:
        if res.design_id not in best_per_design:
            best_per_design[res.design_id] = res

    out_dir.mkdir(parents=True, exist_ok=True)
    write_results_csv(out_dir / "sweep_results_all.csv", all_results_sorted)
    write_results_csv(out_dir / "sweep_results_top.csv", top_overall)
    summarize_to_markdown(out_dir / "sweep_summary.md", top_overall[: min(10, len(top_overall))], best_per_design)

    best_dir.mkdir(parents=True, exist_ok=True)
    for rank, res in enumerate(top_overall[: max(1, args.rerun_top)], start=1):
        target_csv = best_dir / f"rank_{rank:02d}_design_{res.design_id}_combo_{res.combo_index:05d}.csv"
        shutil.copy2(res.stage_csv_path, target_csv)

        cmd = [
            str(bin_path),
            "--seed",
            str(args.seed),
            "--symbols",
            str(args.full_symbols),
            "--snr",
            str(args.snr),
            "--stage-csv",
            str(target_csv),
            "--topology-sim",
            str(res.design_id),
        ]
        rc = run_cmd(cmd, cwd=root, quiet=args.quiet)
        if rc != 0:
            print(f"[WARN] Full rerun failed for rank {rank}")
            continue

        csv_dir, svg_dir = topology_output_paths(root, res.design_id)
        artifact_dir = out_dir / f"full_rerun_rank_{rank:02d}_design_{res.design_id}"
        if artifact_dir.exists():
            shutil.rmtree(artifact_dir)
        (artifact_dir / "csv").mkdir(parents=True, exist_ok=True)
        (artifact_dir / "svg").mkdir(parents=True, exist_ok=True)

        for f in csv_dir.glob("*.csv"):
            shutil.copy2(f, artifact_dir / "csv" / f.name)
        for f in svg_dir.glob("*.svg"):
            shutil.copy2(f, artifact_dir / "svg" / f.name)

    elapsed_total = time.time() - sweep_start
    print(f"[DONE] Sweep completed: {len(all_results_sorted)} valid results in {elapsed_total:.1f}s")
    print(f"[DONE] Full results: {out_dir / 'sweep_results_all.csv'}")
    print(f"[DONE] Top results: {out_dir / 'sweep_results_top.csv'}")
    print(f"[DONE] Summary: {out_dir / 'sweep_summary.md'}")
    print(f"[DONE] Saved top stage CSVs: {best_dir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
