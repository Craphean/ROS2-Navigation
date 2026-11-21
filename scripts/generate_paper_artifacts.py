#!/usr/bin/env python3
"""Aggregate M5 multi-seed benchmark results and emit IEEE figures, LaTeX tables, and Markdown summary."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "hier_explore_bench"))

from hier_explore_bench.eval_metrics import aggregate_runs, compare_methods_significance
from hier_explore_bench.plot_paper_figs import (
    format_latex_table,
    plot_ablation_and_sensitivity,
    plot_alignment_study,
    plot_edge_pareto,
    plot_multiworld_coverage_grid,
    plot_scalability_curves,
)

ALL_METRICS = (
    "final_area_m2",
    "final_coverage_ratio",
    "coverage_2d_ratio",
    "completeness_10cm",
    "completeness_15cm",
    "precision_10cm",
    "fscore_10cm",
    "fscore_15cm",
    "chamfer_l1_m",
    "chamfer_l2_m",
    "occupied_iou_tol1",
    "free_iou",
    "ate_rmse_mean_m",
    "total_path_length_m",
    "coverage_efficiency_m2_per_m",
    "n_auc",
    "t_80_s",
    "load_balance_cv",
    "overlap_ratio",
    "cpu_mean_pct",
    "cpu_peak_pct",
    "ram_peak_mb",
    "cloud_merger_cpu_mean_pct",
    "map_merge_cpu_mean_pct",
    "coordinator_cpu_mean_pct",
    "cycle_ms_p50",
    "cycle_ms_p95",
    "se2_accept_count",
    "se2_degenerate_count",
    "comms_avg_kbps",
    "inter_robot_collisions",
    "min_inter_robot_dist_m",
    "duration_sim_s",
)

WORLDS_META: Sequence[Tuple[str, str, float]] = (
    ("house", "House (150 s)", 150.0),
    ("hospital", "Hospital (180 s)", 180.0),
    ("mine", "Mine (210 s)", 210.0),
    ("warehouse", "Warehouse (180 s)", 180.0),
    ("cmu_indoor", "CMU Indoor (180 s)", 180.0),
    ("cmu_tunnel", "CMU Tunnel (180 s)", 180.0),
)

METHOD_LABELS = {
    "legacy": "Legacy (E0)",
    "e1_2d": "2D-Frontier (E1)",
    "e2_3dgain": "+3D-Gain (E2)",
    "e3_hier": "+Hier (E3)",
    "e4_geodesic": "+Geodesic (E4)",
    "e5_deconflict": "+Deconflict (E5)",
    "full": "Lite-HMRE (Ours)",
    "mtare": "CMU M-TARE",
    "legacy_merge": "Legacy Merge + PyCloud",
    "no_se2": "No SE(2) Align",
    "no_degen_gate": "SE(2) w/o Degen. Gate",
    "gt_odom": "GT-Odom (Oracle)",
}


def find_run_dirs(base_dir: Path) -> List[Path]:
    """Locate all leaf run directories containing metrics.json under base_dir."""
    if not base_dir.is_dir():
        return []
    found = sorted(p.parent for p in base_dir.rglob("metrics.json"))
    # Deduplicate while preserving deterministic seed order
    seen = set()
    unique: List[Path] = []
    for d in found:
        if d not in seen:
            seen.add(d)
            unique.append(d)
    return unique


def load_runs(base_dir: Path) -> List[Dict[str, object]]:
    runs: List[Dict[str, object]] = []
    for rdir in find_run_dirs(base_dir):
        try:
            d = json.loads((rdir / "metrics.json").read_text(encoding="utf-8"))
            d["_run_dir"] = str(rdir)
            runs.append(d)
        except Exception:
            continue
    return runs


def load_timeseries_curves(base_dir: Path) -> List[Tuple[np.ndarray, np.ndarray]]:
    curves: List[Tuple[np.ndarray, np.ndarray]] = []
    for rdir in find_run_dirs(base_dir):
        ts_path = rdir / "timeseries.csv"
        if not ts_path.is_file():
            continue
        xs, ys = [], []
        try:
            with ts_path.open("r", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    xs.append(float(row["sim_time_s"]))
                    ys.append(float(row["explored_area_m2"]))
        except Exception:
            continue
        if len(xs) >= 2:
            curves.append((np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)))
    return curves


def sig_star(runs_a: Sequence[Dict[str, object]], runs_b: Sequence[Dict[str, object]], key: str) -> str:
    sig = compare_methods_significance(runs_a, runs_b, key)
    p = sig.get("welch_p", float("nan"))
    if not math.isfinite(p):
        return ""
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return ""


def fmt_ms(stat: Dict[str, float], fmt: str = ".2f", scale: float = 1.0) -> str:
    m = stat.get("mean", float("nan"))
    s = stat.get("std", float("nan"))
    if not (isinstance(m, (int, float)) and math.isfinite(m)):
        return "--"
    ms = float(m) * scale
    ss = float(s) * scale if isinstance(s, (int, float)) and math.isfinite(s) else 0.0
    return f"{format(ms, fmt)} ± {format(ss, fmt)}"


def generate_all(results_root: Path, out_dir: Path) -> Dict[str, object]:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: Dict[str, object] = {"results_root": str(results_root), "suites": {}}
    md_lines: List[str] = ["# M5 Benchmark Quantitative Summary (4-Core Edge Profile)", ""]

    # ------------------------------------------------------------------
    # Suite 1: Main Multi-World Comparison (Table I & Fig. 3)
    # ------------------------------------------------------------------
    main_methods = ("legacy", "e1_2d", "e3_hier", "mtare", "full")
    main_runs: Dict[str, Dict[str, List[Dict[str, object]]]] = {}
    main_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]] = {}
    worlds_plot_data = []

    table1_rows: List[Tuple[str, Dict[str, Dict[str, float]]]] = []
    table1_breaks: List[int] = []
    table1_sigs: Dict[Tuple[str, str], str] = {}

    md_lines.append("## Suite 1: Main Multi-World Benchmark (Table I / Fig. 3)")
    md_lines.append("")
    md_lines.append(
        "| World | Method | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | Occ IoU | CE (m²/m) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |"
    )
    md_lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")

    for w_idx, (wkey, wtitle, max_x) in enumerate(WORLDS_META):
        main_runs[wkey] = {}
        main_agg[wkey] = {}
        curves_by_m: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        if w_idx > 0:
            table1_breaks.append(len(table1_rows))

        for mkey in main_methods:
            src_dir = (
                results_root / "mtare" / wkey if mkey == "mtare" else results_root / "main" / wkey / mkey
            )
            runs = load_runs(src_dir)
            if not runs:
                continue
            main_runs[wkey][mkey] = runs
            agg = aggregate_runs(runs, ALL_METRICS)
            main_agg[wkey][mkey] = agg
            curves = load_timeseries_curves(src_dir)
            if curves:
                curves_by_m[mkey] = curves

            row_lbl = f"{wkey.replace('_', r'\_')} / {METHOD_LABELS.get(mkey, mkey)}"
            table1_rows.append((row_lbl, agg))
            if mkey == "full" and "e1_2d" in main_runs[wkey]:
                for mk in ("completeness_15cm", "chamfer_l1_m", "final_area_m2", "occupied_iou_tol1"):
                    st = sig_star(runs, main_runs[wkey]["e1_2d"], mk)
                    if st:
                        table1_sigs[(row_lbl, mk)] = st

            md_lines.append(
                f"| {wkey} | {METHOD_LABELS.get(mkey, mkey)} | {len(runs)} | "
                f"{fmt_ms(agg['final_area_m2'], '.1f')} | "
                f"{fmt_ms(agg['completeness_15cm'], '.1f', 100.0)} | "
                f"{fmt_ms(agg['chamfer_l1_m'], '.3f')} | "
                f"{fmt_ms(agg['occupied_iou_tol1'], '.3f')} | "
                f"{fmt_ms(agg['coverage_efficiency_m2_per_m'], '.2f')} | "
                f"{fmt_ms(agg['cpu_mean_pct'], '.1f')} | "
                f"{fmt_ms(agg['ram_peak_mb'], '.0f')} | "
                f"{fmt_ms(agg['cycle_ms_p95'], '.1f')} | "
                f"{fmt_ms(agg['inter_robot_collisions'], '.1f')} |"
            )

        worlds_plot_data.append((wkey, wtitle, max_x, curves_by_m))

    if any(curves for _, _, _, curves in worlds_plot_data):
        plot_multiworld_coverage_grid(worlds_plot_data, out_dir / "fig3_main_curves")
    if main_agg:
        plot_edge_pareto(main_agg, out_dir / "fig3b_edge_pareto")

    if table1_rows:
        t1_tex = format_latex_table(
            table1_rows,
            columns=[
                ("final_area_m2", r"Area ($\text{m}^2$)", ".1f"),
                ("completeness_15cm", r"3D Comp.$_{15}$ (\%)", ".1f"),
                ("chamfer_l1_m", r"CD-$L_1$ (m)", ".3f"),
                ("occupied_iou_tol1", r"Occ. IoU", ".3f"),
                ("coverage_efficiency_m2_per_m", r"CE ($\text{m}^2/\text{m}$)", ".2f"),
                ("cpu_mean_pct", r"CPU (\%)", ".1f"),
                ("cycle_ms_p95", r"P95 (ms)", ".1f"),
            ],
            caption=(
                "Main multi-world exploration benchmark under 4-core edge compute at matched 0.5 m/s max speed "
                "(Mean $\\pm$ Std across 5 seeds; $^*$ $p<0.05$, $^{**}$ $p<0.01$ vs. 2D-Frontier by Welch's $t$-test. "
                "CMU M-TARE runs its official ROS 1/2 AEDE stack capped to 0.5 m/s with GT pose; "
                "all ROS 2 methods run with noisy wheel+IMU EKF odometry)."
            ),
            label="tab:main_results",
            best_modes={
                "final_area_m2": "max",
                "completeness_15cm": "max",
                "chamfer_l1_m": "min",
                "occupied_iou_tol1": "max",
                "coverage_efficiency_m2_per_m": "max",
                "cpu_mean_pct": "min",
                "cycle_ms_p95": "min",
            },
            sig_markers=table1_sigs,
            table_env="table*",
            first_col_header="World / Method",
            scales={"completeness_15cm": 100.0},
            section_breaks=table1_breaks,
        )
        (out_dir / "table1_main_comparison.tex").write_text(t1_tex, encoding="utf-8")

    summary["suites"]["main"] = main_agg

    # ------------------------------------------------------------------
    # Suite 2: 7-Step Component Ablation (Table II)
    # ------------------------------------------------------------------
    ablation_keys = ("legacy", "e1_2d", "e2_3dgain", "e3_hier", "e4_geodesic", "e5_deconflict", "full")
    ablation_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]] = {}
    table2_rows: List[Tuple[str, Dict[str, Dict[str, float]]]] = []
    table2_breaks: List[int] = []

    md_lines.extend(["", "## Suite 2: 7-Step Component Ablation (Table II)", ""])
    md_lines.append(
        "| World | Config | N | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CE (m²/m) | Overlap | Cyc P95 (ms) | Coll |"
    )
    md_lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")

    for w_idx, wkey in enumerate(("mine", "hospital")):
        ablation_agg[wkey] = {}
        if w_idx > 0:
            table2_breaks.append(len(table2_rows))
        for mkey in ablation_keys:
            # E0, E1, E3, E6 come from main/; E2, E4, E5 come from ablation/
            if mkey in ("e2_3dgain", "e4_geodesic", "e5_deconflict"):
                runs = load_runs(results_root / "ablation" / wkey / mkey)
            else:
                runs = main_runs.get(wkey, {}).get(mkey) or load_runs(results_root / "main" / wkey / mkey)
            if not runs:
                continue
            agg = aggregate_runs(runs, ALL_METRICS)
            ablation_agg[wkey][mkey] = agg
            row_lbl = f"{wkey} / {METHOD_LABELS.get(mkey, mkey)}"
            table2_rows.append((row_lbl, agg))
            md_lines.append(
                f"| {wkey} | {METHOD_LABELS.get(mkey, mkey)} | {len(runs)} | "
                f"{fmt_ms(agg['final_area_m2'], '.1f')} | "
                f"{fmt_ms(agg['completeness_15cm'], '.1f', 100.0)} | "
                f"{fmt_ms(agg['chamfer_l1_m'], '.3f')} | "
                f"{fmt_ms(agg['coverage_efficiency_m2_per_m'], '.2f')} | "
                f"{fmt_ms(agg['overlap_ratio'], '.3f')} | "
                f"{fmt_ms(agg['cycle_ms_p95'], '.1f')} | "
                f"{fmt_ms(agg['inter_robot_collisions'], '.1f')} |"
            )

    if table2_rows:
        t2_tex = format_latex_table(
            table2_rows,
            columns=[
                ("final_area_m2", r"Area ($\text{m}^2$)", ".1f"),
                ("completeness_15cm", r"3D Comp.$_{15}$ (\%)", ".1f"),
                ("chamfer_l1_m", r"CD-$L_1$ (m)", ".3f"),
                ("coverage_efficiency_m2_per_m", r"CE ($\text{m}^2/\text{m}$)", ".2f"),
                ("overlap_ratio", r"Overlap", ".3f"),
                ("cycle_ms_p95", r"P95 (ms)", ".1f"),
                ("inter_robot_collisions", r"Coll.", ".1f"),
            ],
            caption="Seven-step incremental component ablation (E0--E6) on Mine and Hospital (5 seeds, 4-core).",
            label="tab:ablation",
            best_modes={
                "final_area_m2": "max",
                "completeness_15cm": "max",
                "chamfer_l1_m": "min",
                "coverage_efficiency_m2_per_m": "max",
                "overlap_ratio": "min",
                "cycle_ms_p95": "min",
                "inter_robot_collisions": "min",
            },
            table_env="table*",
            first_col_header="World / Config",
            scales={"completeness_15cm": 100.0},
            section_breaks=table2_breaks,
        )
        (out_dir / "table2_ablation.tex").write_text(t2_tex, encoding="utf-8")

    summary["suites"]["ablation"] = ablation_agg

    # ------------------------------------------------------------------
    # Suite 3: Noisy Odometry Alignment & Degeneracy Gate Study (Fig. 2 & Table III)
    # ------------------------------------------------------------------
    align_variants = ("legacy_merge", "no_se2", "no_degen_gate", "full", "gt_odom")
    align_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]] = {}
    table3_rows: List[Tuple[str, Dict[str, Dict[str, float]]]] = []
    table3_breaks: List[int] = []

    md_lines.extend(["", "## Suite 3: Noisy Odometry SE(2) Alignment & Degeneracy Study (Fig. 2)", ""])
    md_lines.append(
        "| World | Variant | N | Occ IoU | ATE RMSE (m) | Chamfer-L1 (m) | Prec@10cm (%) | Cloud CPU (%) | SE(2) Accepts | Degen Gates |"
    )
    md_lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")

    for w_idx, wkey in enumerate(("mine", "warehouse")):
        align_agg[wkey] = {}
        if w_idx > 0:
            table3_breaks.append(len(table3_rows))
        for vkey in align_variants:
            if vkey == "full":
                runs = main_runs.get(wkey, {}).get("full") or load_runs(results_root / "main" / wkey / "full")
            else:
                runs = load_runs(results_root / "alignment" / wkey / vkey)
            if not runs:
                continue
            agg = aggregate_runs(runs, ALL_METRICS)
            align_agg[wkey][vkey] = agg
            table3_rows.append((f"{wkey} / {METHOD_LABELS.get(vkey, vkey)}", agg))
            md_lines.append(
                f"| {wkey} | {METHOD_LABELS.get(vkey, vkey)} | {len(runs)} | "
                f"{fmt_ms(agg['occupied_iou_tol1'], '.3f')} | "
                f"{fmt_ms(agg['ate_rmse_mean_m'], '.3f')} | "
                f"{fmt_ms(agg['chamfer_l1_m'], '.3f')} | "
                f"{fmt_ms(agg['precision_10cm'], '.1f', 100.0)} | "
                f"{fmt_ms(agg['cloud_merger_cpu_mean_pct'], '.1f')} | "
                f"{fmt_ms(agg['se2_accept_count'], '.1f')} | "
                f"{fmt_ms(agg['se2_degenerate_count'], '.1f')} |"
            )

    if any(align_agg.get(w) for w in ("mine", "warehouse")):
        plot_alignment_study(align_agg, out_dir / "fig2_alignment")
    if table3_rows:
        t3_tex = format_latex_table(
            table3_rows,
            columns=[
                ("occupied_iou_tol1", r"Occ. IoU", ".3f"),
                ("ate_rmse_mean_m", r"ATE (m)", ".3f"),
                ("chamfer_l1_m", r"CD-$L_1$ (m)", ".3f"),
                ("precision_10cm", r"Prec.$_{10}$ (\%)", ".1f"),
                ("cloud_merger_cpu_mean_pct", r"Cloud CPU (\%)", ".1f"),
                ("se2_accept_count", r"SE(2) Acc.", ".1f"),
            ],
            caption="Online SE(2) map alignment and degeneracy gate study under noisy EKF odometry (5 seeds).",
            label="tab:alignment",
            best_modes={
                "occupied_iou_tol1": "max",
                "ate_rmse_mean_m": "min",
                "chamfer_l1_m": "min",
                "precision_10cm": "max",
                "cloud_merger_cpu_mean_pct": "min",
            },
            table_env="table*",
            first_col_header="World / Variant",
            scales={"precision_10cm": 100.0},
            section_breaks=table3_breaks,
        )
        (out_dir / "table3_alignment.tex").write_text(t3_tex, encoding="utf-8")

    summary["suites"]["alignment"] = align_agg

    # ------------------------------------------------------------------
    # Suite 4: Multi-Robot Scalability N=1..4 (Fig. 4 & Table IV)
    # ------------------------------------------------------------------
    scale_agg: Dict[str, Dict[int, Dict[str, Dict[str, float]]]] = {}
    table4_rows: List[Tuple[str, Dict[str, Dict[str, float]]]] = []
    table4_breaks: List[int] = []

    md_lines.extend(["", "## Suite 4: Multi-Robot Scalability N = 1..4 (Fig. 4)", ""])
    md_lines.append(
        "| World | N Robots | Runs | Area (m²) | 3D Comp@15cm (%) | Load CV | Overlap | Comms (KB/s) | CPU (%) | RAM (MB) | Cyc P95 (ms) | Coll |"
    )
    md_lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")

    for w_idx, wkey in enumerate(("mine", "cmu_indoor")):
        scale_agg[wkey] = {}
        if w_idx > 0:
            table4_breaks.append(len(table4_rows))
        for nrob in (1, 2, 3, 4):
            if nrob == 2:
                runs = main_runs.get(wkey, {}).get("full") or load_runs(results_root / "main" / wkey / "full")
            else:
                runs = load_runs(results_root / "scalability" / wkey / f"n{nrob}")
            if not runs:
                continue
            agg = aggregate_runs(runs, ALL_METRICS)
            scale_agg[wkey][nrob] = agg
            table4_rows.append((f"{wkey.replace('_', r'\_')} / $N={nrob}$", agg))
            md_lines.append(
                f"| {wkey} | {nrob} | {len(runs)} | "
                f"{fmt_ms(agg['final_area_m2'], '.1f')} | "
                f"{fmt_ms(agg['completeness_15cm'], '.1f', 100.0)} | "
                f"{fmt_ms(agg['load_balance_cv'], '.3f')} | "
                f"{fmt_ms(agg['overlap_ratio'], '.3f')} | "
                f"{fmt_ms(agg['comms_avg_kbps'], '.1f')} | "
                f"{fmt_ms(agg['cpu_mean_pct'], '.1f')} | "
                f"{fmt_ms(agg['ram_peak_mb'], '.0f')} | "
                f"{fmt_ms(agg['cycle_ms_p95'], '.1f')} | "
                f"{fmt_ms(agg['inter_robot_collisions'], '.1f')} |"
            )

    if any(scale_agg.get(w) for w in ("mine", "cmu_indoor")):
        plot_scalability_curves(scale_agg, out_dir / "fig4_scalability")
    if table4_rows:
        t4_tex = format_latex_table(
            table4_rows,
            columns=[
                ("final_area_m2", r"Area ($\text{m}^2$)", ".1f"),
                ("completeness_15cm", r"3D Comp.$_{15}$ (\%)", ".1f"),
                ("load_balance_cv", r"Load CV", ".3f"),
                ("comms_avg_kbps", r"Comms (KB/s)", ".1f"),
                ("cpu_mean_pct", r"CPU (\%)", ".1f"),
                ("ram_peak_mb", r"RAM (MB)", ".0f"),
                ("cycle_ms_p95", r"P95 (ms)", ".1f"),
            ],
            caption="Multi-robot scalability ($N=1\\dots 4$) on Mine and CMU Indoor under 4-core edge compute.",
            label="tab:scalability",
            best_modes={"final_area_m2": "max", "completeness_15cm": "max", "cycle_ms_p95": "min"},
            table_env="table*",
            first_col_header="World / Team Size",
            scales={"completeness_15cm": 100.0},
            section_breaks=table4_breaks,
        )
        (out_dir / "table4_scalability.tex").write_text(t4_tex, encoding="utf-8")

    summary["suites"]["scalability"] = {w: {str(k): v for k, v in d.items()} for w, d in scale_agg.items()}

    # ------------------------------------------------------------------
    # Suite 5: Sensor & Voxel Resolution Sensitivity (Table V & Fig. 5b)
    # ------------------------------------------------------------------
    sens_cfgs = (
        ("16beam_005", "16-beam / 0.05 m"),
        ("16beam_010", "16-beam / 0.10 m (Default)"),
        ("16beam_015", "16-beam / 0.15 m"),
        ("32beam_010", "32-beam / 0.10 m"),
    )
    sens_agg: Dict[str, Dict[str, Dict[str, float]]] = {}
    table5_rows: List[Tuple[str, Dict[str, Dict[str, float]]]] = []

    md_lines.extend(["", "## Suite 5: Sensor & Voxel Resolution Sensitivity on Mine (Table V)", ""])
    md_lines.append(
        "| Config | Runs | Area (m²) | 3D Comp@15cm (%) | Chamfer-L1 (m) | CPU (%) | RAM (MB) | Cyc P95 (ms) |"
    )
    md_lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")

    for skey, slabel in sens_cfgs:
        if skey == "16beam_010":
            runs = (main_runs.get("mine", {}).get("full") or load_runs(results_root / "main" / "mine" / "full"))[:3]
        else:
            runs = load_runs(results_root / "sensitivity" / "mine" / skey)
        if not runs:
            continue
        agg = aggregate_runs(runs, ALL_METRICS)
        sens_agg[skey] = agg
        table5_rows.append((slabel, agg))
        md_lines.append(
            f"| {slabel} | {len(runs)} | "
            f"{fmt_ms(agg['final_area_m2'], '.1f')} | "
            f"{fmt_ms(agg['completeness_15cm'], '.1f', 100.0)} | "
            f"{fmt_ms(agg['chamfer_l1_m'], '.3f')} | "
            f"{fmt_ms(agg['cpu_mean_pct'], '.1f')} | "
            f"{fmt_ms(agg['ram_peak_mb'], '.0f')} | "
            f"{fmt_ms(agg['cycle_ms_p95'], '.1f')} |"
        )

    if any(ablation_agg.get(w) for w in ("mine", "hospital")) or sens_agg:
        plot_ablation_and_sensitivity(ablation_agg, sens_agg, out_dir / "fig5_ablation_sensitivity")
    if table5_rows:
        t5_tex = format_latex_table(
            table5_rows,
            columns=[
                ("final_area_m2", r"Area ($\text{m}^2$)", ".1f"),
                ("completeness_15cm", r"3D Comp.$_{15}$ (\%)", ".1f"),
                ("chamfer_l1_m", r"CD-$L_1$ (m)", ".3f"),
                ("cpu_mean_pct", r"CPU (\%)", ".1f"),
                ("ram_peak_mb", r"RAM (MB)", ".0f"),
                ("cycle_ms_p95", r"P95 (ms)", ".1f"),
            ],
            caption="LiDAR beam count and OctoMap voxel resolution sensitivity on Mine (3 seeds, 4-core).",
            label="tab:sensitivity",
            best_modes={
                "completeness_15cm": "max",
                "chamfer_l1_m": "min",
                "cpu_mean_pct": "min",
                "ram_peak_mb": "min",
                "cycle_ms_p95": "min",
            },
            table_env="table",
            first_col_header="Configuration",
            scales={"completeness_15cm": 100.0},
        )
        (out_dir / "table5_sensitivity.tex").write_text(t5_tex, encoding="utf-8")

    summary["suites"]["sensitivity"] = sens_agg

    # ------------------------------------------------------------------
    # Statistical Significance Highlights (Welch t-test & Mann-Whitney U)
    # ------------------------------------------------------------------
    sig_report: Dict[str, Dict[str, Dict[str, float]]] = {}
    md_lines.extend(["", "## Statistical Significance (Lite-HMRE vs. Baselines)", ""])
    md_lines.append("| World | Comparison | Metric | Delta Mean | Welch p | Mann-Whitney U p |")
    md_lines.append("|---|---|---|---:|---:|---:|")
    for wkey, _, _ in WORLDS_META:
        full_r = main_runs.get(wkey, {}).get("full", [])
        if len(full_r) < 2:
            continue
        sig_report[wkey] = {}
        for base in ("legacy", "e1_2d", "e3_hier", "mtare"):
            base_r = main_runs.get(wkey, {}).get(base, [])
            if len(base_r) < 2:
                continue
            for mk in ("final_area_m2", "completeness_15cm", "chamfer_l1_m", "occupied_iou_tol1"):
                st = compare_methods_significance(full_r, base_r, mk)
                sig_report[wkey][f"{base}:{mk}"] = st
                wp = st.get("welch_p", float("nan"))
                mp = st.get("mwu_p", float("nan"))
                dm = st.get("delta_mean", float("nan"))
                if math.isfinite(wp):
                    md_lines.append(
                        f"| {wkey} | full vs {base} | {mk} | {dm:+.4f} | {wp:.4g} | {mp:.4g} |"
                    )
    summary["significance"] = sig_report

    (out_dir / "summary_report.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out_dir / "summary_report.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate IEEE paper figures and LaTeX tables from M5 results.")
    ap.add_argument(
        "--results-root",
        default=str(REPO_ROOT / "benchmark_results" / "m5_paper"),
        help="Root directory of M5 benchmark runs.",
    )
    ap.add_argument(
        "--out-dir",
        default=str(REPO_ROOT / "docs" / "paper_artifacts"),
        help="Output directory for PDF/PNG figures, LaTeX tables, and summary reports.",
    )
    args = ap.parse_args()
    generate_all(Path(args.results_root), Path(args.out_dir))
    print(f"[generate_paper_artifacts] Wrote artifacts to {args.out_dir}")


if __name__ == "__main__":
    main()
