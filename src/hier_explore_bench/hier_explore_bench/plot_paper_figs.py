#!/usr/bin/env python3
"""IEEE double/single-column publication figure & LaTeX table generator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Okabe-Ito colorblind-safe IEEE palette
PALETTE = [
    "#0072B2",  # blue (Ours)
    "#D55E00",  # vermilion (M-TARE)
    "#009E73",  # bluish green
    "#E69F00",  # amber
    "#CC79A7",  # reddish purple
    "#56B4E9",  # sky blue
    "#7F7F7F",  # neutral gray
]

# Consistent method styling across all paper figures
METHOD_STYLES: Dict[str, Dict[str, object]] = {
    "full": {"label": "Lite-HMRE (Ours)", "color": "#0072B2", "ls": "-", "marker": "o", "lw": 1.8, "zorder": 5},
    "mtare": {"label": "CMU M-TARE* (2.0 m/s)", "color": "#D55E00", "ls": "--", "marker": "s", "lw": 1.5, "zorder": 4},
    "e3_hier": {"label": "Hier-Greedy (E3)", "color": "#009E73", "ls": "-.", "marker": "^", "lw": 1.4, "zorder": 3},
    "e1_2d": {"label": "2D-Frontier (E1)", "color": "#E69F00", "ls": ":", "marker": "D", "lw": 1.4, "zorder": 2},
    "legacy": {"label": "Legacy explore_lite (E0)", "color": "#CC79A7", "ls": (0, (3, 1, 1, 1)), "marker": "v", "lw": 1.3, "zorder": 1},
    "e2_3dgain": {"label": "2D+3D Gain (E2)", "color": "#56B4E9", "ls": "--", "marker": "P", "lw": 1.3, "zorder": 2},
    "e4_geodesic": {"label": "+Geodesic (E4)", "color": "#009E73", "ls": "-.", "marker": "^", "lw": 1.4, "zorder": 3},
    "e5_deconflict": {"label": "+Deconflict (E5)", "color": "#332288", "ls": "-", "marker": "X", "lw": 1.5, "zorder": 4},
    "legacy_merge": {"label": "Legacy Merge + PyCloud", "color": "#CC79A7", "ls": ":", "marker": "v", "lw": 1.3, "zorder": 1},
    "no_se2": {"label": "No SE(2) Align", "color": "#E69F00", "ls": "--", "marker": "D", "lw": 1.3, "zorder": 2},
    "no_degen_gate": {"label": "SE(2) w/o Degen. Gate", "color": "#D55E00", "ls": "-.", "marker": "s", "lw": 1.4, "zorder": 3},
    "gt_odom": {"label": "GT-Odom (Oracle)", "color": "#009E73", "ls": "--", "marker": "*", "lw": 1.4, "zorder": 4},
}


def get_method_style(key: str, idx: int = 0) -> Dict[str, object]:
    if key in METHOD_STYLES:
        return METHOD_STYLES[key]
    return {
        "label": key,
        "color": PALETTE[idx % len(PALETTE)],
        "ls": "-",
        "marker": "o",
        "lw": 1.4,
        "zorder": 2,
    }


def configure_ieee_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["DejaVu Sans", "Liberation Sans"],
            "font.size": 8,
            "axes.titlesize": 8.5,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "axes.linewidth": 0.6,
            "lines.linewidth": 1.4,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def _save_fig(fig: plt.Figure, out_stem: Path) -> Path:
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    pdf_path = out_stem.with_suffix(".pdf")
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(out_stem.with_suffix(".png"), dpi=300, bbox_inches="tight")
    plt.close(fig)
    return pdf_path


def _interp_mean_std(
    seed_runs: Sequence[Tuple[np.ndarray, np.ndarray]], grid_x: np.ndarray
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    interp_ys = []
    for xs, ys in seed_runs:
        if len(xs) == 0:
            continue
        order = np.argsort(xs)
        xs_s = np.asarray(xs, dtype=np.float64)[order]
        ys_s = np.asarray(ys, dtype=np.float64)[order]
        interp_ys.append(np.interp(grid_x, xs_s, ys_s, left=ys_s[0], right=ys_s[-1]))
    if not interp_ys:
        return None
    mat = np.vstack(interp_ys)
    mean_y = np.mean(mat, axis=0)
    std_y = np.std(mat, axis=0, ddof=1) if mat.shape[0] > 1 else np.zeros_like(mean_y)
    return mean_y, std_y


def plot_coverage_curves(
    curves_by_method: Dict[str, List[Tuple[np.ndarray, np.ndarray]]],
    out_stem: Path,
    xlabel: str = "Simulation Time (s)",
    ylabel: str = "2D Coverage (%)",
    title: str = "",
    max_x: float = 240.0,
) -> Path:
    """Plot Mean +/- Std coverage curves across seeds on a unified time grid."""
    configure_ieee_style()
    grid_x = np.linspace(0.0, max_x, 200)
    fig, ax = plt.subplots(figsize=(3.5, 2.35), dpi=300)

    for idx, (method, seed_runs) in enumerate(curves_by_method.items()):
        if not seed_runs:
            continue
        res = _interp_mean_std(seed_runs, grid_x)
        if res is None:
            continue
        mean_y, std_y = res
        st = get_method_style(method, idx)
        color = str(st["color"])
        ax.plot(
            grid_x,
            mean_y,
            label=str(st["label"]),
            color=color,
            linestyle=st["ls"],
            linewidth=float(st["lw"]),
            zorder=int(st["zorder"]),
        )
        ax.fill_between(grid_x, mean_y - std_y, mean_y + std_y, color=color, alpha=0.16, linewidth=0)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.5)
    ax.legend(frameon=True, loc="lower right")
    fig.tight_layout()
    return _save_fig(fig, out_stem)


def plot_multiworld_coverage_grid(
    worlds_data: Sequence[Tuple[str, str, float, Dict[str, List[Tuple[np.ndarray, np.ndarray]]]]],
    out_stem: Path,
    ylabel: str = "Explored Area (m²)",
    xlabel: str = "Simulation Time (s)",
    method_order: Sequence[str] = ("full", "mtare", "e3_hier", "e1_2d", "legacy"),
) -> Path:
    """Plot a 2x3 IEEE double-column coverage grid across 6 worlds with a shared top legend.

    ``worlds_data`` entries are ``(world_key, panel_title, max_x, curves_by_method)``.
    """
    configure_ieee_style()
    n_worlds = len(worlds_data)
    ncols = 3 if n_worlds > 4 else 2
    nrows = int(math.ceil(n_worlds / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(7.16, 2.05 * nrows + 0.35), dpi=300)
    axes_flat = np.atleast_1d(axes).ravel()

    legend_handles: Dict[str, object] = {}
    for ax_idx, (wkey, wtitle, max_x, curves_by_method) in enumerate(worlds_data):
        ax = axes_flat[ax_idx]
        grid_x = np.linspace(0.0, max_x, 180)
        ordered_keys = [m for m in method_order if m in curves_by_method] + [
            m for m in curves_by_method if m not in method_order
        ]
        for idx, mkey in enumerate(ordered_keys):
            seed_runs = curves_by_method.get(mkey, [])
            if not seed_runs:
                continue
            res = _interp_mean_std(seed_runs, grid_x)
            if res is None:
                continue
            mean_y, std_y = res
            st = get_method_style(mkey, idx)
            color = str(st["color"])
            (line,) = ax.plot(
                grid_x,
                mean_y,
                label=str(st["label"]),
                color=color,
                linestyle=st["ls"],
                linewidth=float(st["lw"]),
                zorder=int(st["zorder"]),
            )
            ax.fill_between(
                grid_x,
                np.maximum(0.0, mean_y - std_y),
                mean_y + std_y,
                color=color,
                alpha=0.14,
                linewidth=0,
            )
            if mkey not in legend_handles:
                legend_handles[mkey] = (line, str(st["label"]))

        ax.set_title(wtitle, fontweight="bold", pad=4)
        ax.set_xlim(0.0, max_x)
        ax.set_ylim(bottom=0.0)
        ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45)
        if ax_idx >= (nrows - 1) * ncols:
            ax.set_xlabel(xlabel)
        if ax_idx % ncols == 0:
            ax.set_ylabel(ylabel)

    for ax_idx in range(n_worlds, len(axes_flat)):
        axes_flat[ax_idx].axis("off")

    ordered_leg = [m for m in method_order if m in legend_handles] + [
        m for m in legend_handles if m not in method_order
    ]
    handles = [legend_handles[m][0] for m in ordered_leg]
    labels = [legend_handles[m][1] for m in ordered_leg]
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=min(len(labels), 5),
        frameon=True,
        framealpha=0.95,
        columnspacing=1.0,
        handlelength=2.0,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    return _save_fig(fig, out_stem)


def plot_alignment_study(
    world_aggregates: Dict[str, Dict[str, Dict[str, Dict[str, float]]]],
    out_stem: Path,
    worlds: Sequence[Tuple[str, str]] = (("mine", "Mine"), ("warehouse", "Warehouse")),
    variants: Sequence[str] = ("legacy_merge", "no_se2", "no_degen_gate", "full", "gt_odom"),
) -> Path:
    """Plot Fig. 2: Noisy Odometry Online SE(2) Alignment & Degeneracy Gate Study (1x4 panels)."""
    configure_ieee_style()
    metrics = [
        ("occupied_iou_tol1", "2D Map Occ. IoU ↑", 1.0),
        ("ate_rmse_mean_m", "ATE RMSE (m) ↓", 1.0),
        ("chamfer_l1_m", "3D Chamfer-L1 (m) ↓", 1.0),
        ("cloud_merger_cpu_mean_pct", "Cloud Merger CPU (%) ↓", 1.0),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(7.16, 2.35), dpi=300)
    x = np.arange(len(worlds))
    n_var = len(variants)
    width = 0.78 / max(1, n_var)

    for ax, (mkey, ylabel, scale) in zip(axes, metrics):
        for v_idx, vkey in enumerate(variants):
            st = get_method_style(vkey, v_idx)
            means = []
            stds = []
            for wkey, _ in worlds:
                stat = world_aggregates.get(wkey, {}).get(vkey, {}).get(mkey, {})
                m = float(stat.get("mean", float("nan"))) * scale
                s = float(stat.get("std", 0.0)) * scale
                means.append(0.0 if not math.isfinite(m) else m)
                stds.append(0.0 if not math.isfinite(s) else s)
            offset = (v_idx - (n_var - 1) / 2.0) * width
            hatch = "//" if vkey == "gt_odom" else None
            ax.bar(
                x + offset,
                means,
                width=width * 0.92,
                yerr=stds,
                capsize=2.0,
                error_kw={"elinewidth": 0.7, "capthick": 0.7},
                color=str(st["color"]),
                edgecolor="#222222",
                linewidth=0.4,
                hatch=hatch,
                label=str(st["label"]),
                zorder=3,
            )
        ax.set_xticks(x)
        ax.set_xticklabels([wlabel for _, wlabel in worlds])
        ax.set_ylabel(ylabel)
        ax.set_ylim(bottom=0.0)
        ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.06),
        ncol=min(len(labels), 5),
        frameon=True,
        columnspacing=0.9,
        handletextpad=0.4,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.90))
    return _save_fig(fig, out_stem)


def plot_scalability_curves(
    scalability_agg: Dict[str, Dict[int, Dict[str, Dict[str, float]]]],
    out_stem: Path,
    worlds: Sequence[Tuple[str, str]] = (("mine", "Mine"), ("cmu_indoor", "CMU Indoor")),
    robot_counts: Sequence[int] = (1, 2, 3, 4),
) -> Path:
    """Plot Fig. 4: Multi-robot scalability N=1..4 (Area/Speedup, Comms/CPU, Latency/Overlap)."""
    configure_ieee_style()
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.45), dpi=300)
    w_colors = {"mine": "#0072B2", "cmu_indoor": "#D55E00"}
    w_markers = {"mine": "o", "cmu_indoor": "s"}
    ns = np.asarray(robot_counts, dtype=np.int64)

    # Panel (a): Normalized Exploration Speedup (Area(N) / Area(1))
    ax = axes[0]
    for wkey, wlabel in worlds:
        wdata = scalability_agg.get(wkey, {})
        base_area = float(wdata.get(1, {}).get("final_area_m2", {}).get("mean", float("nan")))
        if not math.isfinite(base_area) or base_area <= 1e-6:
            continue
        ys, es = [], []
        for n in robot_counts:
            st = wdata.get(n, {}).get("final_area_m2", {})
            m = float(st.get("mean", float("nan")))
            s = float(st.get("std", 0.0))
            ys.append(m / base_area if math.isfinite(m) else float("nan"))
            es.append(s / base_area if math.isfinite(s) else 0.0)
        ax.errorbar(
            ns,
            ys,
            yerr=es,
            color=w_colors.get(wkey, "#0072B2"),
            marker=w_markers.get(wkey, "o"),
            capsize=2.5,
            label=f"{wlabel}",
        )
    ax.plot(ns, ns, linestyle=":", color="#888888", linewidth=0.9, label="Ideal linear")
    ax.set_xticks(ns)
    ax.set_xlabel("Number of Robots ($N$)")
    ax.set_ylabel("Area Speedup ($A_N / A_1$)")
    ax.set_ylim(0.0, 4.35)
    ax.set_title("(a) Exploration Speedup", fontweight="bold")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45)
    ax.legend(frameon=True, loc="upper left", fontsize=6.2)

    # Panel (b): Total Comms Bandwidth (KB/s) & Peak RAM (GB)
    ax = axes[1]
    ax2 = ax.twinx()
    ax2.spines["top"].set_visible(False)
    max_comms, max_ram = 1.0, 1.0
    for wkey, wlabel in worlds:
        wdata = scalability_agg.get(wkey, {})
        c_m = [float(wdata.get(n, {}).get("comms_avg_kbps", {}).get("mean", float("nan"))) for n in robot_counts]
        c_s = [float(wdata.get(n, {}).get("comms_avg_kbps", {}).get("std", 0.0)) for n in robot_counts]
        r_m = [float(wdata.get(n, {}).get("ram_peak_mb", {}).get("mean", float("nan"))) / 1024.0 for n in robot_counts]
        for m, s in zip(c_m, c_s):
            if math.isfinite(m):
                max_comms = max(max_comms, m + (s if math.isfinite(s) else 0.0))
        for r in r_m:
            if math.isfinite(r):
                max_ram = max(max_ram, r)
        col = w_colors.get(wkey, "#0072B2")
        mk = w_markers.get(wkey, "o")
        ax.errorbar(ns, c_m, yerr=c_s, color=col, marker=mk, capsize=2.2, label=f"{wlabel} Comms")
        ax2.plot(ns, r_m, color=col, marker=mk, linestyle="--", alpha=0.80, linewidth=1.1, label=f"{wlabel} RAM")
    ax.set_xticks(ns)
    ax.set_xlabel("Number of Robots ($N$)")
    ax.set_ylabel("Comms Bandwidth (KB/s)")
    ax2.set_ylabel("Peak RAM (GB)", color="#444444")
    ax.set_ylim(0.0, max_comms * 1.38)
    ax2.set_ylim(0.0, max_ram * 1.38)
    ax.set_title("(b) Bandwidth & Memory", fontweight="bold")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, frameon=True, loc="upper left", fontsize=5.6, ncol=1, framealpha=0.92)

    # Panel (c): Planning P95 Cycle Time (ms) & Load Balance CV
    ax = axes[2]
    ax3 = ax.twinx()
    ax3.spines["top"].set_visible(False)
    max_p95, max_cv = 60.0, 0.25
    for wkey, wlabel in worlds:
        wdata = scalability_agg.get(wkey, {})
        p_m = [float(wdata.get(n, {}).get("cycle_ms_p95", {}).get("mean", float("nan"))) for n in robot_counts]
        p_s = [float(wdata.get(n, {}).get("cycle_ms_p95", {}).get("std", 0.0)) for n in robot_counts]
        cv_m = [float(wdata.get(n, {}).get("load_balance_cv", {}).get("mean", float("nan"))) for n in robot_counts]
        for m, s in zip(p_m, p_s):
            if math.isfinite(m):
                max_p95 = max(max_p95, m + (s if math.isfinite(s) else 0.0))
        for c in cv_m:
            if math.isfinite(c):
                max_cv = max(max_cv, c)
        col = w_colors.get(wkey, "#0072B2")
        mk = w_markers.get(wkey, "o")
        ax.errorbar(ns, p_m, yerr=p_s, color=col, marker=mk, capsize=2.2, label=f"{wlabel} P95")
        ax3.plot(ns, cv_m, color=col, marker=mk, linestyle="--", alpha=0.80, linewidth=1.1, label=f"{wlabel} CV")
    ax.axhline(50.0, color="#666666", linestyle=":", linewidth=0.9, label="50 ms ref.")
    ax.set_xticks(ns)
    ax.set_xlabel("Number of Robots ($N$)")
    ax.set_ylabel("Planning P95 (ms)")
    ax3.set_ylabel("Load Balance CV", color="#444444")
    ax.set_ylim(0.0, max_p95 * 1.42)
    ax3.set_ylim(0.0, max_cv * 1.48)
    ax.set_title("(c) Latency & Load Balance", fontweight="bold")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45)
    h1, l1 = ax.get_legend_handles_labels()
    h3, l3 = ax3.get_legend_handles_labels()
    ax.legend(h1 + h3, l1 + l3, frameon=True, loc="upper left", fontsize=5.4, ncol=1, framealpha=0.92)

    fig.tight_layout(w_pad=1.6)
    return _save_fig(fig, out_stem)


def plot_ablation_and_sensitivity(
    ablation_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]],
    sensitivity_agg: Dict[str, Dict[str, Dict[str, float]]],
    out_stem: Path,
    ablation_order: Sequence[Tuple[str, str]] = (
        ("legacy", "E0"),
        ("e1_2d", "E1"),
        ("e2_3dgain", "E2"),
        ("e3_hier", "E3"),
        ("e4_geodesic", "E4"),
        ("e5_deconflict", "E5"),
        ("full", "E6"),
    ),
    sensitivity_order: Sequence[Tuple[str, str]] = (
        ("16beam_005", "16B / 0.05m"),
        ("16beam_010", "16B / 0.10m"),
        ("16beam_015", "16B / 0.15m"),
        ("32beam_010", "32B / 0.10m"),
    ),
) -> Path:
    """Plot Fig. 5: 7-Step Component Ablation (E0-E6) & Sensor/Voxel Sensitivity."""
    configure_ieee_style()
    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.55), dpi=300)

    # Panel (a): 7-step ablation on mine & hospital (3D Completeness @ 15 cm)
    ax = axes[0]
    xs = np.arange(len(ablation_order))
    w = 0.36
    for idx, (wkey, wlabel, col) in enumerate((("mine", "Mine", "#0072B2"), ("hospital", "Hospital", "#D55E00"))):
        means, stds = [], []
        for mkey, _ in ablation_order:
            st = ablation_agg.get(wkey, {}).get(mkey, {}).get("completeness_15cm", {})
            m = float(st.get("mean", float("nan"))) * 100.0
            s = float(st.get("std", 0.0)) * 100.0
            means.append(0.0 if not math.isfinite(m) else m)
            stds.append(0.0 if not math.isfinite(s) else s)
        ax.bar(
            xs + (idx - 0.5) * w,
            means,
            width=w * 0.92,
            yerr=stds,
            capsize=2.0,
            error_kw={"elinewidth": 0.7},
            color=col,
            edgecolor="#222222",
            linewidth=0.4,
            label=wlabel,
            zorder=3,
        )
    ax.set_xticks(xs)
    ax.set_xticklabels([short for _, short in ablation_order])
    ax.set_xlabel("Incremental Ablation Configuration")
    ax.set_ylabel("3D Completeness @ 15 cm (%)")
    ax.set_ylim(0.0, 108.0)
    ax.set_title("(a) 7-Step Ablation Progression (E0–E6)", fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend(frameon=True, loc="upper center", ncol=2, fontsize=6.8)

    # Panel (b): Sensor & voxel resolution sensitivity on Mine
    ax = axes[1]
    ax2 = ax.twinx()
    ax2.spines["top"].set_visible(False)
    sxs = np.arange(len(sensitivity_order))
    comp_m, comp_s, cpu_m, cpu_s = [], [], [], []
    for skey, _ in sensitivity_order:
        cst = sensitivity_agg.get(skey, {}).get("completeness_15cm", {})
        pst = sensitivity_agg.get(skey, {}).get("cpu_mean_pct", {})
        cm = float(cst.get("mean", float("nan"))) * 100.0
        cs = float(cst.get("std", 0.0)) * 100.0
        pm = float(pst.get("mean", float("nan")))
        ps = float(pst.get("std", 0.0))
        comp_m.append(0.0 if not math.isfinite(cm) else cm)
        comp_s.append(0.0 if not math.isfinite(cs) else cs)
        cpu_m.append(0.0 if not math.isfinite(pm) else pm)
        cpu_s.append(0.0 if not math.isfinite(ps) else ps)

    b1 = ax.bar(
        sxs - 0.18,
        comp_m,
        width=0.34,
        yerr=comp_s,
        capsize=2.0,
        error_kw={"elinewidth": 0.7},
        color="#0072B2",
        edgecolor="#222222",
        linewidth=0.4,
        label="3D Comp. @15cm (%)",
        zorder=3,
    )
    b2 = ax2.bar(
        sxs + 0.18,
        cpu_m,
        width=0.34,
        yerr=cpu_s,
        capsize=2.0,
        error_kw={"elinewidth": 0.7},
        color="#E69F00",
        edgecolor="#222222",
        linewidth=0.4,
        label="Mean CPU Load (%)",
        zorder=3,
    )
    ax.set_xticks(sxs)
    ax.set_xticklabels([lbl for _, lbl in sensitivity_order], fontsize=6.8)
    ax.set_xlabel("LiDAR Beams / OctoMap Voxel Size (Mine)")
    ax.set_ylabel("3D Completeness @ 15 cm (%)", color="#0072B2")
    ax2.set_ylabel("Mean CPU Load (% of 400%)", color="#B87400")
    ax.set_ylim(0.0, 108.0)
    ax2.set_ylim(0.0, 340.0)
    ax.set_title("(b) Sensor & Voxel Sensitivity", fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend([b1, b2], ["3D Comp. @15cm (%)", "Mean CPU (%)"], frameon=True, loc="upper left", ncol=2, fontsize=6.2)

    fig.tight_layout()
    return _save_fig(fig, out_stem)


def format_latex_table(
    rows: Sequence[Tuple[str, Dict[str, Dict[str, float]]]],
    columns: Sequence[Tuple[str, str, str]],
    caption: str = "Quantitative comparison across benchmark environments.",
    label: str = "tab:main_results",
    best_modes: Optional[Dict[str, str]] = None,
    sig_markers: Optional[Dict[Tuple[str, str], str]] = None,
    table_env: str = "table",
    first_col_header: str = "Method",
    scales: Optional[Dict[str, float]] = None,
    section_breaks: Optional[Sequence[int]] = None,
) -> str:
    """Render a booktabs LaTeX table with Mean +/- Std entries, optional bold-best & significance.

    ``columns`` is a sequence of ``(metric_key, header_label, fmt_spec)``.
    ``best_modes`` maps ``metric_key`` -> ``"max"`` or ``"min"`` to bold the best finite mean.
    ``sig_markers`` maps ``(row_name, metric_key)`` -> superscript string such as ``"*"`` or ``"**"``.
    ``scales`` maps ``metric_key`` -> multiplicative factor (e.g. 100.0 for ratios to %).
    """
    best_modes = best_modes or {}
    sig_markers = sig_markers or {}
    scales = scales or {}
    breaks = set(section_breaks or ())

    # Compute best value per column (optionally within each section split by section_breaks)
    sections: List[List[int]] = [[]]
    for idx in range(len(rows)):
        if idx in breaks and sections[-1]:
            sections.append([])
        sections[-1].append(idx)

    best_idx_by_sec_col: Dict[Tuple[int, str], List[int]] = {}
    for sec_i, indices in enumerate(sections):
        for key, _hdr, _fmt in columns:
            mode = best_modes.get(key)
            if mode not in ("max", "min"):
                continue
            sc = scales.get(key, 1.0)
            vals = []
            for r_i in indices:
                m = rows[r_i][1].get(key, {}).get("mean", float("nan"))
                if isinstance(m, (int, float)) and math.isfinite(m):
                    vals.append((r_i, float(m) * sc))
            if not vals:
                continue
            target = max(v for _, v in vals) if mode == "max" else min(v for _, v in vals)
            best_idx_by_sec_col[(sec_i, key)] = [r_i for r_i, v in vals if abs(v - target) <= 1e-9]

    col_spec = "l" + "c" * len(columns)
    headers = " & ".join([first_col_header] + [h for _, h, _ in columns]) + r" \\"
    lines = [
        f"\\begin{{{table_env}}}[t]",
        r"\centering",
        f"\\caption{{{caption}}}",
        f"\\label{{{label}}}",
        r"\footnotesize",
        f"\\begin{{tabular}}{{{col_spec}}}",
        r"\toprule",
        headers,
        r"\midrule",
    ]
    row_to_sec = {r_i: sec_i for sec_i, indices in enumerate(sections) for r_i in indices}
    for r_i, (method_name, agg) in enumerate(rows):
        if r_i in breaks and r_i > 0:
            lines.append(r"\midrule")
        cells = [method_name]
        sec_i = row_to_sec[r_i]
        for key, _hdr, fmt in columns:
            stat = agg.get(key, {})
            sc = scales.get(key, 1.0)
            m = stat.get("mean", float("nan"))
            s = stat.get("std", float("nan"))
            if isinstance(m, (int, float)) and math.isfinite(m):
                ms = float(m) * sc
                ss = float(s) * sc if isinstance(s, (int, float)) and math.isfinite(s) else 0.0
                val_str = f"{format(ms, fmt)} \\pm {format(ss, fmt)}"
                if r_i in best_idx_by_sec_col.get((sec_i, key), []):
                    val_str = f"\\mathbf{{{val_str}}}"
                mark = sig_markers.get((method_name, key), "")
                if mark:
                    val_str = f"{val_str}^{{{mark}}}"
                cells.append(f"${val_str}$")
            else:
                cells.append("--")
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}", f"\\end{{{table_env}}}"])
    return "\n".join(lines) + "\n"
