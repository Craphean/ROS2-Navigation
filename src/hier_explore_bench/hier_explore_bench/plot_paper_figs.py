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
    "mtare": {"label": "CMU M-TARE (0.5 m/s, GT)", "color": "#D55E00", "ls": "--", "marker": "s", "lw": 1.5, "zorder": 4},
    "e3_hier": {"label": "Hier-Greedy (E3)", "color": "#009E73", "ls": "-.", "marker": "^", "lw": 1.4, "zorder": 3},
    "e1_2d": {"label": "2D-Frontier (E1)", "color": "#E69F00", "ls": ":", "marker": "D", "lw": 1.4, "zorder": 2},
    "legacy": {"label": "m-explore-ros2", "color": "#CC79A7", "ls": (0, (3, 1, 1, 1)), "marker": "v", "lw": 1.3, "zorder": 1},
    "e2_3dgain": {"label": "2D+3D Gain (E2)", "color": "#56B4E9", "ls": "--", "marker": "P", "lw": 1.3, "zorder": 2},
    "e4_geodesic": {"label": "+Geodesic (E4)", "color": "#009E73", "ls": "-.", "marker": "^", "lw": 1.4, "zorder": 3},
    "e5_deconflict": {"label": "+Deconflict (E5)", "color": "#332288", "ls": "-", "marker": "X", "lw": 1.5, "zorder": 4},
    "no_se2": {"label": "No SE(2) Align", "color": "#E69F00", "ls": "--", "marker": "D", "lw": 1.3, "zorder": 2},
    "no_phase_corr": {"label": "ICP Only (w/o FFT)", "color": "#CC79A7", "ls": ":", "marker": "v", "lw": 1.3, "zorder": 2},
    "no_degen_gate": {"label": "FFT+ICP (w/o Degen. Gate)", "color": "#D55E00", "ls": "-.", "marker": "s", "lw": 1.4, "zorder": 3},
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
    stress_aggregates: Dict[str, Dict[str, Dict[str, Dict[str, Dict[str, float]]]]],
    out_stem: Path,
    worlds: Sequence[Tuple[str, str]] = (("mine", "Mine"), ("warehouse", "Warehouse")),
    tiers: Sequence[Tuple[str, str]] = (
        ("p00", "0 m, 0°"),
        ("p05", "0.5 m, 5°"),
        ("p10", "1.0 m, 10°"),
    ),
    variants: Sequence[str] = ("no_se2", "no_phase_corr", "no_degen_gate", "full"),
) -> Path:
    """Plot Fig. 2: Initial-Pose Perturbation Stress Test of Online SE(2) Registration (2x3 panels).

    ``stress_aggregates`` maps ``world -> tier -> variant -> metric -> {mean, std}``.
    """
    configure_ieee_style()
    metrics = [
        ("ate_rmse_mean_m", "ATE RMSE (m) ↓", 1.0, None),
        ("occupied_iou_tol1", "2D Map Occ. IoU ↑", 1.0, (0.0, 1.0)),
        ("precision_10cm", "3D Prec. @ 10 cm (%) ↑", 100.0, (0.0, 108.0)),
    ]
    n_rows = len(worlds)
    n_cols = len(metrics)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(7.16, 1.90 * n_rows + 0.45), dpi=300)
    axes_2d = np.atleast_2d(axes)
    x = np.arange(len(tiers))
    n_var = len(variants)
    width = 0.78 / max(1, n_var)
    panel_tags = "abcdefghi"

    variant_labels = {
        "no_se2": "No SE(2) Align",
        "no_phase_corr": "ICP Only (w/o FFT)",
        "no_degen_gate": "FFT+ICP (w/o Degen. Gate)",
        "full": "Lite-HMRE (Full SE(2))",
    }

    for r_idx, (wkey, wlabel) in enumerate(worlds):
        for c_idx, (mkey, ylabel, scale, ylim) in enumerate(metrics):
            ax = axes_2d[r_idx, c_idx]
            for v_idx, vkey in enumerate(variants):
                st = get_method_style(vkey, v_idx)
                means, stds = [], []
                for tkey, _ in tiers:
                    stat = stress_aggregates.get(wkey, {}).get(tkey, {}).get(vkey, {}).get(mkey, {})
                    m = float(stat.get("mean", float("nan"))) * scale
                    s = float(stat.get("std", 0.0)) * scale
                    means.append(0.0 if not math.isfinite(m) else m)
                    stds.append(0.0 if not math.isfinite(s) else s)
                offset = (v_idx - (n_var - 1) / 2.0) * width
                ax.bar(
                    x + offset,
                    means,
                    width=width * 0.90,
                    yerr=stds,
                    capsize=1.8,
                    error_kw={"elinewidth": 0.65, "capthick": 0.65},
                    color=str(st["color"]),
                    edgecolor="#222222",
                    linewidth=0.4,
                    label=variant_labels.get(vkey, str(st["label"])),
                    zorder=3,
                )
            tag = panel_tags[r_idx * n_cols + c_idx]
            clean_title = ylabel.split(" (")[0].replace(" ↑", "").replace(" ↓", "")
            ax.set_title(f"({tag}) {wlabel}: {clean_title}", fontweight="bold", pad=4)
            ax.set_xticks(x)
            ax.set_xticklabels([tlabel for _, tlabel in tiers], fontsize=6.8)
            if r_idx == n_rows - 1:
                ax.set_xlabel("Initial Pose Perturbation")
            ax.set_ylabel(ylabel)
            if ylim is not None:
                ax.set_ylim(*ylim)
            else:
                ax.set_ylim(bottom=0.0)
            ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)

    handles, labels = axes_2d[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=min(len(labels), 4),
        frameon=True,
        framealpha=0.95,
        columnspacing=1.0,
        handletextpad=0.45,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93), h_pad=1.0, w_pad=1.1)
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
    ax.set_ylim(0.0, max_comms * 1.48)
    ax2.set_ylim(0.0, max(max_ram * 1.48, 7.6))
    ax2.axhline(5.80, color="#D55E00", linestyle=":", linewidth=1.0, alpha=0.85, label="M-TARE (N=2) RAM")
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


def plot_edge_pareto(
    main_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]],
    out_stem: Path,
    pareto_worlds: Sequence[Tuple[str, str]] = (("cmu_indoor", "CMU Indoor"), ("cmu_tunnel", "CMU Tunnel")),
    methods: Sequence[str] = ("full", "e3_hier", "e1_2d", "mtare"),
) -> Path:
    """Plot Fig. 3b: Edge Compute Efficiency Pareto (RAM vs P95 Latency & CE vs RAM)."""
    configure_ieee_style()
    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.55), dpi=300)

    # Panel (a): Peak RAM (GB) vs Planning P95 Latency (ms) with 4 GB Edge Feasibility Zone
    ax = axes[0]
    ax.axvspan(0.0, 4.0, color="#E8F5E9", alpha=0.65, zorder=0)
    ax.axvline(4.0, color="#2E7D32", linestyle="--", linewidth=0.9, zorder=1)
    ax.text(2.75, 1085.0, "4 GB Edge SBC Zone", ha="center", va="top", fontsize=6.5, color="#1B5E20", fontweight="bold")

    w_markers = {"cmu_indoor": "o", "cmu_tunnel": "s"}
    plotted_methods = set()
    for wkey, wlabel in pareto_worlds:
        for idx, mkey in enumerate(methods):
            agg = main_agg.get(wkey, {}).get(mkey)
            if not agg:
                continue
            ram_gb = float(agg.get("ram_peak_mb", {}).get("mean", float("nan"))) / 1024.0
            ram_sd = float(agg.get("ram_peak_mb", {}).get("std", 0.0)) / 1024.0
            p95 = float(agg.get("cycle_ms_p95", {}).get("mean", float("nan")))
            p95_sd = float(agg.get("cycle_ms_p95", {}).get("std", 0.0))
            if not (math.isfinite(ram_gb) and math.isfinite(p95)):
                continue
            st = get_method_style(mkey, idx)
            lbl = str(st["label"]) if mkey not in plotted_methods else None
            plotted_methods.add(mkey)
            ax.errorbar(
                ram_gb,
                p95,
                xerr=ram_sd,
                yerr=p95_sd,
                fmt=w_markers.get(wkey, "o"),
                markersize=6.5 if mkey == "full" else 5.5,
                color=str(st["color"]),
                markeredgecolor="#222222",
                markeredgewidth=0.6,
                capsize=2.0,
                elinewidth=0.8,
                label=lbl,
                zorder=4 if mkey == "full" else 3,
            )
    ax.set_xlabel("Peak System RAM (GB) ↓")
    ax.set_ylabel("Planning Cycle P95 Latency (ms) ↓")
    ax.set_xlim(1.5, 6.7)
    ax.set_ylim(0.0, 1150.0)
    ax.set_title("(a) Edge Memory vs. Latency (○ Indoor, □ Tunnel)", fontweight="bold")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend(frameon=True, loc="center right", fontsize=6.0, framealpha=0.95)

    # Panel (b): Coverage Efficiency (CE, m^2/m) across worlds comparing Lite-HMRE vs Baselines
    ax = axes[1]
    ce_worlds = [("house", "House"), ("hospital", "Hosp."), ("mine", "Mine"), ("cmu_indoor", "Indoor"), ("cmu_tunnel", "Tunnel")]
    ce_methods = ("legacy", "e1_2d", "mtare", "full")
    xs = np.arange(len(ce_worlds))
    n_m = len(ce_methods)
    bw = 0.78 / n_m
    for m_idx, mkey in enumerate(ce_methods):
        st = get_method_style(mkey, m_idx)
        vals, errs = [], []
        for wkey, _ in ce_worlds:
            stat = main_agg.get(wkey, {}).get(mkey, {}).get("coverage_efficiency_m2_per_m", {})
            m = float(stat.get("mean", float("nan")))
            s = float(stat.get("std", 0.0))
            vals.append(0.0 if not math.isfinite(m) else m)
            errs.append(0.0 if not math.isfinite(s) else s)
        off = (m_idx - (n_m - 1) / 2.0) * bw
        ax.bar(
            xs + off,
            vals,
            width=bw * 0.92,
            yerr=errs,
            capsize=1.8,
            error_kw={"elinewidth": 0.65},
            color=str(st["color"]),
            edgecolor="#222222",
            linewidth=0.4,
            label=str(st["label"]),
            zorder=3,
        )
    ax.set_xticks(xs)
    ax.set_xticklabels([wl for _, wl in ce_worlds])
    ax.set_ylabel("Coverage Efficiency CE (m²/m) ↑")
    ax.set_ylim(bottom=0.0)
    ax.set_title("(b) Speed-Normalized Spatial Efficiency", fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend(frameon=True, loc="upper left", fontsize=5.8, ncol=1, framealpha=0.94)

    fig.tight_layout()
    return _save_fig(fig, out_stem)


def plot_ablation_and_sensitivity(
    ablation_agg: Dict[str, Dict[str, Dict[str, Dict[str, float]]]],
    sensitivity_agg: Dict[str, Dict[str, Dict[str, float]]],
    out_stem: Path,
    ablation_order: Sequence[Tuple[str, str]] = (
        ("e1_2d", "E1"),
        ("e2_3dgain", "E2"),
        ("e3_hier", "E3"),
        ("e4_geodesic", "E4"),
        ("e5_deconflict", "E5"),
        ("full", "E6"),
    ),
    sensitivity_order: Sequence[Tuple[str, str]] = (
        ("16beam_005", "16B\n0.05m"),
        ("16beam_010", "16B\n0.10m"),
        ("16beam_015", "16B\n0.15m"),
        ("32beam_010", "32B\n0.10m"),
    ),
) -> Path:
    """Plot Fig. 5: 3-panel 6-Step Ablation (3D Comp + CE/Overlap) & Sensor/Voxel Sensitivity."""
    configure_ieee_style()
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.45), dpi=300)

    xs = np.arange(len(ablation_order))
    w = 0.38

    # Panel (a): 7-step ablation 3D Completeness @ 15 cm
    ax = axes[0]
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
            capsize=1.6,
            error_kw={"elinewidth": 0.6},
            color=col,
            edgecolor="#222222",
            linewidth=0.35,
            label=wlabel,
            zorder=3,
        )
    ax.set_xticks(xs)
    ax.set_xticklabels([short for _, short in ablation_order], fontsize=6.5)
    ax.set_xlabel("Ablation Step")
    ax.set_ylabel("3D Comp. @ 15 cm (%) ↑")
    ax.set_ylim(0.0, 108.0)
    ax.set_title("(a) Ablation: 3D Quality", fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend(frameon=True, loc="upper center", ncol=2, fontsize=6.2)

    # Panel (b): 7-step ablation Coverage Efficiency CE (bars) & Path Overlap (dashed line)
    ax = axes[1]
    ax_ov = ax.twinx()
    ax_ov.spines["top"].set_visible(False)
    for idx, (wkey, wlabel, col, mk) in enumerate((("mine", "Mine", "#0072B2", "o"), ("hospital", "Hospital", "#D55E00", "s"))):
        ce_m = [
            float(ablation_agg.get(wkey, {}).get(mkey, {}).get("coverage_efficiency_m2_per_m", {}).get("mean", 0.0))
            for mkey, _ in ablation_order
        ]
        ov_m = [
            float(ablation_agg.get(wkey, {}).get(mkey, {}).get("overlap_ratio", {}).get("mean", 0.0)) * 100.0
            for mkey, _ in ablation_order
        ]
        ce_m = [0.0 if not math.isfinite(v) else v for v in ce_m]
        ov_m = [0.0 if not math.isfinite(v) else v for v in ov_m]
        ax.plot(xs, ce_m, color=col, marker=mk, linewidth=1.3, markersize=4.0, label=f"{wlabel} CE", zorder=4)
        ax_ov.plot(xs, ov_m, color=col, marker=mk, linestyle="--", linewidth=1.0, markersize=3.5, alpha=0.75, label=f"{wlabel} Ovlp", zorder=3)
    ax.set_xticks(xs)
    ax.set_xticklabels([short for _, short in ablation_order], fontsize=6.5)
    ax.set_xlabel("Ablation Step")
    ax.set_ylabel("Coverage Eff. CE (m²/m) ↑")
    ax_ov.set_ylabel("Path Overlap (%) ↓", color="#444444")
    ax.set_ylim(0.0, 4.15)
    ax_ov.set_ylim(0.0, 16.5)
    ax.set_title("(b) Ablation: CE & Overlap", fontweight="bold")
    ax.grid(True, linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_ov.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, frameon=True, loc="upper right", fontsize=5.2, ncol=2, framealpha=0.94)

    # Panel (c): Sensor & voxel resolution sensitivity on Mine
    ax = axes[2]
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
        sxs - 0.19,
        comp_m,
        width=0.36,
        yerr=comp_s,
        capsize=1.8,
        error_kw={"elinewidth": 0.65},
        color="#0072B2",
        edgecolor="#222222",
        linewidth=0.35,
        label="3D Comp.",
        zorder=3,
    )
    b2 = ax2.bar(
        sxs + 0.19,
        cpu_m,
        width=0.36,
        yerr=cpu_s,
        capsize=1.8,
        error_kw={"elinewidth": 0.65},
        color="#E69F00",
        edgecolor="#222222",
        linewidth=0.35,
        label="CPU %",
        zorder=3,
    )
    ax.set_xticks(sxs)
    ax.set_xticklabels([lbl for _, lbl in sensitivity_order], fontsize=6.2)
    ax.set_xlabel("LiDAR / Voxel (Mine)")
    ax.set_ylabel("3D Comp. (%) ↑", color="#0072B2")
    ax2.set_ylabel("CPU (% / 400%) ↓", color="#B87400")
    ax.set_ylim(0.0, 108.0)
    ax2.set_ylim(0.0, 350.0)
    ax.set_title("(c) Sensor & Voxel Trade-off", fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.35, alpha=0.45, zorder=0)
    ax.legend([b1, b2], ["3D Comp. (%)", "CPU (%)"], frameon=True, loc="upper left", ncol=2, fontsize=5.8)

    fig.tight_layout(w_pad=1.3)
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
