#!/usr/bin/env python3
"""IEEE double/single-column publication figure & LaTeX table generator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

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
]


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
        }
    )


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
        interp_ys = []
        for xs, ys in seed_runs:
            if len(xs) == 0:
                continue
            interp_ys.append(np.interp(grid_x, xs, ys, left=ys[0], right=ys[-1]))
        if not interp_ys:
            continue
        mat = np.vstack(interp_ys)
        mean_y = np.mean(mat, axis=0)
        std_y = np.std(mat, axis=0, ddof=1) if mat.shape[0] > 1 else np.zeros_like(mean_y)
        color = PALETTE[idx % len(PALETTE)]
        ax.plot(grid_x, mean_y, label=method, color=color)
        ax.fill_between(grid_x, mean_y - std_y, mean_y + std_y, color=color, alpha=0.18, linewidth=0)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.5)
    ax.legend(frameon=True, loc="lower right")
    fig.tight_layout()

    out_stem.parent.mkdir(parents=True, exist_ok=True)
    pdf_path = out_stem.with_suffix(".pdf")
    fig.savefig(pdf_path)
    fig.savefig(out_stem.with_suffix(".png"), dpi=300)
    plt.close(fig)
    return pdf_path


def format_latex_table(
    rows: Sequence[Tuple[str, Dict[str, Dict[str, float]]]],
    columns: Sequence[Tuple[str, str, str]],
    caption: str = "Quantitative comparison across benchmark environments.",
    label: str = "tab:main_results",
) -> str:
    """Render a booktabs LaTeX table with Mean +/- Std entries.

    ``columns`` is a sequence of ``(metric_key, header_label, fmt_spec)``.
    """
    col_spec = "l" + "c" * len(columns)
    headers = " & ".join(["Method"] + [h for _, h, _ in columns]) + r" \\"
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        f"\\caption{{{caption}}}",
        f"\\label{{{label}}}",
        f"\\begin{{tabular}}{{{col_spec}}}",
        r"\toprule",
        headers,
        r"\midrule",
    ]
    for method_name, agg in rows:
        cells = [method_name]
        for key, _hdr, fmt in columns:
            stat = agg.get(key, {})
            m = stat.get("mean", float("nan"))
            s = stat.get("std", float("nan"))
            if isinstance(m, (int, float)) and math.isfinite(m):
                cells.append(f"${format(m, fmt)} \\pm {format(s, fmt)}$")
            else:
                cells.append("--")
        lines.append(" & ".join(cells) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}"])
    return "\n".join(lines) + "\n"
