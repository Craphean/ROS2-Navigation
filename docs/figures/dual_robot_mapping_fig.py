#!/usr/bin/env python3
"""Journal-style schematic of the dual-robot mapping and exploration stack.

Outputs (180 mm double-column width):
  dual_robot_mapping_en.{svg,pdf,png}
  dual_robot_mapping_zh.{svg,pdf,png}
"""

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

mpl.rcParams.update({
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "svg.fonttype": "none",
    "mathtext.fontset": "dejavusans",
    "axes.linewidth": 0.5,
})

MM = 1.0
W, H = 180.0, 112.0

INK = "#1A1A1A"
GRAY = "#5E5E5E"
MID = "#8A8A8A"
RULE = "#CFCFCF"

C2D = "#3C5E7F"
F2D = "#EDF2F6"
C3D = "#8A6534"
F3D = "#F7F1E6"
CDEC = "#2C5F55"
FDEC = "#E9F1EF"
CEXE = "#4A4A4A"
FEXE = "#F4F4F4"

LINEW = 0.55
ARROW = "#4A4A4A"


def box(ax, x0, y0, x1, y1, title, sub=None, edge=CEXE, face=FEXE,
        tsize=5.4, ssize=4.4, bold=False):
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0,rounding_size=0.8",
        linewidth=LINEW, edgecolor=edge, facecolor=face, zorder=2))
    h = y1 - y0
    if sub:
        ax.text((x0 + x1) / 2, y0 + 0.68 * h, title, ha="center", va="center",
                fontsize=tsize, color=INK, weight="bold" if bold else "normal",
                zorder=3)
        ax.text((x0 + x1) / 2, y0 + 0.29 * h, sub, ha="center", va="center",
                fontsize=ssize, color=GRAY, zorder=3)
    else:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, title, ha="center", va="center",
                fontsize=tsize, color=INK, zorder=3)


def arrow(ax, x0, y0, x1, y1, style="-|>", ls="-", color=ARROW, lw=0.6,
          ms=4.6, z=2):
    ax.add_patch(FancyArrowPatch(
        (x0, y0), (x1, y1), arrowstyle=style, mutation_scale=ms,
        linewidth=lw, color=color, linestyle=ls, shrinkA=0, shrinkB=0,
        zorder=z, joinstyle="miter"))


def build(lang):
    zh = lang == "zh"
    fam = "Noto Sans CJK JP" if zh else "Liberation Sans"
    mpl.rcParams["font.family"] = "sans-serif"
    mpl.rcParams["font.sans-serif"] = [fam, "DejaVu Sans"]

    T = {
        "en": dict(
            bands=["Sensing", "Mapping", "Decision", "Execution"],
            gaz="Gazebo simulation — 2 × TurtleBot3 Waffle",
            gazsub="2D LiDAR 360 × 1 @ 5 Hz  ·  3D LiDAR 32 × 720 @ 5 Hz  ·  "
                   "ros_gz_bridge → /robotX/{scan, points, odom}",
            sl="2D SLAM × 2", slsub="slam_toolbox, per robot",
            mm="Map merge", mmsub="known initial poses",
            mp="/map · world · 0.05 m",
            cm="Cloud merger", cmsub="TF → world · 2 cm voxel · 3 Hz",
            oc="OctoMap", ocsub="0.10 m voxel · 3D occupancy",
            oct="/octomap_point_cloud_centers",
            co="Exploration coordinator", cosub="frontier clustering · 3D gain · "
                "local/global ownership · auction · 0.5 Hz",
            n1="Nav2 · robot 1", n1a="global: static (/map) + scan → NavFn",
            n1b="local: voxel(scan + points) → MPPI @ 20 Hz",
            n2="Nav2 · robot 2",
            f1="Frontier clusters", f1s="2D gap cells, 8-connected",
            f2="3D information gain", f2s=r"$g=1.0\,g_{2D}+40\,g_{3D}$",
            f3="Ownership", f3s=r"$d\leq6$ m, same free component",
            f4="Local pool → TSP tour", f4s="≤ 6 points, ≤ 10 m",
            f5="Global pool → ROI auction", f5s="ROI = g/(d+1) → max bid",
            l2="2D metric layer", l3="3D structural layer",
        ),
        "zh": dict(
            bands=["感知", "建图", "决策", "执行"],
            gaz="Gazebo 仿真 · 2 × TurtleBot3 Waffle",
            gazsub="2D 激光 360×1 @5 Hz  ·  3D 激光 32×720 @5 Hz  ·  "
                   "ros_gz_bridge → /robotX/{scan, points, odom}",
            sl="2D SLAM × 2", slsub="slam_toolbox，每车独立",
            mm="地图合并", mmsub="已知初始位姿",
            mp="/map · world · 0.05 m",
            cm="点云合并", cmsub="TF→world · 2 cm 体素 · 3 Hz",
            oc="OctoMap", ocsub="0.10 m 体素 · 三维占据",
            oct="/octomap_point_cloud_centers",
            co="探索协调器", cosub="前沿聚类 · 3D 增益 · 本地/全局归属 · 竞拍 · 0.5 Hz",
            n1="Nav2 · robot 1", n1a="全局：静态图(/map) + scan → NavFn",
            n1b="局部：voxel(scan + points) → MPPI @20 Hz",
            n2="Nav2 · robot 2",
            f1="前沿簇", f1s="2D 缝隙格，8 连通",
            f2="3D 信息增益", f2s=r"$g=1.0\,g_{2D}+40\,g_{3D}$",
            f3="归属判定", f3s=r"$d\leq6$ m 且同一连通域",
            f4="Local 池 → TSP 巡游", f4s="≤6 点，≤10 m",
            f5="Global 池 → ROI 竞拍", f5s="ROI = g/(d+1)，取最大 bid",
            l2="2D 度量层", l3="3D 结构层",
        ),
    }[lang]

    fig = plt.figure(figsize=(W / 25.4, H / 25.4), dpi=600)
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")

    # ---------------- panel a ----------------
    ax.text(5.0, 107.0, "a", fontsize=8.6, weight="bold", color=INK,
            ha="left", va="center")

    band_y = [(94, 106), (61, 91), (46, 58), (31, 43)]
    for (y0, y1), name in zip(band_y, T["bands"]):
        ax.text(20.0, (y0 + y1) / 2, name, fontsize=5.3, color=MID,
                ha="right", va="center")

    # sensing
    box(ax, 24, 94, 170, 106, T["gaz"], T["gazsub"], edge=CEXE, face=FEXE,
        tsize=5.2, ssize=4.3)

    # trunk + taps into the two mapping branches
    ax.plot([32, 32], [94, 69.5], color=ARROW, lw=0.6, zorder=1,
            solid_capstyle="butt")
    arrow(ax, 32, 83.0, 39, 83.0)
    arrow(ax, 32, 69.5, 39, 69.5)

    ax.text(70, 89.7, T["l2"], ha="center", va="center", fontsize=4.6,
            color=MID)
    ax.text(139, 89.7, T["l3"], ha="center", va="center", fontsize=4.6,
            color=MID)

    # 2D column
    box(ax, 40, 78.5, 100, 88, T["sl"], T["slsub"], edge=C2D, face=F2D,
        tsize=5.2, ssize=4.3)
    arrow(ax, 70, 78.5, 70, 76.3)
    box(ax, 40, 68, 100, 76, T["mm"], T["mmsub"], edge=C2D, face=F2D,
        tsize=5.2, ssize=4.3)
    arrow(ax, 70, 68, 70, 65.8)
    box(ax, 40, 57.5, 100, 65.5, T["mp"], None, edge=C2D, face=F2D,
        tsize=5.0)

    # 3D column
    box(ax, 108, 78.5, 170, 88, T["cm"], T["cmsub"], edge=C3D, face=F3D,
        tsize=5.2, ssize=4.3)
    arrow(ax, 139, 78.5, 139, 76.3)
    box(ax, 108, 68, 170, 76, T["oc"], T["ocsub"], edge=C3D, face=F3D,
        tsize=5.2, ssize=4.3)
    arrow(ax, 139, 68, 139, 65.8)
    box(ax, 108, 57.5, 170, 65.5, T["oct"], None, edge=C3D, face=F3D,
        tsize=4.7)

    # decision
    arrow(ax, 70, 57.5, 70, 54.0)
    arrow(ax, 138, 57.5, 138, 54.0)
    box(ax, 24, 42, 170, 54, T["co"], T["cosub"], edge=CDEC, face=FDEC,
        tsize=5.4, ssize=4.3, bold=False)

    # execution
    arrow(ax, 70, 42, 70, 39.8)
    arrow(ax, 139, 42, 139, 39.8)

    def nav_box(x0, x1, title):
        ax.add_patch(FancyBboxPatch(
            (x0, 28.5), x1 - x0, 11.3,
            boxstyle="round,pad=0,rounding_size=0.8",
            linewidth=LINEW, edgecolor=CEXE, facecolor=FEXE, zorder=2))
        cx = (x0 + x1) / 2
        ax.text(cx, 37.3, title, ha="center", va="center", fontsize=5.2,
                color=INK, zorder=3)
        ax.text(cx, 33.5, T["n1a"], ha="center", va="center", fontsize=4.3,
                color=GRAY, zorder=3)
        ax.text(cx, 30.4, T["n1b"], ha="center", va="center", fontsize=4.3,
                color=GRAY, zorder=3)

    nav_box(24, 102, T["n1"])
    nav_box(108, 170, T["n2"])

    # feedback
    ax.plot([170, 175, 175], [34.0, 34.0, 48.0], color=MID, lw=0.5,
            ls=(0, (2.2, 1.6)), zorder=1, solid_capstyle="butt")
    arrow(ax, 175, 48.0, 170.5, 48.0, ls=(0, (2.2, 1.6)), color=MID, lw=0.5)

    # ---------------- panel b ----------------
    ax.plot([4, 175], [26.0, 26.0], color=RULE, lw=0.5, zorder=1)
    ax.text(5.0, 23.0, "b", fontsize=8.6, weight="bold", color=INK,
            ha="left", va="center")

    box(ax, 8, 12.5, 42, 21.5, T["f1"], T["f1s"], edge=CDEC, face=FDEC,
        tsize=5.0, ssize=4.2)
    arrow(ax, 42, 17.0, 49, 17.0)
    box(ax, 49, 12.5, 89, 21.5, T["f2"], T["f2s"], edge=CDEC, face=FDEC,
        tsize=5.0, ssize=4.2)
    arrow(ax, 89, 17.0, 95, 17.0)
    box(ax, 95, 12.5, 135, 21.5, T["f3"], T["f3s"], edge=CDEC, face=FDEC,
        tsize=5.0, ssize=4.2)

    ax.plot([139, 139], [10.6, 18.4], color=ARROW, lw=0.6, zorder=1,
            solid_capstyle="butt")
    arrow(ax, 135, 17.0, 139, 17.0)
    arrow(ax, 139, 17.8, 143, 17.8)
    arrow(ax, 139, 10.6, 143, 10.6)
    box(ax, 143, 14.2, 175, 21.4, T["f4"], T["f4s"], edge=C2D, face=F2D,
        tsize=4.9, ssize=4.2)
    box(ax, 143, 6.8, 175, 14.0, T["f5"], T["f5s"], edge=C3D, face=F3D,
        tsize=4.9, ssize=4.2)

    stem = f"dual_robot_mapping_{lang}"
    out = "/home/qing/ros2_navigation/docs/figures/" + stem
    fig.savefig(out + ".svg", format="svg")
    fig.savefig(out + ".pdf", format="pdf")
    fig.savefig(out + ".png", format="png", dpi=600)
    plt.close(fig)
    print("wrote", out)


for lang in ("en", "zh"):
    build(lang)
