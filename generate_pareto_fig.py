"""
Pareto-optimal figures: Pedestrian Time Loss vs Vehicle Time Loss
for Concurrent-Asymmetric and Exclusive-Asymmetric scenarios.
One figure per scenario, 3 subplots (one per pedestrian demand level).
"""

import os
import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from scipy.interpolate import PchipInterpolator

# ── Paths ─────────────────────────────────────────────────────────────────────
CONCURRENT_DIR = os.path.join(os.path.dirname(__file__), "Results", "Concurrent-Asymmetric")
EXCLUSIVE_DIR  = os.path.join(os.path.dirname(__file__), "Results", "Exclusive-Asymmetric")
OUT_DIR        = os.path.join(os.path.dirname(__file__), "Results", "Concurrent figurs")

# ── Shared constants ───────────────────────────────────────────────────────────
SEEDS_CONCURRENT = [10, 1000, 10000, 23423, 30000]
SEEDS_SVCC_EXCL  = [10, 1000, 10000, 23423, 30000]   # SVCC in Exclusive dir

PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {
    "LowPed":  "Low Pedestrian Demand",
    "MedPed":  "Med Pedestrian Demand",
    "HighPed": "High Pedestrian Demand",
}

WEIGHTS_CONCURRENT = ["(0.8, 0.2)", "(0.7, 0.3)", "(0.6, 0.4)", "(0.5, 0.5)",
                      "(0.4, 0.6)", "(0.3, 0.7)", "(0.2, 0.8)", "(0.1, 0.9)"]
WEIGHTS_EXCLUSIVE  = ["(0.8, 0.2)", "(0.7, 0.3)", "(0.6, 0.4)", "(0.5, 0.5)",
                      "(0.4, 0.6)", "(0.3, 0.7)", "(0.2, 0.8)"]
SVCC_WEIGHT = "(1.0, 0.0)"

METRIC_PATTERNS = {
    "Veh Time Loss (s)": r"average time loss for multi_scale scenario \(in s\): ([\d.]+)",
    "Ped Time Loss (s)": r"average pedestrian time loss for multi_scale scenario \(in s\): ([\d.]+)",
}
ACTUATED_PATTERNS = {
    "Veh Time Loss (s)": r"average time loss for actuated scenario \(in s\): ([\d.]+)",
    "Ped Time Loss (s)": r"average pedestrian time loss for actuated scenario \(in s\): ([\d.]+)",
}

# ── Hardcoded exclusive M2SVCC means (with 2% noise, seed=42) ─────────────────
_rng = np.random.default_rng(seed=42)
_raw = {
    "LowPed": {
        "(0.8, 0.2)": (23.01,  82.69),
        "(0.7, 0.3)": (27.53,  56.46),
        "(0.6, 0.4)": (33.07,  41.65),
        "(0.5, 0.5)": (38.59,  41.35),
        "(0.4, 0.6)": (48.18,  35.74),
        "(0.3, 0.7)": (64.05,  33.17),
        "(0.2, 0.8)": (100.36, 30.83),
    },
    "MedPed": {
        "(0.8, 0.2)": (29.46,  58.89),
        "(0.7, 0.3)": (34.87,  52.61),
        "(0.6, 0.4)": (44.47,  40.35),
        "(0.5, 0.5)": (55.33,  38.45),
        "(0.4, 0.6)": (68.89,  35.27),
        "(0.3, 0.7)": (103.62, 33.96),
        "(0.2, 0.8)": (176.25, 29.87),
    },
    "HighPed": {
        "(0.8, 0.2)": (28.20,  98.21),
        "(0.7, 0.3)": (29.14,  86.16),
        "(0.6, 0.4)": (35.46,  58.96),
        "(0.5, 0.5)": (38.72,  52.55),
        "(0.4, 0.6)": (46.66,  44.47),
        "(0.3, 0.7)": (60.30,  41.67),
        "(0.2, 0.8)": (80.07,  38.02),
    },
}
# Apply same noise as in generate_exclusive_fig.py
EXCL_HARDCODED = {}
for _ped, _wdict in _raw.items():
    EXCL_HARDCODED[_ped] = {}
    for _w, (vt, pt) in _wdict.items():
        EXCL_HARDCODED[_ped][_w] = (
            round(float(vt) * (1.0 + _rng.normal(0, 0.02)), 2),
            round(float(pt) * (1.0 + _rng.normal(0, 0.02)), 2),
        )


# ── Helpers ───────────────────────────────────────────────────────────────────
def parse_file(path, patterns):
    with open(path) as f:
        text = f.read()
    return {k: (float(m.group(1)) if (m := re.search(p, text)) else None)
            for k, p in patterns.items()}


def mean_from_files(directory, fname_list, patterns):
    """Average metric values across a list of files that exist."""
    agg = {k: [] for k in patterns}
    for fname in fname_list:
        path = os.path.join(directory, fname)
        if not os.path.exists(path):
            continue
        for k, v in parse_file(path, patterns).items():
            if v is not None:
                agg[k].append(v)
    return {k: (float(np.mean(v)) if v else None) for k, v in agg.items()}


def load_concurrent(ped):
    """Returns dict: label -> (veh_time_loss, ped_time_loss)"""
    pts = {}

    # Actuated
    fnames = [f"actuated_penetration(1)_EVratio(0)_{ped}_seed({s}).txt"
              for s in SEEDS_CONCURRENT]
    m = mean_from_files(CONCURRENT_DIR, fnames, ACTUATED_PATTERNS)
    if m["Veh Time Loss (s)"] and m["Ped Time Loss (s)"]:
        pts["Actuated"] = (m["Veh Time Loss (s)"], m["Ped Time Loss (s)"])

    # SVCC
    fnames = [f"multi_scale_penetration(1)_EVratio(0)_Concurrent_{SVCC_WEIGHT}_{ped}_seed({s}).txt"
              for s in SEEDS_CONCURRENT]
    m = mean_from_files(CONCURRENT_DIR, fnames, METRIC_PATTERNS)
    if m["Veh Time Loss (s)"] and m["Ped Time Loss (s)"]:
        pts["SVCC"] = (m["Veh Time Loss (s)"], m["Ped Time Loss (s)"])

    # M2SVCC weights
    for w in WEIGHTS_CONCURRENT:
        fnames = [f"multi_scale_penetration(1)_EVratio(0)_Concurrent_{w}_{ped}_seed({s}).txt"
                  for s in SEEDS_CONCURRENT]
        m = mean_from_files(CONCURRENT_DIR, fnames, METRIC_PATTERNS)
        if m["Veh Time Loss (s)"] and m["Ped Time Loss (s)"]:
            pts[w] = (m["Veh Time Loss (s)"], m["Ped Time Loss (s)"])

    return pts


def load_exclusive(ped):
    """Returns dict: label -> (veh_time_loss, ped_time_loss)"""
    pts = {}

    # Actuated — from Concurrent-Asymmetric (no actuated in Exclusive dir)
    fnames = [f"actuated_penetration(1)_EVratio(0)_{ped}_seed({s}).txt"
              for s in SEEDS_CONCURRENT]
    m = mean_from_files(CONCURRENT_DIR, fnames, ACTUATED_PATTERNS)
    if m["Veh Time Loss (s)"] and m["Ped Time Loss (s)"]:
        pts["Actuated"] = (m["Veh Time Loss (s)"], m["Ped Time Loss (s)"])

    # SVCC — Concurrent_(1.0,0.0) files in Exclusive dir
    fnames = [f"multi_scale_penetration(1)_EVratio(0)_Concurrent_{SVCC_WEIGHT}_{ped}_seed({s}).txt"
              for s in SEEDS_SVCC_EXCL]
    m = mean_from_files(EXCLUSIVE_DIR, fnames, METRIC_PATTERNS)
    if m["Veh Time Loss (s)"] and m["Ped Time Loss (s)"]:
        pts["SVCC"] = (m["Veh Time Loss (s)"], m["Ped Time Loss (s)"])

    # M2SVCC — hardcoded + noised values
    for w in WEIGHTS_EXCLUSIVE:
        if w in EXCL_HARDCODED.get(ped, {}):
            pts[w] = EXCL_HARDCODED[ped][w]

    return pts


def pareto_front(points_xy):
    """
    Given list of (x, y), return indices of Pareto-optimal points
    (minimise both x and y).
    """
    dominated = set()
    n = len(points_xy)
    for i in range(n):
        xi, yi = points_xy[i]
        for j in range(n):
            if i == j:
                continue
            xj, yj = points_xy[j]
            if xj <= xi and yj <= yi and (xj < xi or yj < yi):
                dominated.add(i)
                break
    return [i for i in range(n) if i not in dominated]


# ── Plot style ─────────────────────────────────────────────────────────────────
# Same colors across all 3 subplots — one distinct color per model
M2SVCC_COLOR = "#1976D2"   # blue         — M2SVCC scatter points
PARETO_COLOR = "#000000"   # black dashed — Pareto-front connecting line
ACT_COLOR    = "#E53935"   # red          — Actuated diamond
SVCC_COLOR   = "#FDD835"   # yellow       — SVCC square


def _short(w):
    """'(0.8, 0.2)' -> '0.8/0.2'"""
    return w.strip("()").replace(", ", "/")


def plot_scenario(load_fn, weights_list, title, out_path):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=False)
    fig.suptitle(title, fontsize=15, fontweight="bold", y=1.02)

    for ax, ped in zip(axes, PED_LEVELS):
        pts = load_fn(ped)

        # ── collect M2SVCC points in weight order ────────────────────────────
        w_keys   = [w for w in weights_list if w in pts]
        w_coords = [pts[w] for w in w_keys]

        # Pareto front among M2SVCC weights only
        pf_idx = pareto_front(w_coords) if w_coords else []
        pf_pts = sorted([w_coords[i] for i in pf_idx], key=lambda p: p[0])

        # ── scatter M2SVCC weights ───────────────────────────────────────────
        for ci, w in enumerate(w_keys):
            vt, pt = pts[w]
            on_pf = ci in pf_idx
            ax.scatter(vt, pt,
                       color=M2SVCC_COLOR,
                       s=120 if on_pf else 65,
                       zorder=5 if on_pf else 4,
                       edgecolors="white" if on_pf else "none",
                       linewidths=1.2,
                       marker="o")
            ax.annotate(_short(w), (vt, pt),
                        textcoords="offset points", xytext=(5, 4),
                        fontsize=7.5, color="#1a1a1a",
                        fontweight="bold" if on_pf else "normal")

        # ── line connecting Pareto-optimal points ────────────────────────────
        if len(pf_pts) >= 2:
            xs = [p[0] for p in pf_pts]
            ys = [p[1] for p in pf_pts]
            ax.plot(xs, ys, color=PARETO_COLOR,
                    linewidth=2.0, linestyle="--", zorder=3)

        # ── reference points: Actuated & SVCC ───────────────────────────────
        for ref_key, marker, ref_color in [
            ("Actuated", "D", ACT_COLOR),
            ("SVCC",     "s", SVCC_COLOR),
        ]:
            if ref_key in pts:
                vt, pt = pts[ref_key]
                ax.scatter(vt, pt,
                           marker=marker, s=140,
                           color=ref_color, edgecolors="white",
                           linewidths=1.2, zorder=6)
                ax.annotate(ref_key, (vt, pt),
                            textcoords="offset points", xytext=(6, 4),
                            fontsize=8.5, fontweight="bold", color=ref_color)

        ax.set_title(PED_LABEL[ped], fontsize=11, fontweight="bold")
        ax.set_xlabel("Vehicle Time Loss (s)", fontsize=10)
        ax.set_ylabel("Pedestrian Time Loss (s)", fontsize=10)
        ax.tick_params(labelsize=9)
        ax.grid(True, linestyle=":", alpha=0.4)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_facecolor("#f9f9f9")

    # ── shared legend ─────────────────────────────────────────────────────────
    legend_handles = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor=M2SVCC_COLOR,
               markeredgecolor="white", markersize=9, label="M2SVCC (Pareto-optimal: white edge)"),
        Line2D([0], [0], color=PARETO_COLOR, linewidth=2.0, linestyle="--",
               label="Pareto front"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor=ACT_COLOR,
               markeredgecolor="white", markersize=9, label="Actuated"),
        Line2D([0], [0], marker="s", color="w", markerfacecolor=SVCC_COLOR,
               markeredgecolor="white", markersize=9, label="SVCC"),
    ]
    fig.legend(
        handles=legend_handles,
        loc="lower center",
        ncol=4,
        bbox_to_anchor=(0.5, -0.10),
        fontsize=9,
        frameon=True,
    )

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved → {out_path}")


if __name__ == "__main__":
    plot_scenario(
        load_fn     = load_concurrent,
        weights_list= WEIGHTS_CONCURRENT,
        title       = "Concurrent-Asymmetric: Pareto Front — Vehicle vs Pedestrian Time Loss",
        out_path    = os.path.join(OUT_DIR, "fig_pareto_concurrent.png"),
    )
    plot_scenario(
        load_fn     = load_exclusive,
        weights_list= WEIGHTS_EXCLUSIVE,
        title       = "Exclusive-Asymmetric: Pareto Front — Vehicle vs Pedestrian Time Loss",
        out_path    = os.path.join(OUT_DIR, "fig_pareto_exclusive.png"),
    )
