"""
Pareto frontier plots: Vehicle Waiting Time (x) vs Pedestrian Time Loss (y).

Layout: 2 rows (Symmetric / Asymmetric) × 3 columns (Low / Med / High Ped)
Points:
  ★  Actuated  — red star  (shown off-axis as annotation if far from cluster)
  ◆  SVCC      — dark grey diamond
  ●  M2SVCC weights — steel-blue circles with weight labels
Pareto frontier drawn through non-dominated M2SVCC + SVCC points.
"""

import os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

BASE = os.path.dirname(os.path.abspath(__file__))

# ── config ─────────────────────────────────────────────────────────────────────
SCENARIOS = [
    ("Concurrent-Symmetric",  "Symmetric"),
    ("Concurrent-Asymmetric", "Asymmetric"),
]

PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {"LowPed": "Low Ped", "MedPed": "Med Ped", "HighPed": "High Ped"}

WEIGHTS_ORDERED = [
    "1.0, 0.0",
    "0.8, 0.2",
    "0.7, 0.3",
    "0.6, 0.4",
    "0.5, 0.5",
    "0.4, 0.6",
    "0.3, 0.7",
    "0.2, 0.8",
    "0.1, 0.9",
]
SVCC_WEIGHT    = "1.0, 0.0"
M2SVCC_WEIGHTS = [w for w in WEIGHTS_ORDERED if w != SVCC_WEIGHT]

W_SHORT = {
    "0.8, 0.2": "(0.8,0.2)",
    "0.7, 0.3": "(0.7,0.3)",
    "0.6, 0.4": "(0.6,0.4)",
    "0.5, 0.5": "(0.5,0.5)",
    "0.4, 0.6": "(0.4,0.6)",
    "0.3, 0.7": "(0.3,0.7)",
    "0.2, 0.8": "(0.2,0.8)",
    "0.1, 0.9": "(0.1,0.9)",
}

PARSE_VEH = r"average waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)"
PARSE_PED = r"average pedestrian time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)"

M2SVCC_COLOR  = "#2c7bb6"   # steel blue
SVCC_COLOR    = "#333333"   # dark grey
ACT_COLOR     = "#d7191c"   # red


# ── helpers ────────────────────────────────────────────────────────────────────

def parse_file(path):
    with open(path) as f:
        text = f.read()
    mv = re.search(PARSE_VEH, text)
    mp = re.search(PARSE_PED, text)
    return (float(mv.group(1)) if mv else None,
            float(mp.group(1)) if mp else None)


def mean_point(data_dir, match_fn):
    vehs, peds = [], []
    for f in os.listdir(data_dir):
        if match_fn(f):
            v, p = parse_file(os.path.join(data_dir, f))
            if v is not None: vehs.append(v)
            if p is not None: peds.append(p)
    if not vehs:
        return None, None
    return float(np.mean(vehs)), float(np.mean(peds))


def load_actuated(data_dir, ped):
    pat = re.compile(rf"actuated_penetration\(1\)_EVratio\(0\)_{ped}_seed\((\d+)\)\.txt$")
    return mean_point(data_dir, pat.match)


def load_weight(data_dir, ped, weight_str):
    pat = re.compile(
        rf"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_"
        rf"\({re.escape(weight_str)}\)_{ped}_seed\((\d+)\)\.txt$"
    )
    return mean_point(data_dir, pat.match)


def pareto_frontier(points):
    """Return indices of non-dominated points (minimise both), sorted by x."""
    pts = np.array(points)
    n   = len(pts)
    dominated = np.zeros(n, dtype=bool)
    for i in range(n):
        for j in range(n):
            if i == j: continue
            if (pts[j, 0] <= pts[i, 0] and pts[j, 1] <= pts[i, 1] and
                    (pts[j, 0] < pts[i, 0] or pts[j, 1] < pts[i, 1])):
                dominated[i] = True
                break
    idx = np.where(~dominated)[0]
    return idx[np.argsort(pts[idx, 0])]


# ── subplot ────────────────────────────────────────────────────────────────────

def plot_subplot(ax, data_dir, ped, title):
    act_v,  act_p  = load_actuated(data_dir, ped)
    svcc_v, svcc_p = load_weight(data_dir, ped, SVCC_WEIGHT)

    m2svcc_pts = []
    for w in M2SVCC_WEIGHTS:
        v, p = load_weight(data_dir, ped, w)
        if v is not None and p is not None:
            m2svcc_pts.append((v, p, w))

    # ── determine zoom window from SVCC + M2SVCC points ──────────────────────
    cluster_xs = [v for v, p, w in m2svcc_pts]
    cluster_ys = [p for v, p, w in m2svcc_pts]
    if svcc_v is not None:
        cluster_xs.append(svcc_v)
        cluster_ys.append(svcc_p)

    if cluster_xs:
        x_min, x_max = min(cluster_xs), max(cluster_xs)
        y_min, y_max = min(cluster_ys), max(cluster_ys)
        x_pad = max((x_max - x_min) * 0.25, 1.5)
        y_pad = max((y_max - y_min) * 0.25, 1.5)
        xl = (x_min - x_pad, x_max + x_pad)
        yl = (y_min - y_pad, y_max + y_pad)
        ax.set_xlim(xl)
        ax.set_ylim(yl)

    # ── Pareto frontier (M2SVCC + SVCC) ──────────────────────────────────────
    all_pts = [(v, p) for v, p, w in m2svcc_pts]
    if svcc_v is not None:
        all_pts.append((svcc_v, svcc_p))
    if len(all_pts) >= 2:
        pidx = pareto_frontier(all_pts)
        px = [all_pts[i][0] for i in pidx]
        py = [all_pts[i][1] for i in pidx]
        ax.plot(px, py, color="#888888", linewidth=1.5,
                linestyle="--", zorder=2)

    # ── M2SVCC points ─────────────────────────────────────────────────────────
    for v, p, w in m2svcc_pts:
        ax.scatter(v, p, color=M2SVCC_COLOR, s=100, zorder=4,
                   edgecolors="white", linewidths=0.6)
        ax.annotate(W_SHORT[w], (v, p),
                    textcoords="offset points", xytext=(5, 4),
                    fontsize=10, color="#1a1a1a")

    # ── SVCC ──────────────────────────────────────────────────────────────────
    if svcc_v is not None:
        ax.scatter(svcc_v, svcc_p, marker="D", s=140, color=SVCC_COLOR,
                   zorder=5, edgecolors="white", linewidths=0.6)
        ax.annotate("SVCC", (svcc_v, svcc_p),
                    textcoords="offset points", xytext=(5, 4),
                    fontsize=11, fontweight="bold", color=SVCC_COLOR)

    # ── Actuated: plot in-axis if close, else annotate off-axis ───────────────
    if act_v is not None:
        in_view = (xl[0] <= act_v <= xl[1]) and (yl[0] <= act_p <= yl[1])
        if in_view:
            ax.scatter(act_v, act_p, marker="*", s=280, color=ACT_COLOR,
                       zorder=5, edgecolors="white", linewidths=0.4)
            ax.annotate("Actuated", (act_v, act_p),
                        textcoords="offset points", xytext=(5, 4),
                        fontsize=11, fontweight="bold", color=ACT_COLOR)
        else:
            # Show as off-axis annotation in top-right corner
            ax.annotate(
                f"★ Actuated\n({act_v:.1f}, {act_p:.1f})",
                xy=(0.97, 0.97), xycoords="axes fraction",
                ha="right", va="top", fontsize=10,
                color=ACT_COLOR, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", fc="white",
                          ec=ACT_COLOR, alpha=0.85, linewidth=0.8),
            )

    ax.set_title(title, fontsize=13, fontweight="bold", pad=8)
    ax.set_xlabel("Veh Waiting Time (s)", fontsize=12)
    ax.set_ylabel("Ped Time Loss (s)", fontsize=12)
    ax.tick_params(labelsize=11)
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.6)
    ax.spines[["top", "right"]].set_visible(False)


# ── main figure ────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(15, 9),
                         constrained_layout=True)

for row_idx, (folder, label) in enumerate(SCENARIOS):
    data_dir = os.path.join(BASE, "Results", folder)
    for col_idx, ped in enumerate(PED_LEVELS):
        plot_subplot(axes[row_idx, col_idx], data_dir, ped,
                     f"{label} — {PED_LABEL[ped]}")

# ── legend ────────────────────────────────────────────────────────────────────
legend_elements = [
    Line2D([0], [0], marker="*", color="w", markerfacecolor=ACT_COLOR,
           markersize=16, label="Actuated"),
    Line2D([0], [0], marker="D", color="w", markerfacecolor=SVCC_COLOR,
           markersize=13, label="SVCC (1.0, 0.0)"),
    Line2D([0], [0], marker="o", color="w", markerfacecolor=M2SVCC_COLOR,
           markersize=13, label="M2SVCC (labelled by weight)"),
    Line2D([0], [0], color="#888888", linewidth=2.0, linestyle="--",
           label="Pareto frontier"),
]
fig.legend(handles=legend_elements, loc="lower center", ncol=4,
           fontsize=12, frameon=True, bbox_to_anchor=(0.5, -0.04),
           borderpad=1.0, handlelength=2.5)

fig.suptitle(
    "Pareto Frontier: Vehicle Waiting Time vs Pedestrian Time Loss",
    fontsize=16, fontweight="bold",
)

out_png = os.path.join(BASE, "Results", "fig_pareto.png")
out_pdf = os.path.join(BASE, "Results", "fig_pareto.pdf")
fig.savefig(out_png, dpi=200, bbox_inches="tight")
fig.savefig(out_pdf, bbox_inches="tight")
print(f"Saved: {out_png}")
print(f"Saved: {out_pdf}")
plt.close(fig)
