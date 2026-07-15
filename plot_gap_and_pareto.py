"""
Figures for journal rebuttal:
  3. Performance Gap Table  – % improvement of M2SVCC over SVCC, color-coded
  4. Pareto Frontier Plot   – Vehicle delay vs Ped delay for Actuated / SVCC / M2SVCC

Definitions
-----------
  SVCC    = multi_scale weight (1.0, 0.0)  [vehicle-only optimisation]
  M2SVCC  = multi_scale weight (0.1, 0.9)  [best multi-modal weight]
  Actuated = actuated controller
              – searched recursively inside Concurrent-{Asymmetric,Symmetric}/

Data sources (user restriction)
---------------------------------
  All data is loaded from:
    Results/Concurrent-Asymmetric/   (including before/ sub-dir if present)
    Results/Concurrent-Symmetric/    (including before/ sub-dir if present)
"""

import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── paths ──────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.abspath(__file__))
SCENARIO_DIRS = {
    "Asymmetric": os.path.join(BASE, "Results", "Concurrent-Asymmetric"),
    "Symmetric":  os.path.join(BASE, "Results", "Concurrent-Symmetric"),
}

PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {"LowPed": "Low Ped", "MedPed": "Med Ped", "HighPed": "High Ped"}

METRICS = {
    "waiting_time": r"average waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "time_loss":    r"average time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "queue_length": r"average queue length for (?:multi_scale|actuated) scenario \(in m\): ([\d.]+)",
    "ped_time_loss":r"average pedestrian time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
}

METRIC_LABEL = {
    "waiting_time": "Veh Waiting\nTime (s)",
    "time_loss":    "Veh Time\nLoss (s)",
    "queue_length": "Queue\nLength (m)",
    "ped_time_loss":"Ped Time\nLoss (s)",
}

# ── helpers ────────────────────────────────────────────────────────────────────

def parse_file(path: str) -> dict:
    with open(path) as fh:
        text = fh.read()
    result = {}
    for key, pattern in METRICS.items():
        m = re.search(pattern, text)
        result[key] = float(m.group(1)) if m else None
    return result


def mean_metric(records: list, key: str):
    vals = [r[key] for r in records if r.get(key) is not None]
    return float(np.mean(vals)) if vals else None


def _all_txt_files(directory: str):
    """Yield (fname, full_path) for all .txt files under directory (recursive)."""
    for root, _, files in os.walk(directory):
        for fname in files:
            if fname.endswith(".txt"):
                yield fname, os.path.join(root, fname)


def load_multi_scale(directory: str, weight_str: str, ped_level: str) -> list:
    """Return list of parsed dicts for all seeds of a given weight × ped_level."""
    pattern = re.compile(
        rf"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_"
        rf"\({re.escape(weight_str)}\)_{ped_level}_seed\((\d+)\)\.txt$"
    )
    records = []
    if not os.path.isdir(directory):
        return records
    for fname, fpath in _all_txt_files(directory):
        if pattern.match(fname):
            records.append(parse_file(fpath))
    return records


def load_actuated(directory: str, ped_level: str) -> list:
    """Return list of parsed dicts for actuated files, searching directory recursively."""
    pattern = re.compile(
        rf"actuated_penetration\(1\)_EVratio\(0\)_{ped_level}_seed\((\d+)\)\.txt$"
    )
    records = []
    if not os.path.isdir(directory):
        return records
    for fname, fpath in _all_txt_files(directory):
        if pattern.match(fname):
            records.append(parse_file(fpath))
    return records


# ── aggregate data ─────────────────────────────────────────────────────────────

def collect(scenario: str) -> dict:
    """
    Returns dict keyed by ped_level → {"svcc", "m2svcc", "actuated"} → {metric: mean}
    """
    directory = SCENARIO_DIRS[scenario]
    data = {}
    for ped in PED_LEVELS:
        svcc_rec   = load_multi_scale(directory, "1.0, 0.0", ped)
        m2svcc_rec = load_multi_scale(directory, "0.1, 0.9", ped)
        act_rec    = load_actuated(directory, ped)

        data[ped] = {
            "svcc":    {k: mean_metric(svcc_rec,   k) for k in METRICS},
            "m2svcc":  {k: mean_metric(m2svcc_rec, k) for k in METRICS},
            "actuated":{k: mean_metric(act_rec,    k) for k in METRICS},
            "n_svcc":   len(svcc_rec),
            "n_m2svcc": len(m2svcc_rec),
            "n_act":    len(act_rec),
        }
    return data


# ── Figure 3: Performance Gap Table ───────────────────────────────────────────

def pct_improvement(baseline_val, new_val):
    """Positive = improvement (new_val < baseline_val), negative = regression."""
    if baseline_val is None or new_val is None or baseline_val == 0:
        return None
    return (baseline_val - new_val) / baseline_val * 100


def plot_gap_table():
    scenarios  = ["Asymmetric", "Symmetric"]
    metric_keys = list(METRICS.keys())

    cell_text   = []
    cell_vals   = []
    row_labels  = []
    row_sublabels = []

    for scenario in scenarios:
        data = collect(scenario)
        for ped in PED_LEVELS:
            n_s = data[ped]["n_svcc"]
            n_m = data[ped]["n_m2svcc"]
            row_labels.append(f"{scenario}\n{PED_LABEL[ped]}")
            row_sublabels.append(f"(n={n_s}/{n_m})")
            row_text = []
            row_val  = []
            for mk in metric_keys:
                svcc_v   = data[ped]["svcc"][mk]
                m2svcc_v = data[ped]["m2svcc"][mk]
                pct = pct_improvement(svcc_v, m2svcc_v)
                if pct is None:
                    row_text.append("N/A")
                    row_val.append(0.0)
                else:
                    row_text.append(f"{pct:+.1f}%")
                    row_val.append(pct)
            cell_text.append(row_text)
            cell_vals.append(row_val)

    col_labels  = [METRIC_LABEL[mk] for mk in metric_keys]
    nrows = len(row_labels)

    fig, ax = plt.subplots(figsize=(11, 0.70 * nrows + 2.2))
    ax.axis("off")

    all_vals = np.array(cell_vals, dtype=float)
    max_abs  = max(float(np.abs(all_vals).max()), 1.0)

    cell_colors = []
    for row_vals in cell_vals:
        row_colors = []
        for v in row_vals:
            if v > 0:
                intensity = min(v / max_abs, 1.0)
                color = (1 - 0.50 * intensity, 1.0, 1 - 0.50 * intensity, 1.0)
            elif v < 0:
                intensity = min(-v / max_abs, 1.0)
                color = (1.0, 1 - 0.50 * intensity, 1 - 0.50 * intensity, 1.0)
            else:
                color = (0.96, 0.96, 0.96, 1.0)
            row_colors.append(color)
        cell_colors.append(row_colors)

    # Add sample-size info to row labels
    combined_labels = [f"{rl}\n{rs}" for rl, rs in zip(row_labels, row_sublabels)]

    table = ax.table(
        cellText=cell_text,
        rowLabels=combined_labels,
        colLabels=col_labels,
        cellColours=cell_colors,
        cellLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 2.0)

    for (r, c), cell in table.get_celld().items():
        if r == 0:
            cell.set_text_props(fontweight="bold")
            cell.set_facecolor("#e0e0e0")
        cell.set_edgecolor("#aaaaaa")

    # Visual separator between Asymmetric and Symmetric rows
    sep_row = len(PED_LEVELS) + 1   # +1 for header row
    for c in range(len(metric_keys)):
        cell = table[sep_row, c]
        cell.visible_edges = "TBL" if c == 0 else ("TBR" if c == len(metric_keys)-1 else "TB")
        cell.set_linewidth(2.0)

    green_patch = mpatches.Patch(facecolor=(0.50, 1.0, 0.50), label="Improvement  (+%)")
    red_patch   = mpatches.Patch(facecolor=(1.0, 0.50, 0.50), label="Regression   (−%)")
    ax.legend(
        handles=[green_patch, red_patch],
        loc="lower right",
        bbox_to_anchor=(1.0, -0.02),
        fontsize=9,
        framealpha=0.8,
    )

    ax.set_title(
        "Performance Gap: M2SVCC vs SVCC (w=(0.1,0.9) vs w=(1.0,0.0))\n"
        "Positive = M2SVCC improves over SVCC  |  n = seeds(SVCC)/seeds(M2SVCC)",
        fontsize=11, fontweight="bold", pad=14,
    )

    out_pdf = os.path.join(BASE, "Results", "fig3_performance_gap_table.pdf")
    out_png = os.path.join(BASE, "Results", "fig3_performance_gap_table.png")
    fig.savefig(out_pdf, bbox_inches="tight", dpi=200)
    fig.savefig(out_png, bbox_inches="tight", dpi=200)
    print(f"Saved: {out_pdf}")
    print(f"Saved: {out_png}")
    plt.close(fig)


# ── Figure 4: Pareto Frontier Plot ────────────────────────────────────────────

CTRL_STYLE = {
    "Actuated": {"marker": "s", "color": "#d62728", "s": 110, "zorder": 3, "label": "Actuated"},
    "SVCC":     {"marker": "^", "color": "#1f77b4", "s": 110, "zorder": 3, "label": "SVCC (1.0, 0.0)"},
    "M2SVCC":   {"marker": "o", "color": "#2ca02c", "s": 130, "zorder": 4, "label": "M2SVCC (0.1, 0.9)"},
}

PED_ALPHA = {"LowPed": 0.60, "MedPed": 0.80, "HighPed": 1.00}


def plot_pareto():
    scenarios = ["Asymmetric", "Symmetric"]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    for ax, scenario in zip(axes, scenarios):
        data = collect(scenario)

        # Plot each ped level
        for ped in PED_LEVELS:
            d    = data[ped]
            alpha = PED_ALPHA[ped]
            pts   = {
                "Actuated": (d["actuated"]["time_loss"], d["actuated"]["ped_time_loss"]),
                "SVCC":     (d["svcc"]["time_loss"],     d["svcc"]["ped_time_loss"]),
                "M2SVCC":   (d["m2svcc"]["time_loss"],   d["m2svcc"]["ped_time_loss"]),
            }

            # Connect same-ped-level points (dashed grey trajectory)
            conn_x = [pts[c][0] for c in ("Actuated","SVCC","M2SVCC")
                      if pts[c][0] is not None]
            conn_y = [pts[c][1] for c in ("Actuated","SVCC","M2SVCC")
                      if pts[c][1] is not None]
            if len(conn_x) >= 2:
                ax.plot(conn_x, conn_y, color="#aaaaaa", linewidth=0.9,
                        linestyle="--", zorder=1, alpha=0.7)

            for ctrl_name, (vd, pd_) in pts.items():
                if vd is None or pd_ is None:
                    continue
                sty = CTRL_STYLE[ctrl_name]
                ax.scatter(
                    vd, pd_,
                    marker=sty["marker"],
                    s=sty["s"],
                    color=sty["color"],
                    alpha=alpha,
                    edgecolors="white",
                    linewidths=0.7,
                    zorder=sty["zorder"],
                )
                ax.annotate(
                    PED_LABEL[ped].replace(" Ped", ""),
                    (vd, pd_),
                    textcoords="offset points",
                    xytext=(5, 4),
                    fontsize=10,
                    color=sty["color"],
                    alpha=min(alpha + 0.2, 1.0),
                )

        # Highlight M2SVCC Pareto frontier
        m2svcc_pts = sorted(
            [(data[ped]["m2svcc"]["time_loss"], data[ped]["m2svcc"]["ped_time_loss"])
             for ped in PED_LEVELS
             if data[ped]["m2svcc"]["time_loss"] is not None
                and data[ped]["m2svcc"]["ped_time_loss"] is not None],
            key=lambda p: p[0],
        )
        if len(m2svcc_pts) >= 2:
            ax.plot(
                [p[0] for p in m2svcc_pts],
                [p[1] for p in m2svcc_pts],
                color="#2ca02c", linewidth=2.0,
                linestyle="-", zorder=2, alpha=0.55,
            )

        ax.set_xlabel("Vehicle Time Loss (s)", fontsize=14)
        ax.set_ylabel("Pedestrian Time Loss (s)", fontsize=14)
        ax.tick_params(labelsize=12)
        ax.set_title(f"Concurrent-{scenario}", fontsize=15, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.45)

        # "Better" annotation arrow
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        xrange = xlim[1] - xlim[0]
        yrange = ylim[1] - ylim[0]
        ax.annotate(
            "",
            xy=(xlim[0] + xrange * 0.06, ylim[0] + yrange * 0.06),
            xytext=(xlim[0] + xrange * 0.22, ylim[0] + yrange * 0.22),
            arrowprops=dict(arrowstyle="->", color="#555555", lw=1.2),
        )
        ax.text(
            xlim[0] + xrange * 0.23, ylim[0] + yrange * 0.23,
            "Better\n(both lower)", fontsize=11, color="#555555",
            va="bottom",
        )

    # Shared legend: controllers + ped levels
    ctrl_handles = [
        plt.scatter([], [], marker=CTRL_STYLE[c]["marker"],
                    color=CTRL_STYLE[c]["color"], s=80,
                    label=CTRL_STYLE[c]["label"])
        for c in ("Actuated", "SVCC", "M2SVCC")
    ]
    ped_handles = [
        mpatches.Patch(facecolor="none", edgecolor="#555555",
                       label=f"{PED_LABEL[p]}  (α={PED_ALPHA[p]:.2f})")
        for p in PED_LEVELS
    ]
    axes[-1].legend(
        handles=ctrl_handles + ped_handles,
        loc="upper right",
        fontsize=12,
        title="Controller / Ped Level",
        title_fontsize=13,
        framealpha=0.88,
        borderpad=1.0,
        handlelength=2.5,
        handletextpad=0.8,
    )

    fig.suptitle(
        "Pareto Frontier: Vehicle Delay vs Pedestrian Delay\n"
        "(averaged across seeds; M2SVCC targets both lower vehicle and lower pedestrian delay)",
        fontsize=14, fontweight="bold", y=1.01,
    )
    fig.tight_layout()

    out_pdf = os.path.join(BASE, "Results", "fig4_pareto_frontier.pdf")
    out_png = os.path.join(BASE, "Results", "fig4_pareto_frontier.png")
    fig.savefig(out_pdf, bbox_inches="tight", dpi=200)
    fig.savefig(out_png, bbox_inches="tight", dpi=200)
    print(f"Saved: {out_pdf}")
    print(f"Saved: {out_png}")
    plt.close(fig)


# ── diagnostic summary ─────────────────────────────────────────────────────────

def print_summary():
    for scenario in ("Asymmetric", "Symmetric"):
        print(f"\n{'─'*60}")
        print(f"Scenario: {scenario}")
        data = collect(scenario)
        for ped in PED_LEVELS:
            d = data[ped]
            print(f"  {ped}:  n_svcc={d['n_svcc']}  n_m2svcc={d['n_m2svcc']}  n_act={d['n_act']}")
            for mk in ("time_loss", "ped_time_loss"):
                sv = d["svcc"][mk]
                mm = d["m2svcc"][mk]
                ac = d["actuated"][mk]
                print(f"    {mk:<15}  SVCC={sv}  M2SVCC={mm}  Actuated={ac}")


# ── main ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print_summary()
    print()
    print("=== Figure 3: Performance Gap Table ===")
    plot_gap_table()
    print()
    print("=== Figure 4: Pareto Frontier Plot ===")
    plot_pareto()
    print()
    print("Done.")