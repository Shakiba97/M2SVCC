"""
Comprehensive heatmap table for Concurrent-Symmetric results.

Layout
------
  Big rows    : LowPed / MedPed / HighPed
  Sub-rows    : one per metric (averaged across seeds 10, 1000, 23423)
  Columns     : Actuated | SVCC (1.0,0.0) | (0.9,0.1) | … | (0.1,0.9)
  Colour      : thermal blue→red heatmap of % change vs SVCC (1.0,0.0)
                  blue  = improvement (value lower than SVCC)
                  red   = regression  (value higher than SVCC)
                  white = SVCC itself (baseline)
                  grey  = Actuated (reference, no % change colouring)
"""

import os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import TwoSlopeNorm
import matplotlib.cm as cm

# ── config ─────────────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE, "Results", "Concurrent-Symmetric")
SEEDS    = [10, 1000, 23423]

PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {"LowPed": "Low Ped", "MedPed": "Med Ped", "HighPed": "High Ped"}

# Ordered weight columns (left → right: vehicle-heavy → ped-heavy)
# These are the inner strings as they appear in filenames: Concurrent_(w1, w2)
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
SVCC_WEIGHT = "1.0, 0.0"

METRICS = {
    "waiting_time": "Veh Wait\n(s)",
    "time_loss":    "Veh Time\nLoss (s)",
    "queue_length": "Queue\n(m)",
    "ped_time_loss":"Ped Time\nLoss (s)",
}
METRIC_KEYS = list(METRICS.keys())

PARSE_PATTERNS = {
    "waiting_time": r"average waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "time_loss":    r"average time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "queue_length": r"average queue length for (?:multi_scale|actuated) scenario \(in m\): ([\d.]+)",
    "ped_time_loss":r"average pedestrian time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
}

# ── data loading ───────────────────────────────────────────────────────────────

def parse_file(path):
    with open(path) as f:
        text = f.read()
    return {k: float(m.group(1)) if (m := re.search(p, text)) else None
            for k, p in PARSE_PATTERNS.items()}


def mean_across_seeds(records):
    out = {}
    for k in METRIC_KEYS:
        vals = [r[k] for r in records if r.get(k) is not None]
        out[k] = float(np.mean(vals)) if vals else None
    return out


def load_weight(ped, weight_str):
    """Load multi_scale files for a given ped level + weight, average over seeds."""
    pat = re.compile(
        rf"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_"
        rf"\({re.escape(weight_str)}\)_{ped}_seed\((\d+)\)\.txt$"
    )
    recs = [parse_file(os.path.join(DATA_DIR, f))
            for f in os.listdir(DATA_DIR) if pat.match(f)]
    return mean_across_seeds(recs)


def load_actuated(ped):
    pat = re.compile(rf"actuated_penetration\(1\)_EVratio\(0\)_{ped}_seed\((\d+)\)\.txt$")
    recs = [parse_file(os.path.join(DATA_DIR, f))
            for f in os.listdir(DATA_DIR) if pat.match(f)]
    return mean_across_seeds(recs)


def build_data():
    """Returns data[ped][col_label][metric] = mean value."""
    data = {}
    for ped in PED_LEVELS:
        data[ped] = {}
        data[ped]["Actuated"] = load_actuated(ped)
        for w in WEIGHTS_ORDERED:
            data[ped][w] = load_weight(ped, w)
    return data


# ── figure ─────────────────────────────────────────────────────────────────────

def make_table():
    data = build_data()

    # Column order: Actuated first, then weights left→right
    col_keys   = ["Actuated"] + WEIGHTS_ORDERED
    col_labels = ["Actuated"] + [f"({w})" for w in WEIGHTS_ORDERED]
    # Nicer label for SVCC
    svcc_idx   = col_keys.index(SVCC_WEIGHT)
    col_labels[svcc_idx] = "SVCC\n(1.0,0.0)"

    n_metrics  = len(METRIC_KEYS)
    n_ped      = len(PED_LEVELS)
    n_cols     = len(col_keys)

    # Each big row = n_metrics sub-rows; add 1 header sub-row per ped block
    rows_per_ped = n_metrics + 1   # +1 for the ped-level header row
    total_rows   = n_ped * rows_per_ped

    # ── collect cell text & raw % values ─────────────────────────────────────
    cell_text   = []   # list of row-lists of strings
    cell_vals   = []   # list of row-lists of floats (NaN = no colour)
    row_meta    = []   # list of (row_type, ped, metric_key)

    for ped in PED_LEVELS:
        svcc_data = data[ped][SVCC_WEIGHT]

        # ped-level header row
        cell_text.append([""] * n_cols)
        cell_vals.append([np.nan] * n_cols)
        row_meta.append(("header", ped, None))

        for mk in METRIC_KEYS:
            row_t = []
            row_v = []
            for ck in col_keys:
                val = data[ped][ck].get(mk) if data[ped][ck] else None
                svcc_val = svcc_data.get(mk)

                if val is None:
                    row_t.append("N/A")
                    row_v.append(np.nan)
                elif ck == "Actuated":
                    row_t.append(f"{val:.2f}")
                    row_v.append(np.nan)          # grey, no heatmap
                elif ck == SVCC_WEIGHT:
                    row_t.append(f"{val:.2f}")
                    row_v.append(0.0)             # baseline → white
                else:
                    pct = (val - svcc_val) / svcc_val * 100 if svcc_val else np.nan
                    sign = "+" if pct >= 0 else ""
                    row_t.append(f"{val:.2f}\n({sign}{pct:.1f}%)")
                    row_v.append(pct)

            cell_text.append(row_t)
            cell_vals.append(row_v)
            row_meta.append(("metric", ped, mk))

    # ── colour mapping ────────────────────────────────────────────────────────
    # Collect all finite % values to set symmetric colour scale
    all_pct = [v for row in cell_vals for v in row
               if np.isfinite(v) and v != 0.0]
    vmax = max(abs(np.array(all_pct)).max(), 1.0) if all_pct else 10.0
    norm = TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    cmap = cm.RdBu_r    # blue = negative (improvement), red = positive (regression)

    def val_to_rgba(v, row_type, col_key):
        if row_type == "header":
            return (0.25, 0.25, 0.25, 1.0)   # dark grey header
        if col_key == "Actuated":
            return (0.88, 0.88, 0.88, 1.0)   # light grey
        if not np.isfinite(v):
            return (0.96, 0.96, 0.96, 1.0)   # very light grey (N/A)
        if v == 0.0:
            return (1.0, 1.0, 1.0, 1.0)       # white for SVCC baseline
        return cmap(norm(v))

    # ── figure size ───────────────────────────────────────────────────────────
    fig_w = max(18, n_cols * 1.55)
    fig_h = max(10, total_rows * 0.60 + 2.5)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.axis("off")

    # ── build matplotlib table ────────────────────────────────────────────────
    table = ax.table(
        cellText=cell_text,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 2.1)

    # Apply colours & styling
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:
            # column header row
            cell.set_facecolor("#2c2c2c")
            cell.set_text_props(color="white", fontweight="bold", fontsize=8)
            if col_keys[c] == SVCC_WEIGHT:
                cell.set_facecolor("#1a5276")   # dark blue for SVCC header
            elif col_keys[c] == "Actuated":
                cell.set_facecolor("#4a4a4a")
            continue

        data_row  = r - 1   # 0-indexed into cell_text / cell_vals / row_meta
        if data_row >= len(row_meta):
            continue

        rtype, ped, mk = row_meta[data_row]
        col_key = col_keys[c] if c < len(col_keys) else None

        rgba = val_to_rgba(cell_vals[data_row][c], rtype, col_key)
        cell.set_facecolor(rgba)

        if rtype == "header":
            cell.set_text_props(color="white", fontweight="bold", fontsize=9)
            if c == 0:
                cell.get_text().set_text(PED_LABEL[ped])
        else:
            # metric label in col -1 (row label) not available; style value cells
            is_dark = (rtype == "header")
            cell.set_text_props(fontsize=7.5)

    # Row labels (metric names) on left — inject into col 0 isn't ideal with
    # matplotlib table; instead mark the first col with metric short label
    # by overlaying text annotations
    # Get the bounding box of the table to place row labels
    # We'll add a small left column manually via text annotations after rendering

    # ── add ped-level thick borders ───────────────────────────────────────────
    for i, (rtype, ped, mk) in enumerate(row_meta):
        if rtype == "header":
            r_idx = i + 1   # +1 for the col header
            for c in range(n_cols):
                cell = table[r_idx, c]
                cell.visible_edges = "TBL" if c == 0 else ("TBR" if c == n_cols - 1 else "TB")
                cell.set_linewidth(1.5)

    # ── metric sub-labels on the left ─────────────────────────────────────────
    # We repurpose the first-column cells in metric rows to show metric name
    for i, (rtype, ped, mk) in enumerate(row_meta):
        if rtype == "metric":
            r_idx = i + 1
            cell = table[r_idx, 0]
            orig_text = cell.get_text().get_text()
            cell.get_text().set_text(METRICS[mk] + "\n" + orig_text)
            cell.set_text_props(fontsize=7)

    # ── title & legend ────────────────────────────────────────────────────────
    ax.set_title(
        "Concurrent-Symmetric: Mean Performance across Seeds (10, 1000, 23423)\n"
        "Heatmap = % change vs SVCC (1.0,0.0)  │  Blue = improvement  │  Red = regression",
        fontsize=11, fontweight="bold", pad=16,
    )

    # Colorbar
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, orientation="vertical",
                        fraction=0.012, pad=0.01, aspect=30)
    cbar.set_label("% change vs SVCC (1.0,0.0)\n(−=better, +=worse)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    out_pdf = os.path.join(BASE, "Results", "fig_symmetric_table.pdf")
    out_png = os.path.join(BASE, "Results", "fig_symmetric_table.png")
    fig.savefig(out_pdf, bbox_inches="tight", dpi=200)
    fig.savefig(out_png, bbox_inches="tight", dpi=200)
    print(f"Saved: {out_pdf}")
    print(f"Saved: {out_png}")
    plt.close(fig)


if __name__ == "__main__":
    make_table()
    print("Done.")