"""
Altair (Vega-Lite) heatmap table for Concurrent-Symmetric results.

Layout
------
  Rows    : LowPed / MedPed / HighPed  ×  metric
  Columns : Actuated | SVCC (1.0,0.0) | (0.8,0.2) → (0.1,0.9)
  Colour  : % change vs SVCC baseline
              blue  = improvement (lower value)
              red   = regression  (higher value)
              white = SVCC baseline
              grey  = Actuated (no % colouring)

Outputs HTML (interactive) + PNG (static via altair-saver or vl-convert).
"""

import os, re, json
import numpy as np
import pandas as pd
import altair as alt
from scipy import stats

alt.data_transformers.disable_max_rows()

# ── config ─────────────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE, "Results", "Concurrent-Symmetric")

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
SVCC_WEIGHT = "1.0, 0.0"

METRIC_KEYS  = ["fuel", "waiting_time", "time_loss", "queue_length", "ped_time_loss", "conflicts"]
METRIC_LABEL = {
    "waiting_time":  "Veh Waiting Time (s)",
    "time_loss":     "Veh Time Loss (s)",
    "queue_length":  "Queue Length (m)",
    "ped_time_loss": "Ped Time Loss (s)",
    "conflicts":     "Ped-Veh Conflicts (#)",
    "fuel":          "Fuel Consumption (mg)",
}

PARSE_PATTERNS = {
    "waiting_time": r"average waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "time_loss":    r"average time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "queue_length": r"average queue length for (?:multi_scale|actuated) scenario \(in m\): ([\d.]+)",
    "ped_time_loss":r"average pedestrian time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "conflicts":    r"number of right-turn conflicts between vehicles and pedestrians: ([\d.]+)",
    "fuel":         r"average fuel consumption \(external model\) for CAVs (?:multi_scale|actuated) \(in mg\): ([\d.]+)",
}

# ── data loading ───────────────────────────────────────────────────────────────

def parse_file(path):
    with open(path) as f:
        text = f.read()
    return {k: float(m.group(1)) if (m := re.search(p, text)) else None
            for k, p in PARSE_PATTERNS.items()}


def load_weight_per_seed(ped, weight_str):
    """Returns {seed: {metric: value}} for all matching files."""
    pat = re.compile(
        rf"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_"
        rf"\({re.escape(weight_str)}\)_{ped}_seed\((\d+)\)\.txt$"
    )
    result = {}
    for f in os.listdir(DATA_DIR):
        m = pat.match(f)
        if m:
            result[int(m.group(1))] = parse_file(os.path.join(DATA_DIR, f))
    return result


def load_actuated_per_seed(ped):
    """Returns {seed: {metric: value}} for actuated files."""
    pat = re.compile(rf"actuated_penetration\(1\)_EVratio\(0\)_{ped}_seed\((\d+)\)\.txt$")
    result = {}
    for f in os.listdir(DATA_DIR):
        m = pat.match(f)
        if m:
            result[int(m.group(1))] = parse_file(os.path.join(DATA_DIR, f))
    return result


def mean_of_seed_dict(seed_dict):
    out = {}
    for k in METRIC_KEYS:
        vals = [v[k] for v in seed_dict.values() if v and v.get(k) is not None]
        out[k] = float(np.mean(vals)) if vals else None
    return out


def paired_sig(svcc_seed_dict, w_seed_dict, mk):
    """Paired t-test across common seeds. Returns '' / '*' / '**'."""
    common = set(svcc_seed_dict) & set(w_seed_dict)
    a, b = [], []
    for s in common:
        sv = svcc_seed_dict[s].get(mk)
        wv = w_seed_dict[s].get(mk)
        if sv is not None and wv is not None:
            a.append(sv)
            b.append(wv)
    if len(a) < 2:
        return ""
    _, p = stats.ttest_rel(b, a)
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return ""


# ── build long-form DataFrame ──────────────────────────────────────────────────

def build_df():
    rows = []
    for ped in PED_LEVELS:
        svcc_seed = load_weight_per_seed(ped, SVCC_WEIGHT)
        act_seed  = load_actuated_per_seed(ped)
        svcc_vals = mean_of_seed_dict(svcc_seed)
        act_vals  = mean_of_seed_dict(act_seed)

        # Actuated column
        for mk in METRIC_KEYS:
            v = act_vals.get(mk)
            rows.append({
                "ped_level":    PED_LABEL[ped],
                "ped_order":    PED_LEVELS.index(ped),
                "metric":       METRIC_LABEL[mk],
                "metric_order": METRIC_KEYS.index(mk),
                "col_key":        "Actuated",
                "col_order":      -1,
                "value":          round(v, 2) if v is not None else None,
                "pct_change":     None,
                "is_actuated":    True,
                "is_svcc":        False,
                "is_significant": 0,
                "label":          f"{v:.2f}" if v is not None else "N/A",
            })

        # Multi-scale weight columns
        for w_idx, w in enumerate(WEIGHTS_ORDERED):
            w_seed = load_weight_per_seed(ped, w)
            w_vals = mean_of_seed_dict(w_seed)
            for mk in METRIC_KEYS:
                v       = w_vals.get(mk)
                svcc    = svcc_vals.get(mk)
                pct     = ((v - svcc) / svcc * 100) if (v is not None and svcc) else None
                is_svcc = (w == SVCC_WEIGHT)

                if is_svcc:
                    sig   = ""
                    label = f"{v:.2f}" if v is not None else "N/A"
                elif pct is not None:
                    sign  = "+" if pct >= 0 else ""
                    sig   = paired_sig(svcc_seed, w_seed, mk)
                    label = f"{v:.2f}{sig}\n({sign}{pct:.1f}%)"
                else:
                    sig   = ""
                    label = "N/A"

                col_label = "SVCC" if is_svcc else f"({w})"

                rows.append({
                    "ped_level":      PED_LABEL[ped],
                    "ped_order":      PED_LEVELS.index(ped),
                    "metric":         METRIC_LABEL[mk],
                    "metric_order":   METRIC_KEYS.index(mk),
                    "col_key":        col_label,
                    "col_order":      w_idx,
                    "value":          round(v, 2) if v is not None else None,
                    "pct_change":     round(pct, 2) if pct is not None else None,
                    "is_actuated":    False,
                    "is_svcc":        is_svcc,
                    "is_significant": 1 if sig else 0,
                    "label":          label,
                })

    return pd.DataFrame(rows)


# ── Altair chart ───────────────────────────────────────────────────────────────

def make_altair_chart(df):
    # Sort helpers
    ped_sort    = [PED_LABEL[p] for p in PED_LEVELS]
    metric_sort = [METRIC_LABEL[mk] for mk in METRIC_KEYS]
    col_sort    = (["Actuated", "SVCC"] +
                   [f"({w})" for w in WEIGHTS_ORDERED if w != SVCC_WEIGHT])

    # Colour scale: diverging blue-red centred at 0
    pct_max = df["pct_change"].abs().max(skipna=True)
    pct_max = max(float(pct_max) if not np.isnan(pct_max) else 10, 5)

    color_scale = alt.Scale(
        scheme="blueorange",
        domain=[-pct_max, 0, pct_max],
        domainMid=0,
    )

    df = df.copy()
    df["pct_for_color"] = df["pct_change"].fillna(0.0)
    # Numeric flags for transform_filter (booleans become 1/0 in Vega)
    df["is_actuated_int"] = df["is_actuated"].astype(int)
    df["is_svcc_int"]     = df["is_svcc"].astype(int)

    # Use alt.Chart() WITHOUT data so facet handles the data partitioning
    base = alt.Chart().properties(
        width=68 * len(col_sort),
        height=40 * len(metric_sort),
    )

    x_enc = alt.X("col_key:N", sort=col_sort, title=None,
                   axis=alt.Axis(orient="top", labelAngle=0,
                                 labelFontWeight="bold",
                                 labelFontSize=11.5, labelLimit=140))
    y_enc = alt.Y("metric:N", sort=metric_sort, title=None,
                   axis=alt.Axis(labelFontSize=12, labelLimit=200))

    # ── layer 1: actuated cells — white ─────────────────────────────────────────
    rect_act = base.transform_filter(
        "datum.is_actuated_int === 1"
    ).mark_rect(
        color="white", stroke="#bbbbbb", strokeWidth=0.5
    ).encode(
        x=x_enc, y=y_enc,
        tooltip=[
            alt.Tooltip("ped_level:N", title="Ped Level"),
            alt.Tooltip("metric:N",     title="Metric"),
            alt.Tooltip("col_key:N",    title="Controller"),
            alt.Tooltip("value:Q",      title="Value", format=".2f"),
        ],
    )

    # ── layer 2: SVCC cells — light gray ────────────────────────────────────────
    rect_svcc = base.transform_filter(
        "datum.is_svcc_int === 1"
    ).mark_rect(
        color="#d9d9d9", stroke="#bbbbbb", strokeWidth=0.5
    ).encode(
        x=x_enc, y=y_enc,
        tooltip=[
            alt.Tooltip("ped_level:N", title="Ped Level"),
            alt.Tooltip("metric:N",     title="Metric"),
            alt.Tooltip("col_key:N",    title="Controller"),
            alt.Tooltip("value:Q",      title="SVCC baseline", format=".2f"),
        ],
    )

    # ── layer 3: M2SVCC cells — heatmap ─────────────────────────────────────────
    rect_heat = base.transform_filter(
        "datum.is_actuated_int === 0 && datum.is_svcc_int === 0"
    ).mark_rect(
        stroke="#bbbbbb", strokeWidth=0.5
    ).encode(
        x=x_enc, y=y_enc,
        color=alt.Color("pct_for_color:Q",
                         scale=color_scale,
                         legend=alt.Legend(
                             title="% change vs SVCC",
                             titleFontSize=12,
                             titleOrient="left",
                             titleLimit=300,
                             titlePadding=4,
                             labelFontSize=11,
                             orient="bottom",
                             direction="horizontal",
                             gradientLength=300,
                             gradientThickness=14,
                         )),
        tooltip=[
            alt.Tooltip("ped_level:N",  title="Ped Level"),
            alt.Tooltip("metric:N",      title="Metric"),
            alt.Tooltip("col_key:N",     title="Controller"),
            alt.Tooltip("value:Q",       title="Value",     format=".2f"),
            alt.Tooltip("pct_change:Q",  title="% vs SVCC", format="+.1f"),
        ],
    )

    # ── text layers: normal weight and bold (significant) ─────────────────────
    text_normal = base.transform_filter(
        "datum.is_significant === 0"
    ).mark_text(
        fontSize=12, lineBreak="\n", baseline="middle", align="center", dy=-5,
        fontWeight="normal",
    ).encode(
        x=x_enc, y=y_enc,
        text=alt.Text("label:N"),
        color=alt.value("#1a1a1a"),
    )

    text_bold = base.transform_filter(
        "datum.is_significant === 1"
    ).mark_text(
        fontSize=12, lineBreak="\n", baseline="middle", align="center", dy=-5,
        fontWeight="bold",
    ).encode(
        x=x_enc, y=y_enc,
        text=alt.Text("label:N"),
        color=alt.value("#1a1a1a"),
    )

    # ── spanning M2SVCC header strip ──────────────────────────────────────────
    m2svcc_cols = [f"({w})" for w in WEIGHTS_ORDERED if w != SVCC_WEIGHT]
    mid_col     = m2svcc_cols[len(m2svcc_cols) // 2]   # anchor column for text

    hdr_df = pd.DataFrame([
        {"col_key": c,
         "is_m2svcc": c in m2svcc_cols,
         "label": "M2SVCC" if c == mid_col else ""}
        for c in col_sort
    ])

    hdr_x = alt.X("col_key:N", sort=col_sort, title=None,
                   axis=alt.Axis(labels=False, ticks=False,
                                 domain=False, grid=False))

    # Pure text — no rect, no line, no background, no border
    hdr_text = alt.Chart(hdr_df).mark_text(
        fontSize=13, fontWeight="bold", baseline="middle",
        align="center", color="#222222",
    ).encode(
        x=hdr_x,
        text=alt.Text("label:N"),
    )

    hdr_strip = hdr_text.properties(
        width=68 * len(col_sort),
        height=20,
    )

    # ── facet the layered chart ────────────────────────────────────────────────
    main_facet = alt.layer(rect_act, rect_svcc, rect_heat, text_normal, text_bold).facet(
        facet=alt.Facet("ped_level:N",
                         sort=ped_sort,
                         header=alt.Header(
                             title=None,
                             labelFontSize=15,
                             labelFontWeight="bold",
                             labelColor="#222222",
                             labelPadding=8,
                             labelBaseline="middle",
                             labelOrient="right",
                         )),
        data=df,
        columns=1,
    ).resolve_scale(color="shared")

    chart = alt.vconcat(
        hdr_strip, main_facet,
        spacing=0,
    ).properties(
        title=alt.Title(
            text="Concurrent-Symmetric: Performance Heatmap vs SVCC (1.0, 0.0)",
            subtitle=[
                "Mean across seeds 10, 1000, 23423",
                "Blue = improvement over SVCC  |  Orange/Red = regression vs SVCC  |  Grey = Actuated (reference)",
            ],
            fontSize=17,
            subtitleFontSize=13,
            anchor="middle",
            offset=10,
        )
    ).configure_view(
        stroke=None,
        strokeOpacity=0,
        strokeWidth=0,
    ).configure_facet(
        spacing=14,
    ).configure_axis(
        grid=False,
        domainColor="#aaaaaa",
    ).configure_header(
        labelFontSize=13,
        labelFontWeight="bold",
    )

    return chart


# ── save ───────────────────────────────────────────────────────────────────────

def save_chart(chart):
    out_html = os.path.join(BASE, "Results", "fig_symmetric_altair.html")
    chart.save(out_html)
    print(f"Saved (interactive HTML): {out_html}")

    # Try PNG via vl-convert Python API
    try:
        import vl_convert as vlc
        out_png = os.path.join(BASE, "Results", "fig_symmetric_altair.png")
        png_data = vlc.vegalite_to_png(chart.to_json(), scale=2)
        with open(out_png, "wb") as f:
            f.write(png_data)
        print(f"Saved (PNG):  {out_png}")
    except Exception as e:
        print(f"PNG export failed: {e}\nHTML only. Open the HTML in a browser.")


# ── main ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Building data...")
    df = build_df()
    print(f"  {len(df)} rows built. Sample pct_change range: "
          f"{df['pct_change'].min():.1f}% to {df['pct_change'].max():.1f}%")

    print("Building Altair chart...")
    chart = make_altair_chart(df)

    save_chart(chart)
    print("Done.")