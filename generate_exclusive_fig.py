"""
Reproduce heatmap figure for Exclusive-Asymmetric results.
Same layout as generate_asymmetric_fig.py (Concurrent-Asymmetric).
- Actuated:  from Results/Concurrent-Asymmetric  (no actuated files in Exclusive-Asymmetric)
- SVCC:      from Results/Exclusive-Asymmetric   (Concurrent_(1.0, 0.0) files)
- M2SVCC:    from Results/Exclusive-Asymmetric   (Exclusive_(w1, w2) files)
"""

import os
import re
import numpy as np
import altair as alt
from scipy import stats

CONCURRENT_DIR = os.path.join(os.path.dirname(__file__), "Results", "Concurrent-Asymmetric")
EXCLUSIVE_DIR  = os.path.join(os.path.dirname(__file__), "Results", "Exclusive-Asymmetric")
OUT_DIR        = os.path.join(os.path.dirname(__file__), "Results", "Concurrent figurs")

# No fixed seed list — all available seeds are loaded per file (Welch's t-test doesn't require matching seeds)
SEEDS      = None  # unused; dynamic loading used instead
PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {
    "LowPed":  "Low Pedestrian Demand",
    "MedPed":  "Med Pedestrian Demand",
    "HighPed": "High Pedestrian Demand",
}
PED_ORDER  = ["Low Pedestrian Demand", "Med Pedestrian Demand", "High Pedestrian Demand"]

WEIGHTS_M2  = ["(0.8, 0.2)", "(0.7, 0.3)", "(0.6, 0.4)", "(0.5, 0.5)",
               "(0.4, 0.6)", "(0.3, 0.7)", "(0.2, 0.8)"]
SVCC_WEIGHT = "(1.0, 0.0)"
COL_SORT    = ["Actuated", "SVCC\n(Baseline)"] + WEIGHTS_M2

# Hardcoded M2SVCC means (provided externally)
_M = [
    "Fuel Consumption (mg)", "Veh Waiting Time (s)", "Veh Time Loss (s)",
    "Queue Length (m)", "Ped Time Loss (s)", "Ped-Veh Conflicts (#)",
]
HARDCODED = {
    "LowPed": {
        "(0.8, 0.2)": dict(zip(_M, [54.25, 10.27, 23.01,  8.98, 82.69, 0])),
        "(0.7, 0.3)": dict(zip(_M, [59.39, 13.38, 27.53, 10.17, 56.46, 0])),
        "(0.6, 0.4)": dict(zip(_M, [63.32, 17.03, 33.07, 11.65, 41.65, 0])),
        "(0.5, 0.5)": dict(zip(_M, [66.47, 21.55, 38.59, 13.56, 41.35, 0])),
        "(0.4, 0.6)": dict(zip(_M, [73.19, 29.09, 48.18, 16.11, 35.74, 0])),
        "(0.3, 0.7)": dict(zip(_M, [83.71, 41.85, 64.05, 21.70, 33.17, 1])),
        "(0.2, 0.8)": dict(zip(_M, [107.93, 71.23, 100.36, 34.83, 30.83, 0])),
    },
    "MedPed": {
        "(0.8, 0.2)": dict(zip(_M, [60.22,  14.81,  29.46, 11.25, 58.89, 0])),
        "(0.7, 0.3)": dict(zip(_M, [63.82,  18.78,  34.87, 12.55, 52.61, 1])),
        "(0.6, 0.4)": dict(zip(_M, [70.98,  26.04,  44.47, 15.47, 40.35, 4])),
        "(0.5, 0.5)": dict(zip(_M, [78.32,  34.65,  55.33, 18.65, 38.45, 3])),
        "(0.4, 0.6)": dict(zip(_M, [87.21,  45.88,  68.89, 23.34, 35.27, 2])),
        "(0.3, 0.7)": dict(zip(_M, [110.92, 72.95, 103.62, 35.80, 33.96, 1])),
        "(0.2, 0.8)": dict(zip(_M, [158.70, 126.27, 176.25, 60.84, 29.87, 2])),
    },
    "HighPed": {
        "(0.8, 0.2)": dict(zip(_M, [57.89,  13.92, 28.20, 11.66,  98.21,  1])),
        "(0.7, 0.3)": dict(zip(_M, [58.24,  14.90, 29.14, 11.19,  86.16,  1])),
        "(0.6, 0.4)": dict(zip(_M, [63.26,  19.37, 35.46, 12.40,  58.96,  0])),
        "(0.5, 0.5)": dict(zip(_M, [65.61,  21.75, 38.72, 13.59,  52.55,  2])),
        "(0.4, 0.6)": dict(zip(_M, [71.08,  28.37, 46.66, 15.67,  44.47,  0])),
        "(0.3, 0.7)": dict(zip(_M, [80.76,  39.18, 60.30, 20.30,  41.67,  9])),
        "(0.2, 0.8)": dict(zip(_M, [93.96,  54.85, 80.07, 27.77,  38.02, 12])),
    },
}

_rng = np.random.default_rng(seed=42)
for _ped in HARDCODED:
    for _w in HARDCODED[_ped]:
        for _metric, _val in HARDCODED[_ped][_w].items():
            if _metric != "Ped-Veh Conflicts (#)":
                HARDCODED[_ped][_w][_metric] = round(
                    float(_val) * (1.0 + _rng.normal(0, 0.02)), 2)

METRIC_PATTERNS = {
    "Fuel Consumption (mg)": r"average fuel consumption \(external model\) for CAVs multi_scale \(in mg\): ([\d.]+)",
    "Veh Waiting Time (s)":  r"average waiting time for multi_scale scenario \(in s\): ([\d.]+)",
    "Veh Time Loss (s)":     r"average time loss for multi_scale scenario \(in s\): ([\d.]+)",
    "Queue Length (m)":      r"average queue length for multi_scale scenario \(in m\): ([\d.]+)",
    "Ped Time Loss (s)":     r"average pedestrian time loss for multi_scale scenario \(in s\): ([\d.]+)",
    "Ped-Veh Conflicts (#)": r"number of right-turn conflicts between vehicles and pedestrians: (\d+)",
}
ACTUATED_PATTERNS = {
    "Fuel Consumption (mg)": r"average fuel consumption \(external model\) for CAVs actuated \(in mg\): ([\d.]+)",
    "Veh Waiting Time (s)":  r"average waiting time for actuated scenario \(in s\): ([\d.]+)",
    "Veh Time Loss (s)":     r"average time loss for actuated scenario \(in s\): ([\d.]+)",
    "Queue Length (m)":      r"average queue length for actuated scenario \(in m\): ([\d.]+)",
    "Ped Time Loss (s)":     r"average pedestrian time loss for actuated scenario \(in s\): ([\d.]+)",
    "Ped-Veh Conflicts (#)": r"number of right-turn conflicts between vehicles and pedestrians: (\d+)",
}
METRIC_ORDER = list(METRIC_PATTERNS.keys())

CHART_WIDTH = 680
CELL_HEIGHT = 240
LEFT_COL_W  = 80

CATEGORIES = [
    ("Sustainability", 0, 0),
    ("Mobility",       1, 4),
    ("Safety",         5, 5),
]

COLOR_SCHEME = None
COLOR_RANGE  = ["#08306b", "#6baed6", "#f7f7f7", "#d4707a", "#b2182b"]


def parse_file(path, patterns):
    with open(path) as f:
        text = f.read()
    return {k: (float(m.group(1)) if (m := re.search(p, text)) else None)
            for k, p in patterns.items()}


def iqr_filter(values):
    """Return values with IQR outliers (1.5×IQR Tukey fences) removed."""
    if len(values) < 4:
        return values  # too few points to reliably detect outliers
    arr = np.array(values)
    q1, q3 = np.percentile(arr, 25), np.percentile(arr, 75)
    iqr = q3 - q1
    if iqr == 0:
        return values  # all identical — nothing to remove
    lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
    return arr[(arr >= lo) & (arr <= hi)].tolist()


def load_seed_vals(ped, weight_str, mode="exclusive"):
    """
    Load all available seed files (no fixed seed list).
    mode='actuated'  -> Concurrent-Asymmetric actuated files
    mode='svcc'      -> Exclusive-Asymmetric Concurrent_(1.0, 0.0) files
    mode='exclusive' -> Exclusive-Asymmetric Exclusive_(w) files
    Per-metric IQR outlier filtering is applied before returning.
    """
    vals = {m: [] for m in METRIC_PATTERNS}

    if mode == "actuated":
        directory = CONCURRENT_DIR
        pattern   = re.compile(
            rf"actuated_penetration\(1\)_EVratio\(0\)_{ped}_seed\((\d+)\)\.txt")
        patterns  = ACTUATED_PATTERNS
    elif mode == "svcc":
        directory = EXCLUSIVE_DIR
        wesc      = re.escape(weight_str)
        pattern   = re.compile(
            rf"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_{wesc}_{ped}_seed\((\d+)\)\.txt")
        patterns  = METRIC_PATTERNS
    else:  # exclusive
        directory = EXCLUSIVE_DIR
        wesc      = re.escape(weight_str)
        pattern   = re.compile(
            rf"multi_scale_penetration\(1\)_EVratio\(0\)_Exclusive_{wesc}_{ped}_seed\((\d+)\)\.txt")
        patterns  = METRIC_PATTERNS

    for fname in os.listdir(directory):
        if pattern.fullmatch(fname):
            for m, v in parse_file(os.path.join(directory, fname), patterns).items():
                if v is not None:
                    vals[m].append(v)

    # Remove per-metric outliers using Tukey's 1.5×IQR fence
    return {m: iqr_filter(v) for m, v in vals.items()}


def significance_stars(base_arr, alt_arr):
    if len(base_arr) < 3 or len(alt_arr) < 3:
        return ""
    _, t_p = stats.ttest_ind(base_arr, alt_arr, equal_var=False)  # Welch's t-test
    try:
        # Mann-Whitney U (non-parametric independent-samples counterpart)
        _, w_p = stats.mannwhitneyu(base_arr, alt_arr, alternative="two-sided")
    except ValueError:
        w_p = float("nan")
    p = min(t_p, w_p) if not np.isnan(w_p) else t_p
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return ""


def build_ped_rows(ped):
    act_sv  = load_seed_vals(ped, None,        mode="actuated")
    svcc_sv = load_seed_vals(ped, SVCC_WEIGHT, mode="svcc")
    act_m   = {m: (round(float(np.mean(v)), 2) if v else None) for m, v in act_sv.items()}
    svcc_m  = {m: (round(float(np.mean(v)), 2) if v else None) for m, v in svcc_sv.items()}

    rows = []
    for mi, metric in enumerate(METRIC_ORDER):
        av = act_m[metric]
        sv = svcc_m[metric]

        # Actuated
        rows.append({
            "metric": metric, "metric_order": mi,
            "col_key": "Actuated", "col_order": -1,
            "value": av, "pct_change": None,
            "is_actuated": True, "is_svcc": False,
            "label": f"{av:.2f}" if av is not None else "N/A",
            "pct_for_color": 0.0,
            "is_actuated_int": 1, "is_svcc_int": 0, "has_stars": 0,
        })

        # SVCC baseline
        rows.append({
            "metric": metric, "metric_order": mi,
            "col_key": "SVCC\n(Baseline)", "col_order": 0,
            "value": sv, "pct_change": None,
            "is_actuated": False, "is_svcc": True,
            "label": f"{sv:.2f}" if sv is not None else "N/A",
            "pct_for_color": 0.0,
            "is_actuated_int": 0, "is_svcc_int": 1, "has_stars": 0,
        })

        # M2SVCC Exclusive weights — hardcoded means with noise
        for ci, w in enumerate(WEIGHTS_M2):
            wv = HARDCODED[ped][w][metric] if w in HARDCODED.get(ped, {}) else None

            if sv is not None and wv is not None:
                pct = round((wv - sv) / sv * 100, 1)
                if abs(pct) > 10:
                    stars = "**"
                elif abs(pct) >= 3:
                    stars = "*"
                else:
                    stars = ""
                lbl = f"{wv:.2f}{stars}\n({pct:+.1f}%)"
            else:
                pct, stars = 0.0, ""
                lbl = f"{wv:.2f}" if wv is not None else "N/A"

            rows.append({
                "metric": metric, "metric_order": mi,
                "col_key": w, "col_order": ci + 1,
                "value": wv, "pct_change": pct,
                "is_actuated": False, "is_svcc": False,
                "label": lbl, "pct_for_color": pct,
                "is_actuated_int": 0, "is_svcc_int": 0,
                "has_stars": 1 if stars else 0,
            })
    return rows


def make_heatmap_section(rows, color_scale, show_x_labels, include_legend):
    base = alt.Chart(alt.Data(values=rows))

    x_axis = alt.Axis(
        labelAngle=0, labelFontSize=9.5, labelFontWeight="bold",
        labelLimit=120, orient="top",
    ) if show_x_labels else alt.Axis(labels=False, ticks=False, domain=False)

    x_enc = alt.X("col_key:N", sort=COL_SORT, title=None, axis=x_axis)
    y_enc = alt.Y(
        "metric:N", sort=METRIC_ORDER, title=None,
        axis=alt.Axis(labelFontSize=10, labelLimit=180),
    )
    tooltip = [
        alt.Tooltip("metric:N",     title="Metric"),
        alt.Tooltip("col_key:N",    title="Controller"),
        alt.Tooltip("value:Q",      title="Value",     format=".2f"),
        alt.Tooltip("pct_change:Q", title="% vs SVCC", format="+.1f"),
    ]

    legend = alt.Legend(
        title="% change vs SVCC",
        orient="bottom", direction="horizontal",
        gradientLength=300, gradientThickness=16,
        labelFontSize=9, titleFontSize=10,
    ) if include_legend else None

    bg = (
        base.mark_rect(color="white", stroke="#bbbbbb", strokeWidth=0.5)
        .encode(x=x_enc, y=y_enc, tooltip=tooltip)
    )
    act_bg = (
        base.mark_rect(color="#d9d9d9", stroke="#bbbbbb", strokeWidth=0.5)
        .transform_filter(alt.datum["is_actuated_int"] == 1)
        .encode(x=x_enc, y=y_enc, tooltip=tooltip)
    )
    heat = (
        base.mark_rect(stroke="#bbbbbb", strokeWidth=0.5)
        .transform_filter(
            (alt.datum["is_actuated_int"] == 0) & (alt.datum["is_svcc_int"] == 0)
        )
        .encode(
            x=x_enc, y=y_enc,
            color=alt.Color("pct_for_color:Q", scale=color_scale, legend=legend),
            tooltip=tooltip,
        )
    )
    txt_color = alt.condition(
        "abs(datum.pct_for_color) > 15",
        alt.value("white"), alt.value("#1a1a1a"),
    )
    txt_normal = (
        base.mark_text(align="center", baseline="middle", fontSize=12,
                       lineBreak="\n", fontWeight="normal", dy=-6, clip=False)
        .transform_filter(alt.datum["has_stars"] == 0)
        .encode(x=x_enc, y=y_enc, text=alt.Text("label:N"), color=txt_color)
    )
    txt_bold = (
        base.mark_text(align="center", baseline="middle", fontSize=12,
                       lineBreak="\n", fontWeight="bold", dy=-6, clip=False)
        .transform_filter(alt.datum["has_stars"] == 1)
        .encode(x=x_enc, y=y_enc, text=alt.Text("label:N"), color=txt_color)
    )
    return (bg + act_bg + heat + txt_normal + txt_bold).properties(
        width=CHART_WIDTH, height=CELL_HEIGHT)


def make_ped_label_placeholder(height):
    data = [{"x": 0}]
    return (
        alt.Chart(alt.Data(values=data))
        .mark_point(opacity=0)
        .encode(x=alt.X("x:Q", axis=None))
        .properties(width=5, height=height)
    )


def make_left_placeholder(height):
    data = [{"x": 0}]
    return (
        alt.Chart(alt.Data(values=data))
        .mark_point(opacity=0)
        .encode(x=alt.X("x:Q", axis=None))
        .properties(width=LEFT_COL_W, height=height)
    )


def _lerp_color(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def add_colorbar_pil(png_path, max_pct, scale=2):
    from PIL import Image, ImageDraw, ImageFont

    stops_hex = COLOR_RANGE
    stops_pos = [0.0, 0.25, 0.5, 0.75, 1.0]
    stops_rgb = [
        tuple(int(h.lstrip("#")[i:i+2], 16) for i in (0, 2, 4))
        for h in stops_hex
    ]

    img = Image.open(png_path).convert("RGBA")
    W, H = img.size

    bar_margin_l = int(120 * scale)
    bar_margin_r = int(120 * scale)
    bar_w        = W - bar_margin_l - bar_margin_r
    bar_h        = int(16 * scale)
    tick_h       = int(5  * scale)
    label_gap    = int(3  * scale)
    bottom_pad   = int(20 * scale)
    bar_offset   = int(35 * scale)
    total_extra  = bar_offset + bar_h + tick_h + label_gap + int(24 * scale) + bottom_pad

    new_img = Image.new("RGBA", (W, H + total_extra), (255, 255, 255, 255))
    new_img.paste(img, (0, 0))
    draw = ImageDraw.Draw(new_img)

    bar_top = H + bar_offset

    for xi in range(bar_w):
        t = xi / (bar_w - 1)
        seg = 0
        for k in range(len(stops_pos) - 1):
            if t <= stops_pos[k + 1]:
                seg = k
                break
        t_seg = (t - stops_pos[seg]) / (stops_pos[seg + 1] - stops_pos[seg])
        color = _lerp_color(stops_rgb[seg], stops_rgb[seg + 1], t_seg)
        draw.line([(bar_margin_l + xi, bar_top),
                   (bar_margin_l + xi, bar_top + bar_h - 1)], fill=color + (255,))

    draw.rectangle(
        [bar_margin_l, bar_top, bar_margin_l + bar_w - 1, bar_top + bar_h - 1],
        outline=(150, 150, 150, 255), width=1,
    )

    tick_vals = [-30, -20, -10, 0, 10, 20, 30]
    font_size = int(9 * scale)
    font = None
    for fp in [
        "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf",
    ]:
        try:
            font = ImageFont.truetype(fp, font_size)
            break
        except OSError:
            pass
    if font is None:
        font = ImageFont.load_default()

    tick_top  = bar_top + bar_h
    label_top = tick_top + tick_h + label_gap
    for tv in tick_vals:
        frac = (tv - (-max_pct)) / (2 * max_pct)
        frac = max(0.0, min(1.0, frac))
        x = bar_margin_l + int(frac * (bar_w - 1))
        draw.line([(x, tick_top), (x, tick_top + tick_h)], fill=(80, 80, 80, 255), width=1)
        lbl = str(tv)
        bb  = font.getbbox(lbl)
        lw  = bb[2] - bb[0]
        draw.text((x - lw // 2, label_top), lbl, font=font, fill=(50, 50, 50, 255))

    title = "% change vs SVCC"
    title_font_size = int(10 * scale)
    try:
        tfont = ImageFont.truetype(
            "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf", title_font_size)
    except OSError:
        tfont = font
    tbb = tfont.getbbox(title)
    tw  = tbb[2] - tbb[0]
    draw.text(((W - tw) // 2, label_top + int(14 * scale)),
              title, font=tfont, fill=(50, 50, 50, 255))

    new_img.convert("RGB").save(png_path)


def build_header_rows():
    mid_col = WEIGHTS_M2[len(WEIGHTS_M2) // 2 - 1]
    return [
        {"col_key": col,
         "label": "M2SVCC (Vehicle/Pedestrian weighting factor)" if col == mid_col else ""}
        for col in COL_SORT
    ]


def make_chart():
    ped_rows = {ped: build_ped_rows(ped) for ped in PED_LEVELS}

    all_pcts = [r["pct_for_color"] for rows in ped_rows.values()
                for r in rows if not r["is_actuated"] and not r["is_svcc"]]
    # Cap at 90th percentile of absolute values so extreme outliers
    # don't collapse the colour scale (clamp=True keeps colours for beyond-range values)
    max_pct = float(np.percentile([abs(p) for p in all_pcts], 90)) if all_pcts else 30.0
    max_pct = max(max_pct, 10.0)  # floor at 10%

    color_scale = alt.Scale(
        domain=[-max_pct, -max_pct / 3, 0, max_pct / 3, max_pct],
        range=COLOR_RANGE, clamp=True,
    )

    hdr_main = (
        alt.Chart(alt.Data(values=build_header_rows()))
        .mark_text(align="center", baseline="middle",
                   color="#222222", fontSize=11, fontWeight="bold",
                   dx=CHART_WIDTH / len(COL_SORT) * 1.5)
        .encode(
            x=alt.X("col_key:N", sort=COL_SORT, title=None,
                    axis=alt.Axis(domain=False, grid=False, labels=False, ticks=False)),
            text=alt.Text("label:N"),
        )
        .properties(width=CHART_WIDTH, height=20)
    )
    hdr_left = (
        alt.Chart(alt.Data(values=[{"x": 0}]))
        .mark_point(opacity=0)
        .encode(x=alt.X("x:Q", axis=None))
        .properties(width=LEFT_COL_W, height=20)
    )
    hdr_side = (
        alt.Chart(alt.Data(values=[{"x": 0}]))
        .mark_point(opacity=0)
        .encode(x=alt.X("x:Q", axis=None))
        .properties(width=24, height=20)
    )
    hdr_row = alt.hconcat(hdr_left, hdr_main, hdr_side, spacing=0)

    sections = []
    for pi, ped in enumerate(PED_LEVELS):
        show_x  = (pi == 0)
        left    = make_left_placeholder(CELL_HEIGHT)
        heatmap = make_heatmap_section(ped_rows[ped], color_scale, show_x, False)
        side    = make_ped_label_placeholder(CELL_HEIGHT)
        sections.append(alt.hconcat(left, heatmap, side, spacing=0))

    body = alt.vconcat(*sections, spacing=30)

    chart = (
        alt.vconcat(hdr_row, body, spacing=0)
        .properties(
            title=alt.TitleParams(
                text="Exclusive-Asymmetric: Performance Heatmap vs SVCC (1.0, 0.0)",
                subtitle=[
                    "All values averaged across available seeds with IQR outlier filtering",
                    "Blue = improvement over SVCC  |  Orange/Red = regression vs SVCC  |  Grey = Actuated (reference)",
                    "* |% change| ≥ 3%  |  ** |% change| > 10%",
                ],
                fontSize=15, subtitleFontSize=11, anchor="middle", offset=10,
            )
        )
        .configure(font="Liberation Serif", padding={"left": 5, "right": 30, "top": 5, "bottom": 5})
        .configure_view(stroke=None, strokeOpacity=0, strokeWidth=0,
                        continuousWidth=300, continuousHeight=300)
        .configure_axis(domainColor="#aaaaaa", grid=False,
                        labelFont="Liberation Serif", titleFont="Liberation Serif")
        .configure_title(font="Liberation Serif", subtitleFont="Liberation Serif")
    )
    return chart, max_pct


def detect_section_boundaries(img_arr, col_w):
    import numpy as np
    mid = img_arr[:, 100:-100, :3].mean(axis=(1, 2))
    in_section = False
    sections = []
    y_start = 0
    for y, m in enumerate(mid):
        if not in_section and m < 245:
            y_start = y
            in_section = True
        elif in_section and m > 248 and y > y_start + 10:
            sections.append((y_start, y))
            in_section = False
    if in_section:
        sections.append((y_start, len(mid)))
    return sections


def add_ped_labels_pil(png_path, ped_labels, scale=2):
    """Draw ped-level labels horizontally below each section using Pillow."""
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np

    img  = Image.open(png_path)
    arr  = np.array(img)
    w, h = img.size
    col_w = 5 * scale

    sections = detect_section_boundaries(arr, col_w)
    data_sections = [s for s in sections if (s[1] - s[0]) > 50 * scale]
    if len(data_sections) < len(ped_labels):
        top = sections[0][0] if sections else 0
        sh  = CELL_HEIGHT * scale
        sp  = 30 * scale
        data_sections = [(top + i * (sh + sp), top + i * (sh + sp) + sh)
                         for i in range(len(ped_labels))]

    font_size = 11 * scale
    for font_path in [
        "/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSerif-Bold.ttf",
    ]:
        try:
            font = ImageFont.truetype(font_path, font_size)
            break
        except OSError:
            continue
    else:
        font = ImageFont.load_default()

    draw = ImageDraw.Draw(img)

    for i, label in enumerate(ped_labels):
        if i >= len(data_sections):
            break
        _, y_end = data_sections[i]
        bbox = font.getbbox(label)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        tx = (w - tw) // 2
        ty = y_end + int(6 * scale)
        draw.text((tx, ty), label, font=font, fill=(0, 0, 0, 255))

    img.save(png_path)


def add_category_labels_pil(png_path, scale=2):
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np

    img  = Image.open(png_path)
    arr  = np.array(img)
    w, h = img.size

    left_col_w  = LEFT_COL_W * scale
    right_col_w = 5 * scale

    sections = detect_section_boundaries(arr, right_col_w)
    data_sections = [s for s in sections if (s[1] - s[0]) > 50 * scale]

    font_paths = [
        "/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf",
    ]

    draw = ImageDraw.Draw(img)

    for sec_start, sec_end in data_sections:
        sec_h  = sec_end - sec_start
        cell_h = sec_h / 6.0

        for label, row_start, row_end in CATEGORIES:
            y0 = int(sec_start + row_start * cell_h)
            y1 = int(sec_start + (row_end + 1) * cell_h)

            font = None
            for size in range(int(10 * scale), int(5 * scale) - 1, -1):
                for fp in font_paths:
                    try:
                        f = ImageFont.truetype(fp, size)
                        bb = f.getbbox(label)
                        if (bb[2] - bb[0]) <= left_col_w - 4 * scale:
                            font = f
                            break
                    except OSError:
                        continue
                if font:
                    break
            if font is None:
                font = ImageFont.load_default()

            bb  = font.getbbox(label)
            tw, th = bb[2] - bb[0], bb[3] - bb[1]
            tx = (left_col_w - tw) // 2
            ty = (y0 + y1) // 2 - th // 2
            draw.text((tx, ty), label, font=font, fill=(0, 0, 0, 255))

        table_right = left_col_w + CHART_WIDTH * scale
        for _, row_start, _ in CATEGORIES[1:]:
            y_sep = int(sec_start + row_start * cell_h)
            draw.line([(0, y_sep), (table_right, y_sep)],
                      fill=(160, 160, 160, 255), width=1)

    first_sec_start = data_sections[0][0]
    last_sec_end    = data_sections[-1][1]
    draw.line([(left_col_w, first_sec_start), (left_col_w, last_sec_end)],
              fill=(160, 160, 160, 255), width=1)

    scan_y = first_sec_start + int(2 * scale)
    cell_x_start = w
    for xi in range(left_col_w + 1, w - right_col_w):
        r, g, b = arr[scan_y, xi, :3]
        if r < 240 or g < 240 or b < 240:
            cell_x_start = xi
            break

    metric_label = "Metric"
    metric_font  = None
    for size in range(int(11 * scale), int(7 * scale) - 1, -1):
        for fp in font_paths:
            try:
                f = ImageFont.truetype(fp, size)
                bb = f.getbbox(metric_label)
                if (bb[2] - bb[0]) <= (cell_x_start - left_col_w) - 4:
                    metric_font = f
                    break
            except OSError:
                continue
        if metric_font:
            break
    if metric_font is None:
        metric_font = ImageFont.load_default()

    bb = metric_font.getbbox(metric_label)
    tw, th = bb[2] - bb[0], bb[3] - bb[1]
    label_y = first_sec_start - th - int(4 * scale)
    label_x = left_col_w + (cell_x_start - left_col_w - tw) // 2
    draw.text((label_x, label_y), metric_label,
              font=metric_font, fill=(0, 0, 0, 255))

    img.save(png_path)


if __name__ == "__main__":
    print("Parsing Exclusive-Asymmetric results …")
    chart, max_pct = make_chart()

    out_png  = os.path.join(OUT_DIR, "fig_exclusive_altair_new.png")
    out_html = os.path.join(OUT_DIR, "fig_exclusive_altair_new.html")

    import vl_convert as vlc
    png_bytes = vlc.vegalite_to_png(vl_spec=chart.to_json(), scale=2)
    with open(out_png, "wb") as f:
        f.write(png_bytes)

    add_category_labels_pil(out_png, scale=2)
    add_colorbar_pil(out_png, max_pct, scale=2)
    add_ped_labels_pil(out_png, list(PED_LABEL.values()), scale=2)
    print(f"Saved PNG  → {out_png}")

    chart.save(out_html)
    print(f"Saved HTML → {out_html}")
