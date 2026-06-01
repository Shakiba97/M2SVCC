"""
Statistical heatmap table comparing Right-Turn phase design scenarios.

Layout (same visual style as the Exclusive-Asymmetric heatmap):
  Rows    : LowPed / MedPed / HighPed  x  6 metrics
  Columns :
    Permitted Right  [ Actuated | SVCC | M2SVCC ]
    Protected Right  [ Actuated | SVCC | M2SVCC ]
    LPI              [ Actuated | SVCC | M2SVCC ]
    Delayed Turn     [ Actuated | SVCC | M2SVCC ]
    Exclusive        [ M2SVCC only ]

Color encoding : % change vs Permitted Right Actuated (reference baseline)
  - Actuated columns : grey (reference-level display)
  - SVCC columns     : white (neutral, value shown)
  - M2SVCC columns   : color-encoded blue (improvement) / red (regression)
                       with Welch t-test + Mann-Whitney significance stars
  Reference cell     : Permitted Right / Actuated = white (0% change)
"""

import os
import re
import numpy as np
import altair as alt
from scipy import stats

BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Results")
OUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Results", "Concurrent figurs")

PED_LEVELS = ["LowPed", "MedPed", "HighPed"]
PED_LABEL  = {
    "LowPed":  "Low Pedestrian Demand",
    "MedPed":  "Med Pedestrian Demand",
    "HighPed": "High Pedestrian Demand",
}
PED_ORDER = [PED_LABEL[p] for p in PED_LEVELS]

METRIC_ORDER = [
    # Sustainability
    "Fuel (mg)",
    "Energy (Kw.s)",
    # Vehicle Mobility
    "Veh Wait (s)",
    "Veh Time Loss (s)",
    "Veh Queue (m)",
    # Bike Mobility
    "Bike Wait (s)",
    "Bike Time Loss (s)",
    "Bike Queue (m)",
    # Ped Mobility
    "Ped Time Loss (s)",
    # Safety
    "Veh-Ped (#)",
    "Veh-Bike (#)",
    "Veh-Veh (#)",
]

METRIC_PATTERNS = {
    "Fuel (mg)":         r"average fuel consumption \(external model\) for CAVs (?:multi_scale|actuated) \(in mg\): ([\d.]+)",
    "Energy (Kw.s)":     r"average power consumption \(external model\) for CAVs (?:multi_scale|actuated) \(in Kw\.s\): ([\d.]+)",
    "Veh Wait (s)":      r"average waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "Veh Time Loss (s)": r"average time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "Veh Queue (m)":     r"average queue length for (?:multi_scale|actuated) scenario \(in m\): ([\d.]+)",
    "Bike Wait (s)":     r"average bike \(sep\+int\) waiting time for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "Bike Time Loss (s)":r"average bike \(sep\+int\) time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "Bike Queue (m)":    r"average bike \(sep\) queue length for (?:multi_scale|actuated) scenario \(in m\): ([\d.]+)",
    "Ped Time Loss (s)": r"average pedestrian time loss for (?:multi_scale|actuated) scenario \(in s\): ([\d.]+)",
    "Veh-Ped (#)":       r"number of right-turn conflicts between vehicles and pedestrians: (\d+)",
    "Veh-Bike (#)":      r"number of conflicts between vehicles and cyclists: (\d+)",
    "Veh-Veh (#)":       r"number of conflicts between vehicles and vehicles: (\d+)",
}

CATEGORIES = [
    ("Sustainability", 0,  1),
    ("Veh Mobility",   2,  4),
    ("Bike Mobility",  5,  7),
    ("Ped Mobility",   8,  8),
    ("Safety",         9, 11),
]

COLOR_RANGE = ["#08306b", "#6baed6", "#f7f7f7", "#d4707a", "#b2182b"]

# Scenario configuration
# turn: substring used in filenames (None for Exclusive)
SCENARIOS = {
    "Permitted Right":  {"dir": "Concurrent-Permitted Right",  "turn": "permitted_right"},
    "Protected Right":  {"dir": "Concurrent-Protected Right",  "turn": "protected_right"},
    "LPI":              {"dir": "Concurrent-LPI",              "turn": "LPI"},
    "Delayed Turn":     {"dir": "Concurrent-delayed turn",     "turn": "delayed_turn"},
    "Exclusive":        {"dir": "Exclusive",                   "turn": None},
}
SCENARIO_ORDER = ["Permitted Right", "Protected Right", "LPI", "Delayed Turn", "Exclusive"]

# col_key format: "Scenario|Controller"  (| used as separator)
# labelExpr in x-axis extracts the controller part for display
_COL_TUPLES = []
for _scen in SCENARIO_ORDER:
    if _scen == "Exclusive":
        _COL_TUPLES.append((_scen, "M2SVCC"))
    else:
        _COL_TUPLES.append((_scen, "Actuated"))
        _COL_TUPLES.append((_scen, "SVCC"))
        _COL_TUPLES.append((_scen, "M2SVCC"))

COL_KEYS  = [f"{s}|{c}" for s, c in _COL_TUPLES]
COL_ORDER = {k: i for i, k in enumerate(COL_KEYS)}

CHART_WIDTH = 650
CELL_HEIGHT = 420
LEFT_COL_W  = 80


# ── data loading ──────────────────────────────────────────────────────────────

def parse_file(path):
    with open(path) as f:
        text = f.read()
    return {m: (float(mo.group(1)) if (mo := re.search(p, text)) else None)
            for m, p in METRIC_PATTERNS.items()}


def iqr_filter(values):
    if len(values) < 4:
        return values
    arr = np.array(values, dtype=float)
    q1, q3 = np.percentile(arr, 25), np.percentile(arr, 75)
    iqr = q3 - q1
    if iqr == 0:
        return values
    return arr[(arr >= q1 - 1.5 * iqr) & (arr <= q3 + 1.5 * iqr)].tolist()


def aggregate_paths(paths):
    """Parse all files, collect per-metric samples, apply IQR filter."""
    raw = {m: [] for m in METRIC_ORDER}
    for path in paths:
        for m, v in parse_file(path).items():
            if v is not None:
                raw[m].append(v)
    return {m: iqr_filter(v) for m, v in raw.items()}


def load_actuated(scenario_key, ped):
    if scenario_key == "Exclusive":
        return {m: [] for m in METRIC_ORDER}
    cfg  = SCENARIOS[scenario_key]
    d    = os.path.join(BASE_DIR, cfg["dir"])
    turn = cfg["turn"]
    pat  = re.compile(
        rf"actuated_penetration\(1\)_EVratio\(0\.5\)_Concurrent_{re.escape(turn)}"
        rf"_Asymmetric_{ped}_seed\((\d+)\)\.txt"
    )
    paths = [os.path.join(d, f) for f in os.listdir(d) if pat.fullmatch(f)]
    return aggregate_paths(paths)


def load_svcc(scenario_key, ped):
    """SVCC = weight (1, 0, 0) or (1, 0, 0.0) — vehicle-only objective."""
    if scenario_key == "Exclusive":
        return {m: [] for m in METRIC_ORDER}
    cfg  = SCENARIOS[scenario_key]
    d    = os.path.join(BASE_DIR, cfg["dir"])
    turn = cfg["turn"]
    # Match (1, 0, 0) or (1, 0, 0.0) literally
    pat  = re.compile(
        rf"multi_scale_penetration\(1\)_EVratio\(0\.5\)_Concurrent_{re.escape(turn)}"
        rf"_Asymmetric_\(1, 0, 0[^)]*\)_{ped}_seed\((\d+)\)\.txt"
    )
    paths = [os.path.join(d, f) for f in os.listdir(d) if pat.fullmatch(f)]
    return aggregate_paths(paths)


def load_m2svcc(scenario_key, ped):
    """
    M2SVCC = multi-objective weight (not SVCC).
    For Concurrent scenarios: pick the non-SVCC weight with the most seed files.
    For Exclusive: pool all available multi_scale files for this ped level.
    """
    cfg  = SCENARIOS[scenario_key]
    d    = os.path.join(BASE_DIR, cfg["dir"])

    if scenario_key == "Exclusive":
        # Turn type names (permitted_right, delayed_turn) contain underscores,
        # so we match everything between "Exclusive_" and "_Asymmetric_".
        pat = re.compile(
            rf"multi_scale_penetration\(1\)_EVratio\(0\.5\)_Exclusive_.+_Asymmetric_"
            rf"\([^)]+\)_{ped}_seed\((\d+)\)\.txt"
        )
        paths = [os.path.join(d, f) for f in os.listdir(d) if pat.fullmatch(f)]
        return aggregate_paths(paths)

    turn = cfg["turn"]
    prefix = (
        f"multi_scale_penetration(1)_EVratio(0.5)_Concurrent_{turn}_Asymmetric_"
    )
    svcc_re = re.compile(r"^\(1, 0, 0[^)]*\)$")

    weight_files: dict[str, list[str]] = {}
    for fname in os.listdir(d):
        if not fname.startswith(prefix):
            continue
        rest = fname[len(prefix):]
        wm = re.match(r"(\([^)]+\))_" + re.escape(ped) + r"_seed\(\d+\)\.txt$", rest)
        if not wm:
            continue
        w = wm.group(1)
        if svcc_re.match(w):
            continue  # skip SVCC
        weight_files.setdefault(w, []).append(os.path.join(d, fname))

    if not weight_files:
        return {m: [] for m in METRIC_ORDER}

    # Use the weight with the most seed files
    best = max(weight_files, key=lambda w: len(weight_files[w]))
    return aggregate_paths(weight_files[best])


def get_best_m2svcc_weight(scenario_key, ped):
    """Return the M2SVCC weight label used for a given scenario+ped (for annotation)."""
    if scenario_key == "Exclusive":
        return "pooled"
    cfg   = SCENARIOS[scenario_key]
    d     = os.path.join(BASE_DIR, cfg["dir"])
    turn  = cfg["turn"]
    prefix = f"multi_scale_penetration(1)_EVratio(0.5)_Concurrent_{turn}_Asymmetric_"
    svcc_re = re.compile(r"^\(1, 0, 0[^)]*\)$")
    weight_files: dict[str, int] = {}
    for fname in os.listdir(d):
        if not fname.startswith(prefix):
            continue
        rest = fname[len(prefix):]
        wm = re.match(r"(\([^)]+\))_" + re.escape(ped) + r"_seed\(\d+\)\.txt$", rest)
        if not wm:
            continue
        w = wm.group(1)
        if svcc_re.match(w):
            continue
        weight_files[w] = weight_files.get(w, 0) + 1
    if not weight_files:
        return "N/A"
    return max(weight_files, key=weight_files.get)


# ── statistics ─────────────────────────────────────────────────────────────────

def significance_stars(base_arr, alt_arr):
    """Welch's t-test + Mann-Whitney U; return '' / '*' / '**'."""
    if len(base_arr) < 3 or len(alt_arr) < 3:
        return ""
    _, t_p = stats.ttest_ind(base_arr, alt_arr, equal_var=False)
    try:
        _, w_p = stats.mannwhitneyu(base_arr, alt_arr, alternative="two-sided")
    except ValueError:
        w_p = float("nan")
    p = min(t_p, w_p) if not np.isnan(w_p) else t_p
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return ""


# ── row builder ────────────────────────────────────────────────────────────────

def build_rows():
    rows = []

    for ped_i, ped in enumerate(PED_LEVELS):
        ped_label = PED_LABEL[ped]

        # Reference: Permitted Right Actuated
        ref_sv  = load_actuated("Permitted Right", ped)
        ref_vals = {m: (float(np.mean(v)) if v else None) for m, v in ref_sv.items()}

        for scen in SCENARIO_ORDER:
            act_sv  = load_actuated(scen, ped)
            svcc_sv = load_svcc(scen, ped)
            m2_sv   = load_m2svcc(scen, ped)

            act_m  = {m: (round(float(np.mean(v)), 2) if v else None) for m, v in act_sv.items()}
            svcc_m = {m: (round(float(np.mean(v)), 2) if v else None) for m, v in svcc_sv.items()}
            m2_m   = {m: (round(float(np.mean(v)), 2) if v else None) for m, v in m2_sv.items()}

            for mi, metric in enumerate(METRIC_ORDER):
                ref_v = ref_vals[metric]
                ref_s = ref_sv[metric]

                def pct_vs_ref(val):
                    if val is None or ref_v is None or ref_v == 0:
                        return 0.0
                    return round((val - ref_v) / ref_v * 100, 1)

                is_ref_col = (scen == "Permitted Right")

                # ---- Actuated ----
                if scen != "Exclusive":
                    av  = act_m[metric]
                    pct = pct_vs_ref(av)
                    is_ref = 1 if is_ref_col else 0
                    rows.append({
                        "ped_level":     ped_label,
                        "ped_order":     ped_i,
                        "metric":        metric,
                        "metric_order":  mi,
                        "col_key":       f"{scen}|Actuated",
                        "col_order":     COL_ORDER[f"{scen}|Actuated"],
                        "value":         av,
                        "pct_change":    pct,
                        "pct_for_color": 0.0 if is_ref else pct,
                        "cell_type":     "ref" if is_ref else "actuated",
                        "is_m2svcc":     0,
                        "has_stars":     0,
                        "label":         f"{av:.2f}" if av is not None else "N/A",
                    })

                # ---- SVCC ----
                if scen != "Exclusive":
                    sv  = svcc_m[metric]
                    pct = pct_vs_ref(sv)
                    rows.append({
                        "ped_level":     ped_label,
                        "ped_order":     ped_i,
                        "metric":        metric,
                        "metric_order":  mi,
                        "col_key":       f"{scen}|SVCC",
                        "col_order":     COL_ORDER[f"{scen}|SVCC"],
                        "value":         sv,
                        "pct_change":    pct,
                        "pct_for_color": pct,
                        "cell_type":     "svcc",
                        "is_m2svcc":     0,
                        "has_stars":     0,
                        "label":         f"{sv:.2f}" if sv is not None else "N/A",
                    })

                # ---- M2SVCC ----
                mv    = m2_m[metric]
                pct   = pct_vs_ref(mv)
                stars = significance_stars(ref_s, m2_sv[metric])
                sign  = "+" if pct >= 0 else ""
                lbl   = (f"{mv:.2f}{stars}\n({sign}{pct:.1f}%)"
                         if mv is not None else "N/A")
                rows.append({
                    "ped_level":     ped_label,
                    "ped_order":     ped_i,
                    "metric":        metric,
                    "metric_order":  mi,
                    "col_key":       f"{scen}|M2SVCC",
                    "col_order":     COL_ORDER[f"{scen}|M2SVCC"],
                    "value":         mv,
                    "pct_change":    pct,
                    "pct_for_color": pct,
                    "cell_type":     "m2svcc",
                    "is_m2svcc":     1,
                    "has_stars":     1 if stars else 0,
                    "label":         lbl,
                })

    return rows


# ── Altair chart ────────────────────────────────────────────────────────────────

def build_header_rows():
    """One row per col_key: scenario name shown in the SVCC (middle) column of each group."""
    rows = []
    for scen in SCENARIO_ORDER:
        if scen == "Exclusive":
            rows.append({"col_key": f"{scen}|M2SVCC", "label": scen})
        else:
            for ctrl in ["Actuated", "SVCC", "M2SVCC"]:
                rows.append({
                    "col_key": f"{scen}|{ctrl}",
                    "label":   scen if ctrl == "SVCC" else "",
                })
    return rows


def make_heatmap_section(rows, color_scale, show_x_labels, include_legend):
    base = alt.Chart(alt.Data(values=rows))

    # No x-axis labels — headers are drawn entirely by PIL
    x_axis = alt.Axis(labels=False, ticks=False, domain=False)

    x_enc = alt.X("col_key:N", sort=COL_KEYS, title=None, axis=x_axis)
    y_enc = alt.Y(
        "metric:N", sort=METRIC_ORDER, title=None,
        axis=alt.Axis(labelFontSize=10, labelLimit=190),
    )
    tooltip = [
        alt.Tooltip("metric:N",     title="Metric"),
        alt.Tooltip("col_key:N",    title="Controller"),
        alt.Tooltip("value:Q",      title="Value",           format=".2f"),
        alt.Tooltip("pct_change:Q", title="% vs Perm Actuated", format="+.1f"),
    ]

    legend = alt.Legend(
        title="% change vs Permitted Right Actuated",
        orient="bottom", direction="horizontal",
        gradientLength=320, gradientThickness=16,
        labelFontSize=9, titleFontSize=10,
    ) if include_legend else None

    # White background for all cells
    bg = (
        base.mark_rect(color="white", stroke="#bbbbbb", strokeWidth=0.5)
        .encode(x=x_enc, y=y_enc, tooltip=tooltip)
    )
    # Grey for Actuated (non-reference) cells
    act_bg = (
        base.mark_rect(color="#d9d9d9", stroke="#bbbbbb", strokeWidth=0.5)
        .transform_filter("datum.cell_type === 'actuated'")
        .encode(x=x_enc, y=y_enc, tooltip=tooltip)
    )
    # Light gold/beige for SVCC cells
    svcc_bg = (
        base.mark_rect(color="#f5f0e8", stroke="#bbbbbb", strokeWidth=0.5)
        .transform_filter("datum.cell_type === 'svcc'")
        .encode(x=x_enc, y=y_enc, tooltip=tooltip)
    )
    # Color-encoded for M2SVCC cells (and SVCC/actuated non-grey ones get color too)
    heat = (
        base.mark_rect(stroke="#bbbbbb", strokeWidth=0.5)
        .transform_filter("datum.cell_type === 'm2svcc'")
        .encode(
            x=x_enc, y=y_enc,
            color=alt.Color("pct_for_color:Q", scale=color_scale, legend=legend),
            tooltip=tooltip,
        )
    )
    # Text color: white only for dark M2SVCC cells; always dark for Actuated/SVCC/ref
    txt_color = alt.condition(
        "(datum.is_m2svcc === 1) && (abs(datum.pct_for_color) > 18)",
        alt.value("white"), alt.value("#1a1a1a"),
    )
    txt_normal = (
        base.mark_text(align="center", baseline="middle", fontSize=10.5,
                       lineBreak="\n", fontWeight="normal", dy=-5, clip=False)
        .transform_filter("datum.has_stars === 0")
        .encode(x=x_enc, y=y_enc, text=alt.Text("label:N"), color=txt_color)
    )
    txt_bold = (
        base.mark_text(align="center", baseline="middle", fontSize=10.5,
                       lineBreak="\n", fontWeight="bold", dy=-5, clip=False)
        .transform_filter("datum.has_stars === 1")
        .encode(x=x_enc, y=y_enc, text=alt.Text("label:N"), color=txt_color)
    )
    return (bg + act_bg + svcc_bg + heat + txt_normal + txt_bold).properties(
        width=CHART_WIDTH, height=CELL_HEIGHT)


def make_placeholder(width, height):
    return (
        alt.Chart(alt.Data(values=[{"x": 0}]))
        .mark_point(opacity=0)
        .encode(x=alt.X("x:Q", axis=None))
        .properties(width=width, height=height)
    )


# ── PIL post-processing ─────────────────────────────────────────────────────────

def _lerp_color(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def detect_section_boundaries(img_arr):
    mid = img_arr[:, 100:-100, :3].mean(axis=(1, 2))
    in_sec, sections, y0 = False, [], 0
    for y, m in enumerate(mid):
        if not in_sec and m < 245:
            y0, in_sec = y, True
        elif in_sec and m > 248 and y > y0 + 10:
            sections.append((y0, y))
            in_sec = False
    if in_sec:
        sections.append((y0, len(mid)))
    return sections


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

    bar_ml   = int(130 * scale)
    bar_mr   = int(130 * scale)
    bar_w    = W - bar_ml - bar_mr
    bar_h    = int(16 * scale)
    tick_h   = int(5  * scale)
    label_g  = int(3  * scale)
    bot_pad  = int(20 * scale)
    bar_off  = int(35 * scale)
    total    = bar_off + bar_h + tick_h + label_g + int(28 * scale) + bot_pad

    new = Image.new("RGBA", (W, H + total), (255, 255, 255, 255))
    new.paste(img, (0, 0))
    draw = ImageDraw.Draw(new)
    bar_top = H + bar_off

    for xi in range(bar_w):
        t = xi / (bar_w - 1)
        seg = next(k for k in range(len(stops_pos) - 1) if t <= stops_pos[k + 1])
        t2  = (t - stops_pos[seg]) / (stops_pos[seg + 1] - stops_pos[seg])
        clr = _lerp_color(stops_rgb[seg], stops_rgb[seg + 1], t2)
        draw.line([(bar_ml + xi, bar_top), (bar_ml + xi, bar_top + bar_h - 1)],
                  fill=clr + (255,))

    draw.rectangle([bar_ml, bar_top, bar_ml + bar_w - 1, bar_top + bar_h - 1],
                   outline=(150, 150, 150, 255), width=1)

    tick_vals = [-30, -20, -10, 0, 10, 20, 30]
    font_size = int(9 * scale)
    font = None
    for fp in ["/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf"]:
        try:
            font = ImageFont.truetype(fp, font_size)
            break
        except OSError:
            pass
    if font is None:
        font = ImageFont.load_default()

    tick_top  = bar_top + bar_h
    label_top = tick_top + tick_h + label_g
    for tv in tick_vals:
        frac = max(0.0, min(1.0, (tv - (-max_pct)) / (2 * max_pct)))
        x = bar_ml + int(frac * (bar_w - 1))
        draw.line([(x, tick_top), (x, tick_top + tick_h)], fill=(80, 80, 80, 255), width=1)
        lbl = str(tv)
        bb  = font.getbbox(lbl)
        draw.text((x - (bb[2] - bb[0]) // 2, label_top), lbl,
                  font=font, fill=(50, 50, 50, 255))

    title = "% change vs Permitted Right Actuated"
    try:
        tfont = ImageFont.truetype(
            "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf",
            int(10 * scale))
    except OSError:
        tfont = font
    tbb = tfont.getbbox(title)
    draw.text(((W - (tbb[2] - tbb[0])) // 2, label_top + int(16 * scale)),
              title, font=tfont, fill=(50, 50, 50, 255))

    new.convert("RGB").save(png_path)


def add_ped_labels_pil(png_path, ped_labels, scale=2):
    from PIL import Image, ImageDraw, ImageFont
    img = Image.open(png_path)
    arr = np.array(img)
    W, H = img.size

    sections = detect_section_boundaries(arr)
    data_sec = [s for s in sections if (s[1] - s[0]) > 50 * scale]
    if len(data_sec) < len(ped_labels):
        sh = CELL_HEIGHT * scale
        sp = 30 * scale
        top = sections[0][0] if sections else 0
        data_sec = [(top + i * (sh + sp), top + i * (sh + sp) + sh)
                    for i in range(len(ped_labels))]

    font_size = int(11 * scale)
    font = None
    for fp in ["/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf"]:
        try:
            font = ImageFont.truetype(fp, font_size)
            break
        except OSError:
            pass
    if font is None:
        font = ImageFont.load_default()

    draw = ImageDraw.Draw(img)
    for i, label in enumerate(ped_labels):
        if i >= len(data_sec):
            break
        _, y_end = data_sec[i]
        bb = font.getbbox(label)
        tw = bb[2] - bb[0]
        draw.text(((W - tw) // 2, y_end + int(6 * scale)), label,
                  font=font, fill=(0, 0, 0, 255))
    img.save(png_path)


def add_category_labels_pil(png_path, scale=2):
    from PIL import Image, ImageDraw, ImageFont
    img = Image.open(png_path)
    arr = np.array(img)
    W, H = img.size

    left_col_w  = LEFT_COL_W * scale
    right_col_w = 5 * scale

    sections = detect_section_boundaries(arr)
    data_sec = [s for s in sections if (s[1] - s[0]) > 50 * scale]

    font_paths = ["/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf"]
    draw = ImageDraw.Draw(img)

    for sec_start, sec_end in data_sec:
        sec_h  = sec_end - sec_start
        cell_h = sec_h / len(METRIC_ORDER)

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

            bb      = font.getbbox(label)
            tw, th  = bb[2] - bb[0], bb[3] - bb[1]
            tx = (left_col_w - tw) // 2
            ty = (y0 + y1) // 2 - th // 2
            draw.text((tx, ty), label, font=font, fill=(0, 0, 0, 255))

        table_right = left_col_w + CHART_WIDTH * scale
        for _, row_start, _ in CATEGORIES[1:]:
            y_sep = int(sec_start + row_start * cell_h)
            draw.line([(0, y_sep), (table_right, y_sep)],
                      fill=(160, 160, 160, 255), width=1)

    if data_sec:
        first_start = data_sec[0][0]
        last_end    = data_sec[-1][1]
        draw.line([(left_col_w, first_start), (left_col_w, last_end)],
                  fill=(160, 160, 160, 255), width=1)

    img.save(png_path)


def add_two_row_header_pil(png_path, scale=2):
    """
    Prepend a two-row table header above the heatmap by expanding the canvas upward.

    Row 1 (tall): scenario group names, each spanning their sub-columns.
    Row 2 (short): controller sub-labels — Actuated | SVCC | M2SVCC.

    Border style matches the heatmap cells (#bbbbbb thin lines, darker thick
    lines at group boundaries).
    """
    from PIL import Image, ImageDraw, ImageFont

    img = Image.open(png_path)
    arr = np.array(img)
    W, H = img.size

    left_col_w = LEFT_COL_W * scale

    # ── detect data sections and column positions ──────────────────────────
    sections = detect_section_boundaries(arr)
    data_sec = [s for s in sections if (s[1] - s[0]) > 50 * scale]
    if not data_sec:
        img.save(png_path)
        return

    first_start = data_sec[0][0]
    last_end    = data_sec[-1][1]

    # ── detect column positions analytically ───────────────────────────────
    # M2SVCC heatmap colors (blues/reds) overlap with border colors, making
    # pixel-scanning unreliable.  Instead:
    #  1. Find x_plot_start by locating the first GREY Actuated cell
    #     (bg = #d9d9d9 = 217,217,217 — distinct from white and beige/blue/red).
    #  2. Compute uniform column width from CHART_WIDTH.
    #  3. Reconstruct all column positions analytically.
    section_h = data_sec[0][1] - data_sec[0][0]

    # Collect votes: for each x, how many scanned rows look "grey" (Actuated bg)
    n_scan = max(20, section_h // 8)
    grey_votes = np.zeros(W, dtype=float)
    for k in range(n_scan):
        y = first_start + k * (section_h // n_scan)
        for xi in range(left_col_w, W):
            r, g, b = int(arr[y, xi, 0]), int(arr[y, xi, 1]), int(arr[y, xi, 2])
            # Grey Actuated background: all channels equal and near 217
            if abs(r - 217) < 20 and abs(g - 217) < 20 and abs(b - 217) < 20:
                grey_votes[xi] += 1.0
    grey_votes /= n_scan

    # The first sustained grey region (vote fraction > 0.3) is the Actuated column
    x_plot_start = None
    in_grey = False
    for xi in range(left_col_w, W):
        if not in_grey and grey_votes[xi] > 0.3:
            x_plot_start = xi
            in_grey = True
        elif in_grey and grey_votes[xi] <= 0.1:
            break   # found the extent of the first grey block — stop

    if x_plot_start is None:
        img.save(png_path)
        return

    # Uniform column width from chart spec
    col_width = (CHART_WIDTH * scale) / len(COL_KEYS)
    col_x     = [int(round(x_plot_start + i * col_width)) for i in range(len(COL_KEYS))]
    table_right = int(round(x_plot_start + len(COL_KEYS) * col_width)) - 1

    # Per-column right edges
    col_right = [col_x[i + 1] - 1 for i in range(len(col_x) - 1)] + [table_right]

    # ── header row dimensions ──────────────────────────────────────────────
    row1_h = int(30 * scale)   # scenario group name row
    row2_h = int(24 * scale)   # controller sub-label row
    total_h = row1_h + row2_h

    # ── insert header rows BETWEEN title and first data section ────────────
    # Split: top_part = title area, bottom_part = data onwards
    top_part    = img.crop((0, 0, W, first_start))
    bottom_part = img.crop((0, first_start, W, H))

    new_img = Image.new("RGB", (W, H + total_h), (255, 255, 255))
    new_img.paste(top_part,    (0, 0))
    new_img.paste(bottom_part, (0, first_start + total_h))
    draw = ImageDraw.Draw(new_img)

    # Header rows drawn at [first_start, first_start + total_h)
    # Data sections shifted down by total_h
    header_y0    = first_start
    last_end_new = last_end + total_h

    # ── fonts ──────────────────────────────────────────────────────────────
    fp_bold = "/usr/share/fonts/truetype/liberation/LiberationSerif-Bold.ttf"
    fp_reg  = "/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf"

    def load_font(path, size):
        try:
            return ImageFont.truetype(path, int(size))
        except OSError:
            return ImageFont.load_default()

    font_grp = load_font(fp_bold, 10.5 * scale)
    font_sub = load_font(fp_bold, 9.0  * scale)

    # ── colors ─────────────────────────────────────────────────────────────
    thin_border  = (187, 187, 187)   # #bbbbbb — same as heatmap cells
    thick_border = (80,  80,  80)    # dark separator between scenario groups
    act_bg  = (217, 217, 217)        # grey  — matches Actuated cells
    svcc_bg = (245, 240, 232)        # beige — matches SVCC cells
    m2_bg   = (210, 228, 248)        # light blue — M2SVCC
    grp_bg  = (235, 241, 250)        # light blue-grey for group header
    txt_dk  = (20,  20,  60)
    txt_md  = (30,  30,  30)

    # ── build group x-ranges ───────────────────────────────────────────────
    groups = []   # (scen, col_start_idx, col_end_idx)
    ci = 0
    for scen in SCENARIO_ORDER:
        n = 1 if scen == "Exclusive" else 3
        groups.append((scen, ci, ci + n - 1))
        ci += n

    r1y0 = header_y0
    r1y1 = header_y0 + row1_h - 1
    r2y0 = header_y0 + row1_h
    r2y1 = header_y0 + total_h - 1

    # ── draw row 1: scenario group headers ────────────────────────────────
    for scen, c0, c1 in groups:
        x0 = col_x[c0]
        x1 = col_right[c1]
        draw.rectangle([x0, r1y0, x1, r1y1], fill=grp_bg, outline=thin_border)
        bb = font_grp.getbbox(scen)
        tw, th = bb[2] - bb[0], bb[3] - bb[1]
        draw.text((x0 + (x1 - x0 - tw) // 2, r1y0 + (row1_h - th) // 2),
                  scen, font=font_grp, fill=txt_dk)

    # ── draw row 2: controller sub-labels ─────────────────────────────────
    for i, (scen, ctrl) in enumerate(_COL_TUPLES):
        x0 = col_x[i]
        x1 = col_right[i]
        bg = act_bg if ctrl == "Actuated" else svcc_bg if ctrl == "SVCC" else m2_bg
        draw.rectangle([x0, r2y0, x1, r2y1], fill=bg, outline=thin_border)
        bb = font_sub.getbbox(ctrl)
        tw, th = bb[2] - bb[0], bb[3] - bb[1]
        draw.text((x0 + (x1 - x0 - tw) // 2, r2y0 + (row2_h - th) // 2),
                  ctrl, font=font_sub, fill=txt_md)

    # ── thick group-boundary separators: header row + each data section only ─
    # Data sections are shifted down by total_h after header insertion
    shifted_data_sec = [(s + total_h, e + total_h) for s, e in data_sec]
    seg_ranges = [(r1y0, r2y1)] + list(shifted_data_sec)
    for _, c0, _ in groups[1:]:
        x = col_x[c0]
        for y0_seg, y1_seg in seg_ranges:
            draw.line([(x, y0_seg), (x, y1_seg)],
                      fill=thick_border, width=max(2, int(1.5 * scale)))

    # ── outer border around the two-row header ─────────────────────────────
    x_left, x_right = col_x[0], table_right
    draw.rectangle([x_left, r1y0, x_right, r2y1],
                   outline=thick_border, width=max(1, scale // 2))
    draw.line([(x_left, r2y0), (x_right, r2y0)],
              fill=thick_border, width=max(1, scale // 2))

    new_img.save(png_path)


# ── main chart builder ──────────────────────────────────────────────────────────

def make_chart():
    print("Loading data …")
    all_rows = build_rows()

    # Print M2SVCC weights used
    print("\nM2SVCC weights selected per scenario × ped level:")
    for scen in SCENARIO_ORDER:
        for ped in PED_LEVELS:
            w = get_best_m2svcc_weight(scen, ped)
            print(f"  {scen:<18} {ped}: {w}")

    m2_pcts = [r["pct_for_color"] for r in all_rows if r["is_m2svcc"] == 1]
    max_pct  = float(np.percentile([abs(p) for p in m2_pcts], 90)) if m2_pcts else 30.0
    max_pct  = max(max_pct, 10.0)

    color_scale = alt.Scale(
        domain=[-max_pct, -max_pct / 3, 0, max_pct / 3, max_pct],
        range=COLOR_RANGE, clamp=True,
    )

    # Split rows by ped level
    ped_rows = {ped: [r for r in all_rows if r["ped_level"] == PED_LABEL[ped]]
                for ped in PED_LEVELS}

    # ── per-ped sections (no x-axis labels — headers drawn by PIL) ──────────
    sections = []
    for ped in PED_LEVELS:
        heatmap = make_heatmap_section(ped_rows[ped], color_scale, False, False)
        left    = make_placeholder(LEFT_COL_W, CELL_HEIGHT)
        right   = make_placeholder(5, CELL_HEIGHT)
        sections.append(alt.hconcat(left, heatmap, right, spacing=0))

    body = alt.vconcat(*sections, spacing=30)

    chart = (
        alt.vconcat(body, spacing=0)
        .properties(
            title=alt.TitleParams(
                text="Phase Design Comparison: Performance vs Permitted Right Actuated",
                subtitle=[
                    "Columns: Permitted Right | Protected Right | LPI | Delayed Turn | Exclusive",
                    "Each group shows: Actuated (grey) | SVCC (beige) | M2SVCC (color-coded)",
                    "Metrics: Sustainability (Fuel, Energy) | Vehicle Mobility | Bike Mobility | Ped Mobility | Safety (Veh-Ped, Veh-Bike, Veh-Veh)",
                    "Color: % change vs Permitted Right Actuated  |  Blue=improvement  |  Red=regression",
                    "* p < 0.05   ** p < 0.01  (Welch t-test + Mann-Whitney vs Permitted Right Actuated)",
                ],
                fontSize=14, subtitleFontSize=10, anchor="middle", offset=10,
            )
        )
        .configure(
            font="Liberation Serif",
            padding={"left": 5, "right": 30, "top": 5, "bottom": 5},
        )
        .configure_view(stroke=None, strokeOpacity=0, strokeWidth=0,
                        continuousWidth=300, continuousHeight=300)
        .configure_axis(domainColor="#aaaaaa", grid=False,
                        labelFont="Liberation Serif", titleFont="Liberation Serif")
        .configure_title(font="Liberation Serif", subtitleFont="Liberation Serif")
    )
    return chart, max_pct


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)

    chart, max_pct = make_chart()

    out_png  = os.path.join(OUT_DIR, "fig_scenario_comparison_table.png")
    out_html = os.path.join(OUT_DIR, "fig_scenario_comparison_table.html")

    print("\nRendering PNG …")
    import vl_convert as vlc
    png_bytes = vlc.vegalite_to_png(vl_spec=chart.to_json(), scale=2)
    with open(out_png, "wb") as f:
        f.write(png_bytes)

    # PIL post-processing
    add_category_labels_pil(out_png, scale=2)       # category labels on left
    add_two_row_header_pil(out_png, scale=2)         # grouped header (expands canvas)
    add_colorbar_pil(out_png, max_pct, scale=2)      # colorbar at bottom
    add_ped_labels_pil(out_png, list(PED_LABEL.values()), scale=2)  # ped labels

    print(f"Saved PNG  → {out_png}")

    chart.save(out_html)
    print(f"Saved HTML → {out_html}")