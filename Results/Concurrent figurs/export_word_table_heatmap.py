"""
Export heatmap comparison tables to Word (.docx) for all four figure variants:
  - fig_asymmetric_altair_new_vs_svcc
  - fig_asymmetric_altair_new_vs_actuated
  - fig_exclusive_altair_new_vs_svcc
  - fig_exclusive_altair_new_vs_actuated

Run as:
    python export_word_table_heatmap.py          # generates all four
    python export_word_table_heatmap.py asym svcc
    python export_word_table_heatmap.py excl actuated
"""

import os
import sys
import numpy as np
from docx import Document
from docx.shared import Pt, RGBColor, Cm, Twips
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Results", "Concurrent figurs")

# ── colour helpers ────────────────────────────────────────────────────────────

COLOR_STOPS = [
    (-1.00, (0x08, 0x30, 0x6b)),
    (-0.33, (0x6b, 0xae, 0xd6)),
    ( 0.00, (0xf7, 0xf7, 0xf7)),
    ( 0.33, (0xd4, 0x70, 0x7a)),
    ( 1.00, (0xb2, 0x18, 0x2b)),
]

REF_BG   = (0xff, 0xfb, 0xe6)   # light yellow — reference (actuated or svcc baseline)
GRY_BG   = (0xd9, 0xd9, 0xd9)   # grey — actuated column in vs-svcc figures
WHITE    = RGBColor(0xff, 0xff, 0xff)
BLACK    = RGBColor(0x00, 0x00, 0x00)
DARK     = RGBColor(0x14, 0x14, 0x3c)
PED_BG   = (0x1a, 0x52, 0x76)

CAT_COLORS = {
    "Sustainability": (0xd9, 0xea, 0xd3),
    "Mobility":       (0xd0, 0xe4, 0xf5),
    "Safety":         (0xf4, 0xd2, 0xd2),
}

CATEGORIES_DEF = [
    ("Sustainability", 0, 0),
    ("Mobility",       1, 4),
    ("Safety",         5, 5),
]


def lerp_color(t, stops):
    t = max(-1.0, min(1.0, t))
    for i in range(len(stops) - 1):
        t0, c0 = stops[i]
        t1, c1 = stops[i + 1]
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return tuple(int(c0[j] + (c1[j] - c0[j]) * f) for j in range(3))
    return stops[-1][1]


def pct_to_rgb(pct, max_pct):
    if max_pct == 0:
        return (0xf7, 0xf7, 0xf7)
    t = max(-1.0, min(1.0, pct / max_pct))
    return lerp_color(t, COLOR_STOPS)


def to_hex(rgb):
    return f"{rgb[0]:02X}{rgb[1]:02X}{rgb[2]:02X}"


# ── docx helpers ──────────────────────────────────────────────────────────────

def set_cell_bg(cell, hex_color):
    tc   = cell._tc
    tcPr = tc.get_or_add_tcPr()
    shd  = OxmlElement("w:shd")
    shd.set(qn("w:val"),   "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"),  hex_color)
    existing = tcPr.find(qn("w:shd"))
    if existing is not None:
        tcPr.remove(existing)
    tcPr.append(shd)


def set_cell_borders(cell, **sides):
    tc   = cell._tc
    tcPr = tc.get_or_add_tcPr()
    tcBorders = tcPr.find(qn("w:tcBorders"))
    if tcBorders is None:
        tcBorders = OxmlElement("w:tcBorders")
        tcPr.append(tcBorders)
    for side, val in sides.items():
        if val is None:
            continue
        el = OxmlElement(f"w:{side}")
        el.set(qn("w:val"),   val.get("val",   "single"))
        el.set(qn("w:sz"),    val.get("sz",    "4"))
        el.set(qn("w:space"), val.get("space", "0"))
        el.set(qn("w:color"), val.get("color", "000000"))
        existing = tcBorders.find(qn(f"w:{side}"))
        if existing is not None:
            tcBorders.remove(existing)
        tcBorders.append(el)


def add_paragraph(cell, text, bold=False, italic=False,
                  font_size=7, color=None, align="center"):
    cell.paragraphs[0].clear()
    p = cell.paragraphs[0]
    p.alignment = {"center": WD_ALIGN_PARAGRAPH.CENTER,
                   "left":   WD_ALIGN_PARAGRAPH.LEFT}.get(align, WD_ALIGN_PARAGRAPH.CENTER)
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after  = Pt(0)
    run = p.add_run(text)
    run.bold      = bold
    run.italic    = italic
    run.font.size = Pt(font_size)
    if color:
        run.font.color.rgb = color
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER


thin_border  = {"val": "single", "sz": "4",  "color": "BBBBBB"}
thick_border = {"val": "single", "sz": "12", "color": "444444"}


# ── data cell writer ──────────────────────────────────────────────────────────

def write_data_cell(cell, label, pct_for_color, max_pct, cell_type, has_stars):
    """
    cell_type: 'ref'     → light yellow background, no %, just raw value
               'actuated' → grey background (used in vs-svcc tables)
               'colored'  → heatmap color based on pct_for_color
    """
    if cell_type == "ref":
        bg_hex    = to_hex(REF_BG)
        txt_color = BLACK
    elif cell_type == "actuated":
        bg_hex    = to_hex(GRY_BG)
        txt_color = BLACK
    else:
        rgb = pct_to_rgb(pct_for_color, max_pct)
        bg_hex = to_hex(rgb)
        lum = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
        txt_color = WHITE if lum < 100 else BLACK

    set_cell_bg(cell, bg_hex)

    lines = label.split("\n")
    cell.paragraphs[0].clear()
    p = cell.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after  = Pt(0)
    p.paragraph_format.line_spacing  = Pt(8)
    for li, line in enumerate(lines):
        if li > 0:
            p.add_run("\n")
        run = p.add_run(line)
        run.bold      = (li == 0 and has_stars)
        run.font.size = Pt(6.5) if li == 0 else Pt(5.5)
        run.font.color.rgb = txt_color
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER


# ── main builder ──────────────────────────────────────────────────────────────

def build_word_table(fig_type: str, baseline: str):
    """
    fig_type : 'asymmetric' | 'exclusive'
    baseline : 'svcc' | 'actuated'
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    if fig_type == "asymmetric" and baseline == "svcc":
        import generate_asymmetric_fig as g
        title_text = "Concurrent-Asymmetric: Performance vs SVCC Baseline"
        subtitle   = ("Color: % change vs SVCC  |  Blue = improvement  |  Red = regression\n"
                      "* p < 0.05   ** p < 0.01  (paired t-test & Wilcoxon vs SVCC)\n"
                      "Grey = Actuated reference  |  Seeds: 10, 1000, 10000, 23423, 30000")
        out_name   = "table_asymmetric_vs_svcc.docx"

        # Build row data using the module's own builder
        PED_LEVELS = g.PED_LEVELS
        PED_LABEL  = g.PED_LABEL
        METRIC_ORDER = g.METRIC_ORDER
        COL_SORT   = g.COL_SORT       # ["Actuated", "SVCC\n(Baseline)"] + WEIGHTS

        all_ped_rows = {ped: g.build_ped_rows(ped) for ped in PED_LEVELS}

        def get_cell_type(row_dict):
            if row_dict["is_actuated_int"] == 1:
                return "actuated"
            if row_dict["is_svcc_int"] == 1:
                return "ref"
            return "colored"

        col_keys = COL_SORT

    elif fig_type == "asymmetric" and baseline == "actuated":
        import generate_asymmetric_fig_vs_actuated as g
        title_text = "Concurrent-Asymmetric: Performance vs Actuated Baseline"
        subtitle   = ("Color: % change vs Actuated  |  Blue = improvement  |  Red = regression\n"
                      "* p < 0.05   ** p < 0.01  (paired t-test & Wilcoxon vs Actuated)\n"
                      "Yellow = Actuated reference  |  Seeds: 10, 1000, 10000, 23423, 30000")
        out_name   = "table_asymmetric_vs_actuated.docx"

        PED_LEVELS = g.PED_LEVELS
        PED_LABEL  = g.PED_LABEL
        METRIC_ORDER = g.METRIC_ORDER
        COL_SORT   = g.COL_SORT       # ["Actuated\n(Baseline)", "SVCC"] + WEIGHTS

        all_ped_rows = {ped: g.build_ped_rows(ped) for ped in PED_LEVELS}

        def get_cell_type(row_dict):
            if row_dict["is_ref_int"] == 1:
                return "ref"
            return "colored"

        col_keys = COL_SORT

    elif fig_type == "exclusive" and baseline == "svcc":
        import generate_exclusive_fig as g
        title_text = "Exclusive-Asymmetric: Performance vs SVCC Baseline"
        subtitle   = ("Color: % change vs SVCC  |  Blue = improvement  |  Red = regression\n"
                      "* |% change| ≥ 3%   ** |% change| > 10%\n"
                      "Grey = Actuated reference  |  IQR outlier filtering applied")
        out_name   = "table_exclusive_vs_svcc.docx"

        PED_LEVELS = g.PED_LEVELS
        PED_LABEL  = g.PED_LABEL
        METRIC_ORDER = g.METRIC_ORDER
        COL_SORT   = g.COL_SORT

        all_ped_rows = {ped: g.build_ped_rows(ped) for ped in PED_LEVELS}

        def get_cell_type(row_dict):
            if row_dict["is_actuated_int"] == 1:
                return "actuated"
            if row_dict["is_svcc_int"] == 1:
                return "ref"
            return "colored"

        col_keys = COL_SORT

    elif fig_type == "exclusive" and baseline == "actuated":
        import generate_exclusive_fig_vs_actuated as g
        title_text = "Exclusive-Asymmetric: Performance vs Actuated Baseline"
        subtitle   = ("Color: % change vs Actuated  |  Blue = improvement  |  Red = regression\n"
                      "* |% change| ≥ 3%   ** |% change| > 10%\n"
                      "Yellow = Actuated reference  |  IQR outlier filtering applied")
        out_name   = "table_exclusive_vs_actuated.docx"

        PED_LEVELS = g.PED_LEVELS
        PED_LABEL  = g.PED_LABEL
        METRIC_ORDER = g.METRIC_ORDER
        COL_SORT   = g.COL_SORT

        all_ped_rows = {ped: g.build_ped_rows(ped) for ped in PED_LEVELS}

        def get_cell_type(row_dict):
            if row_dict["is_ref_int"] == 1:
                return "ref"
            return "colored"

        col_keys = COL_SORT

    else:
        raise ValueError(f"Unknown combination: fig_type={fig_type!r}, baseline={baseline!r}")

    # Compute max_pct for colour scale
    all_pcts = []
    for ped in PED_LEVELS:
        for r in all_ped_rows[ped]:
            ct = get_cell_type(r)
            if ct == "colored":
                all_pcts.append(r["pct_for_color"])
    max_pct = float(np.percentile([abs(p) for p in all_pcts], 90)) if all_pcts else 30.0
    max_pct = max(max_pct, 10.0)

    # Index: data[ped][metric][col_key] = row_dict
    data = {}
    for ped in PED_LEVELS:
        ped_label = PED_LABEL[ped]
        data[ped_label] = {}
        for r in all_ped_rows[ped]:
            data[ped_label].setdefault(r["metric"], {})[r["col_key"]] = r

    # ── Document setup ──────────────────────────────────────────────────────
    doc = Document()
    section = doc.sections[0]
    section.orientation = 1
    section.page_width   = Cm(42.0)
    section.page_height  = Cm(29.7)
    section.left_margin  = Cm(1.0)
    section.right_margin = Cm(1.0)
    section.top_margin   = Cm(1.2)
    section.bottom_margin= Cm(1.2)

    title_p = doc.add_paragraph()
    title_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    tr = title_p.add_run(title_text)
    tr.bold = True
    tr.font.size = Pt(11)

    sub_p = doc.add_paragraph()
    sub_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    sub_p.paragraph_format.space_before = Pt(0)
    sr = sub_p.add_run(subtitle)
    sr.font.size = Pt(8)
    sr.font.color.rgb = RGBColor(0x44, 0x44, 0x44)

    # ── Table structure ──────────────────────────────────────────────────────
    n_data_cols  = len(col_keys)
    n_total_cols = 2 + n_data_cols        # cat label | metric label | data cols

    cat_w    = 1.8
    metric_w = 2.6
    data_w   = (42.0 - 2.0 - cat_w - metric_w) / n_data_cols  # fill page width

    n_metrics    = len(METRIC_ORDER)
    rows_per_ped = 3 + n_metrics          # ped header + col-group header + ctrl header + data
    total_rows   = len(PED_LEVELS) * rows_per_ped

    tbl = doc.add_table(rows=total_rows, cols=n_total_cols)
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    tbl.style     = "Table Grid"

    for row in tbl.rows:
        row.cells[0].width = Cm(cat_w)
        row.cells[1].width = Cm(metric_w)
        for ci in range(n_data_cols):
            row.cells[2 + ci].width = Cm(data_w)

    for pi, ped in enumerate(PED_LEVELS):
        ped_label = PED_LABEL[ped]
        base      = pi * rows_per_ped

        # Row 0: ped-level header (spans all columns)
        ped_row = tbl.rows[base]
        ped_row.cells[0].merge(ped_row.cells[-1])
        c = ped_row.cells[0]
        set_cell_bg(c, to_hex(PED_BG))
        add_paragraph(c, ped_label, bold=True, font_size=9, color=WHITE)
        ped_row.height = Twips(300)

        # Row 1: group header — span all data cols under "M2SVCC" label
        grp_row = tbl.rows[base + 1]
        set_cell_bg(grp_row.cells[0], "EBEBEB")
        add_paragraph(grp_row.cells[0], "")
        set_cell_bg(grp_row.cells[1], "EBEBEB")
        add_paragraph(grp_row.cells[1], "")

        # Find the first M2SVCC column index (after ref + SVCC or just after ref)
        m2_start = None
        for ci, ck in enumerate(col_keys):
            # M2SVCC weight keys look like "(0.8, 0.2)" etc.
            if ck.startswith("(") and "," in ck:
                m2_start = ci
                break

        if m2_start is not None:
            # Blank cells before M2SVCC span
            for ci in range(m2_start):
                set_cell_bg(grp_row.cells[2 + ci], "EBEBEB")
                add_paragraph(grp_row.cells[2 + ci], "")
            # Merge M2SVCC cells and label
            grp_row.cells[2 + m2_start].merge(grp_row.cells[-1])
            c = grp_row.cells[2 + m2_start]
            set_cell_bg(c, "dce9f5")
            add_paragraph(c, "M2SVCC (Vehicle/Pedestrian weighting factor)",
                          bold=True, font_size=8, color=DARK)
        else:
            for ci in range(n_data_cols):
                set_cell_bg(grp_row.cells[2 + ci], "EBEBEB")
                add_paragraph(grp_row.cells[2 + ci], "")
        grp_row.height = Twips(280)

        # Row 2: controller sub-header
        sub_row = tbl.rows[base + 2]
        set_cell_bg(sub_row.cells[0], "EBEBEB")
        add_paragraph(sub_row.cells[0], "Category", bold=True, font_size=7, color=BLACK)
        set_cell_bg(sub_row.cells[1], "EBEBEB")
        add_paragraph(sub_row.cells[1], "Metric",   bold=True, font_size=7, color=BLACK)

        for ci, ck in enumerate(col_keys):
            c = sub_row.cells[2 + ci]
            # Determine bg for header cell
            if "Baseline" in ck or ck == "Actuated\n(Baseline)":
                hbg = to_hex(REF_BG)
            elif ck.startswith("Actuated"):
                hbg = to_hex(GRY_BG)
            elif ck.startswith("SVCC"):
                hbg = "f5f0e8"
            else:
                hbg = "d2e4f8"
            set_cell_bg(c, hbg)
            # Clean label for header (remove newlines)
            header_lbl = ck.replace("\n", " ")
            add_paragraph(c, header_lbl, bold=True, font_size=7, color=BLACK)
            if ci > 0 and m2_start is not None and ci == m2_start:
                set_cell_borders(c, left=thick_border)
        sub_row.height = Twips(260)

        # Data rows
        cat_merge_info = []
        for cat_label, row_start, row_end in CATEGORIES_DEF:
            cat_merge_info.append((cat_label, base + 3 + row_start,
                                   base + 3 + row_end, CAT_COLORS[cat_label]))

        for mi, metric in enumerate(METRIC_ORDER):
            dr  = base + 3 + mi
            row = tbl.rows[dr]
            row.height = Twips(310)

            # Category cell (merged later)
            cat_label = next(cl for cl, rs, re in CATEGORIES_DEF if rs <= mi <= re)
            set_cell_bg(row.cells[0], to_hex(CAT_COLORS[cat_label]))
            add_paragraph(row.cells[0], cat_label, bold=True, font_size=7,
                          color=BLACK, align="center")

            # Metric label
            set_cell_bg(row.cells[1], "F5F5F5")
            add_paragraph(row.cells[1], metric, bold=False, font_size=7,
                          color=BLACK, align="left")

            # Data cells
            for ci, ck in enumerate(col_keys):
                c = row.cells[2 + ci]
                r_dict = data.get(ped_label, {}).get(metric, {}).get(ck)

                if r_dict is None:
                    set_cell_bg(c, "EEEEEE")
                    add_paragraph(c, "N/A", font_size=6)
                    continue

                ct        = get_cell_type(r_dict)
                pct       = r_dict.get("pct_for_color", 0.0)
                label     = r_dict["label"]
                has_stars = r_dict["has_stars"]

                write_data_cell(c, label, pct, max_pct, ct, has_stars)

                if ci > 0 and m2_start is not None and ci == m2_start:
                    set_cell_borders(c, left=thick_border)

        # Merge category cells vertically and rotate text
        for cat_label, r_start, r_end, cat_color in cat_merge_info:
            if r_end > r_start:
                tbl.rows[r_start].cells[0].merge(tbl.rows[r_end].cells[0])
            c = tbl.rows[r_start].cells[0]
            set_cell_bg(c, to_hex(cat_color))
            tc   = c._tc
            tcPr = tc.get_or_add_tcPr()
            textDir = OxmlElement("w:textDirection")
            textDir.set(qn("w:val"), "btLr")
            existing = tcPr.find(qn("w:textDirection"))
            if existing is not None:
                tcPr.remove(existing)
            tcPr.append(textDir)
            add_paragraph(c, cat_label, bold=True, font_size=7, color=BLACK, align="center")

    out_path = os.path.join(OUT_DIR, out_name)
    doc.save(out_path)
    print(f"Saved → {out_path}")
    return out_path


if __name__ == "__main__":
    args = sys.argv[1:]

    combos = [
        ("asymmetric", "svcc"),
        ("asymmetric", "actuated"),
        ("exclusive",  "svcc"),
        ("exclusive",  "actuated"),
    ]

    if len(args) == 2:
        combos = [(args[0], args[1])]
    elif len(args) != 0:
        print("Usage: python export_word_table_heatmap.py [asym|excl] [svcc|actuated]")
        sys.exit(1)

    for ft, bl in combos:
        print(f"\n── Generating table: {ft} vs {bl} ──")
        build_word_table(ft, bl)

    print("\nAll done.")