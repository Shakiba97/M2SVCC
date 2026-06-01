"""
Export the scenario comparison table to a Word (.docx) file.
Reuses data-loading logic from generate_scenario_table.py.
"""

import os, sys, math
import numpy as np
from docx import Document
from docx.shared import Pt, RGBColor, Inches, Cm, Twips
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

# ── import data helpers from sibling script ─────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_scenario_table import (
    build_rows, SCENARIO_ORDER, _COL_TUPLES, COL_KEYS,
    METRIC_ORDER, CATEGORIES, PED_LEVELS, PED_LABEL, OUT_DIR,
)

# ── colour helpers ───────────────────────────────────────────────────────────

COLOR_STOPS = [
    (-1.00, (0x08, 0x30, 0x6b)),   # dark blue
    (-0.33, (0x6b, 0xae, 0xd6)),   # mid blue
    ( 0.00, (0xf7, 0xf7, 0xf7)),   # near white
    ( 0.33, (0xd4, 0x70, 0x7a)),   # mid red
    ( 1.00, (0xb2, 0x18, 0x2b)),   # dark red
]

ACT_BG   = (0xd9, 0xd9, 0xd9)   # grey
SVCC_BG  = (0xf5, 0xf0, 0xe8)   # beige
M2_BLUE  = (0xd2, 0xe4, 0xf8)   # light blue (neutral M2)
GRP_BG   = (0xeb, 0xf1, 0xfa)   # scenario group header
PED_BG   = (0x1a, 0x52, 0x76)   # ped-level header
WHITE    = RGBColor(0xff, 0xff, 0xff)
BLACK    = RGBColor(0x00, 0x00, 0x00)
DARK     = RGBColor(0x14, 0x14, 0x3c)

CAT_COLORS = {
    "Sustainability": (0xd9, 0xea, 0xd3),
    "Veh Mobility":   (0xd0, 0xe4, 0xf5),
    "Bike Mobility":  (0xff, 0xf2, 0xcc),
    "Ped Mobility":   (0xf4, 0xe1, 0xd2),
    "Safety":         (0xf4, 0xd2, 0xd2),
}

def to_hex(rgb_tuple):
    return f"{rgb_tuple[0]:02X}{rgb_tuple[1]:02X}{rgb_tuple[2]:02X}"


def lerp_color(t, stops):
    """Interpolate colour at position t in [-1, 1] from stop list."""
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


def rgb_to_hex(r, g, b):
    return f"{r:02X}{g:02X}{b:02X}"


# ── docx XML helpers ─────────────────────────────────────────────────────────

def set_cell_bg(cell, hex_color: str):
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


def set_cell_borders(cell, top=None, bottom=None, left=None, right=None):
    tc   = cell._tc
    tcPr = tc.get_or_add_tcPr()
    tcBorders = tcPr.find(qn("w:tcBorders"))
    if tcBorders is None:
        tcBorders = OxmlElement("w:tcBorders")
        tcPr.append(tcBorders)
    for side, val in [("top", top), ("bottom", bottom),
                      ("left", left), ("right", right)]:
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


def merge_row_cells(row, start, end):
    """Merge cells [start, end] inclusive in a table row (already added)."""
    cells = row.cells
    cells[start].merge(cells[end])


def set_col_width(cell, width_cm):
    tc   = cell._tc
    tcPr = tc.get_or_add_tcPr()
    tcW  = tcPr.find(qn("w:tcW"))
    if tcW is None:
        tcW = OxmlElement("w:tcW")
        tcPr.append(tcW)
    tcW.set(qn("w:w"),    str(int(width_cm * 567)))   # 1 cm ≈ 567 twips
    tcW.set(qn("w:type"), "dxa")


def add_paragraph(cell, text, bold=False, italic=False,
                  font_size=7, color=None, align="center",
                  line_spacing_pt=None):
    cell.paragraphs[0].clear()
    p   = cell.paragraphs[0]
    p.alignment = {
        "center": WD_ALIGN_PARAGRAPH.CENTER,
        "left":   WD_ALIGN_PARAGRAPH.LEFT,
        "right":  WD_ALIGN_PARAGRAPH.RIGHT,
    }.get(align, WD_ALIGN_PARAGRAPH.CENTER)
    if line_spacing_pt:
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after  = Pt(0)
        p.paragraph_format.line_spacing = Pt(line_spacing_pt)
    run = p.add_run(text)
    run.bold   = bold
    run.italic = italic
    run.font.size = Pt(font_size)
    if color:
        run.font.color.rgb = color
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER


thin_border  = {"val": "single", "sz": "4",  "color": "BBBBBB"}
thick_border = {"val": "single", "sz": "12", "color": "444444"}
none_border  = {"val": "none",   "sz": "0",  "color": "FFFFFF"}


# ── main builder ─────────────────────────────────────────────────────────────

def build_word_table():
    print("Loading data …")
    all_rows = build_rows()

    # Compute max_pct for colour scale
    m2_pcts = [r["pct_for_color"] for r in all_rows if r["is_m2svcc"] == 1]
    max_pct = float(np.percentile([abs(p) for p in m2_pcts], 90)) if m2_pcts else 30.0
    max_pct = max(max_pct, 10.0)

    # Index rows: data[ped_label][metric][col_key] = row_dict
    data = {}
    for r in all_rows:
        data.setdefault(r["ped_level"], {}).setdefault(r["metric"], {})[r["col_key"]] = r

    doc = Document()

    # ── page layout: landscape A3 ──────────────────────────────────────────
    section = doc.sections[0]
    section.orientation = 1          # landscape
    section.page_width   = Cm(42.0)
    section.page_height  = Cm(29.7)
    section.left_margin  = Cm(1.0)
    section.right_margin = Cm(1.0)
    section.top_margin   = Cm(1.2)
    section.bottom_margin= Cm(1.2)

    # ── title paragraph ────────────────────────────────────────────────────
    title_p = doc.add_paragraph()
    title_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title_run = title_p.add_run("Phase Design Comparison: Performance vs Permitted Right Actuated")
    title_run.bold = True
    title_run.font.size = Pt(11)
    sub_p = doc.add_paragraph()
    sub_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    sub_p.paragraph_format.space_before = Pt(0)
    sub_run = sub_p.add_run(
        "Color: % change vs Permitted Right Actuated  |  Blue = improvement  |  Red = regression\n"
        "* p < 0.05   ** p < 0.01  (Welch t-test + Mann-Whitney)"
    )
    sub_run.font.size = Pt(8)
    sub_run.font.color.rgb = RGBColor(0x44, 0x44, 0x44)

    # ── table dimensions ───────────────────────────────────────────────────
    # Columns: [cat label] [metric label] [13 data cols]
    n_data_cols  = len(COL_KEYS)   # 13
    n_total_cols = 2 + n_data_cols  # cat + metric + data

    # Column widths (cm)
    cat_w    = 1.8
    metric_w = 2.4
    data_w   = 1.55   # per data column
    total_w  = cat_w + metric_w + n_data_cols * data_w  # ≈ 24.4 cm

    # Rows per ped section:
    #  1 (ped header) + 1 (col-group header) + 1 (ctrl sub-header) + n_metrics (data)
    n_metrics = len(METRIC_ORDER)

    # Total rows:  3 ped sections × (3 header + n_metrics data) rows
    rows_per_ped = 3 + n_metrics
    total_rows   = len(PED_LEVELS) * rows_per_ped

    tbl = doc.add_table(rows=total_rows, cols=n_total_cols)
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    tbl.style     = "Table Grid"

    # Set column widths
    for row in tbl.rows:
        row.cells[0].width = Cm(cat_w)
        row.cells[1].width = Cm(metric_w)
        for ci in range(n_data_cols):
            row.cells[2 + ci].width = Cm(data_w)

    # ── track which row indices hold what type ─────────────────────────────
    # For each ped level p:
    #   row_base = p * rows_per_ped
    #   row_base+0 = ped-level header (spans all cols)
    #   row_base+1 = scenario group header (col-group)
    #   row_base+2 = controller sub-header (Actuated/SVCC/M2SVCC)
    #   row_base+3 … row_base+2+n_metrics = data rows

    for pi, ped in enumerate(PED_LEVELS):
        ped_label = PED_LABEL[ped]
        base      = pi * rows_per_ped

        # ── row 0: ped-level header ────────────────────────────────────────
        ped_row = tbl.rows[base]
        # merge all cells
        ped_row.cells[0].merge(ped_row.cells[-1])
        c = ped_row.cells[0]
        set_cell_bg(c, to_hex(PED_BG))
        add_paragraph(c, ped_label, bold=True, font_size=9, color=WHITE)
        ped_row.height = Twips(300)

        # ── row 1: scenario group header ───────────────────────────────────
        grp_row = tbl.rows[base + 1]
        # blank cat+metric cells
        set_cell_bg(grp_row.cells[0], "EBEBEB")
        add_paragraph(grp_row.cells[0], "")
        set_cell_bg(grp_row.cells[1], "EBEBEB")
        add_paragraph(grp_row.cells[1], "")

        ci = 2
        for scen in SCENARIO_ORDER:
            n_sub = 1 if scen == "Exclusive" else 3
            c0, c1 = ci, ci + n_sub - 1
            grp_row.cells[c0].merge(grp_row.cells[c1])
            c = grp_row.cells[c0]
            set_cell_bg(c, to_hex(GRP_BG))
            add_paragraph(c, scen, bold=True, font_size=8, color=DARK)
            # thick left border for scenario groups (except first)
            if scen != SCENARIO_ORDER[0]:
                set_cell_borders(c, left=thick_border)
            ci += n_sub
        grp_row.height = Twips(280)

        # ── row 2: controller sub-header ───────────────────────────────────
        sub_row = tbl.rows[base + 2]
        set_cell_bg(sub_row.cells[0], "EBEBEB")
        add_paragraph(sub_row.cells[0], "Category", bold=True, font_size=7, color=BLACK)
        set_cell_bg(sub_row.cells[1], "EBEBEB")
        add_paragraph(sub_row.cells[1], "Metric", bold=True, font_size=7, color=BLACK)

        for ci_data, (scen, ctrl) in enumerate(_COL_TUPLES):
            ci   = 2 + ci_data
            c    = sub_row.cells[ci]
            bg   = ACT_BG if ctrl == "Actuated" else SVCC_BG if ctrl == "SVCC" else M2_BLUE
            hex_bg = to_hex(bg)
            set_cell_bg(c, hex_bg)
            add_paragraph(c, ctrl, bold=True, font_size=7, color=BLACK)
            # thick left border at group starts
            if ci_data > 0 and _COL_TUPLES[ci_data - 1][0] != scen:
                set_cell_borders(c, left=thick_border)
        sub_row.height = Twips(260)

        # ── data rows ──────────────────────────────────────────────────────
        # Track category spans for vertical merging
        cat_merge_info = []  # (cat_label, start_data_row_abs, end_data_row_abs, color)
        for cat_label, row_start, row_end, cat_color in [
            (cl, rs, re, CAT_COLORS[cl])
            for cl, rs, re in CATEGORIES
            for cc in [CAT_COLORS.get(cl, WHITE)]
        ]:
            cat_merge_info.append((cat_label, base + 3 + row_start,
                                   base + 3 + row_end, CAT_COLORS[cat_label]))

        for mi, metric in enumerate(METRIC_ORDER):
            dr = base + 3 + mi
            row = tbl.rows[dr]
            row.height = Twips(310)

            # category cell (will be merged later)
            cat_label = next(cl for cl, rs, re in CATEGORIES if rs <= mi <= re)
            cat_color = CAT_COLORS[cat_label]
            set_cell_bg(row.cells[0], to_hex(cat_color))
            add_paragraph(row.cells[0], cat_label, bold=True, font_size=7,
                          color=BLACK, align="center")

            # metric label
            set_cell_bg(row.cells[1], "F5F5F5")
            add_paragraph(row.cells[1], metric, bold=False, font_size=7,
                          color=BLACK, align="left")

            # data cells
            for ci_data, (scen, ctrl) in enumerate(_COL_TUPLES):
                ci  = 2 + ci_data
                c   = row.cells[ci]
                col_key = f"{scen}|{ctrl}"
                r_dict = data.get(ped_label, {}).get(metric, {}).get(col_key)

                if r_dict is None:
                    set_cell_bg(c, "EEEEEE")
                    add_paragraph(c, "N/A", font_size=6)
                    continue

                cell_type = r_dict["cell_type"]
                pct       = r_dict.get("pct_for_color", 0.0)
                label     = r_dict["label"]
                has_stars = r_dict["has_stars"]

                if cell_type == "ref":
                    bg_hex = "FFFFFF"
                    txt_color = BLACK
                elif cell_type == "actuated":
                    bg_hex = to_hex(ACT_BG)
                    txt_color = BLACK
                elif cell_type == "svcc":
                    bg_hex = to_hex(SVCC_BG)
                    txt_color = BLACK
                else:  # m2svcc
                    rgb = pct_to_rgb(pct, max_pct)
                    bg_hex  = rgb_to_hex(*rgb)
                    # white text on dark cells
                    lum = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
                    txt_color = WHITE if lum < 100 else BLACK

                set_cell_bg(c, bg_hex)

                # thick left border at scenario boundaries
                if ci_data > 0 and _COL_TUPLES[ci_data - 1][0] != scen:
                    set_cell_borders(c, left=thick_border)

                # Two-line label: value on first line, pct on second
                lines = label.split("\n")
                c.paragraphs[0].clear()
                p = c.paragraphs[0]
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
                c.vertical_alignment = WD_ALIGN_VERTICAL.CENTER

        # ── merge category label cells vertically ──────────────────────────
        for cat_label, r_start, r_end, cat_color in cat_merge_info:
            if r_end > r_start:
                tbl.rows[r_start].cells[0].merge(tbl.rows[r_end].cells[0])
            c = tbl.rows[r_start].cells[0]
            set_cell_bg(c, to_hex(cat_color))
            # vertical text via rotation
            tc   = c._tc
            tcPr = tc.get_or_add_tcPr()
            textDir = OxmlElement("w:textDirection")
            textDir.set(qn("w:val"), "btLr")   # bottom-to-top = rotated 90°
            existing = tcPr.find(qn("w:textDirection"))
            if existing is not None:
                tcPr.remove(existing)
            tcPr.append(textDir)
            add_paragraph(c, cat_label, bold=True, font_size=7,
                          color=BLACK, align="center")

    # ── save ───────────────────────────────────────────────────────────────
    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, "fig_scenario_comparison_table.docx")
    doc.save(out_path)
    print(f"Saved → {out_path}")


if __name__ == "__main__":
    build_word_table()
    print("Done.")
