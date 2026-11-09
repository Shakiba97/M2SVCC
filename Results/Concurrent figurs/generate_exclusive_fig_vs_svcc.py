"""
Exclusive-Asymmetric heatmap vs SVCC baseline (explicit _vs_svcc output names).
Logic identical to generate_exclusive_fig.py; only output filenames differ.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_exclusive_fig as g

OUT_PNG  = os.path.join(g.OUT_DIR, "fig_exclusive_altair_new_vs_svcc.png")
OUT_HTML = os.path.join(g.OUT_DIR, "fig_exclusive_altair_new_vs_svcc.html")

if __name__ == "__main__":
    print("Parsing Exclusive-Asymmetric results (baseline = SVCC) …")
    chart, max_pct = g.make_chart()

    import vl_convert as vlc
    png_bytes = vlc.vegalite_to_png(vl_spec=chart.to_json(), scale=2)
    with open(OUT_PNG, "wb") as f:
        f.write(png_bytes)

    g.add_category_labels_pil(OUT_PNG, scale=2)
    g.add_colorbar_pil(OUT_PNG, max_pct, scale=2)
    g.add_ped_labels_pil(OUT_PNG, list(g.PED_LABEL.values()), scale=2)
    print(f"Saved PNG  → {OUT_PNG}")

    chart.save(OUT_HTML)
    print(f"Saved HTML → {OUT_HTML}")