"""
Statistical comparison of weighting scenarios vs (1, 0) baseline.
Uses paired t-test and Wilcoxon signed-rank test across seeds.
"""

import os
import re
import numpy as np
from scipy import stats
from itertools import product

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "Results")

METRICS = {
    "waiting_time":   r"average waiting time for multi_scale scenario \(in s\): ([\d.]+)",
    "time_loss":      r"average time loss for multi_scale scenario \(in s\): ([\d.]+)",
    "queue_length":   r"average queue length for multi_scale scenario \(in m\): ([\d.]+)",
    "ped_time_loss":  r"average pedestrian time loss for multi_scale scenario \(in s\): ([\d.]+)",
    "rt_conflicts":   r"number of right-turn conflicts between vehicles and pedestrians: (\d+)",
}

SEEDS       = [10, 1000, 10000, 20000, 23423, 30000]
PED_LEVELS  = ["LowPed", "MedPed", "HighPed"]
WEIGHTS     = ["(0.1, 0.9)", "(0.2, 0.8)", "(0.25, 0.75)",
               "(0.5, 0.5)", "(0.6, 0.4)", "(0.7, 0.3)", "(1, 0)"]


def parse_file(path):
    with open(path) as f:
        text = f.read()
    result = {}
    for key, pattern in METRICS.items():
        m = re.search(pattern, text)
        result[key] = float(m.group(1)) if m else None
    return result


def load_all():
    data = {}  # data[weight][ped_level][seed] = {metric: value}
    for fname in os.listdir(RESULTS_DIR):
        if not fname.endswith(".txt"):
            continue
        # match: multi_scale_penetration(1)_EVratio(0)_Concurrent_(w1, w2)_PedLevel_seed(N).txt
        m = re.match(
            r"multi_scale_penetration\(1\)_EVratio\(0\)_Concurrent_"
            r"(\(\S+, \S+\))_(\w+)_seed\((\d+)\)\.txt",
            fname
        )
        if not m:
            continue
        weight, ped, seed = m.group(1), m.group(2), int(m.group(3))
        if weight not in WEIGHTS or ped not in PED_LEVELS or seed not in SEEDS:
            continue
        path = os.path.join(RESULTS_DIR, fname)
        data.setdefault(weight, {}).setdefault(ped, {})[seed] = parse_file(path)
    return data


def compare(data):
    baseline_weight = "(1, 0)"
    other_weights   = [w for w in WEIGHTS if w != baseline_weight]

    print(f"{'Metric':<18} {'Weight':<14} {'Ped':<9} "
          f"{'Base mean':>10} {'Alt mean':>10} {'Diff%':>7} "
          f"{'t-p':>8} {'W-p':>8} {'Sig':>4}")
    print("-" * 100)

    summary = []  # collect significant findings

    for ped in PED_LEVELS:
        for weight in other_weights:
            for metric in METRICS:
                baseline_vals, alt_vals = [], []
                for seed in SEEDS:
                    try:
                        b = data[baseline_weight][ped][seed][metric]
                        a = data[weight][ped][seed][metric]
                    except KeyError:
                        continue
                    if b is not None and a is not None:
                        baseline_vals.append(b)
                        alt_vals.append(a)

                if len(baseline_vals) < 3:
                    continue

                b_arr = np.array(baseline_vals)
                a_arr = np.array(alt_vals)
                diff_pct = (a_arr.mean() - b_arr.mean()) / b_arr.mean() * 100

                # paired t-test
                _, t_p = stats.ttest_rel(b_arr, a_arr)
                # Wilcoxon (needs at least 3 non-zero diffs)
                diffs = a_arr - b_arr
                try:
                    _, w_p = stats.wilcoxon(diffs)
                except ValueError:
                    w_p = float("nan")

                sig = "*" if t_p < 0.05 or w_p < 0.05 else ""
                sig += "*" if t_p < 0.01 or w_p < 0.01 else ""

                print(f"{metric:<18} {weight:<14} {ped:<9} "
                      f"{b_arr.mean():>10.3f} {a_arr.mean():>10.3f} {diff_pct:>7.1f}% "
                      f"{t_p:>8.4f} {w_p:>8.4f} {sig:>4}")

                if sig:
                    summary.append((metric, weight, ped, b_arr.mean(), a_arr.mean(),
                                    diff_pct, t_p, w_p, sig))

    print("\n" + "=" * 100)
    print("STATISTICALLY SIGNIFICANT RESULTS (p < 0.05 on t-test or Wilcoxon)")
    print("=" * 100)
    if not summary:
        print("  None found.")
    for (metric, weight, ped, bm, am, dp, tp, wp, sig) in summary:
        direction = "IMPROVED" if dp < 0 else "WORSENED"
        print(f"  [{sig}] {metric:<18} weight={weight:<14} ped={ped:<9} "
              f"base={bm:.3f} alt={am:.3f} ({dp:+.1f}%) t-p={tp:.4f} W-p={wp:.4f}  <- {direction}")


if __name__ == "__main__":
    data = load_all()
    compare(data)