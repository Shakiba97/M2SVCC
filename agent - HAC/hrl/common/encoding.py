"""Encodings shared by S_upper and S_lower."""
import numpy as np

from ..upper.signal_plan import NUM_PHASES


def one_hot(k, n=NUM_PHASES):
    v = np.zeros(n, np.float32)
    v[k] = 1.0
    return v
