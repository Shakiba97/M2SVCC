"""One upper decision ("option", Options Framework).

    decision -> [yellow 5 s + all-red 2 s, only if the phase changes] -> 5 s phase-pure window

The option carries what the upper transition needs (S_upper, mask, accumulated costs,
CBF corrections) and the window-end targets s_ref given to the CAVs.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Option:
    k: int
    schedule: list
    n_clear: int
    s_upper: np.ndarray
    mask: np.ndarray
    ptr: int = 0
    veh_int: float = 0.0
    ped_int: float = 0.0
    bike_int: float = 0.0
    corr2: float = 0.0
    targets: dict = field(default_factory=dict)
    # How the phase was chosen ("policy", "random", "forced", or a teacher's "mpc" / "noise" /
    # "fallback") and the teacher's own choice (-1 if none); recorded with the upper transition.
    source: str = "policy"
    teacher_k: int = -1

    @property
    def done(self):
        return self.ptr >= len(self.schedule)

    def duration(self, dt):
        return len(self.schedule) * dt
