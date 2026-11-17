"""Uniform replay buffer used for B_upper and B_lower (Sec. 4.6)."""
import numpy as np
import torch


class ReplayBuffer:
    def __init__(self, capacity, fields):
        self.capacity, self.size, self.ptr = capacity, 0, 0
        self.data = {k: np.zeros((capacity,) + tuple(shape), dtype=dtype) for k, (shape, dtype) in fields.items()}

    def add_batch(self, **items):
        n = len(next(iter(items.values())))
        for i in range(n):
            for k, v in items.items():
                self.data[k][self.ptr] = v[i]
            self.ptr = (self.ptr + 1) % self.capacity
            self.size = min(self.size + 1, self.capacity)

    def sample(self, batch):
        idx = np.random.randint(0, self.size, size=batch)
        return {k: torch.as_tensor(v[idx]) for k, v in self.data.items()}

    def state_dict(self):
        return {"size": self.size, "ptr": self.ptr, "data": {k: v[:self.size] for k, v in self.data.items()}}
