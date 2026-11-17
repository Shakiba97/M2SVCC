"""Small network helpers shared by both SACs."""
import torch
import torch.nn as nn


def mlp(n_in, n_out, hidden):
    return nn.Sequential(nn.Linear(n_in, hidden), nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU(),
                         nn.Linear(hidden, n_out))


def soft_update(target, source, tau):
    with torch.no_grad():
        for t, s in zip(target.parameters(), source.parameters()):
            t.mul_(1 - tau).add_(tau * s)
