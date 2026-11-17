"""Lower SAC (Sec. 6.3): continuous acceleration, one policy shared by every CAV."""
import math

import numpy as np
import torch
import torch.nn.functional as F

from ..common.nets import mlp, soft_update
from ..common.replay import ReplayBuffer


class LowerSAC:
    def __init__(self, obs_dim, cfg):
        self.cfg = cfg
        h = cfg.hidden
        self.actor = mlp(obs_dim, 2, h)
        self.q1, self.q2 = mlp(obs_dim + 1, 1, h), mlp(obs_dim + 1, 1, h)
        self.q1_t, self.q2_t = mlp(obs_dim + 1, 1, h), mlp(obs_dim + 1, 1, h)
        self.q1_t.load_state_dict(self.q1.state_dict())
        self.q2_t.load_state_dict(self.q2.state_dict())
        self.log_alpha = torch.zeros(1, requires_grad=True)
        self.opt_actor = torch.optim.Adam(self.actor.parameters(), lr=cfg.lr_lower)
        self.opt_q = torch.optim.Adam(list(self.q1.parameters()) + list(self.q2.parameters()), lr=cfg.lr_lower)
        self.opt_alpha = torch.optim.Adam([self.log_alpha], lr=cfg.lr_lower)
        if cfg.lower_action_range == "full":
            self.a_center, self.a_half = (cfg.a_max + cfg.a_min) / 2, (cfg.a_max - cfg.a_min) / 2
        else:                       # Sec. 3.4: a_raw = tanh(.) * a_max
            self.a_center, self.a_half = 0.0, cfg.a_max
        self.buffer = ReplayBuffer(cfg.buffer_lower, {
            "s": ((obs_dim,), np.float32), "a": ((1,), np.float32), "r": ((), np.float32),
            "s2": ((obs_dim,), np.float32), "done": ((), np.float32)})

    @property
    def alpha(self):
        return self.log_alpha.exp()

    def _dist(self, s):
        mu, log_std = self.actor(s).chunk(2, -1)
        return mu, torch.clamp(log_std, -5, 2).exp()

    def _sample(self, s):
        mu, std = self._dist(s)
        u = mu + std * torch.randn_like(mu)
        y = torch.tanh(u)
        logp = (-0.5 * ((u - mu) / std) ** 2 - torch.log(std) - 0.5 * math.log(2 * math.pi)).sum(-1)
        logp = logp - torch.log(self.a_half * (1 - y ** 2) + 1e-6).sum(-1)
        return self.a_center + self.a_half * y, logp

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        if len(obs) == 0:
            return np.zeros(0)
        s = torch.as_tensor(np.asarray(obs, dtype=np.float32))
        if deterministic:
            mu, _ = self._dist(s)
            a = self.a_center + self.a_half * torch.tanh(mu)
        else:
            a, _ = self._sample(s)
        return a.squeeze(-1).numpy().astype(float)

    def random_action(self, n):
        return self.a_center + self.a_half * np.random.uniform(-1, 1, size=n)

    def update(self, batch_size):
        b = self.buffer.sample(batch_size)
        s, a, r, s2, d = b["s"], b["a"], b["r"], b["s2"], b["done"]
        gamma = self.cfg.gamma_lo
        with torch.no_grad():
            a2, logp2 = self._sample(s2)
            q_t = torch.min(self.q1_t(torch.cat([s2, a2], -1)), self.q2_t(torch.cat([s2, a2], -1))).squeeze(-1)
            y = r + gamma * (1 - d) * (q_t - self.alpha * logp2)
        sa = torch.cat([s, a], -1)
        q_loss = F.mse_loss(self.q1(sa).squeeze(-1), y) + F.mse_loss(self.q2(sa).squeeze(-1), y)
        self.opt_q.zero_grad()
        q_loss.backward()
        self.opt_q.step()

        a_new, logp = self._sample(s)
        sa_new = torch.cat([s, a_new], -1)
        q_new = torch.min(self.q1(sa_new), self.q2(sa_new)).squeeze(-1)
        pi_loss = (self.alpha.detach() * logp - q_new).mean()
        self.opt_actor.zero_grad()
        pi_loss.backward()
        self.opt_actor.step()

        alpha_loss = -(self.log_alpha * (logp.detach() + self.cfg.lower_target_entropy)).mean()
        self.opt_alpha.zero_grad()
        alpha_loss.backward()
        self.opt_alpha.step()
        soft_update(self.q1_t, self.q1, self.cfg.tau_target)
        soft_update(self.q2_t, self.q2, self.cfg.tau_target)
        return {"lower_q_loss": q_loss.item(), "lower_pi_loss": pi_loss.item(), "lower_alpha": float(self.alpha.detach())}

    def state_dict(self):
        return {k: getattr(self, k).state_dict() for k in ("actor", "q1", "q2", "q1_t", "q2_t")} | {
            "log_alpha": self.log_alpha.detach().clone()}

    def load_state_dict(self, sd):
        for k in ("actor", "q1", "q2", "q1_t", "q2_t"):
            getattr(self, k).load_state_dict(sd[k])
        with torch.no_grad():
            self.log_alpha.copy_(sd["log_alpha"])
