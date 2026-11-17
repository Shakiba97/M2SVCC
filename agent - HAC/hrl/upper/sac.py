"""Upper SAC (Sec. 6.3): discrete phase choice with action masking (Sec. 5.3) and SMDP
discounting: an option lasts clearance + window, so the bootstrap uses
gamma_up ** (duration / window).
"""
import numpy as np
import torch
import torch.nn.functional as F

from ..common.nets import mlp, soft_update
from ..common.replay import ReplayBuffer


class UpperSAC:
    """Discrete SAC (Christodoulou, 2019) with masked logits: illegal phases get -inf."""

    def __init__(self, obs_dim, n_actions, cfg):
        self.cfg, self.n = cfg, n_actions
        h = cfg.hidden
        self.actor = mlp(obs_dim, n_actions, h)
        self.q1, self.q2 = mlp(obs_dim, n_actions, h), mlp(obs_dim, n_actions, h)
        self.q1_t, self.q2_t = mlp(obs_dim, n_actions, h), mlp(obs_dim, n_actions, h)
        self.q1_t.load_state_dict(self.q1.state_dict())
        self.q2_t.load_state_dict(self.q2.state_dict())
        self.log_alpha = torch.zeros(1, requires_grad=True)
        self.opt_actor = torch.optim.Adam(self.actor.parameters(), lr=cfg.lr_upper)
        self.opt_q = torch.optim.Adam(list(self.q1.parameters()) + list(self.q2.parameters()), lr=cfg.lr_upper)
        self.opt_alpha = torch.optim.Adam([self.log_alpha], lr=cfg.lr_upper)
        self.buffer = ReplayBuffer(cfg.buffer_upper, {
            "s": ((obs_dim,), np.float32), "a": ((), np.int64), "r": ((), np.float32),
            "s2": ((obs_dim,), np.float32), "done": ((), np.float32), "duration": ((), np.float32),
            "mask": ((n_actions,), np.float32), "mask2": ((n_actions,), np.float32)})

    @property
    def alpha(self):
        return self.log_alpha.exp()

    def _policy(self, s, mask):
        logits = self.actor(s).masked_fill(mask <= 0, float("-inf"))
        logp = torch.log_softmax(logits, -1)
        p = logp.exp()
        logp = torch.where(mask > 0, logp, torch.zeros_like(logp))   # avoid 0 * -inf
        return p, logp

    @torch.no_grad()
    def act(self, obs, mask, deterministic=False):
        s = torch.as_tensor(np.asarray(obs, dtype=np.float32)).unsqueeze(0)
        m = torch.as_tensor(np.asarray(mask, dtype=np.float32)).unsqueeze(0)
        p, _ = self._policy(s, m)
        if deterministic:
            return int(p.argmax(-1))
        return int(torch.multinomial(p, 1))

    @torch.no_grad()
    def value(self, obs, mask):
        """V(s) = sum_a pi(a|s) [min Q(s,a) - alpha log pi(a|s)] (used by the lookahead)."""
        s = torch.as_tensor(np.asarray(obs, dtype=np.float32)).reshape(-1, len(obs[0]) if np.ndim(obs) == 2 else len(obs))
        m = torch.as_tensor(np.asarray(mask, dtype=np.float32)).reshape(s.shape[0], -1)
        p, logp = self._policy(s, m)
        q = torch.min(self.q1(s), self.q2(s))
        return (p * (q - self.alpha * logp)).sum(-1).numpy()

    def update(self, batch_size, update_actor=True):
        """One SAC update. update_actor=False runs only the critic step (and the target update):
        used to pre-train the critic and, after an MPC warm start, while the actor is frozen."""
        b = self.buffer.sample(batch_size)
        s, a, r, s2, d, dur, m2 = b["s"], b["a"], b["r"], b["s2"], b["done"], b["duration"], b["mask2"]
        disc = self.cfg.gamma_up ** (dur / self.cfg.window)
        with torch.no_grad():
            p2, logp2 = self._policy(s2, m2)
            q_t = torch.min(self.q1_t(s2), self.q2_t(s2))
            v2 = (p2 * (q_t - self.alpha * logp2)).sum(-1)
            y = r + disc * (1 - d) * v2
        q1 = self.q1(s).gather(1, a.unsqueeze(1)).squeeze(1)
        q2 = self.q2(s).gather(1, a.unsqueeze(1)).squeeze(1)
        q_loss = F.mse_loss(q1, y) + F.mse_loss(q2, y)
        self.opt_q.zero_grad()
        q_loss.backward()
        self.opt_q.step()

        if not update_actor:
            soft_update(self.q1_t, self.q1, self.cfg.tau_target)
            soft_update(self.q2_t, self.q2, self.cfg.tau_target)
            return {"upper_q_loss": q_loss.item(), "upper_alpha": float(self.alpha.detach())}

        p, logp = self._policy(s, b["mask"])
        with torch.no_grad():
            q = torch.min(self.q1(s), self.q2(s))
        pi_loss = (p * (self.alpha.detach() * logp - q)).sum(-1).mean()
        self.opt_actor.zero_grad()
        pi_loss.backward()
        self.opt_actor.step()
        # Entropy target per state from its number of legal phases; states with a single
        # legal phase (G_p or min-green hold) carry no choice and are left out.
        entropy = -(p * logp).sum(-1).detach()
        n_legal = b["mask"].sum(-1)
        target = self.cfg.upper_target_entropy_ratio * torch.log(torch.clamp(n_legal, min=1))
        free = (n_legal > 1).float()
        alpha_loss = (self.log_alpha * (entropy - target) * free).sum() / torch.clamp(free.sum(), min=1)
        self.opt_alpha.zero_grad()
        alpha_loss.backward()
        self.opt_alpha.step()
        soft_update(self.q1_t, self.q1, self.cfg.tau_target)
        soft_update(self.q2_t, self.q2, self.cfg.tau_target)
        return {"upper_q_loss": q_loss.item(), "upper_pi_loss": pi_loss.item(), "upper_alpha": float(self.alpha.detach())}

    def state_dict(self):
        return {k: getattr(self, k).state_dict() for k in ("actor", "q1", "q2", "q1_t", "q2_t")} | {
            "log_alpha": self.log_alpha.detach().clone()}

    def load_state_dict(self, sd):
        for k in ("actor", "q1", "q2", "q1_t", "q2_t"):
            getattr(self, k).load_state_dict(sd[k])
        with torch.no_grad():
            self.log_alpha.copy_(sd["log_alpha"])
