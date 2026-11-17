"""Command-line handling shared by the HAC scripts."""
import argparse
import os

import hrl  # noqa: F401  (sys.path setup)
from hac_config import load_config
from hrl.common.paths import HAC_DIR


def parser(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--config", help="JSON file with HACConfig overrides")
    p.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                   help="override one HACConfig field (repeatable), e.g. --set episodes=50")
    p.add_argument("--run-name", help="folder under 'agent - HAC/runs/' (default: config run_name)")
    return p


def setup(args):
    cfg = load_config(args.config, args.set)
    if args.run_name:
        cfg.run_name = args.run_name
    run_dir = os.path.join(HAC_DIR, "runs", cfg.run_name)
    os.makedirs(run_dir, exist_ok=True)
    return cfg, run_dir
