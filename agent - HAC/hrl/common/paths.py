"""Put the M2SVCC repository root and the agent folder on sys.path.

The agent folder name ("agent - HAC") is not a valid Python package name, so the
scripts import this module first instead of relying on package-relative imports.
"""
import os
import sys

HAC_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPO_ROOT = os.path.dirname(HAC_DIR)
NETWORK_DIR = os.path.join(REPO_ROOT, "environment", "network_model")

for _p in (REPO_ROOT, HAC_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Do not leave new .pyc files inside the repository's tracked folders.
sys.dont_write_bytecode = True

# The repo's metric code creates matplotlib figures; never open windows.
os.environ.setdefault("MPLBACKEND", "Agg")

# SUMO backend (README "SUMO version and backend"). "libsumo" runs SUMO inside this process
# instead of a separate sumo process behind a TraCI socket: about 3x faster per training step,
# identical results. It works through SUMO's LIBSUMO_AS_TRACI switch, which makes every later
# `import traci` (here and in the repo's environment/ and agent/ code) load libsumo, so it is
# decided here, before anything imports traci. sumo-gui needs TraCI: HAC_SUMO_BACKEND=traci.
_backend = os.environ.get("HAC_SUMO_BACKEND", "libsumo")
if _backend not in ("libsumo", "traci"):
    raise ValueError(f"HAC_SUMO_BACKEND must be 'libsumo' or 'traci', not {_backend!r}")
if "traci" not in sys.modules:
    if _backend == "traci":
        os.environ.pop("LIBSUMO_AS_TRACI", None)
    else:
        import importlib.util
        if importlib.util.find_spec("libsumo") is None:
            print("WARNING: libsumo is not installed, using TraCI (pip install libsumo==1.26.0)")
        else:
            os.environ.setdefault("LIBSUMO_AS_TRACI", "quiet")
