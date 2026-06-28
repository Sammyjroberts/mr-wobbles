"""paths.py — resolve the repo's data directories from the installed package.

Code lives in src/balancer/; the data artifacts (printed STL, computed gains,
summaries) live at the repo root. With an editable install (`uv sync`), the
package stays in place, so walking up from this file lands on the repo root.
"""
from pathlib import Path

# src/balancer/paths.py -> parents[2] == repo root
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CAD_DIR = PROJECT_ROOT / "cad"
OUTPUTS_DIR = PROJECT_ROOT / "outputs"
HARDWARE_DIR = PROJECT_ROOT / "hardware"

STL_PATH = CAD_DIR / "balancer_chassis_v1.stl"
KC_REAL_PATH = OUTPUTS_DIR / "Kc_real.npy"
