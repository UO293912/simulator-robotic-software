"""Pytest bootstrap local para los tests del simulador 3D."""
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIMULATOR_DIR = PROJECT_ROOT / "simulator"

for candidate in (PROJECT_ROOT, SIMULATOR_DIR):
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)
