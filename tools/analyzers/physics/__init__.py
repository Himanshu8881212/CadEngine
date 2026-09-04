"""physics — independent reference structural FEA for part verification.

This package provides a clean-room, benchmark-validated hex8 linear-elastic
solver (`reference_fea`) and a geometric region resolver (`resolve_selector`)
used by the orchestrator to *verify* delivered parts. It is deliberately
independent of any per-part ``analysis.py`` and of any SIMP density weighting:
it analyses the as-built binary occupancy (``rho >= 0.5``).

Dependencies: numpy + scipy only. No network, no ``agents.*`` imports.
"""

from __future__ import annotations

from .fea import reference_fea, reference_modal
from .buckling import reference_buckling
from .convergence import convergence_study, coarsen_field
from .dfam import reference_dfam
from .printability import printability_report
from .selectors import (
    element_mask_to_node_ids,
    resolve_selector,
)

__all__ = [
    "reference_fea",
    "reference_modal",
    "reference_buckling",
    "convergence_study",
    "coarsen_field",
    "reference_dfam",
    "printability_report",
    "resolve_selector",
    "element_mask_to_node_ids",
]
