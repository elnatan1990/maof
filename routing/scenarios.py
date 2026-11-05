"""The exercise scenario and its variants. Every number quoted in the document
comes from running these through `reroute()`; nothing is typed by hand."""
from __future__ import annotations

from .reroute import Box, Constraints, Run, Section

# Rigid structural element (concrete beam): X in [4.0, 4.6], Y in [0, 10], Z in [2.6, 3.2]
BEAM = Box((4.0, 0.0, 2.6), (4.6, 10.0, 3.2))
BEAM_MOVED = Box((4.4, 0.0, 2.6), (5.0, 10.0, 3.2))        # the architect moved it +0.40 m in X
A = (2.0, 5.0, 2.9)
B = (7.0, 5.0, 2.9)

# Assumption: downstand beam flush with the slab soffit -> the soffit is at the beam top (3.20).
PLENUM = Box((0.0, 0.0, 2.0), (10.0, 10.0, 3.2))
SOFFIT_Z = 3.2

# A realistic duct: 400 x 300 mm, 25 mm insulation, 15 mm flange allowance -> half-height 0.19 m.
# Its run sits at 2.85 (not 2.90): at 2.90 the duct TOP would be 11 cm from the slab, inside the clearance.
REAL_SECTION = Section(width=0.40, height=0.30, insulation=0.025, flange=0.015)
A_REAL = (2.0, 5.0, 2.85)
B_REAL = (7.0, 5.0, 2.85)

SCENARIOS = {
    "default":      "the exercise as given: centre-line duct, 15 cm clearance, 45 deg offsets",
    "ninety":       "same, with 90 deg offsets",
    "moved":        "the beam moved +0.40 m in X (the real-time event of section 2)",
    "real-section": "a 400x300 insulated duct (run at 2.85) under a 2.70 m suspended ceiling -> NO_ROUTE",
    "real-low":     "the same duct with the ceiling at 2.00 m -> feasible at centre-line 2.26",
    "gravity":      "a gravity drain -> needs_review (downstream re-slope)",
    "no-soffit":    "no slab information and a 2.50 m ceiling -> the router goes OVER",
}


def scenario(name: str = "default") -> tuple[Run, list[Box], Constraints]:
    base = Constraints(clearance=0.15, bend=45, min_straight=0.30, soffit_z=SOFFIT_Z, zone=PLENUM)
    if name == "default":
        return Run("duct-1", A, B), [BEAM], base
    if name == "ninety":
        return Run("duct-1", A, B), [BEAM], Constraints(**{**vars(base), "bend": 90})
    if name == "moved":
        return Run("duct-1", A, B), [BEAM_MOVED], base
    if name == "real-section":
        return (Run("duct-1", A_REAL, B_REAL, section=REAL_SECTION), [BEAM],
                Constraints(**{**vars(base), "ceiling_z": 2.70}))
    if name == "real-low":
        return (Run("duct-1", A_REAL, B_REAL, section=REAL_SECTION), [BEAM],
                Constraints(**{**vars(base), "ceiling_z": 2.00}))
    if name == "gravity":
        return Run("drain-1", A, B, system="gravity_pipe"), [BEAM], base
    if name == "no-soffit":
        return (Run("duct-1", A, B), [BEAM],
                Constraints(**{**vars(base), "soffit_z": None, "ceiling_z": 2.50, "zone": None}))
    raise KeyError(f"unknown scenario {name!r}; one of {', '.join(SCENARIOS)}")
