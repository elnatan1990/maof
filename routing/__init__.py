"""Deterministic MEP rerouting spike.

Public API:
    reroute(run, obstacles, constraints) -> Route      (Tier-1 line-probe router)
    validate(waypoints, ctx) -> list[Violation]        (the deterministic rule set)
    safety_gate(proposal, ctx, fallback) -> Decision   (AI output never bypasses validate)
"""
from .reroute import Box, Constraints, Route, Run, Section, reroute
from .validate import Context, Decision, Violation, safety_gate, validate

__all__ = [
    "Box", "Constraints", "Route", "Run", "Section", "reroute",
    "Context", "Decision", "Violation", "safety_gate", "validate",
]
