"""The deterministic rule set, and the AI safety gate built on it.

`validate()` is a pure, versioned function: the SAME code checks the router's
own output and anything an ML model proposes. The gate never lets a proposal
reach the model without passing it:

    propose (AI, untrusted) -> validate -> repair (bounded) -> validate ->
    fallback (the deterministic route, valid by construction) -> audit record
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import Optional

from .reroute import EPS, Box, Section, Vec, angle_deg, dist, segment_extents, segment_penetrates, snap, vsnap

RULESET_VERSION = "1.0"


@dataclass(frozen=True)
class Violation:
    rule: str
    message: str
    at: Optional[Vec] = None

    def to_json(self) -> dict:
        return {"rule": self.rule, "message": self.message, "at": list(self.at) if self.at else None}


@dataclass
class Context:
    start: Vec
    end: Vec
    obstacles: list[Box]
    clearance: float = 0.15
    section: Section = Section()
    allowed_angles: tuple[float, ...] = (45.0, 90.0)
    min_straight: float = 0.30
    ceiling_z: Optional[float] = None
    ceiling_clearance: float = 0.02
    soffit_z: Optional[float] = None
    zone: Optional[Box] = None
    system: str = "duct"
    min_slope: float = 0.01             # gravity pipes: >= 1 % (2 % for small bores, per local code)


# --- rules (each returns a list of violations) ----------------------------

def rule_endpoints(pts, ctx):
    v = []
    if len(pts) < 2:
        return [Violation("endpoints", "fewer than two waypoints")]
    if vsnap(pts[0]) != vsnap(ctx.start):
        v.append(Violation("endpoints", "first waypoint is not the run start", pts[0]))
    if vsnap(pts[-1]) != vsnap(ctx.end):
        v.append(Violation("endpoints", "last waypoint is not the run end", pts[-1]))
    return v


def rule_clearance(pts, ctx):
    """The duct body (centre-line + its half-extents on each axis, see
    `duct_extents`) must not enter the obstacle inflated by the clearance.
    Touching the inflated face is allowed: that is exactly the clearance."""
    v = []
    for i in range(len(pts) - 1):
        e = segment_extents(pts, i, ctx.section)
        margin = tuple(snap(ctx.clearance + ek) for ek in e)
        for box in ctx.obstacles:
            if segment_penetrates(pts[i], pts[i + 1], box.inflated(margin)):
                d = snap(box.distance_to_segment(pts[i], pts[i + 1]))
                v.append(Violation("clearance", f"segment {i} passes {d:.3f} m (centre-line) from an obstacle; "
                                                f"the duct surface needs {ctx.clearance:.2f} m", pts[i]))
    return v


def rule_bend_angles(pts, ctx):
    v = []
    for i in range(1, len(pts) - 1):
        u = tuple(b - a for a, b in zip(pts[i - 1], pts[i]))
        w = tuple(b - a for a, b in zip(pts[i], pts[i + 1]))
        a = angle_deg(u, w)
        if a >= 0.5 and all(abs(a - ok) > 0.5 for ok in ctx.allowed_angles):
            v.append(Violation("bend_angle", f"{a:.1f} deg bend is not in {ctx.allowed_angles}", pts[i]))
    return v


def rule_min_straight(pts, ctx):
    v = []
    for i in range(len(pts) - 1):
        d = dist(pts[i], pts[i + 1])
        if d < ctx.min_straight - EPS and d > EPS:
            v.append(Violation("min_straight", f"segment {i} is {d:.3f} m, minimum {ctx.min_straight:.2f} m", pts[i]))
    return v


def rule_ceiling(pts, ctx):
    if ctx.ceiling_z is None:
        return []
    floor = ctx.ceiling_z + ctx.section.half_height + ctx.ceiling_clearance
    return [Violation("ceiling", f"duct bottom below the ceiling line (centre {p[2]:.3f} < {floor:.3f})", p)
            for p in pts if p[2] < floor - EPS]


def rule_soffit(pts, ctx):
    if ctx.soffit_z is None:
        return []
    top = ctx.soffit_z - ctx.section.half_height - ctx.clearance
    return [Violation("soffit", f"duct top too close to the slab (centre {p[2]:.3f} > {top:.3f})", p)
            for p in pts if p[2] > top + EPS]


def rule_zone(pts, ctx):
    if ctx.zone is None:
        return []
    return [Violation("zone", "waypoint outside the coordination zone", p) for p in pts if not ctx.zone.contains(p)]


def rule_gravity_slope(pts, ctx):
    if ctx.system != "gravity_pipe":
        return []
    v = []
    for i in range(len(pts) - 1):
        p, q = pts[i], pts[i + 1]
        horiz = math.hypot(q[0] - p[0], q[1] - p[1])
        dz = q[2] - p[2]
        if dz > EPS:
            v.append(Violation("gravity_slope", "a gravity pipe may not rise", p))
        elif horiz > EPS and -dz < ctx.min_slope * horiz - EPS:
            v.append(Violation("gravity_slope", f"slope {(-dz / horiz) * 100:.2f} % below the minimum {ctx.min_slope * 100:.0f} %", p))
    return v


RULES = [rule_endpoints, rule_clearance, rule_bend_angles, rule_min_straight,
         rule_ceiling, rule_soffit, rule_zone, rule_gravity_slope]


def validate(waypoints: list[Vec], ctx: Context) -> list[Violation]:
    pts = [vsnap(p) for p in waypoints]
    out: list[Violation] = []
    for rule in RULES:
        out.extend(rule(pts, ctx))
    return out


# --- the safety gate -------------------------------------------------------

@dataclass
class Decision:
    status: str                      # accepted | repaired | fallback
    waypoints: list[Vec]
    violations_before: list[Violation] = field(default_factory=list)
    violations_after: list[Violation] = field(default_factory=list)
    audit: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {
            "status": self.status,
            "waypoints": [list(p) for p in self.waypoints],
            "violations_before": [v.to_json() for v in self.violations_before],
            "violations_after": [v.to_json() for v in self.violations_after],
            "audit": self.audit,
        }


def _repair(pts: list[Vec], ctx: Context, passes: int = 3) -> list[Vec]:
    """Bounded, deterministic repair: push every interior waypoint that sits
    inside an inflated obstacle out through its nearest face. Endpoints are
    never moved (they are the run's connectors)."""
    margin = tuple(ctx.clearance + ctx.section.half_max for _ in range(3))
    boxes = [b.inflated(margin) for b in ctx.obstacles]
    pts = [vsnap(p) for p in pts]
    for _ in range(passes):
        moved = False
        for i in range(1, len(pts) - 1):
            for box in boxes:
                p = pts[i]
                if not all(box.lo[k] + EPS < p[k] < box.hi[k] - EPS for k in range(3)):
                    continue
                # smallest push to a face, axes in a fixed order for determinism
                pushes = []
                for k in range(3):
                    pushes.append((p[k] - box.lo[k], k, box.lo[k]))
                    pushes.append((box.hi[k] - p[k], k, box.hi[k]))
                _, k, face = min(pushes)
                q = list(p)
                q[k] = face
                pts[i] = vsnap(q)
                moved = True
        if not moved:
            break
    return pts


def safety_gate(proposal: list[Vec], ctx: Context, fallback: list[Vec],
                model_version: str = "unknown") -> Decision:
    """Accept an AI proposal only if the deterministic rules pass; otherwise
    repair it (bounded) and re-check; otherwise use the deterministic route."""
    before = validate(proposal, ctx)
    audit = {
        "ruleset_version": RULESET_VERSION,
        "model_version": model_version,
        "input_sha256": hashlib.sha256(json.dumps(proposal, sort_keys=True).encode()).hexdigest()[:16],
        "rules_checked": [r.__name__ for r in RULES],
    }
    if not before:
        return Decision("accepted", [vsnap(p) for p in proposal], audit=audit)
    repaired = _repair(proposal, ctx)
    after = validate(repaired, ctx)
    if not after:
        return Decision("repaired", repaired, before, [], audit)
    fb_violations = validate(fallback, ctx)
    audit["fallback_valid"] = not fb_violations
    return Decision("fallback", [vsnap(p) for p in fallback], before, after, audit)
