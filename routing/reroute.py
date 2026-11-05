"""Tier-1 deterministic line-probe router.

Reroutes a straight, axis-aligned MEP run (duct / pipe) around ONE blocking
obstacle (an axis-aligned bounding box) with a mandatory clearance, using clean
45 deg or 90 deg offsets. Metres, project shared coordinates, snapped to
millimetres. Standard library only.

How it works (the whole algorithm is ~60 lines, see `reroute()`):
  1. Inflate the obstacle by (clearance + the duct half-extent in the bend
     plane). This is a conservative Minkowski sum: a box, not a rounded box.
     A 45 deg diagonal that leaves an inflated corner keeps margin*sqrt(2) from
     the real corner, so the box is safe for diagonals too.
  2. Probe: does the straight run penetrate the inflated box? If not, keep it.
  3. Otherwise evaluate the four offset candidates (under, over, left, right).
     Each one has an explicit feasibility test against the ceiling, the slab
     soffit, the zone and the other obstacles. No feasible candidate ->
     NO_ROUTE with a reason (the platform raises a coordination issue).
  4. Build the waypoints for every feasible candidate (45 deg trapezoid or
     90 deg rectangle), cost them (added length + a per-bend penalty), pick
     the cheapest; ties are broken in a fixed order, so the result is stable.
  5. Return the route with its fittings, metrics and the measured clearance.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

Vec = tuple[float, float, float]

BEND_PENALTY_M = 0.25      # one bend "costs" 25 cm of extra duct (fitting + pressure loss)
AXIS_NAMES = ("x", "y", "z")
EPS = 1e-9


def snap(v: float) -> float:
    """Snap to millimetres. All geometry is compared at mm precision."""
    return round(v + 0.0, 3)


def vsnap(p: Iterable[float]) -> Vec:
    x, y, z = (snap(c) for c in p)
    return (x, y, z)


def dist(p: Vec, q: Vec) -> float:
    return math.dist(p, q)


def angle_deg(u: Vec, v: Vec) -> float:
    """Angle between two direction vectors, in degrees (0 = collinear)."""
    nu, nv = math.sqrt(sum(c * c for c in u)), math.sqrt(sum(c * c for c in v))
    if nu < EPS or nv < EPS:
        return 0.0
    c = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v)) / (nu * nv)))
    return round(math.degrees(math.acos(c)), 1)


@dataclass(frozen=True)
class Box:
    """Axis-aligned bounding box, metres."""
    lo: Vec
    hi: Vec

    def inflated(self, margin: Vec) -> "Box":
        return Box(
            vsnap(c - m for c, m in zip(self.lo, margin)),
            vsnap(c + m for c, m in zip(self.hi, margin)),
        )

    def distance_to_point(self, p: Vec) -> float:
        return math.sqrt(sum(max(lo - c, 0.0, c - hi) ** 2 for c, lo, hi in zip(p, self.lo, self.hi)))

    def closest_on_segment(self, p: Vec, q: Vec) -> Vec:
        """The point of the segment nearest to the box. Distance-to-box is
        convex along a segment, so a ternary search on t in [0, 1] finds it."""
        def at(t: float) -> Vec:
            return tuple(a + (b - a) * t for a, b in zip(p, q))
        def f(t: float) -> float:
            return self.distance_to_point(at(t))
        lo, hi = 0.0, 1.0
        for _ in range(80):
            m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            if f(m1) <= f(m2):
                hi = m2
            else:
                lo = m1
        return min((at(t) for t in (lo, hi, 0.0, 1.0)), key=self.distance_to_point)

    def distance_to_segment(self, p: Vec, q: Vec) -> float:
        return self.distance_to_point(self.closest_on_segment(p, q))

    def clamp(self, p: Vec) -> Vec:
        return tuple(min(max(c, lo), hi) for c, lo, hi in zip(p, self.lo, self.hi))

    def contains(self, p: Vec, eps: float = EPS) -> bool:
        return all(lo - eps <= c <= hi + eps for c, lo, hi in zip(p, self.lo, self.hi))

    def to_json(self) -> dict:
        return {"min": list(self.lo), "max": list(self.hi)}


def segment_penetrates(p: Vec, q: Vec, box: Box) -> bool:
    """True if the segment passes through the OPEN interior of the box.
    Touching a face, an edge or a corner is not a penetration (that is what
    "exactly at the clearance" looks like)."""
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = q[i] - p[i]
        if abs(d) < EPS:
            if p[i] <= box.lo[i] + EPS or p[i] >= box.hi[i] - EPS:
                return False
            continue
        a, b = (box.lo[i] - p[i]) / d, (box.hi[i] - p[i]) / d
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t1 - t0 <= EPS:
            return False
    return t1 - t0 > EPS


@dataclass(frozen=True)
class Section:
    """Duct / pipe cross-section. The exercise models the duct as a line
    (all zeros). Half-extents include insulation and the flange allowance."""
    width: float = 0.0
    height: float = 0.0
    insulation: float = 0.0
    flange: float = 0.0

    @property
    def half_width(self) -> float:
        return self.width / 2 + self.insulation + self.flange

    @property
    def half_height(self) -> float:
        return self.height / 2 + self.insulation + self.flange

    @property
    def half_max(self) -> float:
        return max(self.half_width, self.half_height)

    def half_along(self, axis: int) -> float:
        """Half-extent of the straight duct along a perpendicular axis."""
        return self.half_height if axis == 2 else self.half_width


@dataclass(frozen=True)
class Run:
    id: str
    start: Vec
    end: Vec
    system: str = "duct"            # duct | pipe | gravity_pipe
    section: Section = Section()


@dataclass(frozen=True)
class Constraints:
    clearance: float = 0.15         # mandatory margin from any obstacle surface, m
    bend: int = 45                  # preferred offset: 45 or 90
    min_straight: float = 0.30      # minimum straight run between fittings, m
    ceiling_z: Optional[float] = None       # top of the suspended ceiling (Z_c), if known
    ceiling_clearance: float = 0.02         # margin above the ceiling, m
    soffit_z: Optional[float] = None        # slab soffit, if known (beam top in the scenario)
    zone: Optional[Box] = None              # the coordination zone (plenum bounds)
    side: str = "auto"                      # auto | under | over | left | right


@dataclass
class Candidate:
    side: str
    feasible: bool
    reason: str = ""
    cost_m: Optional[float] = None
    waypoints: list[Vec] = field(default_factory=list)


@dataclass
class Route:
    status: str                       # ok | no_route | needs_review
    waypoints: list[Vec]
    reason: str = ""
    side: str = ""
    fittings: list[dict] = field(default_factory=list)
    length_m: float = 0.0
    added_m: float = 0.0
    bends: int = 0
    min_clearance_m: Optional[float] = None
    candidates: list[Candidate] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def to_json(self) -> dict:
        return {
            "status": self.status,
            "reason": self.reason or None,
            "side": self.side or None,
            "waypoints": [list(p) for p in self.waypoints],
            "fittings": self.fittings,
            "metrics": {
                "length_m": self.length_m,
                "added_m": self.added_m,
                "bends": self.bends,
                "min_clearance_m": self.min_clearance_m,
            },
            "candidates": [
                {"side": c.side, "feasible": c.feasible, "reason": c.reason or None, "cost_m": c.cost_m}
                for c in self.candidates
            ],
            "notes": self.notes,
        }


# --- metrics -------------------------------------------------------------

def polyline_length(pts: list[Vec]) -> float:
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def fittings_of(pts: list[Vec]) -> list[dict]:
    out = []
    for i in range(1, len(pts) - 1):
        u = tuple(b - a for a, b in zip(pts[i - 1], pts[i]))
        v = tuple(b - a for a, b in zip(pts[i], pts[i + 1]))
        a = angle_deg(u, v)
        if a >= 0.5:
            out.append({"at": list(pts[i]), "angle_deg": a})
    return out


def segment_dir(p: Vec, q: Vec) -> Vec:
    return tuple(b - a for a, b in zip(p, q))


def duct_extents(d: Vec, neighbours: list[Vec], section: Section) -> Vec:
    """Half-extents of the duct BODY along each axis, for a segment with
    direction `d`. The width stays horizontal (Y for a run along X); the
    height follows the bend plane, so a vertical leg carries the height along
    the horizontal axis of the plane it bends in (found from its neighbours).
    A diagonal gets the height on both in-plane axes (conservative)."""
    hw, hh = section.half_width, section.half_height
    moving = [k for k in range(3) if abs(d[k]) > EPS]
    if moving == [2]:                       # vertical leg
        plane = 0
        for n in neighbours:
            horiz = [k for k in (0, 1) if abs(n[k]) > EPS]
            if horiz:
                plane = horiz[0]
                break
        e = [hw, hw, 0.0]
        e[plane] = hh
        return (e[0], e[1], e[2])
    e = [0.0, 0.0, 0.0]
    for k in range(3):
        if k in moving:
            e[k] = 0.0 if len(moving) == 1 else (hh if 2 in moving else hw)
        else:
            e[k] = hh if k == 2 else hw
    return (e[0], e[1], e[2])


def segment_extents(pts: list[Vec], i: int, section: Section) -> Vec:
    neighbours = []
    if i > 0:
        neighbours.append(segment_dir(pts[i - 1], pts[i]))
    if i + 2 < len(pts):
        neighbours.append(segment_dir(pts[i + 1], pts[i + 2]))
    return duct_extents(segment_dir(pts[i], pts[i + 1]), neighbours, section)


def min_clearance(pts: list[Vec], obstacles: Iterable[Box], section: Section) -> Optional[float]:
    """Smallest distance between the duct SURFACE and any obstacle: the
    centre-line distance minus the duct half-extent on the axis (or axes) of
    closest approach. For the exercise's line model it is the plain
    centre-line distance."""
    obstacles = list(obstacles)
    if not obstacles or len(pts) < 2:
        return None
    best = None
    for i in range(len(pts) - 1):
        e = segment_extents(pts, i, section)
        for box in obstacles:
            p = box.closest_on_segment(pts[i], pts[i + 1])
            delta = tuple(a - b for a, b in zip(p, box.clamp(p)))
            axes = [k for k in range(3) if abs(delta[k]) > 1e-9]
            d = math.sqrt(sum(c * c for c in delta))
            clear = d - (max(e[k] for k in axes) if axes else max(e))
            best = clear if best is None else min(best, clear)
    return snap(best)


# --- the router ------------------------------------------------------------

def _run_axis(run: Run) -> int:
    moving = [i for i in range(3) if abs(run.end[i] - run.start[i]) > EPS]
    if len(moving) != 1:
        raise ValueError("Tier-1 expects an axis-aligned straight run")
    return moving[0]


def _blocker(run: Run, obstacles: list[Box], c: Constraints) -> Optional[Box]:
    """The first obstacle the straight run penetrates, inflated per axis by
    clearance + the straight duct's half-extent along that axis."""
    a = _run_axis(run)
    hits = []
    for box in obstacles:
        margin = tuple(c.clearance + (0.0 if i == a else run.section.half_along(i)) for i in range(3))
        if segment_penetrates(run.start, run.end, box.inflated(margin)):
            hits.append((box.lo[a], box))
    hits.sort(key=lambda t: t[0])
    return hits[0][1] if hits else None


def _side_name(axis: int, sign: int) -> str:
    if axis == 2:
        return "under" if sign < 0 else "over"
    return "left" if sign < 0 else "right"


def _offset_waypoints(run: Run, box: Box, axis_o: int, sign: int, c: Constraints) -> list[Vec]:
    """Waypoints for one candidate. Trapezoid (45 deg) or rectangle (90 deg)."""
    a = _run_axis(run)
    s = 1 if run.end[a] > run.start[a] else -1
    h = run.section.half_along(axis_o)
    margin = [0.0, 0.0, 0.0]
    margin[a] = margin[axis_o] = c.clearance + h
    inf = box.inflated(tuple(margin))
    target = inf.hi[axis_o] if sign > 0 else inf.lo[axis_o]   # the offset coordinate
    base = run.start[axis_o]
    d = abs(target - base)
    entry = inf.lo[a] if s > 0 else inf.hi[a]
    exit_ = inf.hi[a] if s > 0 else inf.lo[a]
    lead = d if c.bend == 45 else 0.0                       # horizontal run of a 45 deg offset

    def at(along: float, off: float) -> Vec:
        p = list(run.start)
        p[a], p[axis_o] = along, off
        return vsnap(p)

    return [
        run.start,
        at(entry - s * lead, base),
        at(entry, target),
        at(exit_, target),
        at(exit_ + s * lead, base),
        run.end,
    ]


def _feasibility(run: Run, box: Box, obstacles: list[Box], axis_o: int, sign: int,
                 c: Constraints, pts: list[Vec]) -> str:
    """'' when feasible, otherwise the reason code."""
    a = _run_axis(run)
    h = run.section.half_along(axis_o)
    target = pts[2][axis_o]
    if axis_o == 2 and sign > 0 and run.system == "gravity_pipe":
        return "GRAVITY_CANNOT_RISE"
    if axis_o == 2 and sign < 0 and c.ceiling_z is not None:
        if target - h - c.ceiling_clearance < c.ceiling_z - EPS:
            return "PLENUM_TOO_SHALLOW"
    if axis_o == 2 and sign > 0 and c.soffit_z is not None:
        if target + h + c.clearance > c.soffit_z + EPS:
            return "SOFFIT_TOO_LOW"
    if c.zone is not None:
        hw = run.section.half_along(axis_o)
        if target - hw < c.zone.lo[axis_o] - EPS or target + hw > c.zone.hi[axis_o] + EPS:
            return "OUTSIDE_ZONE"
    s = 1 if run.end[a] > run.start[a] else -1
    if s * (pts[1][a] - run.start[a]) < c.min_straight - EPS or s * (run.end[a] - pts[4][a]) < c.min_straight - EPS:
        return "INSUFFICIENT_STRAIGHT"
    others = [b for b in obstacles if b is not box]
    for b in others:
        margin = tuple(c.clearance + run.section.half_max for _ in range(3))
        inf = b.inflated(margin)
        if any(segment_penetrates(pts[i], pts[i + 1], inf) for i in range(len(pts) - 1)):
            return "SECOND_OBSTACLE"      # Tier-2 territory
    return ""


def reroute(run: Run, obstacles: list[Box], c: Constraints = Constraints()) -> Route:
    """Tier-1: route one straight run around the first obstacle it hits."""
    a = _run_axis(run)
    run = Run(run.id, vsnap(run.start), vsnap(run.end), run.system, run.section)
    straight = polyline_length([run.start, run.end])
    box = _blocker(run, obstacles, c)
    if box is None:
        return Route("ok", [run.start, run.end], side="straight", length_m=snap(straight),
                     added_m=0.0, bends=0, min_clearance_m=min_clearance([run.start, run.end], obstacles, run.section),
                     notes=["straight run is clear"])

    perpendicular = [i for i in range(3) if i != a]
    order = sorted(perpendicular, key=lambda i: 0 if i == 2 else 1)   # z first: under, over, then lateral
    wanted = None if c.side == "auto" else c.side
    candidates: list[Candidate] = []
    for axis_o in order:
        for sign in (-1, 1):
            name = _side_name(axis_o, sign)
            pts = _offset_waypoints(run, box, axis_o, sign, c)
            reason = _feasibility(run, box, obstacles, axis_o, sign, c, pts)
            if wanted and name != wanted:
                reason = reason or "NOT_REQUESTED"
            if reason:
                candidates.append(Candidate(name, False, reason))
                continue
            added = polyline_length(pts) - straight
            cost = added + BEND_PENALTY_M * len(fittings_of(pts))
            candidates.append(Candidate(name, True, "", snap(cost), pts))

    feasible = [k for k in candidates if k.feasible]
    if not feasible:
        # the headline reason is the preferred candidate's (under, then over, then lateral)
        primary = next((k for k in candidates if k.reason != "NOT_REQUESTED"), candidates[0])
        return Route("no_route", [], reason=primary.reason, candidates=candidates,
                     notes=["no feasible offset: raise a coordination issue (sleeve / bulkhead / move the beam)"])

    best = min(feasible, key=lambda k: k.cost_m)        # min() keeps the first of equal costs -> stable tie-break
    pts = best.waypoints
    fittings = fittings_of(pts)
    status, notes = "ok", []
    if run.system == "gravity_pipe":
        status = "needs_review"
        notes.append("GRAVITY_RESLOPE_DOWNSTREAM: the drop propagates; the downstream run must be re-sloped")
    return Route(status, pts, side=best.side, fittings=fittings,
                 length_m=snap(polyline_length(pts)), added_m=snap(polyline_length(pts) - straight),
                 bends=len(fittings), min_clearance_m=min_clearance(pts, obstacles, run.section),
                 candidates=candidates, notes=notes)
