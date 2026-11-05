"""The numbers the document quotes are asserted here; the document cannot drift."""
import json
import random
from pathlib import Path

import pytest

from routing import Box, Constraints, Context, Run, Section, reroute, safety_gate, validate
from routing.reroute import fittings_of, polyline_length, segment_penetrates
from routing.scenarios import A, B, BEAM, BEAM_MOVED, PLENUM, scenario

ROOT = Path(__file__).resolve().parents[1]


def ctx_for(run, obstacles, c):
    return Context(run.start, run.end, obstacles, c.clearance, run.section, (45.0, 90.0), c.min_straight,
                   c.ceiling_z, c.ceiling_clearance, c.soffit_z, c.zone, run.system)


# --- section 1: the quoted numbers -----------------------------------------

def test_default_45_degree_route_matches_the_document():
    run, obstacles, c = scenario("default")
    r = reroute(run, obstacles, c)
    assert r.status == "ok" and r.side == "under"
    assert r.waypoints == [(2.0, 5.0, 2.9), (3.4, 5.0, 2.9), (3.85, 5.0, 2.45),
                           (4.75, 5.0, 2.45), (5.2, 5.0, 2.9), (7.0, 5.0, 2.9)]
    assert r.length_m == 5.373 and r.added_m == 0.373
    assert r.bends == 4 and {f["angle_deg"] for f in r.fittings} == {45.0}
    assert r.min_clearance_m == 0.15
    assert validate(r.waypoints, ctx_for(run, obstacles, c)) == []


def test_ninety_degree_route_is_longer():
    run, obstacles, c = scenario("ninety")
    r = reroute(run, obstacles, c)
    assert r.waypoints == [(2.0, 5.0, 2.9), (3.85, 5.0, 2.9), (3.85, 5.0, 2.45),
                           (4.75, 5.0, 2.45), (4.75, 5.0, 2.9), (7.0, 5.0, 2.9)]
    assert r.length_m == 5.9 and r.bends == 4 and {f["angle_deg"] for f in r.fittings} == {90.0}
    assert validate(r.waypoints, ctx_for(run, obstacles, c)) == []


def test_over_is_infeasible_because_the_beam_top_is_the_soffit():
    run, obstacles, c = scenario("default")
    r = reroute(run, obstacles, c)
    over = next(k for k in r.candidates if k.side == "over")
    assert not over.feasible and over.reason == "SOFFIT_TOO_LOW"
    lateral = [k for k in r.candidates if k.side in ("left", "right")]
    assert all(not k.feasible and k.reason == "OUTSIDE_ZONE" for k in lateral)


def test_two_bend_v_shape_would_clash():
    v = [A, (4.3, 5.0, 2.45), B]
    inflated = BEAM.inflated((0.15, 0.15, 0.15))
    assert segment_penetrates(v[0], v[1], inflated) and segment_penetrates(v[1], v[2], inflated)
    assert validate(v, ctx_for(Run("duct-1", A, B), [BEAM], Constraints()))  # clearance + angle violations


# --- section 2: the beam moved +40 cm --------------------------------------

def test_moved_beam_keeps_the_topology_and_shifts_four_points():
    run, obstacles, c = scenario("moved")
    r = reroute(run, obstacles, c)
    assert r.waypoints == [(2.0, 5.0, 2.9), (3.8, 5.0, 2.9), (4.25, 5.0, 2.45),
                           (5.15, 5.0, 2.45), (5.6, 5.0, 2.9), (7.0, 5.0, 2.9)]
    assert r.length_m == 5.373 and r.bends == 4
    old = reroute(*scenario("default")).waypoints
    assert validate(old, ctx_for(run, obstacles, c))  # the old route is no longer valid


# --- feasibility, sections, gravity ----------------------------------------

def test_real_section_under_a_normal_ceiling_has_no_route():
    run, obstacles, c = scenario("real-section")
    r = reroute(run, obstacles, c)
    assert r.status == "no_route" and r.reason == "PLENUM_TOO_SHALLOW"
    assert r.waypoints == []


def test_real_section_with_a_low_ceiling_routes_at_2_26():
    run, obstacles, c = scenario("real-low")
    r = reroute(run, obstacles, c)
    assert r.status == "ok"
    assert r.waypoints[2][2] == 2.26 and r.waypoints[2][0] == 3.66
    assert r.min_clearance_m >= 0.15
    assert validate(r.waypoints, ctx_for(run, obstacles, c)) == []


def test_without_slab_information_and_a_high_ceiling_the_router_goes_over():
    run, obstacles, c = scenario("no-soffit")
    r = reroute(run, obstacles, c)
    assert r.side == "over" and r.waypoints[2][2] == 3.35


def test_gravity_pipe_needs_review_and_never_goes_over():
    run, obstacles, c = scenario("gravity")
    r = reroute(run, obstacles, c)
    assert r.status == "needs_review" and r.side == "under"
    assert next(k for k in r.candidates if k.side == "over").reason == "GRAVITY_CANNOT_RISE"


def test_a_clear_run_is_left_alone():
    run = Run("duct-2", (2.0, 5.0, 2.3), (7.0, 5.0, 2.3))
    r = reroute(run, [BEAM], Constraints(soffit_z=3.2))
    assert r.waypoints == [run.start, run.end] and r.bends == 0


def test_random_beams_always_keep_the_clearance():
    rng = random.Random(7)
    for _ in range(200):
        x0 = rng.uniform(2.6, 5.6)
        beam = Box((round(x0, 3), 0.0, 2.6), (round(x0 + rng.uniform(0.2, 0.8), 3), 10.0, 3.2))
        run = Run("duct-1", A, B)
        c = Constraints(clearance=rng.choice([0.10, 0.15, 0.20]), bend=rng.choice([45, 90]), soffit_z=3.2, zone=PLENUM)
        r = reroute(run, [beam], c)
        if r.status == "no_route":
            assert r.reason == "INSUFFICIENT_STRAIGHT"
            continue
        assert r.min_clearance_m >= c.clearance
        assert validate(r.waypoints, ctx_for(run, [beam], c)) == []


# --- section 3: the safety gate ---------------------------------------------

def test_validator_catches_a_clash_and_a_bad_angle():
    ctx = ctx_for(Run("duct-1", A, B), [BEAM], Constraints(soffit_z=3.2))
    straight = validate([A, B], ctx)
    assert {v.rule for v in straight} == {"clearance"}
    kinked = validate([A, (3.0, 5.0, 2.7), (3.85, 5.0, 2.45), (4.75, 5.0, 2.45), (5.2, 5.0, 2.9), B], ctx)
    assert "bend_angle" in {v.rule for v in kinked}


def test_gate_repairs_an_ai_proposal_that_cuts_the_corner():
    run, obstacles, c = scenario("default")
    ctx = ctx_for(run, obstacles, c)
    good = reroute(run, obstacles, c).waypoints
    ai = list(good)
    ai[2] = (3.9, 5.0, 2.5)           # inside the inflated box: 10 cm from the beam, not 15
    ai[3] = (4.7, 5.0, 2.5)
    d = safety_gate(ai, ctx, good, model_version="pattern-model-0.3")
    assert d.status in ("repaired", "fallback")
    assert validate(d.waypoints, ctx) == []
    assert d.violations_before and d.audit["ruleset_version"] == "1.0"


def test_gate_falls_back_when_repair_cannot_help():
    run, obstacles, c = scenario("default")
    ctx = ctx_for(run, obstacles, c)
    good = reroute(run, obstacles, c).waypoints
    d = safety_gate([A, (4.3, 5.0, 2.45), B], ctx, good)
    assert d.status == "fallback" and d.waypoints == good and d.audit["fallback_valid"] is True


def test_gate_accepts_a_valid_proposal_unchanged():
    run, obstacles, c = scenario("ninety")
    ctx = ctx_for(run, obstacles, c)
    good = reroute(run, obstacles, c).waypoints
    d = safety_gate(good, ctx, good)
    assert d.status == "accepted" and d.waypoints == good


# --- the API contract examples validate against their schemas ---------------

def _check(instance, schema, path="$"):
    """A minimal JSON-Schema checker (type, required, properties, items, enum)."""
    t = schema.get("type")
    kinds = {"object": dict, "array": list, "string": str, "number": (int, float), "integer": int, "boolean": bool, "null": type(None)}
    if t:
        ok = any(isinstance(instance, kinds[x]) and not (x == "number" and isinstance(instance, bool)) for x in ([t] if isinstance(t, str) else t))
        assert ok, f"{path}: expected {t}, got {type(instance).__name__}"
    if "enum" in schema:
        assert instance in schema["enum"], f"{path}: {instance!r} not in {schema['enum']}"
    if isinstance(instance, dict):
        for key in schema.get("required", []):
            assert key in instance, f"{path}: missing required {key!r}"
        for key, sub in schema.get("properties", {}).items():
            if key in instance:
                _check(instance[key], sub, f"{path}.{key}")
        if schema.get("additionalProperties") is False:
            extra = set(instance) - set(schema.get("properties", {}))
            assert not extra, f"{path}: unexpected keys {sorted(extra)}"
    if isinstance(instance, list) and "items" in schema:
        for i, item in enumerate(instance):
            _check(item, schema["items"], f"{path}[{i}]")
        if "minItems" in schema:
            assert len(instance) >= schema["minItems"], f"{path}: fewer than {schema['minItems']} items"


@pytest.mark.parametrize("name", ["model-delta", "reroute-request", "reroute-response", "routing-proposal",
                                  "proposal-decision", "error"])
def test_contract_examples_validate(name):
    schema = json.loads((ROOT / "schema" / f"{name}.schema.json").read_text(encoding="utf-8"))
    for example in sorted((ROOT / "schema" / "examples").glob(f"{name}*.json")):
        _check(json.loads(example.read_text(encoding="utf-8")), schema, example.name)


def test_reroute_response_example_is_the_scripts_own_output():
    example = json.loads((ROOT / "schema" / "examples" / "reroute-response.json").read_text(encoding="utf-8"))
    r = reroute(*scenario("default"))
    assert example["waypoints"] == [list(p) for p in r.waypoints]
    assert example["metrics"]["length_m"] == r.length_m
