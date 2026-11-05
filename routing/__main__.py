"""python -m routing [--scenario NAME] [--bends 45|90] [--json] [--all]

Prints the rerouted waypoints and metrics for the exercise scenario (or a
variant). `--json` prints the machine-readable result; `--all --json` prints
every scenario (this is what the companion web page embeds)."""
from __future__ import annotations

import argparse
import json
import sys

from .reroute import Constraints, reroute
from .scenarios import SCENARIOS, scenario
from .validate import Context, validate


def run_one(name: str, bends: int | None):
    run, obstacles, c = scenario(name)
    if bends:
        c = Constraints(**{**vars(c), "bend": bends})
    route = reroute(run, obstacles, c)
    ctx = Context(run.start, run.end, obstacles, c.clearance, run.section, (45.0, 90.0), c.min_straight,
                  c.ceiling_z, c.ceiling_clearance, c.soffit_z, c.zone, run.system)
    violations = validate(route.waypoints, ctx) if route.waypoints else []
    return run, obstacles, c, route, violations


def describe(name: str, run, obstacles, c, route, violations) -> str:
    lines = [f"== {name}: {SCENARIOS[name]}",
             f"   run {run.id} {run.start} -> {run.end}  system={run.system}  "
             f"section half-extents w={run.section.half_width:.3f} h={run.section.half_height:.3f}",
             f"   obstacles {[o.to_json() for o in obstacles]}",
             f"   clearance {c.clearance} m  bend {c.bend} deg  ceiling {c.ceiling_z}  soffit {c.soffit_z}",
             f"   -> {route.status.upper()}" + (f" ({route.reason})" if route.reason else "") + (f"  side={route.side}" if route.side else "")]
    for p in route.waypoints:
        lines.append(f"      ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    if route.waypoints:
        lines.append(f"   length {route.length_m:.3f} m  added {route.added_m:+.3f} m  bends {route.bends} "
                     f"{sorted({f['angle_deg'] for f in route.fittings})}  min clearance {route.min_clearance_m} m")
    for k in route.candidates:
        lines.append(f"   candidate {k.side:<6} {'ok' if k.feasible else 'no'} {k.reason or ''} {('cost ' + str(k.cost_m)) if k.cost_m is not None else ''}")
    if route.waypoints:
        lines.append("   validator: " + ("PASS" if not violations else "; ".join(f"{v.rule}: {v.message}" for v in violations)))
    else:
        lines.append("   validator: n/a (no route)")
    for n in route.notes:
        lines.append(f"   note: {n}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m routing")
    ap.add_argument("--scenario", default="default", choices=sorted(SCENARIOS))
    ap.add_argument("--bends", type=int, choices=(45, 90))
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--all", action="store_true", help="every scenario")
    args = ap.parse_args(argv)
    names = list(SCENARIOS) if args.all else [args.scenario]
    results = {}
    for name in names:
        run, obstacles, c, route, violations = run_one(name, args.bends)
        if args.json:
            results[name] = {
                "description": SCENARIOS[name],
                "run": {"id": run.id, "start": list(run.start), "end": list(run.end), "system": run.system,
                        "section": vars(run.section)},
                "obstacles": [o.to_json() for o in obstacles],
                "constraints": {k: (v.to_json() if hasattr(v, "to_json") else v) for k, v in vars(c).items()},
                "route": route.to_json(),
                "validation": {"passed": not violations, "violations": [v.to_json() for v in violations]},
            }
        else:
            print(describe(name, run, obstacles, c, route, violations))
            print()
    if args.json:
        json.dump(results if args.all else results[names[0]], sys.stdout, indent=2)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
