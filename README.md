# Real-Time MEP Coordination — Design & Algorithmic Spike

**Elnatan Davidovitch** · elnatan1990@gmail.com · 050-6998223 · [linkedin.com/in/elndav](https://www.linkedin.com/in/elndav)
Companion: **[interactive page](https://maof-mep.vercel.app)** (3-D scene — move the beam yourself) · **[PDF](docs/MEP-Coordination-Spike.pdf)** · [תקציר בעברית (PDF)](docs/MEP-Coordination-Spike-HE.pdf) · `python -m routing` prints every number quoted below.

**Assumptions.** (1) The beam is a downstand flush with the slab: its top (Z = 3.20) is the soffit, so routing *over* it is infeasible. (2) The suspended-ceiling height `Z_c` comes from the architectural model; the exercise gives none, so the worked example leaves it open and a second case shows it biting. (3) The exercise's duct is a centre-line; a real 400 × 300 insulated duct is the second case. (4) Metres, project shared coordinates; the plugin converts Revit's internal feet. (5) The architect moves the beam +0.40 m in X.

## 1. Spatial routing — a deterministic line-probe router, validator-last

**Representation.** Obstacles are axis-aligned boxes inflated by *clearance + the duct's half-extent in the bend plane*: a conservative Minkowski sum (a box, not a rounded box). A 45° diagonal leaving an inflated corner keeps margin·√2 ≈ 0.21 m from the real corner, so the box is safe for diagonals too. Everything is snapped to and compared at millimetres, so "exactly 15 cm" is a clean pass, not a rounding accident.

**Why a geometric offset, not a search.** Ducts are axis-aligned with 45°/90° fittings, and a moved beam is one blocker on a straight run. For that case an analytic *line probe* (the Hightower / Mikami–Tabuchi family) is exact, deterministic and sub-millisecond:

1. **Probe** — does the straight run penetrate the inflated box? No → keep it.
2. **Candidates** — under, over, left, right, each with an explicit feasibility test. Under: `z = beam_bottom − c − h`, feasible iff `z − h − c_ceiling ≥ Z_c`; over: `z + h + c ≤ soffit`; lateral: inside the zone. None may cut a second obstacle; a gravity pipe may never rise.
3. **Waypoints** — a 45° trapezoid (or 90° rectangle) between the inflated faces; cost = added length + 0.25 m per bend; cheapest wins, ties break in a fixed order, so the result is stable.
4. **No feasible candidate → `NO_ROUTE(PLENUM_TOO_SHALLOW)`** and a coordination issue (BCF) for structural: sleeve, local bulkhead, or move the beam. A coordination tool must say *why* there is no route.
5. **Validate** — the rule set the AI gate uses (§3) checks the router's own output.

| approach | deterministic | bends | latency (estimates, one plenum zone) | note |
| --- | --- | --- | --- | --- |
| **geometric offset** (chosen: one blocker) | yes | 45°/90° by construction | < 1 ms | exact; one obstacle per probe |
| sparse orthogonal graph + A\* with turn penalty (the generalisation) | yes | orthogonal, then a chamfer pass → 45° | 1–30 ms | nodes only at inflated face coordinates; state = (node, direction) |
| A\* on a 5 cm voxel grid | yes | staircase → smoothing | 50 ms – 2 s; 800k voxels per 10 × 10 × 1 m | resolution fights clearance accuracy |
| 3-D visibility graph | yes | arbitrary angles | ms | 3-D shortest path among polyhedra is NP-hard; the vertex graph is a heuristic |
| RRT\* | no (sampled) | random → smoothing | 100 ms – s | for high-DOF kinematics, not ducts |

*With more time:* Tier 2 is the sparse graph, run only when the probe meets a second obstacle; its chamfer post-pass turns each Z-step into a 45° offset so both tiers emit the same fitting style.

![Cross-section at Y = 5: beam, clearance envelope, original and rerouted duct](docs/diagrams/route-xz.svg)

**Worked result** (`python -m routing`): inflated box X ∈ [3.85, 4.75], Z ∈ [2.45, 3.35]; over is infeasible (soffit), lateral is outside the zone (the beam spans all of Y) → **under**:
`A (2.0, 5.0, 2.90) → (3.40, 5.0, 2.90) → (3.85, 5.0, 2.45) → (4.75, 5.0, 2.45) → (5.20, 5.0, 2.90) → B (7.0, 5.0, 2.90)` — **5.373 m** (+0.373 m, +7.5 %), four 45° fittings, clearance 0.150 m on the flat run and 0.212 m at the diagonals. The 90° variant is 5.90 m (+18 %) with the same four fittings: 45° wins on length and pressure loss. A two-bend "V" would need 11–14° legs and passes 0.09 m from the beam: rejected. Real section (400 × 300, 25 mm insulation, 15 mm flange, h = 0.19): centre-line 2.26, duct bottom 2.07 — feasible only if the ceiling is below 2.05; under a 2.70 ceiling the answer is `NO_ROUTE`.

```text
reroute(run, obstacles, c):                              # routing/reroute.py, ~60 lines of logic
    box = first obstacle whose inflated box the straight run penetrates
    if box is None: return straight
    for side in (under, over, left, right):              # fixed order = stable tie-break
        inf    = box.inflated(c.clearance + run.section.half_in_plane(side))
        target = inf.face(side);  d = |target - run.level|;  lead = d if c.bend == 45 else 0
        pts    = [A, A.at(inf.entry - lead), (inf.entry, target), (inf.exit, target), B.at(inf.exit + lead), B]
        if not feasible(side, target, ceiling, soffit, zone, other_obstacles, min_straight): note reason; continue
        cost[side] = length(pts) - |AB| + 0.25 * bends(pts)
    best = argmin(cost) or NO_ROUTE(reason of the preferred side)
    assert validate(best.pts) == []                      # the gate's rule set, applied to ourselves
```

## 2. Real-time data flow — a 40 cm beam move, Revit-native

![Pipeline: Revit add-in, WebSocket gateway, NATS JetStream, spatial engine, push to the engineer and the architect](docs/diagrams/pipeline.svg)

1. **Capture (the plugin).** `Application.DocumentChanged` fires after the architect's transaction commits (Added/Modified/Deleted ids). Filter to geometric categories and the primary design option; compare `Element.VersionGuid` + a bbox hash so parameter edits are dropped. **One transaction = one `ModelDelta`**, applied atomically to the twin; no debounce (a move is one transaction), coalesce only while a delta for the same element is in flight. Feet → metres via `UnitUtils`, shared coordinates via `ActiveProjectLocation.GetTotalTransform()`; identity `{modelId}:{UniqueId}`, version `VersionGuid`. Echo suppression: our own accepted-proposal transactions are recognised by name and not re-sent. `DocumentChanged` = *live preview* proposal, `DocumentSynchronizedWithCentral` = *committed*. The beam lives in a linked RVT the mechanical engineer sees only after a link reload: the twin bridges that gap (ghost beam + rerouted duct before the reload, re-validation after it).
2. **Transport.** WebSocket + protobuf/MessagePack for *both* desktop and web: Revit add-ins are .NET Framework 4.8 sharing one AppDomain, where `Grpc.Net.Client`/`Google.Protobuf` clash with other vendors' add-ins and HTTP/2 bidi streaming needs WinHttp on Windows 11; grpc-web has no client streaming. gRPC stays service-to-service.
3. **Cloud.** Gateway (project-scoped tokens, ACL per subject) → **NATS JetStream**, partitioned by project (a zone is a fan-out concern, not an ordering key) → spatial engine. The *twin* is a materialised view of the event log (bounding volumes + centre-lines, ~100 B per element, 50k ≈ 5 MB; rebuilt from the log on failover) in an R-tree: update the beam in O(log n), query the runs intersecting old ∪ new inflated boxes, reroute, validate, append a `RoutingProposal` — at-least-once delivery, idempotent consumer keyed by (id, VersionGuid). One beam affecting 200 runs becomes **one `ChangeImpact{proposals[]}`**, pushed to the mechanical engineer *and* to the architect: "your move affects 3 runs" is the most valuable real-time signal in the product.
4. **Apply (the plugin).** An `ExternalEvent` draws the proposal as a DirectContext3D overlay; the model is not edited until the engineer accepts. Accept is a **compare-and-swap on the proposal's read-set** `{elementId: VersionGuid}` (beam, duct, other obstacles), otherwise `STALE_VERSION`. Reconnect: outbox + last acknowledged sequence; replay from it, or a zone snapshot if the log was compacted.
5. **Within seconds, nothing freezes.** Deltas (< 1 KB), never models (100 MB – 2 GB); the heavy model never leaves the client; an overlay, not an edit; one impact message, not N pushes; an incremental index. Budget (estimates): extract 5–20 ms · upload 20–60 · bus < 10 · index < 1 · route < 1 · validate < 1 · persist ~5 · push 20–60 · apply 10–30 → **p50 target 0.5 s, p99 target 3 s**. Not under our control: Revit's regeneration after the commit and `ExternalEvent` idle scheduling (a modal dialog delays it).

## 3. AI safety gate and the API contract

![Safety gate: the model emits weights or candidates, the validator decides, repair is bounded, fallback is deterministic, every decision is audited](docs/diagrams/safety-gate.svg)

**The guarantee comes from what the model may output.** The pattern model emits *cost weights or a ranking over deterministic candidates* — never geometry — so its best output is still a route the router built and validated. In the weaker mode (candidate waypoints in a strict JSON schema) the same `validate()` runs: clearance with the duct body, ceiling clearance on the duct *bottom*, gravity slope (monotonic, ≥ 1 %), bends ∈ {45°, 90°}, minimum straight, soffit, zone, endpoints. Fail → bounded deterministic repair (push waypoints out through the nearest inflated face, three passes) → re-validate → otherwise **fallback** to the deterministic route, valid by construction. Every decision writes an audit record (ruleset version, model version, input hash, rules passed and failed). Shadow mode first, a kill switch, and property tests: random obstacles + random perturbations ⇒ zero accepted violations (`tests/`).

**Contract** ([`schema/`](schema), examples in [`schema/examples/`](schema/examples)): `schemaVersion`, metres, shared coordinates, ids `{modelId}:{UniqueId}`, versions = `VersionGuid`, `requestId`/`transactionId` as idempotency keys. `ModelDelta` (plugin → engine, one committed transaction) · `RerouteRequest` → `RerouteResponse` (sync) · `RoutingProposal` inside a `ChangeImpact` (push) · `ProposalDecision` (accept/reject with the read-set, plus the created element ids) · `Error {code: NO_ROUTE | STALE_VERSION | INVALID_INPUT, message, details}`.

```json
// RerouteRequest (trimmed)
{ "schemaVersion": "1.0", "requestId": "req-01HZX4M2Q9", "projectId": "prj-7f2a",
  "run": { "id": "duct-1", "systemType": "duct", "start": [2.0, 5.0, 2.9], "end": [7.0, 5.0, 2.9],
           "targetElementIds": ["MEP-L03:7b1c2d3e-…-0001a2b3"], "size": "400x300", "system": "Supply Air" },
  "constraints": { "clearance_m": 0.15, "bendAngles": [45, 90], "preferredBend": 45, "minStraight_m": 0.30,
                   "soffitZ": 3.2, "zone": { "min": [0, 0, 2.0], "max": [10, 10, 3.2] } },
  "obstacles": [ { "id": "STR-L03:0a1b2c3d-…-000e1f20", "versionGuid": "0c2f8e1a-…",
                   "bbox": { "min": [4.0, 0, 2.6], "max": [4.6, 10, 3.2] } } ] }
```

```json
// RerouteResponse (trimmed) — the script's own output for the request above
{ "schemaVersion": "1.0", "requestId": "req-01HZX4M2Q9", "status": "ok", "side": "under",
  "waypoints": [[2.0,5.0,2.9], [3.4,5.0,2.9], [3.85,5.0,2.45], [4.75,5.0,2.45], [5.2,5.0,2.9], [7.0,5.0,2.9]],
  "fittings": [ { "at": [3.4,5.0,2.9], "angleDeg": 45 }, { "at": [3.85,5.0,2.45], "angleDeg": 45 },
                { "at": [4.75,5.0,2.45], "angleDeg": 45 }, { "at": [5.2,5.0,2.9], "angleDeg": 45 } ],
  "metrics": { "length_m": 5.373, "added_m": 0.373, "bends": 4, "min_clearance_m": 0.15 },
  "validation": { "passed": true, "rulesetVersion": "1.0", "violations": [] },
  "readSet": { "STR-L03:0a1b2c3d-…-000e1f20": "0c2f8e1a-…", "MEP-L03:7b1c2d3e-…-0001a2b3": "9a8b7c6d-…" },
  "candidates": [ { "side": "under", "feasible": true, "cost_m": 1.373 }, { "side": "over", "feasible": false, "reason": "SOFFIT_TOO_LOW" },
                  { "side": "left", "feasible": false, "reason": "OUTSIDE_ZONE" }, { "side": "right", "feasible": false, "reason": "OUTSIDE_ZONE" } ] }
```

**Delegation.** *Plugin (hers):* event capture and filtering, delta extraction and unit conversion, the overlay, accept = `Duct` elements + `NewElbowFitting` per `fittings[]`, echo suppression, the outbox. *Engine (mine):* gateway, twin, router, validator, persistence, push. *Shared:* the schemas, the example fixtures, a contract test both CIs run, and a **mock engine** (static JSON from `schema/examples/`) so the plugin is built and demoed before the engine exists.

---

```bash
python -m routing                    # the exercise: waypoints, metrics, candidates, validator verdict
python -m routing --scenario moved   # +0.40 m; also: ninety, real-section, real-low, gravity, no-soffit  (--all --json)
python -m pytest                     # 22 tests: the quoted numbers, feasibility, the gate, the contract examples
```

`routing/reroute.py` the router and geometry · `routing/validate.py` the rules and `safety_gate()` · `routing/scenarios.py` the exercise and variants · `schema/` the contract · `docs/` the PDF and diagrams · `web/` the companion page (optional, built after the time-box; every number in it comes from the script).
