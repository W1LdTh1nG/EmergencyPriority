# Game.dll decompile findings — Emergency Vehicle mod

Date: 2026-09-08. Source: `Game.dll` in this folder (Game Pass build, file dated 2026-06-27; newest named
version constant in `Game.Version` is `1.5.7f1 [6157.21012]`). Decompiled with `ilspycmd 9.1.0.7988`
(the 11.x package fails to install as a dotnet tool):

```
dotnet tool install -g ilspycmd --version 9.1.0.7988
ilspycmd -p -o <outdir> -r "D:\Games\XBoxGames\Cities- Skylines II - PC Edition\Content\Cities2_Data\Managed" Game.dll
```

Line numbers below refer to that decompile output (`Game.Simulation/*.cs` etc.).

---

## 1. Verdict on §5 (ghosting): VIABLE, and simpler than hypothesised

The hypothesis was "`Blocker` is written by navigation and consumed by the mover". Half right:

- `CarNavigationSystem.UpdateNavigationJob` (Burst) computes everything and writes **both** `Blocker` and
  `CarNavigation { m_TargetPosition, m_TargetRotation, m_MaxSpeed }` to the component store
  (`CarNavigationSystem.cs:1581-1582, 1650-1651, 1971-1975`).
- `CarMoveSystem.CarMoveJob` reads **`CarNavigation` only** — it never touches `Blocker`
  (`CarMoveSystem.cs`: type handles are `Car`, `CarNavigation`, `CarCurrentLane`, `PrefabRef`, `PseudoRandomSeed`
  read-only; `Moving`, `Transform`, `TransformFrame` read-write). `Blocker` is consumed by
  `StuckMovingObjectSystem` and the vehicle AI systems, not by movement.
- The mover sets the velocity vector to exactly `|CarNavigation.m_MaxSpeed|` in magnitude
  (`MathUtils.TryNormalize(ref value4, num3)` — verified in `Colossal.Mathematics.MathUtils:2001`, it rescales to
  `newLength`). Sign bit of `m_MaxSpeed` = reversing.
- Position advances by `velocity * (4/15 s)` per vehicle tick (a vehicle ticks once per 16 frames; both
  systems use the `UpdateFrame == frameIndex & 0xF` shared-component filter, so nav and move for a given
  vehicle happen in the **same frame**).

**Frame order** (`Game.Common/SystemOrder.cs:309-336`, all `GameSimulation`):
`CarNavigationSystem` → `CarNavigationSystem.Actions` → `AmbulanceAISystem`, `FireEngineAISystem`,
`PoliceCarAISystem`, … → `VehicleOutOfControlSystem` → `CarMoveSystem`.

So a mod system with `UpdateAfter<Mine, CarNavigationSystem.Actions>` + `UpdateBefore<Mine, CarMoveSystem>`
(or simply `UpdateBefore<Mine, AmbulanceAISystem>`), using the same `UpdateFrame` filter, can overwrite
`CarNavigation.m_MaxSpeed` and the mover will drive at that speed. No Burst code is bypassed; the value is
plain component data between two jobs.

### Why a raised speed actually moves the car
`UpdateNavigationTarget` advances the look-ahead target by `num7 = max(1 m, m_MaxSpeed * dt) + pivotOffset`
along the lane (`:1667`, `MoveTarget` calls `:1803-1862`) and finally clamps
`m_MaxSpeed = min(m_MaxSpeed, distance(pos, target) / dt)` (`:1971-1975`). For a blocked (speed ≈ 0) car the
target sits ≥ 1 m ahead, so the car may be driven at up to ~3.75 m/s (1 m / 0.267 s) without overshooting
the target. Next tick the target is re-advanced from the new position. **Crawl speed must therefore be
≤ ~3.5 m/s** (≈ 12 km/h) unless you also move `m_TargetPosition` yourself.

### Vanilla already ghosts — `CarLaneFlags.IgnoreBlocker`
`CarLaneSpeedIterator.UpdateMaxSpeed` (`:1537`):
```
maxBrakingSpeed = math.select(maxBrakingSpeed, 3f, ignore && maxBrakingSpeed < 3f);
```
where `ignore = (laneObject == m_Ignore)` and `m_Ignore = blocker.m_Blocker` iff `CarCurrentLane.m_LaneFlags`
has `IgnoreBlocker` (`CarNavigationSystem.cs:1543`). Vanilla sets the flag after a blocked-lane repath
(`:1151`) so a car drives *through* the single entity that blocked it at a floor of 3 m/s. That is exactly
the "pass slowly through" effect wanted, proving cars can overlap without side effects. Caveat: `CheckBlocker`
(`:1377-1382`) clears `IgnoreBlocker` whenever the iterator's blocker changes, so using this flag directly
would need re-setting every tick and gives one-tick stalls per car in a queue. Prefer the post-nav override.

### No collision/accident risk
`ObjectCollisionSystem` query requires `Any = { OutOfControl }` (`ObjectCollisionSystem.cs:700-716`). Normal
vehicles never collide. `VehicleCollisionIterator` is only run for `CarLaneFlags.Area` lanes (parking lots),
`CarNavigationSystem.cs:1928-1970`, not roads.

### Stuck detection won't fire on a crawling responder
`StuckMovingObjectSystem` (interval 4, `UpdateFrame = (frameIndex>>2) % 16`) marks `PathFlags.Stuck` only when
`Blocker.m_MaxSpeed < 6` (byte units = m/s × 2.295 → < 2.6 m/s) **and** following the `Blocker` chain forms a
cycle back to the vehicle (or its target/leader) or exceeds 100 links (`:71-131, 133-204`). It is a deadlock
detector, not a timer. A responder crawling at 3 m/s reports `Blocker.m_MaxSpeed ≈ 7` if you also rewrite the
byte; if you leave `Blocker` untouched it stays as nav computed it, which is fine because the responder is
moving so it will not be in a static cycle for long. Recommended: leave `Blocker` alone.

### Recommended ghosting implementation
System `EmergencyGhostSystem : GameSystemBase`, `GameSimulation`, ordered after `CarNavigationSystem.Actions`
and before `AmbulanceAISystem`; query `Car, CarNavigation, Blocker, CarCurrentLane, CarNavigationLane,
UpdateFrame`, exclude `Deleted, Temp, OutOfControl, ParkedCar, TripSource`; same shared-component filter as
nav (`frameIndex % 16`). Per entity:

1. `(car.m_Flags & CarFlags.Emergency) == 0` → skip.
2. "Still navigating" guard: skip if `CarNavigationLane` buffer empty or `CarCurrentLane.m_LaneFlags` has
   `EndOfPath | EndReached | ParkingSpace | Area | Connection | ResetSpeed`.
3. Skip if `navigation.m_MaxSpeed < 0` (reversing) or `blocker.m_Blocker == Entity.Null`.
4. Ghost only when `blocker.m_Type == BlockerType.Continuing` (car ahead in same/merging lane) and the blocker
   has a `Car` component (not a pedestrian → `Temporary`, not a train, not `Signal`/`Limit`/`Caution`).
   `Crossing`/`Oncoming` deliberately excluded to avoid visibly driving through cross traffic.
5. `crawl = clamp(setting, 0, 3.5)`; `allowed = currentSpeed + prefabCarData.m_Acceleration * (4/15)` so the
   ramp-up looks natural; `navigation.m_MaxSpeed = max(navigation.m_MaxSpeed, min(crawl, allowed))`.
   Write back `CarNavigation`.

Because `max()` is used, the override only kicks in when the car ahead is slower than the crawl, i.e. a real
jam. Moving traffic is untouched. Setting off → system disabled → exact vanilla.

---

## 2. §4 claims re-verified

| Claim | Status | Evidence |
|---|---|---|
| `CarFlags.Emergency` = sirens; cleared on ParkCar / when returning | ✔ | `AmbulanceAISystem.cs:742-760`, `FireEngineAISystem.cs:507, 825-841`, `PoliceCarAISystem.cs:961-981` |
| Flag not cleared on arrival (fire engine keeps it while extinguishing) | ✔ | Only `ParkCar`/return paths clear it; use nav-buffer / EndOfPath guard |
| `GetPriority` = 108 for Emergency, 100 civilian | ✔ | `VehicleUtils.cs:657-660` |
| Responders ignore signals/stop signs (`m_Priority < 108` gate) | ✔ | `CarLaneSpeedIterator.cs:412, 425, 440, 540, 553, 567`; `LaneSignalFlags.Physical` still blocks |
| `LaneSignal` fields, `m_Default`, Physical | ✔ | `Game.Net/LaneSignal.cs`; `LaneSignalFlags { CanExtend=1, Physical=2 }` |
| `TrafficLightSystem.GetNextSignalGroup` picks highest `m_Priority` and consumes petition | ✔ | `TrafficLightSystem.cs:360-396` (`m_Petitioner = Null; m_Priority = m_Default`), interval 4 |
| Vanilla petition walk bounded by braking distance | ✔ | `CarNavigationSystem.ReserveNavigationLanes` `:2274-2345`: walk budget ≈ 2×braking distance + vehicle length; ≈ 0 when stopped |
| Nav buffer is a rolling window | ✔ | `FillNavigationPaths`, `navigationLanes.RemoveAt(0)` at `:1922` |
| `StuckMovingObjectSystem` sets `PathFlags.Stuck`; AI deletes on `IsStuck` | ✔ | `StuckMovingObjectSystem.cs:125`; `AmbulanceAISystem.cs:234-237`, `FireEngineAISystem.cs:393-396`, `PoliceCarAISystem.cs:284-287` |
| `RequireNewPath` = Obsolete set and none of Pending/Failed/Stuck | ✔ | `VehicleUtils.cs:199-206` |
| "Reroute-on-blocked test" `m_Blocker != Null && Type != Temporary && m_MaxSpeed < 6` | ⚠ | This is the **StuckMovingObjectSystem pre-filter** (`:71-79, 100`), not an AI reroute test. The AI systems have no blocked-reroute logic at all; EmergencyPriority's mod uses it as its own heuristic. Fine to reuse. |

Extra: `FireEngineAISystem.cs:434` — an Emergency fire engine whose lane is `IsBlocked` and which is
"close enough" to the fire ends navigation and fights the fire from where it stands. Ghosting reduces how
often this triggers; harmless.

---

## 3. Goal #3 (keep the junction clear) — there is a vanilla mechanism: `LaneReservation`

`Game.Net.LaneReservation { m_Blocker, m_Next{offset,priority}, m_Prev{offset,priority} }`, `GetPriority()` =
max(next, prev). Written from `CarNavigationSystem.Actions.UpdateLaneReservationsJob` (`:2604-2628`, only ever
*raises* `m_Next` priority/offset). Rotated by `NetLaneReservationSystem` each frame for the lane's
`UpdateFrame` bucket: `prev = next; next = 0` (`NetLaneReservationSystem.cs`). So a reservation written once is
honoured for 16–32 frames; write it every ≤16 frames to hold it.

Consumers:
- **Cars**: `CarLaneSpeedIterator.CheckOverlappingLanes` `:1264-1276` — for every lane overlapping the one being
  entered, if that lane's reservation `priority > (own priority ± yield delta)` the car brakes to a stop before
  the overlap (`BlockerType.Crossing`, `m_Blocker = Null`). Civilians are 100, so a **108 reservation on the
  junction lanes stops all cross traffic**, signalled or not. Also `:155-158`: a lane reserved at exactly 102
  halves civilian drive speed (vanilla uses this via `ReserveOtherLanesInGroup(lane, 102)` for parallel lanes
  next to a responder), and `:1189`: a civilian whose *own* lane is reserved ≥108 gets effective priority 106
  so it clears out rather than yielding.
- **Pedestrians**: `CreatureTargetIterator.CheckOverlapLane` `:62-76` — if the road lane they're about to cross
  has reservation `priority >= 108` they stop at the kerb and queue. **Pedestrians come free.**

Vanilla emergency vehicles already enqueue 108 reservations along the nav buffer (`ReserveNavigationLanes`
`:2318-2325, 2346-2372`) but with the same braking-distance budget as the signal petition, i.e. nothing useful
when stopped. The mod's job is therefore the same as the green wave: walk `CarNavigationLane` with a distance
budget and write `LaneReservation.m_Next.m_Priority = 108, m_Offset = 255` on every lane that has the
component (junction lanes do; you can restrict to lanes owned by a `Node` if you want only the box). Do it in
the same system/tick as the signal petition. Nothing persisted in saves; stop writing and it decays within
32 frames.

Not to be used: `LaneSignal.m_Blocker` (that is the traffic-light system's own "who holds the bridge" marker).

---

## 4. Goal #1 (despawn) — unchanged, EmergencyPriority already does it
Confirmed against current code: after `StuckMovingObjectSystem` (interval 4), clear `Stuck`, set `Obsolete`.
Note `RequireNewPath` refuses while `Pending` is set, so also don't touch vehicles mid-pathfind.

---

## 5. Comparison with EmergencyPriority (fetched 2026-09-08, `main`)
- `EmergencyRepathSystem`: main-thread, interval 4, `UpdateAfter<…, StuckMovingObjectSystem>`, query
  `Car + Blocker + PathOwner` excl. `OutOfControl/Deleted/Temp`. Despawn guard + reroute with backoff.
- `GreenLightPrioritySystem`: main-thread, interval 4, `UpdateBefore<…, TrafficLightSystem>`, walks
  `CarNavigationLane` with a distance budget writing `LaneSignal.m_Priority = 108, m_Petitioner = entity`.
- Neither touches `CarNavigation` or `LaneReservation`. Ghosting and box-clearing are new work; both slot in
  as one additional system each. Main-thread is acceptable at these entity counts, but a Burst `IJobChunk` is
  straightforward for the ghost system since it needs no managed state.

**Decision recommendation:** fork EmergencyPriority (MIT), keep its two systems, add
`EmergencyGhostSystem` (post-nav `m_MaxSpeed` override) and extend `GreenLightPrioritySystem`'s walk to also
write `LaneReservation` (rename to e.g. `JunctionPrioritySystem`). Ship order unchanged: despawn → green wave →
reservations (cars + pedestrians in one go) → ghosting.

---

## 6. Open items for implementation
- Confirm at runtime that `Node`-owned lanes are the only ones with `LaneReservation`; if edge lanes have it too,
  reserving them at 108 will also stop side-street entries (probably desirable but should be a setting).
- The `ReserveOtherLanesInGroup(lane, 102)` half-speed effect could be extended along the same walk to make
  parallel-lane traffic slow down ahead of the responder (cheap, cosmetic, optional).
- Ghost crawl default 3 m/s; expose 1–3.5 m/s in settings.
- Emergency buses (evacuation) also carry `CarFlags.Emergency`; decide whether to include (query-level toggle).

---

## 7. Dispatch: who answers a new fire (added 2026-09-10)

Question: when a fire breaks out while an engine is returning to station, does the game send a spare from the
station or redirect the closer returning engine? **The closer one, and a returning engine is a candidate.**

- `FireRescueDispatchSystem` runs one pathfind per request whose targets are seeded by `FirePathfindSetup`:
  - every `Game.Buildings.FireStation` with `FireStationFlags.HasFreeFireEngines` (own district target) or
    `HasAvailableFireEngines` (any station serving the district) — `FirePathfindSetup.cs:84-103`;
  - every `Game.Vehicles.FireEngine` on the road that is `Returning` and not `Disabled`; it is seeded as a target at
    its current location (`targetSeeker2.FindTargets(entity3, num)`, `:169-177`). `Empty` engines are excluded
    (the AI only calls `SelectNextDispatch` when `(state & (Returning|Empty|Disabled)) == Returning`,
    `FireEngineAISystem.cs:469-472`).
  - engines still on a call: eligible for a *second* request appended after the current one, evaluated from the
    end of their current path (`CheckServiceDispatches`, `:636-650`).
- Cheapest path wins, so whichever is nearer by road — station or returning engine — gets the job.
- On acceptance the engine clears `Returning` (and `Extinguishing|Rescueing`), takes the fire as target and gets
  `CarFlags.Emergency | StayOnRoad` back in `ResetPath` (`:747`, `:826-841`). From then on the mod treats it as a
  full responder again.
- Pathfind weights differ: responding engines weight time only and ignore `ForbidCombustionEngines |
  ForbidHeavyTraffic` (`:610-615`); returning engines use balanced weights plus `SpecialParking` and a random
  cost, i.e. they wander home like ordinary traffic (`:616-621`).

Mod relevance: a returning engine drives as ordinary traffic exactly when it might be redirected; the redirect
itself is what lights it up, so the ambulance-only "lights in traffic" rule does not need extending to fire
engines. Ambulances are the special case because a non-critical transport keeps a patient aboard without sirens.
