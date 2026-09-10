# Cities: Skylines II — Emergency Vehicle Mod: Handoff Notes

Status: **feasibility phase**. No code written yet. Next action is decompiling `Game.dll` to answer one question (see §5) before committing to the toolchain setup.

---

## 1. Goal

A CS2 code mod that changes emergency vehicle behaviour **only while they are responding with sirens on**. Vehicles without sirens must behave exactly as vanilla.

Desired behaviours:

1. **No despawn.** Responders that get stuck in traffic must not be deleted; they should re-route instead.
2. **"Ghost" through blocking traffic.** A responder blocked by vehicles ahead should pass slowly through them (simulating cars pulling aside). Physically moving the civilian cars out of the way is *not* wanted — it fights the lane-following systems and is expected to be problematic.
3. **Keep the next intersection clear.** Vehicles and pedestrians should not enter the junction ahead of the responder, leaving it free.

Environment: CS2 is the **Xbox / Game Pass PC Edition**, installed under `D:\Games\XBoxGames\Cities- Skylines II - PC Edition\Content\Cities2_Data\Managed`. Game Pass install folders carry restrictive ACLs, so `Game.dll` has been **copied to `C:\Users\John\Downloads`** for decompiling.

---

## 2. Modding landscape (verified Sep 2026)

- CS2 is Unity **DOTS / ECS** (Entities + Burst). Unity 2022.3.7f1 acts as the SDK; .NET 8; official NuGet C# project template; publish to Paradox Mods via project context-menu targets. Wiki: https://cs2.paradoxwikis.com/Modding_Toolchain
- **Do not plan on Harmony for the traffic sim.** The interesting code (`CarLaneSpeedIterator`, navigation/move jobs) is Burst-compiled; Harmony patches the managed IL and the game runs the native Burst build. The robust pattern is **pure ECS**: your own `GameSystemBase`, ordered before/after the relevant vanilla system with `updateSystem.UpdateBefore<Mine, Vanilla>(SystemUpdatePhase.GameSimulation)`, reading and writing the same components the game does.
- Community references:
  - https://github.com/optimus-code/Cities2Modding (general notes)
  - https://github.com/Mimonsi/CS2Modding (getting started)
  - https://github.com/krzychu124/Traffic (lane connector / priorities — largest ECS traffic mod, good for how to touch junction data and persist it in saves)

---

## 3. Prior art — READ THIS FIRST

**Emergency Priority** by AmicusDeus — MIT, ~650 lines, pure ECS, no Harmony.
Source: https://github.com/AmicusDeus/EmergencyPriority
Paradox Mods: https://mods.paradoxplaza.com/mods/150540/Windows

It already implements goal #1 and most of goal #3. Its source comments are unusually thorough and cite vanilla line numbers. Files:

- `EmergencyRepathSystem.cs` — despawn guard + reroute-on-blocked with exponential backoff.
- `GreenLightPrioritySystem.cs` — "green wave": petitions traffic lights along the responder's route.
- `Mod.cs` — system registration (`UpdateAfter<EmergencyRepathSystem, StuckMovingObjectSystem>`, `UpdateBefore<GreenLightPrioritySystem, TrafficLightSystem>`, both `GameSimulation` phase, update interval 4).
- `Setting.cs`, `Translations.cs` — options UI boilerplate.

Decision pending: **fork it** or **build fresh using it as reference**. Either way, don't re-derive what it already proves.

---

## 4. Verified vanilla internals (from EmergencyPriority's source comments — re-verify against the decompile)

### Identifying "sirens on"
- `Game.Vehicles.Car.m_Flags & CarFlags.Emergency` — set for fire engines, ambulances, police on emergency calls, evacuation buses. Cleared when returning to station / parked (`ParkCar`, `ResetPath` once Returning).
- **Trap:** the flag is *not* cleared on arrival. A fire engine parked at a fire with `FireEngineFlags.Extinguishing` still carries it for the whole fire. Guard with the game's own "still navigating" test: skip if `CarNavigationLane` buffer is empty or `CarCurrentLane.m_LaneFlags` has `EndOfPath | ParkingSpace`. Being *stopped* must NOT be part of the test — a responder halted in a queue is exactly who needs help.
- This gives the "ignore rules when no sirens" requirement essentially for free — it's just the query filter.

### Despawn (goal #1)
- `StuckMovingObjectSystem` sets `PathFlags.Stuck` on `PathOwner.m_State`.
- `AmbulanceAISystem` / `FireEngineAISystem`: `IsStuck => AddComponent<Deleted>`; the rescue request then enters exponential backoff.
- Fix: on a main-thread system ordered after `StuckMovingObjectSystem`, clear `Stuck` and set `Obsolete` on `PathOwner.m_State`. `VehicleUtils.RequireNewPath` then triggers a fresh, congestion-aware pathfind with the AI's own emergency weights.
- Reroute-on-blocked test used by vanilla: `Blocker.m_Blocker != Entity.Null && m_Type != BlockerType.Temporary && m_MaxSpeed < 6`.
- Pathfinder congestion cost comes from `NetData`/`m_FlowOffset` — a smoothed time-of-day average, not live traffic — so a mid-jam repath may return the same route; hence backoff.

### Signals / junctions (goal #3)
- `VehicleUtils.GetPriority` returns **108** for `CarFlags.Emergency`; civilians are **100**.
- `CarLaneSpeedIterator` gates its stop branch on `m_Priority < 108` (signals) and stop signs likewise — **responders already run reds**. What stops them is the queue of civilian cars obeying the red in front.
- `Game.Net.LaneSignal` on lanes: `m_Signal`, `m_Priority`, `m_Petitioner`, `m_Default` (resting 0; level-crossing track/waterway groups −1). `LaneSignalFlags.Physical` = barriers/moveable bridges — the one case a responder is genuinely signal-blocked; leave alone.
- `TrafficLightSystem.GetNextSignalGroup` picks the group with the highest `m_Priority` across the junction's lanes, and **consumes** the petition (`m_Petitioner = Entity.Null; m_Priority = m_Default`). So a petition is a per-update lease: write it every 4 frames while the lane is in the responder's nav buffer and it both turns green *and holds* green; stop writing and the junction resumes cycling. Nothing to unwind, nothing persisted in the save.
- Vanilla does petition (`CarNavigationSystem` ~:2326) but the walk is bounded by braking distance, which is ~0 for a stopped vehicle — so vanilla petitions nothing exactly when it matters. The mod walks the nav buffer with a distance budget instead.
- `CarNavigationLane` buffer is a rolling window of at most ~13 entries (`CarNavigationSystem` `FillNavigationPaths`).
- **Never hard-set `LaneSignal.m_Signal` to Go** — junctions recompute only every 64 frames (interval 4 × 16 UpdateFrame buckets), so you'd get conflicting greens. Go through the petition so vanilla's Ending/Changing clearance phases run.
- Not covered by prior art: actively preventing cars/pedestrians entering the box at **unsignalled** junctions. Candidates to investigate: `Game.Net.LaneReservation` (`m_Blocker`, `m_Offset`), writing `Blocker` onto cross-traffic vehicles, `LaneSignal` where present. Pedestrians are a separate family (`Game.Creatures`, pedestrian lane systems) — treat as stretch goal.

---

## 5. The open question — ghosting (goal #2)

Follow-distance limiting happens inside `CarLaneSpeedIterator` (Burst job) → cannot be patched.

**Hypothesis to test in the decompile:** the `Blocker` component (`m_Blocker`, `m_MaxSpeed`, `m_Type`) on the follower is *written* by one system (navigation/lane iteration) and *consumed* by another (the move/speed system) in a later system in the frame. If so, a mod system ordered between them can overwrite `Blocker` on Emergency-flagged cars each tick (e.g. `m_Blocker = Entity.Null`, `m_MaxSpeed` = a slow crawl), and the responder will drive through the car ahead. CS2 cars have no physics collisions, so they simply overlap visually — which is the "ghost slowly through" effect wanted. The crawl speed is what makes it look like cars are pulling aside rather than a clip-through.

What to find in `Game.dll`:
1. Every reader and writer of `Game.Vehicles.Blocker` (or wherever it lives) and their system ordering / update phase.
2. Whether the consumer reads `Blocker` from the component store or recomputes it inline in the same job (if inline → hypothesis dead).
3. Whether `Moving.m_Velocity` / `CarCurrentLane` speed is clamped elsewhere using lane-object occupancy (`LaneObject` buffers) that would re-block it regardless.
4. `StuckMovingObjectSystem` thresholds, to make sure a crawling responder doesn't trip Stuck.

If the hypothesis is dead, the fallback is taking over the responder's movement entirely (drive its `Transform` along the path yourself, hand back when clear). That is a large, fragile piece of work and was the reason for "check first before setting up the toolchain".

---

## 6. Plan

1. Decompile `C:\Users\John\Downloads\Game.dll` (ILSpy / `ilspycmd`). Answer §5. Also re-verify every claim in §4 against the current game version.
2. Decide: fork EmergencyPriority or fresh project.
3. Set up toolchain (Unity 2022.3.7f1, .NET 8, NuGet template).
4. Ship in order of risk: despawn guard → signal petition / green wave → box-clearing at unsignalled junctions → ghosting (if viable) → pedestrians (stretch).
5. All behaviour opt-in via the options UI, off = exact vanilla.

---

## 7. Status update — 2026-09-08

Decompile done (`ilspycmd 9.1.0.7988`, output was kept in a session scratchpad; command is in the findings file).
**§5 answered: ghosting is viable.** `CarMoveSystem` reads `CarNavigation.m_MaxSpeed`, not `Blocker`; a system
ordered between `CarNavigationSystem.Actions` and `CarMoveSystem` can raise it to a ≤3.5 m/s crawl. Vanilla
itself already does a 3 m/s drive-through via `CarLaneFlags.IgnoreBlocker`. Goal #3 has a vanilla mechanism
too: `LaneReservation` at priority 108 stops both cars and pedestrians from entering the reserved lane.
Full evidence, line references and the recommended design: `DECOMPILE_FINDINGS.md`. Next: fork
EmergencyPriority and set up the toolchain (plan step 2–3).

---

## 8. Fork status — 2026-09-10

**Decision:** not published by us. The work goes back to AmicusDeus as a pull request for release in the
original Emergency Priority; a renamed local build stays on this machine. Nothing here touches Paradox Mods.

**Repos and branches**
- Local clone: `EmergencyVehicle\EmergencyPriority\`. Remotes: `upstream` = AmicusDeus/EmergencyPriority,
  `origin` = https://github.com/W1LdTh1nG/EmergencyPriority (John's fork; its `main` is untouched = upstream).
- `ghost` (`eb2dccf`, 9 commits on upstream `main`): the hand-back. Feature only — no rename, no publish-config
  change. **Open the PR from `W1LdTh1nG:ghost` into `AmicusDeus:main`.**
- `local-ghost` (`3002a08`): `ghost` + one identity commit (project renamed `EmergencyPriority_Ghost`, settings
  file `EmergencyPriority_Ghost`, options title "Emergency Priority Ghost", upstream ModId removed). This is the
  only branch to build from.

**Workflow (learned the hard way):** edit + commit on `ghost`, then `git checkout local-ghost && git merge ghost`,
`dotnet build -c Debug` there only, `git push origin ghost local-ghost`. Never build on `ghost`: its csproj still
carries the upstream name, so the toolchain deploys a second copy to `Mods\EmergencyPriority` and the game loads
both mods at once (happened 2026-09-08; every system ran twice). The OnLoad log line now prints the assembly
name and path so a duplicate is visible immediately. After upstream merges: pull their `main`, merge into
`local-ghost`, rebuild.

**What the `ghost` branch adds** (all in `EmergencyGhostSystem.cs` + `JunctionClearSystem.cs`, each behaviour a
toggle, master switch off = exact vanilla):
1. Drive through stopped traffic (same-lane, or stationary cross traffic in a gridlocked box/roundabout) at a
   configurable speed (1–10 m/s, default 6); above ~3.5 m/s the nav target is advanced along the lane.
2. Slow traffic pulls over: civilian directly ahead in the same lane brakes to a stop for a responder behind it.
3. Free-lane pass: queued responder uses an empty neighbouring lane (either side, LHT/RHT agnostic), pinned with
   `FixedLane`, returns to its home lane 30 m before the junction.
4. Ambulances with a patient aboard but no sirens light up when held by traffic, off again ~5 s after clear.
5. Keep the junctions ahead clear: `LaneReservation` priority 108 along the nav buffer (default 60 m) — stops
   cross traffic and pedestrians at unsignalled junctions and roundabouts too.
6. Stall handling: 5 s no progress → hands-off (alternating); 30 s → give up (fresh path, green wave and
   reservations released for it); `StuckDespawnSeconds` (default 120, 0 = never) → deleted like vanilla would.
   This slider replaced upstream's "Prevent stuck responders from despawning" toggle (that guard is now always on).
7. Strings in all 11 locales, README updated, `DECOMPILE_FINDINGS.md` included (local paths scrubbed).

**Verified in play (2026-09-09/10):** traffic lights and roundabouts clear ahead of responders, pedestrians hold at
the kerb, responders creep/scoot through queues, ambulances light up on transports. Two deadlocks found and
fixed via the mod log's `[SelfTest]` counters (angled responder wanting to reverse; car-park stall holding the
street outside). Not yet stress-tested: the free-lane pass at scale, `stalls`/`gaveUp`/`despawned` frequency.

**Tuning knobs left as constants** (in `EmergencyGhostSystem.cs`): pull-over distance 15 m / slow-traffic
threshold 8 m/s; free-lane needs 30 m clear, 60 m of lane left, returns at 30 m; lights on <3 m/s, off ≥6 m/s
after 5 s; stall 0.5 m / 5 s / 30 s / hands-off 5 s and 60 s.

**Other mods present:** C2VM Traffic Lights Enhancement is installed — first suspect if the green wave misbehaves
at its junctions.

---

## 9. Fixes from the 2026-09-09 evening session

Pull request is open: `W1LdTh1nG:ghost` → `AmicusDeus:main`. It tracks the branch, so every push to `ghost`
lands in it. Heads now: `ghost` = `1923ca3` (14 commits on upstream `main`), `local-ghost` = `79bd07b`.

Five fixes, all found from play and confirmed via the mod log rather than guessed:

1. **Pull-over needs a moving responder** (`506cac3`). `CarFlags.Emergency` is not cleared on arrival; a loading
   ambulance at the kerb held every civilian ahead of it stopped. Pull-over now needs a non-empty nav buffer and
   a driving lane state — the guard the other features already had.
2. **Stall report** (`25d8f6f`). One `[Stall]` log line per watchdog trip with everything the job saw (position,
   flags, path state, nav vs actual speed, reverse request, blocker entity/type/byte/is-car/speed, pass/lit,
   ambulance state). Diagnose from the log; don't theorise.
3. **Junction clearing: not in car parks, released at 15 s** (`9fa6bf0`). An ambulance wedged in a car park had its
   driveway reserved at 108 and every pedestrian on the pavement queued at that crossing in both directions.
   Junction clearing skips Area/Connection lanes; both walks drop a responder with no progress for 15 s;
   progress radius 0.5 m → 3 m so a vehicle jiggling between bays counts as stalled.
4. **Push through the don't-block-the-box rule** (`6312161`). Stall report showed `navSpeed=0, reverse=True,
   blocker=0, type=Continuing`: `CheckSpace` refusing entry to the next lane (no room beyond) — fails for ever in a
   car park's short lanes, and nav's reverse-to-realign at zero speed seals it. A "Continuing" limit with no
   entity is now pushed through; other no-entity limits (reservation yields, barriers, speed limits) still respected.
   `pushedSpaceRule` counter in the status line (36 in the first hour).
5. **Stopped responders** (`1923ca3`). The game removes `Moving` when it parks a vehicle in place
   (`AmbulanceAISystem.StopVehicle` while loading). Such a vehicle is invisible to the ghost job (query requires
   `Moving`) so the watchdog can never give up on it, while the walks kept reserving for it. Both walks now skip
   `Game.Objects.Stopped`. Plus a `[Sitting]` line every 15 s per siren-on car that is not moving, including the
   ones without `Moving` — the report the stall report can't produce.

**Log reading, one hour after the last fix (23:04–00:01):** ~8,400 pushes, 840 through stopped cross traffic, 36
through the box rule, ~4,000 pull-overs, 71 lights on/off, 6 passes all returned; 3 watchdog trips (all
reservation yields at junctions, self-cleared within 5 s), 0 give-ups, 0 despawns; 26 upstream reroutes. All
long `[Sitting]` entries were legitimate: a busy critical-transport ambulance at successive pickups/hospital, and
two non-ambulance responders parked at their incident with EndOfPath. First boring log of the project.

**Lessons:** the two diagnostics ([Stall], [Sitting]) paid for themselves within an hour — keep them in the
upstream branch. Every deploy needs the game closed; the log resets on each load, so read it before restarting.
Entity ids are reassigned on load, so a vehicle can't be tracked across restarts by id — use position.

---

## 10. 2026-09-10 session: refactor, the circling ambulance, the bridge that came and went

Heads: `ghost` = `045de41` (17 commits on upstream `main`, all in the open PR), `local-ghost` = `fa2333e`.

**Refactor (`ac9f146`).** `EmergencyGhostSystem.cs` (1,142 lines, a 255-line `Execute` doing five things) split into
seven partial-class files, no behaviour change: `.cs` core (job handles, `Execute` as a sequence of named calls,
lifecycle, sweep), `.Ghost.cs` (push + re-target), `.PullOver.cs`, `.Pass.cs`, `.Lights.cs`, `.Watchdog.cs`
(+ `GivenUp`), `.Diagnostics.cs` ([Stall]/[Sitting]/[SelfTest]). Verified by diffing the set of code lines before
and after (only extracted signatures, dispatch calls, renamed parameters) and a clean compile.

**Circling ambulance (`91fdea5`).** An ambulance drove round and round in the carriageway, blocking everything.
Cause: my earlier "force forward when nav asks to reverse" fix — right when the target is ahead, wrong when it
is beside/behind (the turning circle can never reach it). And because it was moving, the position-based
watchdog never tripped. Fixes: (a) target more than ~80° off heading → grant nav's own reverse at 2 m/s
(`kForwardMinHeading`, `kReverseSpeed`, counter `reversed`); (b) watchdog progress is now "3 m further along
the lane, or a new lane" (`StallState.m_LastLane/m_LastCurveX`) instead of "3 m through space", so circling and
car-park jiggling count as stalled. Confirmed in play: the ambulance did a three-point turn and left; the log
showed one stage-1 trip on a vehicle doing 6 m/s with no lane progress, then nothing.

**Debug bridge — built, used, removed.** A file-based command bridge (`DebugBridgeSystem`, local-ghost only:
`command.txt` → `bridge.log`, commands responders/vehicle/near/watch/lane/settings) was added to interrogate the
running game, then removed the same evening at John's request: he wants a **standalone, generic MCP bridge mod**
instead. Notes for that live in `D:\VS\Projects\Personal\git\CS2Mods\MCP_Bridge\NOTES.md`. What stayed on
`ghost`: `EmergencyGhostSystem.DescribeInternal(entity, frame)` (`045de41`), a text dump of the job's private
per-vehicle state (stall/hands-off/give-up, pass, lights) for any external tool.

**Safe compile check on `ghost`:** `dotnet build -c Debug -p:LocalModsPath=<scratch dir>` — `Mod.targets` sets
`DeployDir = LocalModsPath\TargetName`, so the build deploys to the scratch folder instead of the game. Used
before every commit tonight; the game's Mods folder only ever saw `local-ghost` builds.

**Log reading after the fixes (22:02–22:07 load):** two stall reports at one junction — the circling transport
(caught within 5 s by the new progress test, one trip, gone) and a dispatched ambulance queued behind it
(reservation yield, two alternating trips, gone). No warnings. `reversed` counter arrives with the next status line.

**Open:** nothing known broken. Constants still hard-coded as listed in §8 plus `kForwardMinHeading` 0.17 and
`kReverseSpeed` 2 m/s. PR awaiting AmicusDeus.
