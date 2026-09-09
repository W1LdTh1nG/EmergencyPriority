# PR title

Responders drive through jams: ghost-through, pull-over, free-lane pass, junction clearing, transport lights, stall handling

# PR description (paste below the title)

Hi — first, thanks for Emergency Priority; the source comments made it possible to build on it instead of starting over. This branch adds the missing half of the problem: your mod fixes *routing and signals*, this adds *driving*. A responder that is boxed in now gets through the traffic instead of waiting for it, and everything is done the same way you did it — pure ECS, writing only fields the game itself writes, no Harmony, nothing in the save, master switch off = exact vanilla.

## What it adds

Two new systems, `EmergencyGhostSystem` (Burst job) and `JunctionClearSystem` (main thread, same shape as `GreenLightPrioritySystem`). Each behaviour has its own toggle.

1. **Drive through stopped traffic.** A responder blocked by a stopped car in its own lane, or by stationary cross traffic in a gridlocked junction/roundabout, drives through it at a configurable speed (1–10 m/s, default 6). Moving cross traffic, pedestrians and signals are never driven through. The game has no collisions between driving vehicles (`ObjectCollisionSystem` only queries `OutOfControl`), so the brief overlap is harmless — and vanilla already does exactly this at 3 m/s for one entity via `CarLaneFlags.IgnoreBlocker`.
2. **Slow traffic pulls over.** In slow traffic the civilian directly ahead of a responder in the same lane brakes to a stop for it, so the responder passes car after car to the head of the queue.
3. **Free-lane pass.** A queued responder moves into an empty neighbouring lane (either side; LHT/RHT agnostic), passes the queue there, and cuts back into its home lane 30 m before the junction. Pinned with `CarLaneFlags.FixedLane` so `UpdateOptimalLane` doesn't revert it; the pin dies with the lane, so vanilla is back in charge at the junction.
4. **Ambulances light up to get through traffic.** A transport without sirens (vanilla only lights a critical patient) gets `CarFlags.Emergency` while traffic holds it, and loses it ~5 s after it's clear. The AI only rewrites that flag on path/park/dispatch events, so the two don't fight.
5. **Keep the junctions ahead clear.** `LaneReservation.m_Next.m_Priority = 108` along the nav buffer with a distance budget (default 60 m) — the same reservations `ReserveNavigationLanes` already writes, just further than braking distance. `CarLaneSpeedIterator.CheckOverlappingLanes` then stops civilians short of any crossing lane and `CreatureTargetIterator` holds pedestrians at the kerb. Works at unsignalled junctions and roundabouts, and composes with the green wave at lights.
6. **Stall handling.** A responder making no progress while still en route: 5 s → the mod backs off (alternating); 30 s → gives up on it (fresh path; green wave and reservations released for it, so the street outside a blocked car park doesn't jam); `StuckDespawnSeconds` (default 120, 0 = never) → deleted, as the vanilla AI does with a stuck responder, so the call gets a fresh unit.

## How it works (the seam)

`CarMoveSystem` reads **only** `CarNavigation` — never `Blocker` — and drives at exactly `|m_MaxSpeed|`. Nav and move for a vehicle happen in the same frame (same `UpdateFrame` bucket), with the AI systems in between. A system ordered after `CarNavigationSystem.Actions` therefore gets the last word on `CarNavigation.m_MaxSpeed` / `m_TargetPosition` (and can start a lane change through `CarCurrentLane.m_ChangeLane`, which nav then blends itself). Above ~3.5 m/s the nav target has to be advanced along the lane, which is done with the same `GetLaneOffset`/`GetLanePosition` helpers `MoveTarget` uses. Every claim is line-referenced to a decompile of the 1.6.0.0 build in `DECOMPILE_FINDINGS.md`, and the source comments cite the same lines.

## Changes to existing code (kept minimal)

- `EmergencyRepathSystem`: the "blocked" test reads `Blocker.m_MaxSpeed`, which the ghost job now writes back (a crawling responder was being re-pathed every `RerouteAfterSeconds`, and each re-path emptied its nav buffer). The despawn-guard toggle is folded into `StuckDespawnSeconds` (0 = never = old toggle on); the guard itself is now always on while the mod is.
- `GreenLightPrioritySystem`: one line — skip a responder the watchdog has given up on.
- `Mod.cs`: two system registrations, locale entries, and the OnLoad line now logs the assembly name/path (I once had two copies of the mod loaded side by side and it took a while to notice).
- `Setting.cs`, `Translations.cs`, `README.md`: new options and strings. Translations are machine-generated in the same style as the existing ones — corrections welcome.

## Tested

Several hours in a ~60k city, LHT (UK). Traffic lights and roundabouts clear ahead of responders, pedestrians hold, responders creep/scoot through queues, transports light up. Two deadlocks were found through the `[SelfTest]` counters in the log and fixed (an angled responder whose nav wanted to reverse; a car-park stall holding the street outside). Not stress-tested at scale: the free-lane pass, and how often the 30 s / 120 s stages fire. The `[SelfTest] ghost status` line breaks every skipped responder down by reason, which is the first thing to read if something looks wrong.

## Things you may want to tune

Constants at the top of `EmergencyGhostSystem.cs`: pull-over trigger distance (15 m) and slow-traffic threshold (8 m/s); free-lane pass needs 30 m clear and 60 m of lane, returns at 30 m; lights on below 3 m/s, off above 6 m/s after 5 s; stall thresholds 0.5 m / 5 s / 30 s. Defaults in `Setting.cs`.

Happy to split this into smaller PRs if you'd rather review it that way.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
