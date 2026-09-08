# Emergency Priority

Keeps responding fire engines, ambulances and police cars alive and moving through traffic jams in **Cities: Skylines II**.

**Paradox Mods:** https://mods.paradoxplaza.com/mods/150540/Windows

## What it does
Vanilla CS2 deletes a responding emergency vehicle the moment it's flagged as stuck in traffic, and never re-evaluates its route after dispatch. This mod:

- **Despawn guard** — when a responder is flagged stuck, it requests a fresh route instead of being deleted, so it keeps responding.
- **Re-route around congestion** — a responder blocked behind traffic for too long looks for a new route (vanilla only prices congestion once, at dispatch).
- **Green lights along the route** — the traffic lights ahead of a responder are asked for green and hold it until it has passed. The green is for the queue of ordinary cars in front, which is what actually stops a responder (it already runs reds itself).
- **Keep the junctions ahead clear** — the lanes a responder is about to drive through are reserved for it at emergency priority, so cross traffic stops short of them and pedestrians wait at the kerb until it has passed. This is the game's own right-of-way mechanism, asked further ahead than vanilla does, so it works at unsignalled junctions and roundabouts as well as at lights.
- **Drive through stopped traffic** — a responder boxed in by stopped cars in its own lane, or by stopped cross traffic in a gridlocked junction or roundabout, drives through them at a configurable speed, as if the queue had pulled aside. Moving cross traffic, pedestrians and signals are never driven through.
- **Slow traffic pulls over** — in slow-moving traffic, the car directly ahead of a responder in the same lane brakes to a stop for it, so the responder passes car after car to the head of the queue.
- **Use a free lane to pass the queue** — a queued responder moves into an empty neighbouring lane, passes the queue there, and cuts back into its own lane just before the junction, even if that lane is for a different turn. Works on either side and for left- or right-hand traffic.
- **Ambulances light up to get through traffic** — an ambulance carrying a patient without its sirens on (vanilla only uses them for a critical patient) switches them on when traffic holds it up and off again once through, getting priority and all of the above while lit.

Everything is opt-in; turn the mod off for exact vanilla behaviour.

## Options (Options → Mods → Emergency Priority)
- Enable / disable
- Prevent stuck responders from despawning
- Re-route around congestion
- Re-route after N seconds blocked (slider)
- Green lights along the route, and how far ahead they react (slider)
- Keep the junctions ahead clear, and how far ahead (slider)
- Drive through stopped traffic, and the speed through parted traffic (slider)
- Slow traffic pulls over
- Use a free lane to pass the queue
- Ambulances light up to get through traffic

## Under the hood (for the curious / security-minded)
- **Pure ECS — no Harmony patches.** It reads emergency vehicles (`CarFlags.Emergency`) and writes only fields the game itself writes: `PathOwner.m_State` (the repath request), `LaneSignal.m_Priority`/`m_Petitioner` (the signal petition), `LaneReservation.m_Next.m_Priority` (the right-of-way reservation), and — for the drive-through, pull-over and free-lane features — `CarNavigation.m_MaxSpeed`/`m_TargetPosition`, `CarCurrentLane`'s lane-change fields, `Blocker.m_MaxSpeed` and (for the lights option) `CarFlags.Emergency` on transporting ambulances, in the same frame slot between the game's navigation and movement jobs. Nothing is written into the save; every effect stops the moment the mod does.
- **No vehicle is ever moved sideways or teleported.** The drive-through works because the game has no collisions between driving vehicles; a responder simply overlaps the car it passes for a moment.
- **No network access at all** — no HTTP, no sockets. Nothing leaves your machine.
- **Filesystem:** writes only its own settings file and a log (`EmergencyPriority.Mod.log`) in the game's log folder. Nothing else.
- **Dependencies:** none beyond the base game.

You don't have to take my word for it — the full source is here, and a compiled .NET mod decompiles cleanly (ILSpy/dnSpy) if you want to confirm the DLL matches.

## Build from source
Requires the official CS2 modding toolchain. `dotnet build -c Release` compiles and deploys to your local Mods folder.

## License
[MIT](LICENSE).

---

*Made with [Claude Code](https://claude.com/claude-code), Anthropic's agentic coding tool.*
