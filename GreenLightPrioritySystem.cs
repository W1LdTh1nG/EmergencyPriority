using System.Collections.Generic;
using Game;
using Game.Net;
using Game.Simulation;
using Game.Vehicles;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace EmergencyPriority
{
    // "Green wave" — the traffic lights along a responder's route are asked for green while it approaches, and they
    // HOLD that green until it has passed. Both halves are the game's own mechanism; vanilla simply never asks.
    //
    // WHAT THIS IS ACTUALLY FOR. A responder does NOT stop at red lights in vanilla: CarLaneSpeedIterator gates its
    // stop branch on `m_Priority < 108` (:412 for signals, :440 for stop signs) and VehicleUtils.GetPriority returns
    // 108 for CarFlags.Emergency. The ambulance runs the red already. What stops it is the QUEUE OF CIVILIAN CARS in
    // front of it, which obey the red at priority 100. So the green is for the queue, to drain it out of the way.
    // (The one case where the responder itself is signal-blocked is LaneSignalFlags.Physical — level-crossing
    // barriers and moveable bridges. Nothing here changes those, and nothing should.)
    //
    // THE VANILLA GAP. Vanilla walks this same nav-lane buffer and petitions each signalled lane
    // (CarNavigationSystem.cs:2326), but the walk is bounded by the vehicle's BRAKING DISTANCE (:2274-2275) and gated
    // on `num > 1f` / `num2 > 0f` (:2280, :2307). A stopped responder has a braking distance of ~0, so the loop never
    // runs and nothing is petitioned — precisely when it is needed most. This system restores the walk with a floor
    // under the budget instead of a ceiling.
    //
    // WHY A STANDING PETITION IS ALSO THE HOLD. TrafficLightSystem.GetNextSignalGroup (:359-438) takes the highest
    // m_Priority across the junction's lanes. 108 beats civilian 100, and a lane's resting m_Default is 0 (only
    // level-crossing track/waterway groups get -1, TrafficLightInitializationSystem.cs:590-596), so a live petition
    // always wins the selection. The group scan then starts at the CURRENT group while preferChange is false (:421)
    // and the fallback returns the current group (:438) — so when the responder's direction is already green, the
    // function returns the group that is already running, the `!=` test at :219 fails, the state machine returns
    // false, and the phase does not advance. The green holds, for free, for as long as we keep asking.
    //
    // AND THE RELEASE IS AUTOMATIC. GetNextSignalGroup CONSUMES what it reads: `m_Petitioner = Entity.Null;
    // m_Priority = m_Default` (:394-395). So a petition is a per-update lease, not a latch. The moment the responder
    // clears a junction and that lane leaves its navigation buffer we stop writing, and the junction resumes normal
    // cycling on its very next update. There is nothing to unwind on uninstall and nothing to leave in a save — the
    // game itself resets the only field we touch.
    //
    // WHAT THIS DELIBERATELY DOES NOT DO. It never writes LaneSignal.m_Signal. Hard-setting a lane to Go bypasses the
    // phase machine, and because each junction only recomputes every 64 frames (TrafficLightSystem.GetUpdateInterval
    // is 4, filtered across 16 UpdateFrame buckets) the cross direction can keep its own Go for up to a second —
    // two conflicting greens aimed at cars that DO obey signals. Going through the petition instead means vanilla's
    // Ending/Changing clearance phases still run (RequireEnding, :342, exists exactly to force one when a conflicting
    // lane is still Go). That costs a few seconds on a red-to-green switch and is the reason the detection distance
    // is generous: the aim is for the light to already be green when the responder arrives, not to flip it there.
    //
    // Ordered before TrafficLightSystem at the same interval, so every petition is fresh when it reads.
    public partial class GreenLightPrioritySystem : GameSystemBase
    {
        // Vanilla's own emergency priority (VehicleUtils.GetPriority). Not an invented number: matching it means we
        // are asking with exactly the authority the game already grants a responder, no more.
        private const sbyte kEmergencyPriority = 108;

        private const uint kWatchRefreshFrames = 64;

        private EntityQuery m_CarQuery;
        private SimulationSystem m_Sim;
        private readonly List<Entity> m_Watched = new List<Entity>();
        private uint m_LastWatchRefresh;
        private uint m_LastLog;

        // Session counters for the [SelfTest] line.
        private int m_Petitions;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_Sim = World.GetOrCreateSystemManaged<SimulationSystem>();
            m_CarQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[]
                {
                    ComponentType.ReadOnly<Car>(),
                    ComponentType.ReadOnly<CarCurrentLane>(),
                    ComponentType.ReadOnly<CarNavigationLane>(),
                },
                None = new[]
                {
                    ComponentType.ReadOnly<Game.Common.Deleted>(),
                    ComponentType.ReadOnly<Game.Tools.Temp>(),
                },
            });
            RequireForUpdate(m_CarQuery);
        }

        // Matches TrafficLightSystem's own interval so our write always lands in the same frame it reads, immediately
        // before it. A junction consumes the petition when its UpdateFrame bucket comes round; re-writing every 4
        // frames guarantees a live petition is present whichever bucket that is.
        public override int GetUpdateInterval(SystemUpdatePhase phase) => 4;

        protected override void OnUpdate()
        {
            EmergencyPrioritySetting s = Mod.ActiveSetting;
            if (s == null || !s.Enabled || !s.GreenLightPriority)
                return;

            uint frame = m_Sim.frameIndex;
            if (m_Watched.Count == 0 || frame - m_LastWatchRefresh >= kWatchRefreshFrames)
                RefreshWatchSet(frame);

            float budgetMetres = s.GreenLightDistance;

            for (int i = m_Watched.Count - 1; i >= 0; i--)
            {
                Entity e = m_Watched[i];
                if (!EntityManager.Exists(e) || !EntityManager.HasComponent<Car>(e)
                    || !EntityManager.HasComponent<CarCurrentLane>(e) || !EntityManager.HasBuffer<CarNavigationLane>(e))
                {
                    m_Watched.RemoveAt(i);
                    continue;
                }
                // The Emergency flag drops the moment the mission phase changes (returning to station, parked). A
                // responder on its way home is ordinary traffic and must not keep holding junctions open.
                if ((EntityManager.GetComponentData<Car>(e).m_Flags & CarFlags.Emergency) == 0)
                {
                    m_Watched.RemoveAt(i);
                    continue;
                }

                // A responder the ghost watchdog has given up on is not coming: stop holding the lights for it.
                if (EmergencyGhostSystem.GivenUp.Contains(e) || EntityManager.HasComponent<Game.Objects.Stopped>(e))
                    continue;

                CarCurrentLane current = EntityManager.GetComponentData<CarCurrentLane>(e);
                DynamicBuffer<CarNavigationLane> lanes = EntityManager.GetBuffer<CarNavigationLane>(e, isReadOnly: true);

                // 🚨 ARRIVED RESPONDERS MUST NOT PETITION. CarFlags.Emergency is NOT cleared on arrival: the fire
                // engine AI clears it in ParkCar (back at the station) and in ResetPath once Returning, so a unit
                // parked at a burning building with FireEngineFlags.Extinguishing set still carries the flag for the
                // whole duration of the fire. Without this guard it would hold every junction within the detection
                // distance green for minutes, starving the cross traffic at every fire and accident in the city —
                // the exact opposite of what this mod is for.
                //
                // The test is the game's own "is this vehicle still navigating" condition: FillNavigationPaths stops
                // topping up the lane buffer on EndOfPath / ParkingSpace / Waypoint (CarNavigationSystem.cs:816), so
                // an arrived vehicle both carries those flags and runs its buffer down to empty. Being STOPPED is
                // deliberately not part of the test — a responder halted in a queue is precisely who needs the green.
                if (lanes.Length == 0
                    || (current.m_LaneFlags & (Game.Vehicles.CarLaneFlags.EndOfPath | Game.Vehicles.CarLaneFlags.ParkingSpace)) != 0)
                    continue;

                float remaining = budgetMetres;

                // The lane the vehicle is on first: on a signalled connector it is the one holding up the queue, and
                // it is the lane whose group the junction must serve. Only the part still ahead counts against the
                // budget — .x is the current position along the curve, .z the lane exit (cf. CarNavigationSystem:2278).
                Petition(current.m_Lane, e);
                remaining -= LaneLength(current.m_Lane, math.abs(current.m_CurvePosition.z - current.m_CurvePosition.x));

                if (remaining <= 0f)
                    continue;

                // Then the upcoming lanes, in order, until the budget runs out. This buffer is a rolling window the
                // game keeps topped up to at most 13 entries (CarNavigationSystem.cs:818), so it is inherently a
                // near-route view — there is no way to reach across the whole city from here, which is just as well.
                for (int j = 0; j < lanes.Length; j++)
                {
                    CarNavigationLane nav = lanes[j];
                    Petition(nav.m_Lane, e);
                    remaining -= LaneLength(nav.m_Lane, math.abs(nav.m_CurvePosition.y - nav.m_CurvePosition.x));
                    if (remaining <= 0f)
                        break;
                }
            }

            if (frame - m_LastLog >= 16384)
            {
                m_LastLog = frame;
                Mod.log.Info($"[SelfTest] greenLight status: enabled={s.Enabled} greenLight={s.GreenLightPriority} distance={s.GreenLightDistance}m watchedResponders={m_Watched.Count} petitionsTotal={m_Petitions}");
            }
        }

        // Raise the lane's signal petition to emergency priority. Lanes without a LaneSignal are skipped, so on an
        // unsignalled street this whole pass costs one HasComponent per lane and writes nothing.
        //
        // Only ever RAISES: if something already asked at 108 or above (another responder converging on the same
        // junction) we leave it alone rather than stealing the petitioner slot, so two ambulances do not fight over
        // one junction — GetNextSignalGroup merges equal-priority group masks by itself (:381-385).
        private void Petition(Entity lane, Entity vehicle)
        {
            if (lane == Entity.Null || !EntityManager.HasComponent<LaneSignal>(lane))
                return;
            LaneSignal signal = EntityManager.GetComponentData<LaneSignal>(lane);
            if (signal.m_Priority >= kEmergencyPriority)
                return;
            signal.m_Priority = kEmergencyPriority;
            signal.m_Petitioner = vehicle;
            EntityManager.SetComponentData(lane, signal);
            m_Petitions++;
        }

        // Metres of the given lane covered by a curve-position span. A lane with no Curve contributes nothing to the
        // budget rather than aborting the walk — better to petition one junction too many than to stop short.
        private float LaneLength(Entity lane, float curveSpan)
        {
            if (lane == Entity.Null || !EntityManager.HasComponent<Curve>(lane))
                return 0f;
            return EntityManager.GetComponentData<Curve>(lane).m_Length * curveSpan;
        }

        private void RefreshWatchSet(uint frame)
        {
            m_LastWatchRefresh = frame;
            m_Watched.Clear();
            NativeArray<Entity> cars = m_CarQuery.ToEntityArray(Allocator.Temp);
            for (int i = 0; i < cars.Length; i++)
            {
                if ((EntityManager.GetComponentData<Car>(cars[i]).m_Flags & CarFlags.Emergency) != 0)
                    m_Watched.Add(cars[i]);
            }
            cars.Dispose();
        }
    }
}
