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
    // "Keep the junction clear" — the lanes a responder is about to drive through are reserved at emergency priority,
    // so cross traffic stops short of them and pedestrians hold at the kerb until the responder is past. This is the
    // game's own right-of-way mechanism; vanilla just does not ask far enough ahead.
    //
    // THE MECHANISM. Every car lane carries Game.Net.LaneReservation (CarLane prefab, :57): two {offset, priority}
    // slots, m_Next and m_Prev, and GetPriority() = max of the two. Consumers:
    //   - Cars. CarLaneSpeedIterator.CheckOverlappingLanes (:1264-1276): for each lane overlapping the one being
    //     entered, if that lane's reservation priority beats the car's own effective priority, the car brakes to a
    //     stop before the overlap (BlockerType.Crossing). Civilians are 100 (VehicleUtils.GetPriority), so 108 on a
    //     junction's lanes stops every civilian about to cross them — signalled or not, roundabouts included. Two
    //     side effects help too: a lane reserved at exactly 102 halves civilian speed (:155-158, which is what
    //     vanilla's ReserveOtherLanesInGroup does for the responder's neighbours), and a civilian whose OWN lane is
    //     reserved >= 108 gets effective priority 106 (:1189), i.e. it clears out rather than yielding.
    //   - Pedestrians. CreatureTargetIterator.CheckOverlapLane (:62-76): a road lane reserved at >= 108 makes the
    //     pedestrian about to cross it stop and queue at the kerb.
    //   - Lifetime. NetLaneReservationSystem rotates each lane once per 16 frames: prev = next, next = 0. A
    //     reservation written once is therefore honoured for 16-32 frames; re-writing every 4 frames holds it. Stop
    //     writing and it decays by itself. Nothing is persisted in the save.
    //
    // THE VANILLA GAP. CarNavigationSystem.ReserveNavigationLanes (:2274-2372) already writes 108 reservations along
    // a responder's nav buffer, but the walk budget is ~2x braking distance plus a vehicle length (:2346-2349). A
    // responder stopped in a queue has a braking distance of ~0 and reserves nothing beyond its own bonnet — so
    // the junction ahead fills up with cross traffic exactly when it is needed clear. This system walks the same
    // buffer with a distance budget instead, exactly like GreenLightPrioritySystem does for signals.
    //
    // WHAT IT WRITES. Only m_Next.m_Priority, and only upwards (to 108 if lower) — the same rule vanilla's
    // UpdateLaneReservationsJob applies (:2610-2626). Offset is left alone: vanilla's own extended emergency walk
    // writes offset 0 with the priority (:2361), which means "priority only", so two responders converging on one
    // junction do not block each other. LaneReservation.m_Blocker is not touched.
    //
    // Interval 4 (a quarter of the rotation period) so a reservation is always live whichever bucket the lane is in.
    public partial class JunctionClearSystem : GameSystemBase
    {
        // Vanilla's own emergency priority (VehicleUtils.GetPriority). Exactly the authority a responder already has.
        private const byte kEmergencyPriority = 108;

        private const uint kWatchRefreshFrames = 64;

        private EntityQuery m_CarQuery;
        private SimulationSystem m_Sim;
        private readonly List<Entity> m_Watched = new List<Entity>();
        private uint m_LastWatchRefresh;
        private uint m_LastLog;
        private int m_Reservations;

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

        public override int GetUpdateInterval(SystemUpdatePhase phase) => 4;

        protected override void OnUpdate()
        {
            EmergencyPrioritySetting s = Mod.ActiveSetting;
            if (s == null || !s.Enabled || !s.JunctionClear)
                return;

            uint frame = m_Sim.frameIndex;
            if (m_Watched.Count == 0 || frame - m_LastWatchRefresh >= kWatchRefreshFrames)
                RefreshWatchSet(frame);

            float budgetMetres = s.JunctionClearDistance;

            for (int i = m_Watched.Count - 1; i >= 0; i--)
            {
                Entity e = m_Watched[i];
                if (!EntityManager.Exists(e) || !EntityManager.HasComponent<Car>(e)
                    || !EntityManager.HasComponent<CarCurrentLane>(e) || !EntityManager.HasBuffer<CarNavigationLane>(e))
                {
                    m_Watched.RemoveAt(i);
                    continue;
                }
                if ((EntityManager.GetComponentData<Car>(e).m_Flags & CarFlags.Emergency) == 0)
                {
                    m_Watched.RemoveAt(i);
                    continue;
                }
                // A responder the ghost watchdog has stalled or given up on is not coming: stop holding the road
                // for it. (Seen in play: an ambulance wedged in a car park, its driveway reserved, and every
                // pedestrian on the pavement queued at that crossing in both directions.)
                if (EmergencyGhostSystem.GivenUp.Contains(e))
                    continue;

                CarCurrentLane current = EntityManager.GetComponentData<CarCurrentLane>(e);
                DynamicBuffer<CarNavigationLane> lanes = EntityManager.GetBuffer<CarNavigationLane>(e, isReadOnly: true);

                // Not while it is still inside a car park or on a building connection: reserving the driveway and
                // 60 m of street while it threads the bays is premature, and a lot can take a while to cross. It
                // gets the reservations the moment it is on a road lane.
                if ((current.m_LaneFlags & (Game.Vehicles.CarLaneFlags.Area | Game.Vehicles.CarLaneFlags.Connection)) != 0)
                    continue;

                // Same "still navigating" guard as the green-light system: CarFlags.Emergency is NOT cleared on
                // arrival, and a fire engine parked at a fire must not hold the surrounding junctions for the whole
                // blaze. Being stopped is deliberately not part of the test.
                if (lanes.Length == 0
                    || (current.m_LaneFlags & (Game.Vehicles.CarLaneFlags.EndOfPath | Game.Vehicles.CarLaneFlags.ParkingSpace)) != 0)
                    continue;

                // The lane it is on (so traffic crossing the box it is already in yields), then the lanes ahead until
                // the budget runs out. The buffer is a rolling window of at most ~13 lanes (CarNavigationSystem:818).
                float remaining = budgetMetres;
                Reserve(current.m_Lane);
                remaining -= LaneLength(current.m_Lane, math.abs(current.m_CurvePosition.z - current.m_CurvePosition.x));
                for (int j = 0; j < lanes.Length && remaining > 0f; j++)
                {
                    CarNavigationLane nav = lanes[j];
                    Reserve(nav.m_Lane);
                    remaining -= LaneLength(nav.m_Lane, math.abs(nav.m_CurvePosition.y - nav.m_CurvePosition.x));
                }
            }

            if (frame - m_LastLog >= 16384)
            {
                m_LastLog = frame;
                Mod.log.Info($"[SelfTest] junctionClear status: enabled={s.Enabled} junctionClear={s.JunctionClear} distance={s.JunctionClearDistance}m watchedResponders={m_Watched.Count} reservationsTotal={m_Reservations}");
            }
        }

        // Raise the lane's pending reservation to emergency priority. Only ever raises, and never touches the offset
        // or blocker fields, so it composes with whatever vanilla wrote for the same lane this frame.
        private void Reserve(Entity lane)
        {
            if (lane == Entity.Null || !EntityManager.HasComponent<LaneReservation>(lane))
                return;
            LaneReservation reservation = EntityManager.GetComponentData<LaneReservation>(lane);
            if (reservation.m_Next.m_Priority >= kEmergencyPriority)
                return;
            reservation.m_Next.m_Priority = kEmergencyPriority;
            EntityManager.SetComponentData(lane, reservation);
            m_Reservations++;
        }

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
