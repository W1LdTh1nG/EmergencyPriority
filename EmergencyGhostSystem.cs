using Game;
using Game.Common;
using Game.Net;
using Game.Objects;
using Game.Pathfind;
using Game.Prefabs;
using Game.Simulation;
using Game.Tools;
using Game.Vehicles;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using CarLaneFlags = Game.Vehicles.CarLaneFlags;

namespace EmergencyPriority
{
    // "Ghosting" — a responder that is boxed in by stopped traffic drives THROUGH it instead of waiting, as if the
    // queue had pulled aside for the siren. One Burst job, one pass over the current UpdateFrame bucket, five
    // behaviours, each in its own file of this partial class:
    //
    //   EmergencyGhostSystem.Ghost.cs       — drive through stopped traffic (the push, and the nav re-target)
    //   EmergencyGhostSystem.PullOver.cs    — slow traffic pulls over for a responder behind it
    //   EmergencyGhostSystem.Pass.cs        — free-lane pass: use an empty neighbouring lane, return before the junction
    //   EmergencyGhostSystem.Lights.cs      — transporting ambulances light up to get through traffic
    //   EmergencyGhostSystem.Watchdog.cs    — back off / give up / despawn a responder that stops making progress
    //   EmergencyGhostSystem.Diagnostics.cs — [Stall], [Sitting] and [SelfTest] log lines
    //
    // Nothing is moved sideways out of the way (that fights the lane-following code); the responder simply overlaps
    // the civilian for a moment. Cities: Skylines II has no physics collisions between driving vehicles —
    // ObjectCollisionSystem only queries `Any = { OutOfControl }` (ObjectCollisionSystem.cs:700-716) — so the overlap
    // has no side effects.
    //
    // HOW THE MOVEMENT PIPELINE ACTUALLY WORKS. Per 16-frame vehicle tick, CarNavigationSystem's Burst job runs the
    // CarLaneSpeedIterator, decides how fast the car may go and writes the result to the component store as
    // CarNavigation { m_TargetPosition, m_MaxSpeed } (CarNavigationSystem.cs:1581, 1650, 1971). CarMoveSystem then
    // reads CarNavigation — and ONLY CarNavigation; it never looks at Blocker — and sets the velocity vector to
    // exactly |m_MaxSpeed| in magnitude (CarMoveSystem: MathUtils.TryNormalize(ref value4, num3)). Both systems use
    // the same UpdateFrame bucket (frameIndex % 16), so nav and move for a given vehicle happen in the same frame,
    // with the vehicle AI systems in between (SystemOrder.cs:309-336). A system ordered after
    // CarNavigationSystem.Actions and before CarMoveSystem therefore gets the last word on m_MaxSpeed, and can raise
    // it from "stopped behind that car" to a chosen speed, without patching anything Burst-compiled. The same seam
    // lets it LOWER a civilian's speed in the civilian's own bucket, and start a lane change (CarCurrentLane
    // .m_ChangeLane) that the nav job then carries out with its own blending (:1673-1690).
    //
    // Runs as a Burst IJobChunk over one UpdateFrame bucket per frame: ~1/16 of all cars, one flag test each; the
    // pull-over scan reads one lane's LaneObject buffer per slow civilian, the free-lane scan two per queued
    // responder. Ordered after CarNavigationSystem.Actions (the nav job and its reservation/signal flush) and, by
    // construction of the vanilla list, before the vehicle AI systems and CarMoveSystem.
    public partial class EmergencyGhostSystem : GameSystemBase
    {
        // Vehicle simulation step used by CarNavigationSystem and CarMoveSystem (4/15 s per 16-frame tick).
        private const float kTimeStep = 4f / 15f;

        // Blocker.m_MaxSpeed byte scale (CarNavigationSystem.cs:1421) and the "not blocked" threshold everyone
        // reads it against (StuckMovingObjectSystem.cs:100, EmergencyRepathSystem).
        private const float kBlockerSpeedScale = 2.2949998f;
        private const byte kBlockerNotBlocked = 6;

        // Lane states in which a vehicle is not "driving down a road" — parked, arrived, at a building door, on a
        // connection lane, in a parking area — and must not be pushed or stopped. EndOfPath/ParkingSpace are the
        // same guard the green-light system uses (CarFlags.Emergency is NOT cleared on arrival); the rest are lane
        // kinds where the nav target is not a point on a road curve.
        private const CarLaneFlags kNotDrivingFlags =
            CarLaneFlags.EndOfPath | CarLaneFlags.EndReached | CarLaneFlags.ParkingSpace | CarLaneFlags.TransformTarget
            | CarLaneFlags.Area | CarLaneFlags.Connection | CarLaneFlags.ResetSpeed;

        // m_Stats slots. Session counters for the [SelfTest] line: what happened to every siren-on car whose nav
        // speed was below the ghost speed this tick, plus the pull-over and free-lane counts. The first thing to
        // read when "it didn't ghost".
        private const int kPushed = 0;          // speed raised (same-lane blocker)
        private const int kSkipNoBlocker = 1;   // slow but nothing named as the blocker (speed limit, caution, spawn)
        private const int kSkipCrossing = 2;    // waiting for a gap in MOVING cross traffic — junction/roundabout entry
        private const int kSkipOncoming = 3;    // head-on on a two-way lane
        private const int kSkipOtherType = 4;   // Signal/Limit/Caution/Spawn/Temporary with an entity attached
        private const int kSkipNotCar = 5;      // Continuing, but the blocker is not a car (train, creature)
        private const int kSkipNotDriving = 6;  // empty nav buffer (arrived / re-pathing) or non-road lane state
        private const int kSkipReversing = 7;
        private const int kPushedCrossing = 8;  // speed raised through a STOPPED crossing-lane blocker (gridlock)
        private const int kRetargeted = 9;      // of the pushes, how many also moved the nav target forward
        private const int kPulledOver = 10;     // civilians braked for a responder behind them
        private const int kPassStarted = 11;    // free-lane passes begun
        private const int kPassReturned = 12;   // ... and returns into the home lane begun
        private const int kLitUp = 13;          // transporting ambulances given lights for a jam
        private const int kLitOff = 14;         // ... and lights taken away again once clear
        private const int kStalls = 15;         // watchdog trips (5 s without movement -> 5 s hands-off)
        private const int kForcedForward = 16;  // nav wanted to reverse behind a stopped car; pushed forward instead
        private const int kGaveUp = 17;         // 30 s without progress: hands off, repath, released from the walks
        private const int kDespawned = 18;      // StuckDespawnSeconds without progress: deleted like vanilla would
        private const int kPushedSpaceRule = 19; // pushed through the don't-block-the-box rule (no blocker entity)
        private const int kReversed = 20;       // nav wanted to reverse and the target was behind: reverse granted
        private const int kStatCount = 21;

        [BurstCompile]
        private partial struct GhostJob : IJobChunk
        {
            [ReadOnly] public EntityTypeHandle m_EntityType;
            public ComponentTypeHandle<Car> m_CarType;
            [ReadOnly] public ComponentTypeHandle<Game.Objects.Transform> m_TransformType;
            [ReadOnly] public ComponentTypeHandle<Moving> m_MovingType;
            [ReadOnly] public ComponentTypeHandle<PrefabRef> m_PrefabRefType;
            [ReadOnly] public BufferTypeHandle<CarNavigationLane> m_NavigationLaneType;
            public ComponentTypeHandle<CarNavigation> m_NavigationType;
            public ComponentTypeHandle<CarCurrentLane> m_CurrentLaneType;
            public ComponentTypeHandle<Blocker> m_BlockerType;
            public ComponentTypeHandle<PathOwner> m_PathOwnerType;
            public EntityCommandBuffer m_CommandBuffer;

            [ReadOnly] public ComponentLookup<Car> m_CarData;
            [ReadOnly] public ComponentLookup<Controller> m_ControllerData;
            [ReadOnly] public ComponentLookup<Moving> m_MovingData;
            [ReadOnly] public ComponentLookup<Curve> m_CurveData;
            [ReadOnly] public ComponentLookup<NodeLane> m_NodeLaneData;
            [ReadOnly] public ComponentLookup<SlaveLane> m_SlaveLaneData;
            [ReadOnly] public ComponentLookup<Game.Net.CarLane> m_CarLaneData;
            [ReadOnly] public ComponentLookup<Owner> m_OwnerData;
            [ReadOnly] public ComponentLookup<PrefabRef> m_PrefabRefData;
            [ReadOnly] public ComponentLookup<NetLaneData> m_PrefabLaneData;
            [ReadOnly] public ComponentLookup<CarData> m_PrefabCarData;
            [ReadOnly] public ComponentLookup<ObjectGeometryData> m_PrefabObjectGeometryData;
            [ReadOnly] public BufferLookup<LaneObject> m_LaneObjects;
            [ReadOnly] public BufferLookup<Game.Net.SubLane> m_SubLanes;
            [ReadOnly] public ComponentLookup<Game.Vehicles.Ambulance> m_AmbulanceData;
            // Read-only views of two types this job also holds read-write chunk handles for (the responder is in
            // another chunk/bucket than the civilian asking about it). Single-threaded schedule, so no race.
            [ReadOnly, NativeDisableContainerSafetyRestriction] public ComponentLookup<CarCurrentLane> m_CurrentLaneData;
            [ReadOnly, NativeDisableContainerSafetyRestriction] public BufferLookup<CarNavigationLane> m_NavigationLaneData;

            public NativeParallelHashMap<Entity, PassState> m_Passes;
            public NativeParallelHashMap<Entity, LitState> m_Lit;
            public NativeParallelHashMap<Entity, StallState> m_Stalls;
            public NativeQueue<StallReport> m_Reports;

            public float m_GhostSpeed;
            public bool m_PullOver;
            public bool m_UseFreeLane;
            public bool m_LightsInTraffic;
            public uint m_DespawnFrames;
            public uint m_Frame;

            // See the k* slots. Single-threaded schedule, so plain counters are safe.
            public NativeArray<int> m_Stats;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                NativeArray<Entity> entities = chunk.GetNativeArray(m_EntityType);
                NativeArray<Car> cars = chunk.GetNativeArray(ref m_CarType);
                NativeArray<Game.Objects.Transform> transforms = chunk.GetNativeArray(ref m_TransformType);
                NativeArray<Moving> movings = chunk.GetNativeArray(ref m_MovingType);
                NativeArray<PrefabRef> prefabRefs = chunk.GetNativeArray(ref m_PrefabRefType);
                NativeArray<CarNavigation> navigations = chunk.GetNativeArray(ref m_NavigationType);
                NativeArray<CarCurrentLane> lanes = chunk.GetNativeArray(ref m_CurrentLaneType);
                NativeArray<Blocker> blockers = chunk.GetNativeArray(ref m_BlockerType);
                NativeArray<PathOwner> pathOwners = chunk.GetNativeArray(ref m_PathOwnerType);
                BufferAccessor<CarNavigationLane> navLanes = chunk.GetBufferAccessor(ref m_NavigationLaneType);

                for (int i = 0; i < chunk.Count; i++)
                {
                    CarNavigation navigation = navigations[i];

                    // Sign bit set = the nav job wants the vehicle to reverse. For a civilian that is none of our
                    // business. For a responder it is checked again in PushThrough: boxed in by a stopped car it is
                    // pushed forward instead, otherwise left alone.
                    bool wantsReverse = (math.asuint(navigation.m_MaxSpeed) >> 31) != 0;
                    if (wantsReverse && (cars[i].m_Flags & CarFlags.Emergency) == 0)
                        continue;

                    CarCurrentLane lane = lanes[i];
                    bool driving = navLanes[i].Length != 0 && (lane.m_LaneFlags & kNotDrivingFlags) == 0;

                    if ((cars[i].m_Flags & CarFlags.Emergency) == 0)
                    {
                        Car civilian = cars[i];
                        if (TryLightUp(entities[i], ref civilian, blockers[i], driving, navigation))
                        {
                            cars[i] = civilian;
                            continue;
                        }
                        if (TryPullOver(entities[i], lane, driving, ref navigation, movings[i], prefabRefs[i]))
                            navigations[i] = navigation;
                        continue;
                    }

                    // ---- Responder with sirens on. ----

                    Blocker blocker = blockers[i];

                    Car responder = cars[i];
                    if (TryDropLights(entities[i], ref responder, driving, navigation, blocker))
                    {
                        cars[i] = responder;
                        continue;
                    }

                    // Watchdog: hands off a responder that has stopped making progress despite our help; give up on
                    // it after 30 s; despawn it after the configured time.
                    bool enRoute = navLanes[i].Length != 0
                        && (lane.m_LaneFlags & (CarLaneFlags.EndOfPath | CarLaneFlags.EndReached | CarLaneFlags.ParkingSpace)) == 0;
                    PathOwner pathOwner = pathOwners[i];
                    int verdict = Watchdog(entities[i], transforms[i].m_Position, enRoute, ref lane, ref pathOwner, out int trippedStage);
                    if (trippedStage != 0)
                        ReportStall(trippedStage, entities[i], transforms[i].m_Position, cars[i], lane, navigation,
                            math.length(movings[i].m_Velocity), wantsReverse, driving, enRoute, blocker, pathOwner);
                    if (verdict != 0)
                    {
                        lanes[i] = lane;
                        pathOwners[i] = pathOwner;
                        if (verdict == 2)
                            m_CommandBuffer.AddComponent(entities[i], default(Deleted));
                        continue;
                    }

                    // Free-lane pass bookkeeping runs every tick for a responder with a pass in progress, and may
                    // start one for a responder queued behind a same-lane blocker.
                    if (UpdatePass(entities[i], cars[i], driving, navigation, blocker, ref lane))
                        lanes[i] = lane;

                    // The push itself: drive through whatever stopped thing is holding it below ghost speed.
                    PushThrough(i, navigations, lanes, blockers, navigation, lane, blocker, driving, wantsReverse,
                        movings[i], prefabRefs[i], transforms[i]);
                }
            }
        }

        private EntityQuery m_CarQuery;
        private SimulationSystem m_Sim;
        private EndFrameBarrier m_EndFrameBarrier;
        private NativeArray<int> m_Stats;
        private NativeParallelHashMap<Entity, PassState> m_Passes;
        private NativeParallelHashMap<Entity, LitState> m_Lit;
        private NativeParallelHashMap<Entity, StallState> m_Stalls;
        private NativeQueue<StallReport> m_Reports;
        private uint m_LastSweep;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_Sim = World.GetOrCreateSystemManaged<SimulationSystem>();
            m_EndFrameBarrier = World.GetOrCreateSystemManaged<EndFrameBarrier>();
            m_Stats = new NativeArray<int>(kStatCount, Allocator.Persistent);
            m_Passes = new NativeParallelHashMap<Entity, PassState>(64, Allocator.Persistent);
            m_Lit = new NativeParallelHashMap<Entity, LitState>(64, Allocator.Persistent);
            m_Stalls = new NativeParallelHashMap<Entity, StallState>(64, Allocator.Persistent);
            m_Reports = new NativeQueue<StallReport>(Allocator.Persistent);
            m_CarQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[]
                {
                    ComponentType.ReadWrite<Car>(),
                    ComponentType.ReadOnly<Game.Objects.Transform>(),
                    ComponentType.ReadOnly<Moving>(),
                    ComponentType.ReadOnly<PrefabRef>(),
                    ComponentType.ReadOnly<CarNavigationLane>(),
                    ComponentType.ReadOnly<UpdateFrame>(),
                    ComponentType.ReadWrite<CarNavigation>(),
                    ComponentType.ReadWrite<CarCurrentLane>(),
                    ComponentType.ReadWrite<Blocker>(),
                    ComponentType.ReadWrite<PathOwner>(),
                },
                None = new[]
                {
                    ComponentType.ReadOnly<Deleted>(),
                    ComponentType.ReadOnly<Temp>(),
                    ComponentType.ReadOnly<OutOfControl>(),
                    ComponentType.ReadOnly<ParkedCar>(),
                    ComponentType.ReadOnly<TripSource>(),
                },
            });
            RequireForUpdate(m_CarQuery);
            CreateDiagnosticsQuery();
        }

        protected override void OnDestroy()
        {
            if (m_Stats.IsCreated)
                m_Stats.Dispose();
            if (m_Passes.IsCreated)
                m_Passes.Dispose();
            if (m_Lit.IsCreated)
                m_Lit.Dispose();
            if (m_Stalls.IsCreated)
                m_Stalls.Dispose();
            if (m_Reports.IsCreated)
                m_Reports.Dispose();
            GivenUp.Clear();
            base.OnDestroy();
        }

        // Every frame: the nav/move pair for a given vehicle both land in its own UpdateFrame bucket, and there is one
        // bucket per frame. Skipping frames would leave 15/16 of vehicles untouched.
        public override int GetUpdateInterval(SystemUpdatePhase phase) => 1;

        protected override void OnUpdate()
        {
            EmergencyPrioritySetting s = Mod.ActiveSetting;
            if (s == null || !s.Enabled || !s.GhostThroughJams)
                return;

            uint frame = m_Sim.frameIndex;
            m_CarQuery.ResetFilter();
            m_CarQuery.SetSharedComponentFilter(new UpdateFrame(frame % 16));

            // Main-thread reads of the job's state. Last frame's job is long complete (CarMoveSystem waited on it),
            // so this Complete() costs nothing.
            if ((frame & 3) == 0)
            {
                Dependency.Complete();
                DrainStallReports();
                SittingReport(frame);
                PublishGivenUp(frame);
            }

            GhostJob job = new GhostJob
            {
                m_EntityType = GetEntityTypeHandle(),
                m_CarType = GetComponentTypeHandle<Car>(isReadOnly: false),
                m_TransformType = GetComponentTypeHandle<Game.Objects.Transform>(isReadOnly: true),
                m_MovingType = GetComponentTypeHandle<Moving>(isReadOnly: true),
                m_PrefabRefType = GetComponentTypeHandle<PrefabRef>(isReadOnly: true),
                m_NavigationLaneType = GetBufferTypeHandle<CarNavigationLane>(isReadOnly: true),
                m_NavigationType = GetComponentTypeHandle<CarNavigation>(isReadOnly: false),
                m_CurrentLaneType = GetComponentTypeHandle<CarCurrentLane>(isReadOnly: false),
                m_BlockerType = GetComponentTypeHandle<Blocker>(isReadOnly: false),
                m_PathOwnerType = GetComponentTypeHandle<PathOwner>(isReadOnly: false),
                m_CommandBuffer = m_EndFrameBarrier.CreateCommandBuffer(),
                m_CarData = GetComponentLookup<Car>(isReadOnly: true),
                m_ControllerData = GetComponentLookup<Controller>(isReadOnly: true),
                m_MovingData = GetComponentLookup<Moving>(isReadOnly: true),
                m_CurveData = GetComponentLookup<Curve>(isReadOnly: true),
                m_NodeLaneData = GetComponentLookup<NodeLane>(isReadOnly: true),
                m_SlaveLaneData = GetComponentLookup<SlaveLane>(isReadOnly: true),
                m_CarLaneData = GetComponentLookup<Game.Net.CarLane>(isReadOnly: true),
                m_OwnerData = GetComponentLookup<Owner>(isReadOnly: true),
                m_PrefabRefData = GetComponentLookup<PrefabRef>(isReadOnly: true),
                m_PrefabLaneData = GetComponentLookup<NetLaneData>(isReadOnly: true),
                m_PrefabCarData = GetComponentLookup<CarData>(isReadOnly: true),
                m_PrefabObjectGeometryData = GetComponentLookup<ObjectGeometryData>(isReadOnly: true),
                m_LaneObjects = GetBufferLookup<LaneObject>(isReadOnly: true),
                m_SubLanes = GetBufferLookup<Game.Net.SubLane>(isReadOnly: true),
                m_AmbulanceData = GetComponentLookup<Game.Vehicles.Ambulance>(isReadOnly: true),
                m_CurrentLaneData = GetComponentLookup<CarCurrentLane>(isReadOnly: true),
                m_NavigationLaneData = GetBufferLookup<CarNavigationLane>(isReadOnly: true),
                m_Passes = m_Passes,
                m_Lit = m_Lit,
                m_Stalls = m_Stalls,
                m_Reports = m_Reports,
                m_GhostSpeed = math.clamp(s.GhostCrawlSpeed, 0.5f, kMaxGhostSpeed),
                m_PullOver = s.TrafficPullsOver,
                m_UseFreeLane = s.GhostUsesFreeLane,
                m_LightsInTraffic = s.LightsInTraffic,
                m_DespawnFrames = (uint)math.max(0, s.StuckDespawnSeconds) * 60u,
                m_Frame = frame,
                m_Stats = m_Stats,
            };
            // Single-threaded on purpose: the bucket is small, m_Stats is a plain counter and the maps are plain maps.
            Dependency = JobChunkExtensions.Schedule(job, m_CarQuery, Dependency);
            m_EndFrameBarrier.AddJobHandleForProducer(Dependency);

            // Sweep the maps for responders that have despawned or stood down, so they cannot grow unbounded.
            if (frame - m_LastSweep >= 1024)
            {
                m_LastSweep = frame;
                Dependency.Complete();
                NativeArray<Entity> keys = m_Passes.GetKeyArray(Allocator.Temp);
                for (int i = 0; i < keys.Length; i++)
                {
                    Entity e = keys[i];
                    if (!EntityManager.Exists(e) || !EntityManager.HasComponent<Car>(e)
                        || (EntityManager.GetComponentData<Car>(e).m_Flags & CarFlags.Emergency) == 0)
                        m_Passes.Remove(e);
                }
                keys.Dispose();
                // Lit ambulances whose entity is gone, or all of them if the option was switched off (the AI restores
                // the flag on its next path; until then they simply stay lit).
                keys = m_Lit.GetKeyArray(Allocator.Temp);
                for (int i = 0; i < keys.Length; i++)
                {
                    if (!s.LightsInTraffic || !EntityManager.Exists(keys[i]))
                        m_Lit.Remove(keys[i]);
                }
                keys.Dispose();
                keys = m_Stalls.GetKeyArray(Allocator.Temp);
                for (int i = 0; i < keys.Length; i++)
                {
                    Entity e = keys[i];
                    if (!EntityManager.Exists(e) || !EntityManager.HasComponent<Car>(e)
                        || (EntityManager.GetComponentData<Car>(e).m_Flags & CarFlags.Emergency) == 0)
                        m_Stalls.Remove(e);
                }
                keys.Dispose();
            }

            LogStatus(frame, s, job.m_GhostSpeed);
        }
    }
}
