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
    // "Ghosting" — a responder that is boxed in by stopped traffic in its own lane drives THROUGH the car in front
    // instead of sitting behind it, as if the queue had pulled aside for the siren; with "traffic pulls over" on, the
    // car in front of a responder in slow traffic actually stops for it, so the responder scoots past car after car
    // to the head of the queue; and with "use a free lane" on, a responder queued behind traffic moves into an empty
    // neighbouring lane, passes the queue there, and cuts back into its own lane just before the junction. Nothing is
    // moved sideways out of the way (that fights the lane-following code); the responder simply overlaps the
    // civilian for a moment. Cities: Skylines II has no physics collisions between driving vehicles —
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
    // WHY THE TARGET HAS TO MOVE TOO. The nav job places m_TargetPosition only `max(1 m, speed*dt) + pivot` ahead
    // along the lane (:1667), which for a blocked car (speed ≈ 0) is ~1 m, and it clamps m_MaxSpeed to
    // (distance to target / dt) so the mover never overshoots (:1971-1975). Raising the speed alone therefore tops
    // out near 3.75 m/s. For anything faster this system re-targets: it walks the current lane's curve forward from
    // the nav target by the missing distance, using the same lane-offset helpers the nav job uses
    // (VehicleUtils.GetLaneOffset/GetLanePosition, :2501-2530), writes the new point back to CarNavigation and the
    // matching curve position to CarCurrentLane.m_CurvePosition.x — which is exactly what MoveTarget itself does.
    // Next tick MoveTarget binary-searches from that x to the lane end for the point at its own distance from the
    // car's ACTUAL position (:2501-2530), so a car that has moved further than it expected is handled by design.
    // The re-target never leaves the current lane and is skipped during a lane change, so at every lane boundary
    // (including the short connector lanes inside a junction) the responder briefly drops back to the nav-capped
    // creep, then picks up again on the next lane.
    //
    // WHY THE FREE-LANE PASS NEEDS FixedLane. Vanilla picks a vehicle's lane on each road with
    // CarLaneSelectIterator.UpdateOptimalLane (:383-634): a lane from which the next path lane is unreachable costs
    // 1,000,000, a stopped car ahead costs 0.49 — so a left-turning responder never leaves the left-turn lane, however
    // long the queue. It re-runs whenever the blocker changes (CheckBlocker sets UpdateOptimalLane, :1394/:1405) and
    // would revert a lane change it did not choose — UNLESS CarLaneFlags.FixedLane is set on CarCurrentLane, in which
    // case it keeps whatever m_ChangeLane/m_Lane already say (:388). The flag lives only until the vehicle moves onto
    // the next lane, where the nav job replaces the flags wholesale (:1919). So: start the change ourselves, pin it
    // with FixedLane, and start the change BACK into the original lane kReturnDistance before the lane ends — the
    // original lane is by construction the one vanilla chose for the upcoming turn. Should the return not finish
    // by the stop line, vanilla's own pop drops the vehicle onto the path's connector regardless (:1911-1917).
    //
    // LIGHTS ON TO GET THROUGH TRAFFIC. An ambulance only carries CarFlags.Emergency while dispatched to a healthcare
    // request or while its patient is flagged Critical (AmbulanceAISystem.ResetPath, :742-760); a routine transport
    // back to hospital drives as ordinary traffic and queues like everyone else. Real crews put the lights on for the
    // jam and off again once through. So: an ambulance with a patient aboard (AmbulanceFlags.Transporting) that is
    // held below kLightsOnSpeed by traffic gets the Emergency flag set here — which is everything at once: priority
    // 108, the doubled speed-limit factor, the warning lights (CarMoveSystem raises TransformFlags.WarningLights from
    // the flag), the green wave, and every feature in this file — and gets it cleared again kLightsOffDelay after it
    // was last held up. The AI only rewrites the flag when a new path lands, when it parks, or when it takes a
    // dispatch (:347, :697, :742-760), so the two never fight: if the AI does reset it mid-jam, the next tick simply
    // lights it up again.
    //
    // VANILLA ALREADY DOES THE DRIVE-THROUGH, ONCE. CarLaneSpeedIterator.UpdateMaxSpeed floors the speed at 3 m/s
    // for the single entity named by CarCurrentLane.m_LaneFlags & IgnoreBlocker (:1537
    // `select(v, 3f, ignore && v < 3f)`), which the game sets after a blocked-lane repath (:1151).
    //
    // WHAT IT DELIBERATELY DOES NOT DO.
    //  - Only when the blocker is a Car (trailers resolved to their controller). BlockerType.Continuing (the vehicle
    //    ahead in the same or a merging lane) qualifies whenever it is slower than the ghost speed. BlockerType.Crossing
    //    (a vehicle on a lane that crosses the one being entered — junction box, roundabout ring) qualifies only if
    //    that vehicle is itself STATIONARY: a gridlocked roundabout is stopped traffic and gets driven through, but a
    //    responder waiting for a gap in flowing cross traffic keeps waiting — driving visibly through moving cars is
    //    not "cars pull aside". Oncoming (head-on on a two-way lane) and Temporary (pedestrians) are never touched,
    //    nor are Signal/Limit/Caution; a responder already ignores reds itself (priority 108).
    //  - Uses max() on the responder's speed, so it only applies when the car in front is slower than the ghost speed.
    //  - Ramps at the vehicle's own acceleration so the pass starts smoothly rather than snapping to speed.
    //  - Pull-over is same-lane only, only for civilians already in slow traffic, only with the responder close
    //    behind, and brakes at the civilian's own braking rate. The civilian resumes on its own once the responder
    //    is ahead of it (it is then simply following the responder).
    //  - The free-lane pass only uses a lane in the same direction group (SlaveLane m_MinIndex..m_MaxIndex — the
    //    same set ReserveNavigationLanes treats as "other lanes in group"), only one that is clear for kFreeAhead
    //    metres, only when there is room to pass and come back, and never a lane the vehicle may not drive on
    //    (VehicleUtils.GetForbiddenLaneFlags). Both neighbours are considered, so it works for a right turn, a left
    //    turn, left-hand or right-hand traffic alike; the "home" lane is just the one it started in.
    //  - Writes the granted speed back into the responder's Blocker.m_MaxSpeed (same byte units the nav job uses,
    //    :1421) and leaves m_Blocker/m_Type alone. That byte is what everyone downstream reads to decide "is this
    //    vehicle blocked": StuckMovingObjectSystem's deadlock walk stops at any link >= 6 (:100, :121), and
    //    EmergencyRepathSystem's blocked clock uses the same `< 6` test. Left at the nav value of 0, a crawling
    //    responder would be re-routed every RerouteAfterSeconds, and each re-route empties its nav buffer — which
    //    pauses the ghost until the new path lands. Observed in play as stop-go crawling before the byte was written.
    //  - Runs as a Burst IJobChunk over one UpdateFrame bucket per frame: ~1/16 of all cars, one flag test each; the
    //    pull-over scan reads one lane's LaneObject buffer per slow civilian, the free-lane scan two per queued
    //    responder.
    //
    // Ordered after CarNavigationSystem.Actions (the nav job and its reservation/signal flush) and, by construction of
    // the vanilla list, before the vehicle AI systems and CarMoveSystem.
    public partial class EmergencyGhostSystem : GameSystemBase
    {
        // Vehicle simulation step used by CarNavigationSystem and CarMoveSystem (4/15 s per 16-frame tick).
        private const float kTimeStep = 4f / 15f;

        // Slider ceiling. 10 m/s is 36 km/h — plenty for threading a parted queue.
        public const float kMaxGhostSpeed = 10f;

        // Extra look-ahead beyond speed*dt when re-targeting, so the mover's "braking" flag logic and small overshoots
        // never leave the target behind the car.
        private const float kTargetMargin = 0.5f;

        // Blocker.m_MaxSpeed byte scale (CarNavigationSystem.cs:1421) and the "not blocked" threshold everyone
        // reads it against (StuckMovingObjectSystem.cs:100, EmergencyRepathSystem).
        private const float kBlockerSpeedScale = 2.2949998f;
        private const byte kBlockerNotBlocked = 6;

        // A crossing-lane blocker slower than this is "stopped" (gridlock, accident tailback) rather than passing.
        private const float kStationarySpeed = 0.5f;

        // Pull-over: a civilian counts as "in slow traffic" below this nav speed (8 m/s ≈ 29 km/h), and stops when a
        // responder is within this many metres behind it in the same lane.
        private const float kSlowTrafficSpeed = 8f;
        private const float kPullOverDistance = 15f;

        // Free-lane pass: a neighbouring lane must be clear for this far ahead to be worth moving into; anything
        // whose tail is within kBehindMargin behind us counts as alongside and blocks it. The pass is only started
        // with at least kMinPassRun of lane left (room to move over, pass, and come back), and the return into the
        // home lane starts kReturnDistance before the lane ends.
        private const float kFreeAhead = 30f;
        private const float kBehindMargin = 8f;
        private const float kMinPassRun = 60f;
        private const float kReturnDistance = 30f;

        // Lights in traffic: a transporting ambulance held below kLightsOnSpeed by a vehicle lights up; it stays lit
        // while held below kLightsOffSpeed and goes dark kLightsOffDelayFrames (~5 s at 60 sim frames/s) after it
        // was last held up, so a stop-start queue does not strobe it.
        private const float kLightsOnSpeed = 3f;
        private const float kLightsOffSpeed = 6f;
        private const uint kLightsOffDelayFrames = 300;

        // Re-target only when the new point is within ~35° of the vehicle's heading (cos 35° ≈ 0.82). A target more
        // than 1 m away AND more than 45° off heading makes a STOPPED vehicle's nav job decide to reverse to realign
        // (CarNavigationSystem.cs:2002-2013); with a blocker ahead its allowed speed is 0, so it "reverses" at zero
        // speed for ever. Vanilla's own 1 m target can never trip that; a pushed-out one can. Seen in play as an
        // angled ambulance sitting behind a pulled-over truck with `reversing` ticking up in the log.
        private const float kRetargetMinHeading = 0.82f;

        // Watchdog, three stages, all measured as "no kStallMove of progress while the vehicle still has somewhere to
        // go" (nav buffer non-empty, no EndOfPath/EndReached/ParkingSpace — so a fire engine parked at a blaze never
        // counts as stalled). Sim runs 60 frames/s.
        //  1. kStallFrames (5 s): kHandsOffFrames of nothing from us (no push, no pull-over on its behalf, any pass
        //     abandoned and unpinned) so the vanilla nav can untangle whatever we got it into, then we resume;
        //     alternating while it stays stuck.
        //  2. kGiveUpFrames (30 s): give up. Hands off for kGiveUpHandsOffFrames, a fresh path requested (Obsolete —
        //     the same thing EmergencyRepathSystem does, but that only fires on a Blocker, and a vehicle wedged in a
        //     parking lot may have none), and GivenUp published so GreenLightPrioritySystem / JunctionClearSystem
        //     stop holding the roads outside for a vehicle that is not coming. Seen in play: an ambulance stuck in
        //     a car park with the street outside jammed by its own green wave and reservations.
        //  3. StuckDespawnSeconds (setting, 0 = never): despawn it — Deleted, exactly what the vanilla AI does to a
        //     responder it considers stuck (AmbulanceAISystem.cs:234-237) — so the request is served by a fresh
        //     unit instead of sitting behind a vehicle that will never arrive. Lights are left alone throughout.
        private const float kStallMove = 0.5f;
        private const uint kStallFrames = 300;
        private const uint kHandsOffFrames = 300;
        private const uint kGiveUpFrames = 1800;
        private const uint kGiveUpHandsOffFrames = 3600;

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
        private const int kStatCount = 19;

        // An ambulance we lit up, keyed by entity: when it was last held up by traffic.
        public struct LitState
        {
            public uint m_LastBlockedFrame;
        }

        // Watchdog state per responder: where it last made progress, until when we are keeping our hands off, when
        // stage 1 last tripped (so it alternates instead of latching), and whether stage 2 has fired for this stall.
        public struct StallState
        {
            public float3 m_LastPosition;
            public uint m_LastMoveFrame;
            public uint m_HandsOffUntil;
            public uint m_LastTrip;
            public bool m_GaveUp;
        }

        // Responders stage 2 has given up on, for the main-thread walks (green wave, junction clearing) to skip.
        // Refreshed from the job's map every few frames; read-only for everyone else.
        public static readonly System.Collections.Generic.HashSet<Entity> GivenUp = new System.Collections.Generic.HashSet<Entity>();

        // One free-lane pass in progress, keyed by responder. Removed when the responder is back in its home lane,
        // leaves the road, or stops responding.
        public struct PassState
        {
            public Entity m_HomeLane;
            public Entity m_PassLane;
            public Entity m_Edge;
            public bool m_Returning;
        }

        [BurstCompile]
        private struct GhostJob : IJobChunk
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
                    // business. For a responder it is checked again below: boxed in by a stopped car it is pushed
                    // forward instead, otherwise left alone.
                    bool wantsReverse = (math.asuint(navigation.m_MaxSpeed) >> 31) != 0;
                    if (wantsReverse && (cars[i].m_Flags & CarFlags.Emergency) == 0)
                        continue;

                    CarCurrentLane lane = lanes[i];
                    bool driving = navLanes[i].Length != 0 && (lane.m_LaneFlags & kNotDrivingFlags) == 0;

                    if ((cars[i].m_Flags & CarFlags.Emergency) == 0)
                    {
                        // A transporting ambulance held up by traffic: lights on. Everything else (priority, ghost,
                        // green wave) follows from the flag on its next tick.
                        if (m_LightsInTraffic && m_AmbulanceData.TryGetComponent(entities[i], out Game.Vehicles.Ambulance ambulance))
                        {
                            Blocker held = blockers[i];
                            bool transporting = driving && (ambulance.m_State & AmbulanceFlags.Transporting) != 0
                                && (ambulance.m_State & (AmbulanceFlags.AtTarget | AmbulanceFlags.Disembarking | AmbulanceFlags.Disabled)) == 0;
                            if (transporting && held.m_Blocker != Entity.Null && navigation.m_MaxSpeed < kLightsOnSpeed
                                && (held.m_Type == BlockerType.Continuing || held.m_Type == BlockerType.Crossing))
                            {
                                Car lit = cars[i];
                                lit.m_Flags |= CarFlags.Emergency;
                                cars[i] = lit;
                                m_Lit[entities[i]] = new LitState { m_LastBlockedFrame = m_Frame };
                                m_Stats[kLitUp]++;
                                continue;
                            }
                            // Not (or no longer) a candidate: forget any entry left behind by an AI flag reset.
                            m_Lit.Remove(entities[i]);
                        }

                        if (m_PullOver && driving && navigation.m_MaxSpeed < kSlowTrafficSpeed
                            && ResponderCloseBehind(entities[i], lane))
                        {
                            // Brake to a stop at the civilian's own braking rate — vanilla's speed range floor
                            // (VehicleUtils.CalculateSpeedRange) — so it looks like a stop, not a freeze.
                            float braked = math.max(0f, math.length(movings[i].m_Velocity)
                                - m_PrefabCarData[prefabRefs[i].m_Prefab].m_Braking * kTimeStep);
                            if (braked < navigation.m_MaxSpeed)
                            {
                                navigation.m_MaxSpeed = braked;
                                navigations[i] = navigation;
                                m_Stats[kPulledOver]++;
                            }
                        }
                        continue;
                    }

                    // ---- Responder with sirens on. ----

                    Blocker blocker = blockers[i];

                    // An ambulance we lit up for a jam: keep the lights while it is still held up, drop them once it
                    // has been clear for the delay or has stopped transporting.
                    if (m_Lit.TryGetValue(entities[i], out LitState litState))
                    {
                        bool stillTransporting = driving
                            && m_AmbulanceData.TryGetComponent(entities[i], out Game.Vehicles.Ambulance litAmbulance)
                            && (litAmbulance.m_State & AmbulanceFlags.Transporting) != 0;
                        bool heldUp = blocker.m_Blocker != Entity.Null && navigation.m_MaxSpeed < kLightsOffSpeed
                            && (blocker.m_Type == BlockerType.Continuing || blocker.m_Type == BlockerType.Crossing);
                        if (stillTransporting && heldUp)
                        {
                            litState.m_LastBlockedFrame = m_Frame;
                            m_Lit[entities[i]] = litState;
                        }
                        else if (!stillTransporting || m_Frame - litState.m_LastBlockedFrame >= kLightsOffDelayFrames)
                        {
                            Car dark = cars[i];
                            dark.m_Flags &= ~CarFlags.Emergency;
                            cars[i] = dark;
                            m_Lit.Remove(entities[i]);
                            m_Passes.Remove(entities[i]);
                            m_Stats[kLitOff]++;
                            continue;
                        }
                    }

                    // Watchdog: hands off a responder that has stopped making progress despite our help; give up on
                    // it after 30 s; despawn it after the configured time.
                    bool enRoute = navLanes[i].Length != 0
                        && (lane.m_LaneFlags & (CarLaneFlags.EndOfPath | CarLaneFlags.EndReached | CarLaneFlags.ParkingSpace)) == 0;
                    PathOwner pathOwner = pathOwners[i];
                    int verdict = Watchdog(entities[i], transforms[i].m_Position, enRoute, ref lane, ref pathOwner);
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

                    // Already allowed to go at least ghost speed: whatever is in front is not holding it up.
                    if (navigation.m_MaxSpeed >= m_GhostSpeed)
                        continue;

                    // Still navigating a road (the empty-buffer half is the "arrived" test, and also covers a
                    // re-path in flight — the buffer is cleared until the new path lands).
                    if (!driving)
                    {
                        m_Stats[kSkipNotDriving]++;
                        continue;
                    }

                    // Boxed in by a vehicle ahead, and that vehicle is a car. Blocker is what the nav job just
                    // computed this frame for this vehicle, so it is current.
                    if (blocker.m_Blocker == Entity.Null)
                    {
                        m_Stats[kSkipNoBlocker]++;
                        continue;
                    }
                    bool crossing = blocker.m_Type == BlockerType.Crossing;
                    if (blocker.m_Type != BlockerType.Continuing && !crossing)
                    {
                        m_Stats[blocker.m_Type == BlockerType.Oncoming ? kSkipOncoming : kSkipOtherType]++;
                        continue;
                    }
                    // A trailer (truck, articulated bus) is its own lane object with no Car component; the vehicle
                    // that owns it is in Controller.m_Controller — same resolution the nav job does (:1387).
                    Entity blockerVehicle = blocker.m_Blocker;
                    if (m_ControllerData.TryGetComponent(blockerVehicle, out Controller controller))
                        blockerVehicle = controller.m_Controller;
                    if (!m_CarData.HasComponent(blockerVehicle))
                    {
                        m_Stats[kSkipNotCar]++;
                        continue;
                    }
                    // Cross traffic only when it is stopped. A car with no Moving component is parked/stopped; one
                    // with it must be at a standstill. Same-lane blockers need no such test: the max() below already
                    // leaves a faster car ahead alone, because the nav speed is then above the ghost speed.
                    if (crossing && m_MovingData.TryGetComponent(blockerVehicle, out Moving blockerMoving)
                        && math.lengthsq(blockerMoving.m_Velocity) > kStationarySpeed * kStationarySpeed)
                    {
                        m_Stats[kSkipCrossing]++;
                        continue;
                    }

                    // Nav wanted to back up to realign, but there is a stopped car in front and it is a responder:
                    // go forward through it instead. (A reversing responder with a MOVING blocker was skipped above
                    // via the crossing test or is about to be left alone by the ghost-speed test.)
                    if (wantsReverse)
                    {
                        navigation.m_MaxSpeed = 0f;
                        m_Stats[kForcedForward]++;
                    }

                    // Ramp up at the vehicle's own acceleration so a stopped responder eases into the pass.
                    float currentSpeed = math.length(movings[i].m_Velocity);
                    CarData prefabCar = m_PrefabCarData[prefabRefs[i].m_Prefab];
                    float desired = math.min(m_GhostSpeed, currentSpeed + prefabCar.m_Acceleration * kTimeStep);

                    // The nav target is ~1 m ahead for a blocked car. Push it out to what `desired` needs, along the
                    // current lane only (never across a lane change or past the lane end), then apply vanilla's own
                    // no-overshoot clamp against wherever the target ended up.
                    float3 position = transforms[i].m_Position;
                    float distance = math.distance(position, navigation.m_TargetPosition);
                    float need = desired * kTimeStep + kTargetMargin;
                    bool retargeted = false;
                    if (distance < need && lane.m_ChangeLane == Entity.Null)
                        retargeted = AdvanceTarget(ref navigation, ref lane, prefabRefs[i], position,
                            math.forward(transforms[i].m_Rotation), need, ref distance);

                    float target = math.min(desired, distance / kTimeStep);
                    if (target <= navigation.m_MaxSpeed)
                        continue;

                    navigation.m_MaxSpeed = target;
                    navigations[i] = navigation;
                    if (retargeted)
                    {
                        lanes[i] = lane;
                        m_Stats[kRetargeted]++;
                    }

                    // Publish the granted speed as the blocker-limited speed so the stuck detector and the re-route
                    // clock see a moving vehicle. Floored at the "not blocked" threshold: a responder that IS moving,
                    // however slowly, must not be treated as jammed by either.
                    blocker.m_MaxSpeed = (byte)math.max(kBlockerNotBlocked,
                        math.clamp((int)math.round(target * kBlockerSpeedScale), 0, 255));
                    blockers[i] = blocker;
                    m_Stats[crossing ? kPushedCrossing : kPushed]++;
                }
            }

            // Watchdog for one responder. Returns 0 = assist as normal, 1 = hands off this tick, 2 = despawn it.
            // Abandons and unpins any pass whenever it trips, so vanilla's lane selection is free again.
            private int Watchdog(Entity entity, float3 position, bool enRoute, ref CarCurrentLane lane, ref PathOwner pathOwner)
            {
                if (!enRoute)
                {
                    m_Stalls.Remove(entity);
                    return 0;
                }
                if (!m_Stalls.TryGetValue(entity, out StallState stall))
                    stall = new StallState { m_LastPosition = position, m_LastMoveFrame = m_Frame };
                if (math.distancesq(position, stall.m_LastPosition) > kStallMove * kStallMove)
                {
                    // Progress: everything is forgiven. (A give-up's hands-off still runs its course.)
                    stall.m_LastPosition = position;
                    stall.m_LastMoveFrame = m_Frame;
                    stall.m_GaveUp = false;
                }
                uint stalled = m_Frame - stall.m_LastMoveFrame;

                // Stage 3: gone for good, like the vanilla AI would do with a stuck responder.
                if (m_DespawnFrames != 0 && stalled >= m_DespawnFrames)
                {
                    m_Stalls.Remove(entity);
                    m_Passes.Remove(entity);
                    m_Lit.Remove(entity);
                    m_Stats[kDespawned]++;
                    return 2;
                }

                bool tripped = false;

                // Stage 2: give up — long hands-off, fresh path, released from the main-thread walks.
                if (!stall.m_GaveUp && stalled >= kGiveUpFrames)
                {
                    stall.m_GaveUp = true;
                    stall.m_HandsOffUntil = m_Frame + kGiveUpHandsOffFrames;
                    if ((pathOwner.m_State & (PathFlags.Pending | PathFlags.Failed | PathFlags.Obsolete)) == 0)
                        pathOwner.m_State |= PathFlags.Obsolete;
                    tripped = true;
                    m_Stats[kGaveUp]++;
                }
                // Stage 1: short hands-off, alternating with assistance while the stall lasts.
                else if (!stall.m_GaveUp && stalled >= kStallFrames && m_Frame >= stall.m_HandsOffUntil
                    && m_Frame - stall.m_LastTrip >= kStallFrames + kHandsOffFrames)
                {
                    stall.m_HandsOffUntil = m_Frame + kHandsOffFrames;
                    stall.m_LastTrip = m_Frame;
                    tripped = true;
                    m_Stats[kStalls]++;
                }

                if (tripped && m_Passes.TryGetValue(entity, out PassState pass))
                {
                    StartLaneChange(ref lane, pass.m_HomeLane);
                    lane.m_LaneFlags &= ~CarLaneFlags.FixedLane;
                    m_Passes.Remove(entity);
                }
                m_Stalls[entity] = stall;
                return (m_Frame < stall.m_HandsOffUntil) ? 1 : 0;
            }

            private bool IsHandsOff(Entity entity)
            {
                return m_Stalls.TryGetValue(entity, out StallState stall) && m_Frame < stall.m_HandsOffUntil;
            }

            // Free-lane pass state machine for one responder. Returns true if CarCurrentLane was changed.
            private bool UpdatePass(Entity entity, Car car, bool driving, CarNavigation navigation, Blocker blocker, ref CarCurrentLane lane)
            {
                Entity edge = m_OwnerData.TryGetComponent(lane.m_Lane, out Owner owner) ? owner.m_Owner : Entity.Null;

                if (m_Passes.TryGetValue(entity, out PassState pass))
                {
                    // Off the road the pass was on (the nav job popped the next lane and replaced the flags), or no
                    // longer driving at all: vanilla is back in charge, forget the pass.
                    if (!driving || edge != pass.m_Edge)
                    {
                        m_Passes.Remove(entity);
                        return false;
                    }
                    if (pass.m_Returning)
                    {
                        if (lane.m_ChangeLane == Entity.Null && lane.m_Lane == pass.m_HomeLane)
                            m_Passes.Remove(entity);
                        return false;
                    }
                    if (RemainingOnLane(lane) > kReturnDistance)
                        return false;

                    // Time to come back: change into the home lane (or reverse a change still in progress).
                    StartLaneChange(ref lane, pass.m_HomeLane);
                    pass.m_Returning = true;
                    m_Passes[entity] = pass;
                    m_Stats[kPassReturned]++;
                    return true;
                }

                // Consider starting a pass: queued behind a same-lane car, not already changing lane, on a
                // multi-lane road with room to pass and come back, and vanilla has not pinned the lane itself.
                if (!m_UseFreeLane || !driving || edge == Entity.Null
                    || lane.m_ChangeLane != Entity.Null || (lane.m_LaneFlags & CarLaneFlags.FixedLane) != 0
                    || navigation.m_MaxSpeed >= m_GhostSpeed
                    || blocker.m_Blocker == Entity.Null || blocker.m_Type != BlockerType.Continuing
                    || !m_SlaveLaneData.TryGetComponent(lane.m_Lane, out SlaveLane slave)
                    || !m_SubLanes.TryGetBuffer(edge, out DynamicBuffer<Game.Net.SubLane> subLanes)
                    || RemainingOnLane(lane) < kMinPassRun)
                    return false;

                int last = math.min(slave.m_MaxIndex, subLanes.Length - 1);
                int mine = -1;
                for (int k = slave.m_MinIndex; k <= last; k++)
                {
                    if (subLanes[k].m_SubLane == lane.m_Lane)
                    {
                        mine = k;
                        break;
                    }
                }
                if (mine < 0)
                    return false;

                Game.Net.CarLaneFlags forbidden = VehicleUtils.GetForbiddenLaneFlags(car, isBicycle: false);
                Entity best = Entity.Null;
                float bestRun = 0f;
                for (int side = -1; side <= 1; side += 2)
                {
                    int k = mine + side;
                    if (k < slave.m_MinIndex || k > last)
                        continue;
                    Game.Net.SubLane subLane = subLanes[k];
                    if ((subLane.m_PathMethods & PathMethod.Road) == 0
                        || !m_CarLaneData.TryGetComponent(subLane.m_SubLane, out Game.Net.CarLane carLane)
                        || (carLane.m_Flags & forbidden) != 0)
                        continue;
                    float run = FreeRunAhead(entity, subLane.m_SubLane, lane);
                    if (run >= kFreeAhead && run > bestRun)
                    {
                        best = subLane.m_SubLane;
                        bestRun = run;
                    }
                }
                if (best == Entity.Null)
                    return false;

                Entity home = lane.m_Lane;
                StartLaneChange(ref lane, best);
                lane.m_LaneFlags |= CarLaneFlags.FixedLane;
                m_Passes.TryAdd(entity, new PassState { m_HomeLane = home, m_PassLane = best, m_Edge = edge });
                m_Stats[kPassStarted]++;
                return true;
            }

            // Begin a lane change to `target`, mirroring how UpdateOptimalLane manipulates the same fields (:600-623):
            // a change already under way to a different lane is reversed rather than restarted.
            private static void StartLaneChange(ref CarCurrentLane lane, Entity target)
            {
                lane.m_LaneFlags &= ~(CarLaneFlags.TurnLeft | CarLaneFlags.TurnRight);
                if (lane.m_Lane == target)
                {
                    if (lane.m_ChangeLane == Entity.Null)
                        return;
                    if (lane.m_ChangeProgress == 0f)
                    {
                        lane.m_ChangeLane = Entity.Null;
                        return;
                    }
                    lane.m_Lane = lane.m_ChangeLane;
                    lane.m_ChangeLane = target;
                    lane.m_ChangeProgress = math.saturate(1f - lane.m_ChangeProgress);
                    return;
                }
                if (lane.m_ChangeLane == target)
                    return;
                lane.m_ChangeLane = target;
                lane.m_ChangeProgress = 0f;
            }

            // Metres of the current lane still ahead of the nav target (.x is the target's curve parameter, .z the exit).
            private float RemainingOnLane(CarCurrentLane lane)
            {
                if (!m_CurveData.TryGetComponent(lane.m_Lane, out Curve curve))
                    return 0f;
                return curve.m_Length * math.abs(lane.m_CurvePosition.z - lane.m_CurvePosition.x);
            }

            // How many metres of `candidate` are clear ahead of our position, up to the lane end. Anything whose tail
            // is less than kBehindMargin behind us counts as alongside and makes the lane unusable (returns 0).
            // Lanes in one group share their curve parameterisation (the nav job blends between them by one
            // curve position, :1837), so our own m_CurvePosition is valid on the candidate.
            private float FreeRunAhead(Entity self, Entity candidate, CarCurrentLane lane)
            {
                if (!m_CurveData.TryGetComponent(candidate, out Curve curve) || curve.m_Length < 0.01f)
                    return 0f;
                bool forward = lane.m_CurvePosition.z >= lane.m_CurvePosition.x;
                float myX = lane.m_CurvePosition.x;
                float run = curve.m_Length * math.abs(lane.m_CurvePosition.z - myX);
                if (!m_LaneObjects.TryGetBuffer(candidate, out DynamicBuffer<LaneObject> objects))
                    return run;
                for (int j = 0; j < objects.Length; j++)
                {
                    LaneObject laneObject = objects[j];
                    if (laneObject.m_LaneObject == self
                        || (m_ControllerData.TryGetComponent(laneObject.m_LaneObject, out Controller c) && c.m_Controller == self))
                        continue;
                    float2 span = laneObject.m_CurvePosition;
                    float lo = math.min(span.x, span.y);
                    float hi = math.max(span.x, span.y);
                    // Along-lane metres relative to us, positive ahead.
                    float sMin = (forward ? lo - myX : myX - hi) * curve.m_Length;
                    float sMax = (forward ? hi - myX : myX - lo) * curve.m_Length;
                    if (sMax < -kBehindMargin)
                        continue;
                    if (sMin <= 0f)
                        return 0f;
                    run = math.min(run, sMin);
                }
                return run;
            }

            // Move the nav target further along the current lane so that it is `need` metres from `position`.
            // Mirrors MoveTarget (CarNavigationSystem.cs:2501-2530): m_CurvePosition.x is the curve parameter of the
            // target, .z the lane exit, and the lateral offset comes from GetLaneOffset/GetLanePosition with the
            // vehicle's own lane position (sign-flipped on a lane traversed backwards, :1859).
            private bool AdvanceTarget(ref CarNavigation navigation, ref CarCurrentLane lane, PrefabRef prefabRef, float3 position, float3 heading, float need, ref float distance)
            {
                if (!m_CurveData.TryGetComponent(lane.m_Lane, out Curve curve) || curve.m_Length < 0.01f
                    || !m_PrefabRefData.TryGetComponent(lane.m_Lane, out PrefabRef lanePrefabRef)
                    || !m_PrefabLaneData.TryGetComponent(lanePrefabRef.m_Prefab, out NetLaneData laneData)
                    || !m_PrefabObjectGeometryData.TryGetComponent(prefabRef.m_Prefab, out ObjectGeometryData geometry))
                    return false;

                float3 cp = lane.m_CurvePosition;
                float direction = math.sign(cp.z - cp.x);
                if (direction == 0f)
                    return false;

                float t = cp.x + direction * (need - distance) / curve.m_Length;
                t = math.clamp(t, math.min(cp.x, cp.z), math.max(cp.x, cp.z));
                if (t == cp.x)
                    return false;

                m_NodeLaneData.TryGetComponent(lane.m_Lane, out NodeLane nodeLane);
                float lanePosition = math.select(lane.m_LanePosition, -lane.m_LanePosition, cp.z < cp.x);
                float laneOffset = VehicleUtils.GetLaneOffset(geometry, laneData, nodeLane, t, lanePosition, isBicycle: false);
                float3 newTarget = VehicleUtils.GetLanePosition(curve.m_Bezier, t, laneOffset);
                float newDistance = math.distance(position, newTarget);
                if (newDistance <= distance)
                    return false;

                // Off-heading target: keep vanilla's 1 m target and let the vehicle creep and straighten first
                // (see kRetargetMinHeading).
                if (math.dot(heading, math.normalizesafe(newTarget - position)) < kRetargetMinHeading)
                    return false;

                navigation.m_TargetPosition = newTarget;
                lane.m_CurvePosition.x = t;
                distance = newDistance;
                return true;
            }

            // Is there a siren-on car within kPullOverDistance behind `self` in its own lane? LaneObject entries carry
            // each vehicle's curve position (.x, the same "target t" CarCurrentLane.m_CurvePosition.x holds), and the
            // lane's traversal direction is the sign of (.z - .x) on our own CarCurrentLane.
            private bool ResponderCloseBehind(Entity self, CarCurrentLane lane)
            {
                if (!m_LaneObjects.TryGetBuffer(lane.m_Lane, out DynamicBuffer<LaneObject> objects) || objects.Length < 2
                    || !m_CurveData.TryGetComponent(lane.m_Lane, out Curve curve))
                    return false;

                bool forward = lane.m_CurvePosition.z >= lane.m_CurvePosition.x;
                float myX = lane.m_CurvePosition.x;
                float maxSpan = kPullOverDistance / math.max(0.01f, curve.m_Length);
                for (int j = 0; j < objects.Length; j++)
                {
                    LaneObject laneObject = objects[j];
                    if (laneObject.m_LaneObject == self)
                        continue;
                    if (!m_CarData.TryGetComponent(laneObject.m_LaneObject, out Car other)
                        || (other.m_Flags & CarFlags.Emergency) == 0)
                        continue;
                    float span = forward ? myX - laneObject.m_CurvePosition.x : laneObject.m_CurvePosition.x - myX;
                    if (span <= 0f || span > maxSpan || IsHandsOff(laneObject.m_LaneObject))
                        continue;
                    // Only for a responder that is still going somewhere. CarFlags.Emergency is NOT cleared on
                    // arrival: an ambulance loading its patient at the kerb, or a fire engine at a blaze, keeps it
                    // for the duration — and must not hold everything ahead of it stopped meanwhile. Same guard as
                    // the push logic (kNotDrivingFlags + non-empty nav buffer). Seen in play: a bus and a bike
                    // parked in front of a loading ambulance for as long as it stood there.
                    if (!m_NavigationLaneData.TryGetBuffer(laneObject.m_LaneObject, out DynamicBuffer<CarNavigationLane> responderLanes)
                        || responderLanes.Length == 0
                        || !m_CurrentLaneData.TryGetComponent(laneObject.m_LaneObject, out CarCurrentLane responderLane)
                        || (responderLane.m_LaneFlags & kNotDrivingFlags) != 0)
                        continue;
                    return true;
                }
                return false;
            }
        }

        private EntityQuery m_CarQuery;
        private SimulationSystem m_Sim;
        private EndFrameBarrier m_EndFrameBarrier;
        private NativeArray<int> m_Stats;
        private NativeParallelHashMap<Entity, PassState> m_Passes;
        private NativeParallelHashMap<Entity, LitState> m_Lit;
        private NativeParallelHashMap<Entity, StallState> m_Stalls;
        private uint m_LastLog;
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

            // Publish the given-up set for the main-thread walks. Last frame's job is long complete (CarMoveSystem
            // waited on it), so this Complete() costs nothing.
            if ((frame & 3) == 0)
            {
                Dependency.Complete();
                GivenUp.Clear();
                if (m_Stalls.Count() != 0)
                {
                    NativeKeyValueArrays<Entity, StallState> stalls = m_Stalls.GetKeyValueArrays(Allocator.Temp);
                    for (int i = 0; i < stalls.Length; i++)
                    {
                        if (stalls.Values[i].m_GaveUp && frame < stalls.Values[i].m_HandsOffUntil)
                            GivenUp.Add(stalls.Keys[i]);
                    }
                    stalls.Dispose();
                }
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

            // Sweep passes whose responder has despawned or stood down, so the map cannot grow unbounded.
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

            if (frame - m_LastLog >= 16384)
            {
                m_LastLog = frame;
                Dependency.Complete();
                Mod.log.Info($"[SelfTest] ghost status: enabled={s.Enabled} ghost={s.GhostThroughJams} pullOver={s.TrafficPullsOver} freeLane={s.GhostUsesFreeLane} lights={s.LightsInTraffic} speed={job.m_GhostSpeed:0.0}m/s"
                    + $" pushed={m_Stats[kPushed]} pushedCrossing={m_Stats[kPushedCrossing]} retargeted={m_Stats[kRetargeted]} pulledOver={m_Stats[kPulledOver]}"
                    + $" passes={m_Stats[kPassStarted]} returns={m_Stats[kPassReturned]} passesLive={m_Passes.Count()}"
                    + $" litUp={m_Stats[kLitUp]} litOff={m_Stats[kLitOff]} litLive={m_Lit.Count()}"
                    + $" stalls={m_Stats[kStalls]} forcedForward={m_Stats[kForcedForward]} gaveUp={m_Stats[kGaveUp]} despawned={m_Stats[kDespawned]} givenUpLive={GivenUp.Count}"
                    + $" skipped: noBlocker={m_Stats[kSkipNoBlocker]} crossingMoving={m_Stats[kSkipCrossing]}"
                    + $" oncoming={m_Stats[kSkipOncoming]} otherType={m_Stats[kSkipOtherType]} notCar={m_Stats[kSkipNotCar]}"
                    + $" notDriving={m_Stats[kSkipNotDriving]} reversing={m_Stats[kSkipReversing]}");
            }
        }
    }
}
