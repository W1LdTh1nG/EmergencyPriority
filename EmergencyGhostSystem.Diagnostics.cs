using Game.Common;
using Game.Objects;
using Game.Pathfind;
using Game.Tools;
using Game.Vehicles;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace EmergencyPriority
{
    // Diagnostics. Three kinds of log line, all cheap enough to leave on:
    //   [Stall]    one per watchdog trip, everything the job saw for that vehicle at that moment, plus where it was.
    //              The first thing to read when a responder sits somewhere it should not.
    //   [Sitting]  every 15 s, one per siren-on car that is not moving — including the ones the job cannot see
    //              because the game removed Moving (AmbulanceAISystem.StopVehicle). The report the stall report
    //              cannot produce.
    //   [SelfTest] every ~4.5 min, the session counters (see the k* slots).
    public partial class EmergencyGhostSystem
    {
        public struct StallReport
        {
            public Entity m_Entity;
            public uint m_Frame;
            public int m_Stage;
            public float3 m_Position;
            public uint m_CarFlags;
            public uint m_LaneFlags;
            public Entity m_Lane;
            public bool m_Changing;
            public float m_NavSpeed;
            public float m_Speed;
            public bool m_WantsReverse;
            public bool m_Driving;
            public bool m_EnRoute;
            public Entity m_Blocker;
            public byte m_BlockerType;
            public byte m_BlockerByte;
            public bool m_BlockerIsCar;
            public float m_BlockerSpeed;   // -1 = no Moving component
            public bool m_Pass;
            public bool m_Lit;
            public int m_AmbulanceState;   // -1 = not an ambulance
            public ushort m_PathState;
        }

        private EntityQuery m_SittingQuery;
        private uint m_LastSittingReport;
        private uint m_LastLog;

        private void CreateDiagnosticsQuery()
        {
            // Every siren-on car, whether or not it still has Moving.
            m_SittingQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[]
                {
                    ComponentType.ReadOnly<Car>(),
                    ComponentType.ReadOnly<Game.Objects.Transform>(),
                    ComponentType.ReadOnly<CarCurrentLane>(),
                    ComponentType.ReadOnly<CarNavigation>(),
                    ComponentType.ReadOnly<CarNavigationLane>(),
                    ComponentType.ReadOnly<Blocker>(),
                    ComponentType.ReadOnly<PathOwner>(),
                },
                None = new[]
                {
                    ComponentType.ReadOnly<Deleted>(),
                    ComponentType.ReadOnly<Temp>(),
                    ComponentType.ReadOnly<ParkedCar>(),
                },
            });
        }

        // Main thread, after the job is complete.
        private void DrainStallReports()
        {
            while (m_Reports.TryDequeue(out StallReport r))
            {
                Mod.log.Info($"[Stall] stage={r.m_Stage} frame={r.m_Frame} vehicle={r.m_Entity.Index}:{r.m_Entity.Version}"
                    + $" at=({r.m_Position.x:0},{r.m_Position.z:0}) carFlags=0x{r.m_CarFlags:X} ambulance={r.m_AmbulanceState}"
                    + $" lane={r.m_Lane.Index} laneFlags=0x{r.m_LaneFlags:X} changing={r.m_Changing} driving={r.m_Driving} enRoute={r.m_EnRoute}"
                    + $" path=0x{r.m_PathState:X} navSpeed={r.m_NavSpeed:0.00} speed={r.m_Speed:0.00} reverse={r.m_WantsReverse}"
                    + $" blocker={r.m_Blocker.Index} type={(BlockerType)r.m_BlockerType} byte={r.m_BlockerByte} isCar={r.m_BlockerIsCar} blockerSpeed={r.m_BlockerSpeed:0.00}"
                    + $" pass={r.m_Pass} lit={r.m_Lit}");
            }
        }

        // Main thread, after the job is complete.
        private void SittingReport(uint frame)
        {
            if (frame - m_LastSittingReport < 900)
                return;
            m_LastSittingReport = frame;
            NativeArray<Entity> sitting = m_SittingQuery.ToEntityArray(Allocator.Temp);
            for (int i = 0; i < sitting.Length; i++)
            {
                Entity e = sitting[i];
                Car car = EntityManager.GetComponentData<Car>(e);
                if ((car.m_Flags & CarFlags.Emergency) == 0)
                    continue;
                bool hasMoving = EntityManager.HasComponent<Moving>(e);
                float speed = hasMoving ? math.length(EntityManager.GetComponentData<Moving>(e).m_Velocity) : 0f;
                if (hasMoving && speed > 0.1f)
                    continue;
                Game.Objects.Transform t = EntityManager.GetComponentData<Game.Objects.Transform>(e);
                CarCurrentLane cl = EntityManager.GetComponentData<CarCurrentLane>(e);
                CarNavigation nav = EntityManager.GetComponentData<CarNavigation>(e);
                Blocker bl = EntityManager.GetComponentData<Blocker>(e);
                PathOwner po = EntityManager.GetComponentData<PathOwner>(e);
                int navLen = EntityManager.GetBuffer<CarNavigationLane>(e, isReadOnly: true).Length;
                int amb = EntityManager.HasComponent<Game.Vehicles.Ambulance>(e) ? (int)EntityManager.GetComponentData<Game.Vehicles.Ambulance>(e).m_State : -1;
                bool stopped = EntityManager.HasComponent<Game.Objects.Stopped>(e);
                uint stalledFor = m_Stalls.TryGetValue(e, out StallState ss) ? frame - ss.m_LastMoveFrame : 0;
                Mod.log.Info($"[Sitting] vehicle={e.Index}:{e.Version} at=({t.m_Position.x:0},{t.m_Position.z:0}) moving={hasMoving} stopped={stopped} speed={speed:0.00}"
                    + $" carFlags=0x{(uint)car.m_Flags:X} ambulance={amb} lane={cl.m_Lane.Index} laneFlags=0x{(uint)cl.m_LaneFlags:X} changing={cl.m_ChangeLane != Entity.Null}"
                    + $" navLen={navLen} path=0x{(ushort)po.m_State:X} navSpeed={nav.m_MaxSpeed:0.00} blocker={bl.m_Blocker.Index} type={bl.m_Type} byte={bl.m_MaxSpeed}"
                    + $" stalledFrames={stalledFor} givenUp={GivenUp.Contains(e)}");
            }
            sitting.Dispose();
        }

        // Main thread, every ~4.5 min. Completes the job to read the counters.
        private void LogStatus(uint frame, EmergencyPrioritySetting s, float ghostSpeed)
        {
            if (frame - m_LastLog < 16384)
                return;
            m_LastLog = frame;
            Dependency.Complete();
            Mod.log.Info($"[SelfTest] ghost status: enabled={s.Enabled} ghost={s.GhostThroughJams} pullOver={s.TrafficPullsOver} freeLane={s.GhostUsesFreeLane} lights={s.LightsInTraffic} speed={ghostSpeed:0.0}m/s"
                + $" pushed={m_Stats[kPushed]} pushedCrossing={m_Stats[kPushedCrossing]} pushedSpaceRule={m_Stats[kPushedSpaceRule]} retargeted={m_Stats[kRetargeted]} pulledOver={m_Stats[kPulledOver]}"
                + $" passes={m_Stats[kPassStarted]} returns={m_Stats[kPassReturned]} passesLive={m_Passes.Count()}"
                + $" litUp={m_Stats[kLitUp]} litOff={m_Stats[kLitOff]} litLive={m_Lit.Count()}"
                + $" stalls={m_Stats[kStalls]} forcedForward={m_Stats[kForcedForward]} gaveUp={m_Stats[kGaveUp]} despawned={m_Stats[kDespawned]} givenUpLive={GivenUp.Count}"
                + $" skipped: noBlocker={m_Stats[kSkipNoBlocker]} crossingMoving={m_Stats[kSkipCrossing]}"
                + $" oncoming={m_Stats[kSkipOncoming]} otherType={m_Stats[kSkipOtherType]} notCar={m_Stats[kSkipNotCar]}"
                + $" notDriving={m_Stats[kSkipNotDriving]} reversing={m_Stats[kSkipReversing]}");
        }

        private partial struct GhostJob
        {
            // Job side: snapshot everything the job saw for a responder the watchdog just tripped on.
            private void ReportStall(int stage, Entity entity, float3 position, Car car, CarCurrentLane lane, CarNavigation navigation,
                float speed, bool wantsReverse, bool driving, bool enRoute, Blocker blocker, PathOwner pathOwner)
            {
                Entity blockerVehicleForReport = blocker.m_Blocker;
                if (m_ControllerData.TryGetComponent(blockerVehicleForReport, out Controller reportController))
                    blockerVehicleForReport = reportController.m_Controller;
                m_Reports.Enqueue(new StallReport
                {
                    m_Entity = entity,
                    m_Frame = m_Frame,
                    m_Stage = stage,
                    m_Position = position,
                    m_CarFlags = (uint)car.m_Flags,
                    m_LaneFlags = (uint)lane.m_LaneFlags,
                    m_Lane = lane.m_Lane,
                    m_Changing = lane.m_ChangeLane != Entity.Null,
                    m_NavSpeed = navigation.m_MaxSpeed,
                    m_Speed = speed,
                    m_WantsReverse = wantsReverse,
                    m_Driving = driving,
                    m_EnRoute = enRoute,
                    m_Blocker = blocker.m_Blocker,
                    m_BlockerType = (byte)blocker.m_Type,
                    m_BlockerByte = blocker.m_MaxSpeed,
                    m_BlockerIsCar = m_CarData.HasComponent(blockerVehicleForReport),
                    m_BlockerSpeed = m_MovingData.TryGetComponent(blockerVehicleForReport, out Moving reportMoving) ? math.length(reportMoving.m_Velocity) : -1f,
                    m_Pass = m_Passes.ContainsKey(entity),
                    m_Lit = m_Lit.ContainsKey(entity),
                    m_AmbulanceState = m_AmbulanceData.TryGetComponent(entity, out Game.Vehicles.Ambulance reportAmbulance) ? (int)reportAmbulance.m_State : -1,
                    m_PathState = (ushort)pathOwner.m_State,
                });
            }
        }
    }
}
