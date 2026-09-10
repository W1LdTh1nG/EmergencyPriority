using System;
using System.IO;
using System.Text;
using Game;
using Game.Common;
using Game.Net;
using Game.Objects;
using Game.Pathfind;
using Game.Prefabs;
using Game.Simulation;
using Game.Vehicles;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace EmergencyPriority
{
    // LOCAL BUILD ONLY — not part of the upstream branch. A file-based command bridge so the game can be
    // interrogated while it runs, without guessing from screenshots: drop a one-line command into
    //   %USERPROFILE%\AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\EmergencyPriority_Ghost\command.txt
    // and the answer is appended to bridge.log next to it. Polled every 30 frames; costs one File.Exists when idle.
    // No sockets, no network — the upstream README's promise still holds even for this build.
    //
    // Commands (one per file):
    //   help
    //   settings                       current option values
    //   responders                     every siren-on car: state, lane, blocker, nav, ghost internals
    //   vehicle <index>                everything about one vehicle (entity index as printed in the log)
    //   near <x> <z> <radius>          all cars within radius of a map position (as printed in [Stall]/[Sitting])
    //   watch <index> <seconds>        one compact line per vehicle tick (16 frames) for that long
    //   lane <index>                   a lane: flags, reservation, signal, owner, length, objects on it
    public partial class DebugBridgeSystem : GameSystemBase
    {
        private const uint kPollFrames = 30;

        private SimulationSystem m_Sim;
        private EmergencyGhostSystem m_Ghost;
        private EntityQuery m_CarQuery;
        private string m_Dir;
        private string m_CommandPath;
        private string m_LogPath;
        private uint m_LastPoll;
        private Entity m_Watch;
        private uint m_WatchUntil;
        private uint m_WatchLast;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_Sim = World.GetOrCreateSystemManaged<SimulationSystem>();
            m_Ghost = World.GetOrCreateSystemManaged<EmergencyGhostSystem>();
            m_CarQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<Car>(), ComponentType.ReadOnly<Game.Objects.Transform>() },
                None = new[] { ComponentType.ReadOnly<Deleted>(), ComponentType.ReadOnly<Game.Tools.Temp>() },
            });
            string local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
            string localLow = Path.Combine(Path.GetDirectoryName(local) ?? local, "LocalLow");
            m_Dir = Path.Combine(localLow, "Colossal Order", "Cities Skylines II", "ModsData", "EmergencyPriority_Ghost");
            m_CommandPath = Path.Combine(m_Dir, "command.txt");
            m_LogPath = Path.Combine(m_Dir, "bridge.log");
            try { Directory.CreateDirectory(m_Dir); } catch (Exception e) { Mod.log.Warn($"[Bridge] cannot create {m_Dir}: {e.Message}"); }
            Mod.log.Info($"[Bridge] listening for {m_CommandPath}");
        }

        protected override void OnUpdate()
        {
            uint frame = m_Sim.frameIndex;

            if (m_Watch != Entity.Null)
            {
                if (frame >= m_WatchUntil || !EntityManager.Exists(m_Watch))
                {
                    Out($"watch {m_Watch.Index} ended");
                    m_Watch = Entity.Null;
                }
                else if (frame - m_WatchLast >= 16)
                {
                    m_WatchLast = frame;
                    Out("  " + Brief(m_Watch, frame));
                }
            }

            if (frame - m_LastPoll < kPollFrames)
                return;
            m_LastPoll = frame;

            string command;
            try
            {
                if (!File.Exists(m_CommandPath))
                    return;
                command = File.ReadAllText(m_CommandPath).Trim();
                File.Delete(m_CommandPath);
            }
            catch (Exception e)
            {
                Mod.log.Warn($"[Bridge] read failed: {e.Message}");
                return;
            }
            if (command.Length == 0)
                return;

            try
            {
                Out($"> {command}   (frame {frame})");
                Run(command, frame);
                Out("< done");
            }
            catch (Exception e)
            {
                Out($"< error: {e}");
            }
        }

        private void Run(string command, uint frame)
        {
            string[] a = command.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            switch (a[0].ToLowerInvariant())
            {
                case "help":
                    Out("help | settings | responders | vehicle <index> | near <x> <z> <radius> | watch <index> <seconds> | lane <index>");
                    break;
                case "settings":
                {
                    EmergencyPrioritySetting s = Mod.ActiveSetting;
                    Out($"enabled={s.Enabled} despawnAfter={s.StuckDespawnSeconds}s reroute={s.AutoReroute} rerouteAfter={s.RerouteAfterSeconds}s greenLight={s.GreenLightPriority}/{s.GreenLightDistance}m junctionClear={s.JunctionClear}/{s.JunctionClearDistance}m ghost={s.GhostThroughJams} speed={s.GhostCrawlSpeed}m/s pullOver={s.TrafficPullsOver} freeLane={s.GhostUsesFreeLane} lights={s.LightsInTraffic}");
                    break;
                }
                case "responders":
                {
                    NativeArray<Entity> cars = m_CarQuery.ToEntityArray(Allocator.Temp);
                    int n = 0;
                    for (int i = 0; i < cars.Length; i++)
                    {
                        if ((EntityManager.GetComponentData<Car>(cars[i]).m_Flags & CarFlags.Emergency) == 0)
                            continue;
                        n++;
                        Out("  " + Brief(cars[i], frame));
                    }
                    cars.Dispose();
                    Out($"{n} responders");
                    break;
                }
                case "vehicle":
                {
                    Entity e = FindCar(int.Parse(a[1]));
                    if (e == Entity.Null) { Out("no such car"); break; }
                    Full(e, frame);
                    break;
                }
                case "near":
                {
                    float x = float.Parse(a[1]), z = float.Parse(a[2]), r = float.Parse(a[3]);
                    NativeArray<Entity> cars = m_CarQuery.ToEntityArray(Allocator.Temp);
                    int n = 0;
                    for (int i = 0; i < cars.Length; i++)
                    {
                        float3 p = EntityManager.GetComponentData<Game.Objects.Transform>(cars[i]).m_Position;
                        if (math.distance(new float2(p.x, p.z), new float2(x, z)) > r)
                            continue;
                        n++;
                        Out("  " + Brief(cars[i], frame));
                    }
                    cars.Dispose();
                    Out($"{n} cars within {r} m of ({x},{z})");
                    break;
                }
                case "watch":
                {
                    Entity e = FindCar(int.Parse(a[1]));
                    if (e == Entity.Null) { Out("no such car"); break; }
                    m_Watch = e;
                    m_WatchUntil = frame + (uint)(float.Parse(a[2]) * 60f);
                    m_WatchLast = 0;
                    Out($"watching {e.Index} for {a[2]} s");
                    break;
                }
                case "lane":
                {
                    Entity lane = FindEntity(int.Parse(a[1]));
                    if (lane == Entity.Null) { Out("no such entity"); break; }
                    Out("  " + LaneInfo(lane));
                    if (EntityManager.HasBuffer<LaneObject>(lane))
                    {
                        DynamicBuffer<LaneObject> objs = EntityManager.GetBuffer<LaneObject>(lane, isReadOnly: true);
                        for (int i = 0; i < objs.Length; i++)
                            Out($"    object {objs[i].m_LaneObject.Index} at {objs[i].m_CurvePosition.x:0.000}-{objs[i].m_CurvePosition.y:0.000}"
                                + (EntityManager.HasComponent<Car>(objs[i].m_LaneObject) ? " " + Brief(objs[i].m_LaneObject, frame) : ""));
                    }
                    break;
                }
                default:
                    Out("unknown command; try help");
                    break;
            }
        }

        // One line: who, where, what state, what nav sees, what the ghost job holds.
        private string Brief(Entity e, uint frame)
        {
            StringBuilder sb = new StringBuilder();
            sb.Append($"car {e.Index}:{e.Version}");
            Car car = EntityManager.GetComponentData<Car>(e);
            sb.Append($" flags=0x{(uint)car.m_Flags:X}{((car.m_Flags & CarFlags.Emergency) != 0 ? "(EMERGENCY)" : "")}");
            if (EntityManager.HasComponent<Game.Vehicles.Ambulance>(e))
                sb.Append($" ambulance={EntityManager.GetComponentData<Game.Vehicles.Ambulance>(e).m_State}");
            if (EntityManager.HasComponent<Game.Vehicles.FireEngine>(e))
                sb.Append($" fireEngine={EntityManager.GetComponentData<Game.Vehicles.FireEngine>(e).m_State}");
            if (EntityManager.HasComponent<Game.Vehicles.PoliceCar>(e))
                sb.Append($" police={EntityManager.GetComponentData<Game.Vehicles.PoliceCar>(e).m_State}");
            if (EntityManager.HasComponent<Game.Objects.Transform>(e))
            {
                Game.Objects.Transform t = EntityManager.GetComponentData<Game.Objects.Transform>(e);
                sb.Append($" at=({t.m_Position.x:0},{t.m_Position.z:0})");
            }
            bool moving = EntityManager.HasComponent<Moving>(e);
            sb.Append(moving ? $" speed={math.length(EntityManager.GetComponentData<Moving>(e).m_Velocity):0.00}" : " NO-MOVING");
            if (EntityManager.HasComponent<Game.Objects.Stopped>(e)) sb.Append(" STOPPED");
            if (EntityManager.HasComponent<ParkedCar>(e)) sb.Append(" PARKED");
            if (EntityManager.HasComponent<CarCurrentLane>(e))
            {
                CarCurrentLane cl = EntityManager.GetComponentData<CarCurrentLane>(e);
                sb.Append($" lane={cl.m_Lane.Index}{(cl.m_ChangeLane != Entity.Null ? "->" + cl.m_ChangeLane.Index + $"@{cl.m_ChangeProgress:0.00}" : "")} curve=({cl.m_CurvePosition.x:0.000},{cl.m_CurvePosition.y:0.000},{cl.m_CurvePosition.z:0.000}) laneFlags={cl.m_LaneFlags}");
            }
            if (EntityManager.HasComponent<CarNavigation>(e))
            {
                CarNavigation nav = EntityManager.GetComponentData<CarNavigation>(e);
                sb.Append($" navSpeed={nav.m_MaxSpeed:0.00} target=({nav.m_TargetPosition.x:0.0},{nav.m_TargetPosition.z:0.0})");
            }
            if (EntityManager.HasComponent<Blocker>(e))
            {
                Blocker b = EntityManager.GetComponentData<Blocker>(e);
                sb.Append($" blocker={b.m_Blocker.Index}/{b.m_Type}/byte{b.m_MaxSpeed}");
            }
            if (EntityManager.HasComponent<PathOwner>(e))
                sb.Append($" path={EntityManager.GetComponentData<PathOwner>(e).m_State}");
            if (EntityManager.HasBuffer<CarNavigationLane>(e))
                sb.Append($" navLen={EntityManager.GetBuffer<CarNavigationLane>(e, isReadOnly: true).Length}");
            if ((car.m_Flags & CarFlags.Emergency) != 0)
                sb.Append(m_Ghost.DescribeInternal(e, frame));
            return sb.ToString();
        }

        private void Full(Entity e, uint frame)
        {
            Out("  " + Brief(e, frame));
            if (EntityManager.HasComponent<Game.Objects.Transform>(e))
            {
                Game.Objects.Transform t = EntityManager.GetComponentData<Game.Objects.Transform>(e);
                float3 f = math.forward(t.m_Rotation);
                sb0.Clear();
                sb0.Append($"  heading=({f.x:0.00},{f.z:0.00})");
                if (EntityManager.HasComponent<CarNavigation>(e))
                {
                    CarNavigation nav = EntityManager.GetComponentData<CarNavigation>(e);
                    float3 d = nav.m_TargetPosition - t.m_Position;
                    sb0.Append($" targetDistance={math.length(d):0.00} targetDot={math.dot(f, math.normalizesafe(d)):0.00}");
                }
                Out(sb0.ToString());
            }
            if (EntityManager.HasComponent<Target>(e))
                Out($"  target entity={EntityManager.GetComponentData<Target>(e).m_Target.Index}");
            if (EntityManager.HasComponent<CarCurrentLane>(e))
            {
                CarCurrentLane cl = EntityManager.GetComponentData<CarCurrentLane>(e);
                Out("  current " + LaneInfo(cl.m_Lane));
                if (cl.m_ChangeLane != Entity.Null)
                    Out("  changing-to " + LaneInfo(cl.m_ChangeLane));
            }
            if (EntityManager.HasComponent<Blocker>(e))
            {
                Blocker b = EntityManager.GetComponentData<Blocker>(e);
                if (b.m_Blocker != Entity.Null && EntityManager.Exists(b.m_Blocker))
                {
                    Entity bv = b.m_Blocker;
                    if (EntityManager.HasComponent<Controller>(bv))
                        bv = EntityManager.GetComponentData<Controller>(bv).m_Controller;
                    Out("  blocker " + (EntityManager.HasComponent<Car>(bv) ? Brief(bv, frame) : $"entity {b.m_Blocker.Index} (not a car)"));
                }
            }
            if (EntityManager.HasBuffer<CarNavigationLane>(e))
            {
                DynamicBuffer<CarNavigationLane> nl = EntityManager.GetBuffer<CarNavigationLane>(e, isReadOnly: true);
                for (int i = 0; i < nl.Length; i++)
                    Out($"  nav[{i}] {nl[i].m_Lane.Index} flags={nl[i].m_Flags} curve=({nl[i].m_CurvePosition.x:0.000},{nl[i].m_CurvePosition.y:0.000}) " + LaneInfo(nl[i].m_Lane));
            }
        }

        private readonly StringBuilder sb0 = new StringBuilder();

        private string LaneInfo(Entity lane)
        {
            if (lane == Entity.Null || !EntityManager.Exists(lane))
                return "lane none";
            StringBuilder sb = new StringBuilder();
            sb.Append($"lane {lane.Index}");
            if (EntityManager.HasComponent<Game.Net.CarLane>(lane))
                sb.Append($" carLane={EntityManager.GetComponentData<Game.Net.CarLane>(lane).m_Flags}");
            if (EntityManager.HasComponent<Curve>(lane))
                sb.Append($" length={EntityManager.GetComponentData<Curve>(lane).m_Length:0.0}");
            if (EntityManager.HasComponent<Owner>(lane))
            {
                Entity o = EntityManager.GetComponentData<Owner>(lane).m_Owner;
                sb.Append($" owner={o.Index}{(EntityManager.HasComponent<Game.Net.Node>(o) ? "(node)" : EntityManager.HasComponent<Game.Net.Edge>(o) ? "(edge)" : "")}");
            }
            if (EntityManager.HasComponent<SlaveLane>(lane))
            {
                SlaveLane s = EntityManager.GetComponentData<SlaveLane>(lane);
                sb.Append($" group={s.m_MinIndex}..{s.m_MaxIndex}");
            }
            if (EntityManager.HasComponent<LaneReservation>(lane))
            {
                LaneReservation r = EntityManager.GetComponentData<LaneReservation>(lane);
                sb.Append($" reservation=next({r.m_Next.m_Priority}@{r.m_Next.m_Offset})/prev({r.m_Prev.m_Priority}@{r.m_Prev.m_Offset})");
            }
            if (EntityManager.HasComponent<LaneSignal>(lane))
            {
                LaneSignal s = EntityManager.GetComponentData<LaneSignal>(lane);
                sb.Append($" signal={s.m_Signal}/prio{s.m_Priority}/{s.m_Flags}");
            }
            if (EntityManager.HasBuffer<LaneObject>(lane))
                sb.Append($" objects={EntityManager.GetBuffer<LaneObject>(lane, isReadOnly: true).Length}");
            return sb.ToString();
        }

        private Entity FindCar(int index)
        {
            NativeArray<Entity> cars = m_CarQuery.ToEntityArray(Allocator.Temp);
            Entity found = Entity.Null;
            for (int i = 0; i < cars.Length; i++)
                if (cars[i].Index == index) { found = cars[i]; break; }
            cars.Dispose();
            return found;
        }

        private Entity FindEntity(int index)
        {
            // Try successive versions; entities are recycled with increasing version numbers.
            for (int v = 1; v < 4096; v++)
            {
                Entity e = new Entity { Index = index, Version = v };
                if (EntityManager.Exists(e))
                    return e;
            }
            return Entity.Null;
        }

        private void Out(string line)
        {
            try { File.AppendAllText(m_LogPath, line + Environment.NewLine); }
            catch (Exception e) { Mod.log.Warn($"[Bridge] write failed: {e.Message}"); }
        }
    }
}
