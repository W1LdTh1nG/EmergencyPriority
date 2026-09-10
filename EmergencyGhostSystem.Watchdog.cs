using Game.Net;
using Game.Pathfind;
using Game.Vehicles;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using CarLaneFlags = Game.Vehicles.CarLaneFlags;

namespace EmergencyPriority
{
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
    public partial class EmergencyGhostSystem
    {
        // "Progress" = advancing kStallMove along the lane it is on, or moving to another lane. Measured along the
        // road rather than through space on purpose: a vehicle driving round in a circle in the carriageway, or
        // jiggling between the bays of a car park, covers plenty of ground and gets nowhere (both seen in play).
        private const float kStallMove = 3f;

        // The main-thread walks (green wave, junction clearing) stop holding lights and lanes for a responder that
        // has made no progress for this long — earlier than the 30 s give-up, because those holds are what jam the
        // street (and the pavement: pedestrians hold at a reserved crossing) while it sits there. Still long enough
        // for the normal few-second wait at a roundabout entry while the ring stops for it.
        private const uint kReleaseWalksFrames = 900;
        private const uint kStallFrames = 300;
        private const uint kHandsOffFrames = 300;
        private const uint kGiveUpFrames = 1800;
        private const uint kGiveUpHandsOffFrames = 3600;

        // Watchdog state per responder: where it last made progress, until when we are keeping our hands off, when
        // stage 1 last tripped (so it alternates instead of latching), and whether stage 2 has fired for this stall.
        public struct StallState
        {
            public Entity m_LastLane;
            public float m_LastCurveX;
            public uint m_LastMoveFrame;
            public uint m_HandsOffUntil;
            public uint m_LastTrip;
            public bool m_GaveUp;
        }

        // Responders the main-thread walks (green wave, junction clearing) must skip: no progress for
        // kReleaseWalksFrames, or given up on (stage 2) and still in its hands-off. Refreshed from the job's map
        // every few frames; read-only for everyone else.
        public static readonly System.Collections.Generic.HashSet<Entity> GivenUp = new System.Collections.Generic.HashSet<Entity>();

        // Main thread, after the job is complete: rebuild GivenUp from the stall map.
        private void PublishGivenUp(uint frame)
        {
            GivenUp.Clear();
            if (m_Stalls.Count() != 0)
            {
                NativeKeyValueArrays<Entity, StallState> stalls = m_Stalls.GetKeyValueArrays(Allocator.Temp);
                for (int i = 0; i < stalls.Length; i++)
                {
                    StallState st = stalls.Values[i];
                    if ((st.m_GaveUp && frame < st.m_HandsOffUntil) || frame - st.m_LastMoveFrame >= kReleaseWalksFrames)
                        GivenUp.Add(stalls.Keys[i]);
                }
                stalls.Dispose();
            }
        }

        private partial struct GhostJob
        {
            // Watchdog for one responder. Returns 0 = assist as normal, 1 = hands off this tick, 2 = despawn it.
            // Abandons and unpins any pass whenever it trips, so vanilla's lane selection is free again.
            private int Watchdog(Entity entity, float3 position, bool enRoute, ref CarCurrentLane lane, ref PathOwner pathOwner, out int trippedStage)
            {
                trippedStage = 0;
                if (!enRoute)
                {
                    m_Stalls.Remove(entity);
                    return 0;
                }
                if (!m_Stalls.TryGetValue(entity, out StallState stall))
                    stall = new StallState { m_LastLane = lane.m_Lane, m_LastCurveX = lane.m_CurvePosition.x, m_LastMoveFrame = m_Frame };
                float laneLength = m_CurveData.TryGetComponent(lane.m_Lane, out Curve curve) ? curve.m_Length : 0f;
                if (lane.m_Lane != stall.m_LastLane
                    || math.abs(lane.m_CurvePosition.x - stall.m_LastCurveX) * laneLength >= kStallMove)
                {
                    // Progress: everything is forgiven. (A give-up's hands-off still runs its course.)
                    stall.m_LastLane = lane.m_Lane;
                    stall.m_LastCurveX = lane.m_CurvePosition.x;
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
                    trippedStage = 2;
                    m_Stats[kGaveUp]++;
                }
                // Stage 1: short hands-off, alternating with assistance while the stall lasts.
                else if (!stall.m_GaveUp && stalled >= kStallFrames && m_Frame >= stall.m_HandsOffUntil
                    && m_Frame - stall.m_LastTrip >= kStallFrames + kHandsOffFrames)
                {
                    stall.m_HandsOffUntil = m_Frame + kHandsOffFrames;
                    stall.m_LastTrip = m_Frame;
                    tripped = true;
                    trippedStage = 1;
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
        }
    }
}
