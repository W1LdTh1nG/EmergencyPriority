using Game.Common;
using Game.Net;
using Game.Pathfind;
using Game.Vehicles;
using Unity.Entities;
using Unity.Mathematics;
using CarLaneFlags = Game.Vehicles.CarLaneFlags;

namespace EmergencyPriority
{
    // Free-lane pass — a responder queued behind traffic moves into an empty neighbouring lane, passes the queue
    // there, and cuts back into its own lane just before the junction.
    //
    // WHY IT NEEDS FixedLane. Vanilla picks a vehicle's lane on each road with CarLaneSelectIterator.UpdateOptimalLane
    // (:383-634): a lane from which the next path lane is unreachable costs 1,000,000, a stopped car ahead costs 0.49
    // — so a left-turning responder never leaves the left-turn lane, however long the queue. It re-runs whenever the
    // blocker changes (CheckBlocker sets UpdateOptimalLane, :1394/:1405) and would revert a lane change it did not
    // choose — UNLESS CarLaneFlags.FixedLane is set on CarCurrentLane, in which case it keeps whatever
    // m_ChangeLane/m_Lane already say (:388). The flag lives only until the vehicle moves onto the next lane, where
    // the nav job replaces the flags wholesale (:1919). So: start the change ourselves, pin it with FixedLane, and
    // start the change BACK into the original lane kReturnDistance before the lane ends — the original lane is by
    // construction the one vanilla chose for the upcoming turn. Should the return not finish by the stop line,
    // vanilla's own pop drops the vehicle onto the path's connector regardless (:1911-1917).
    //
    // Only uses a lane in the same direction group (SlaveLane m_MinIndex..m_MaxIndex — the same set
    // ReserveNavigationLanes treats as "other lanes in group"), only one that is clear for kFreeAhead metres, only
    // when there is room to pass and come back, and never a lane the vehicle may not drive on
    // (VehicleUtils.GetForbiddenLaneFlags). Both neighbours are considered, so it works for a right turn, a left
    // turn, left-hand or right-hand traffic alike; the "home" lane is just the one it started in.
    public partial class EmergencyGhostSystem
    {
        // A neighbouring lane must be clear for this far ahead to be worth moving into; anything whose tail is
        // within kBehindMargin behind us counts as alongside and blocks it. The pass is only started with at least
        // kMinPassRun of lane left (room to move over, pass, and come back), and the return into the home lane
        // starts kReturnDistance before the lane ends.
        private const float kFreeAhead = 30f;
        private const float kBehindMargin = 8f;
        private const float kMinPassRun = 60f;
        private const float kReturnDistance = 30f;

        // One free-lane pass in progress, keyed by responder. Removed when the responder is back in its home lane,
        // leaves the road, or stops responding.
        public struct PassState
        {
            public Entity m_HomeLane;
            public Entity m_PassLane;
            public Entity m_Edge;
            public bool m_Returning;
        }

        private partial struct GhostJob
        {
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
        }
    }
}
