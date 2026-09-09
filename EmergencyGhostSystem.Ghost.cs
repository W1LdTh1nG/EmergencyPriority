using Game.Net;
using Game.Objects;
using Game.Prefabs;
using Game.Vehicles;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace EmergencyPriority
{
    // Drive through stopped traffic — the push.
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
    //    nor are Signal/Limit/Caution; a responder already ignores reds itself (priority 108). The one no-entity
    //    limit that IS pushed through is the don't-block-the-box rule (see PushThrough).
    //  - Uses max() on the responder's speed, so it only applies when the car in front is slower than the ghost speed.
    //  - Ramps at the vehicle's own acceleration so the pass starts smoothly rather than snapping to speed.
    //  - Writes the granted speed back into the responder's Blocker.m_MaxSpeed (same byte units the nav job uses,
    //    :1421) and leaves m_Blocker/m_Type alone. That byte is what everyone downstream reads to decide "is this
    //    vehicle blocked": StuckMovingObjectSystem's deadlock walk stops at any link >= 6 (:100, :121), and
    //    EmergencyRepathSystem's blocked clock uses the same `< 6` test. Left at the nav value of 0, a crawling
    //    responder would be re-routed every RerouteAfterSeconds, and each re-route empties its nav buffer — which
    //    pauses the ghost until the new path lands. Observed in play as stop-go crawling before the byte was written.
    public partial class EmergencyGhostSystem
    {
        // Slider ceiling. 10 m/s is 36 km/h — plenty for threading a parted queue.
        public const float kMaxGhostSpeed = 10f;

        // Extra look-ahead beyond speed*dt when re-targeting, so the mover's "braking" flag logic and small overshoots
        // never leave the target behind the car.
        private const float kTargetMargin = 0.5f;

        // A crossing-lane blocker slower than this is "stopped" (gridlock, accident tailback) rather than passing.
        private const float kStationarySpeed = 0.5f;

        // Re-target only when the new point is within ~35° of the vehicle's heading (cos 35° ≈ 0.82). A target more
        // than 1 m away AND more than 45° off heading makes a STOPPED vehicle's nav job decide to reverse to realign
        // (CarNavigationSystem.cs:2002-2013); with a blocker ahead its allowed speed is 0, so it "reverses" at zero
        // speed for ever. Vanilla's own 1 m target can never trip that; a pushed-out one can. Seen in play as an
        // angled ambulance sitting behind a pulled-over truck with `reversing` ticking up in the log.
        private const float kRetargetMinHeading = 0.82f;

        private partial struct GhostJob
        {
            // The push for one responder. Writes navigations/lanes/blockers[i] when it raises the speed.
            private void PushThrough(int i, NativeArray<CarNavigation> navigations, NativeArray<CarCurrentLane> lanes, NativeArray<Blocker> blockers,
                CarNavigation navigation, CarCurrentLane lane, Blocker blocker, bool driving, bool wantsReverse,
                Moving moving, PrefabRef prefabRef, Game.Objects.Transform transform)
            {
                // Already allowed to go at least ghost speed: whatever is in front is not holding it up.
                if (navigation.m_MaxSpeed >= m_GhostSpeed)
                    return;

                // Still navigating a road (the empty-buffer half is the "arrived" test, and also covers a
                // re-path in flight — the buffer is cleared until the new path lands).
                if (!driving)
                {
                    m_Stats[kSkipNotDriving]++;
                    return;
                }

                // Blocker is what the nav job just computed this frame for this vehicle, so it is current.
                bool crossing = blocker.m_Type == BlockerType.Crossing;
                bool spaceRule = false;
                if (blocker.m_Blocker == Entity.Null)
                {
                    // No entity: a rule, not a vehicle. "Continuing" with no entity is the don't-block-the-box
                    // check (CarLaneSpeedIterator.IterateNextLane -> CheckSpace, :404-457): don't enter the
                    // next lane unless there is room beyond it. In a car park's short lanes that can fail for
                    // ever with nothing actually in the way, and a responder is the vehicle the box should
                    // clear FOR — so push through it. Anything else with no entity (a reservation yield, a
                    // physical barrier, a speed limit) is left alone.
                    if (blocker.m_Type != BlockerType.Continuing)
                    {
                        m_Stats[kSkipNoBlocker]++;
                        return;
                    }
                    spaceRule = true;
                }
                else
                {
                    if (blocker.m_Type != BlockerType.Continuing && !crossing)
                    {
                        m_Stats[blocker.m_Type == BlockerType.Oncoming ? kSkipOncoming : kSkipOtherType]++;
                        return;
                    }
                    // A trailer (truck, articulated bus) is its own lane object with no Car component; the
                    // vehicle that owns it is in Controller.m_Controller — same resolution the nav job does.
                    Entity blockerVehicle = blocker.m_Blocker;
                    if (m_ControllerData.TryGetComponent(blockerVehicle, out Controller controller))
                        blockerVehicle = controller.m_Controller;
                    if (!m_CarData.HasComponent(blockerVehicle))
                    {
                        m_Stats[kSkipNotCar]++;
                        return;
                    }
                    // Cross traffic only when it is stopped. A car with no Moving component is parked/stopped;
                    // one with it must be at a standstill. Same-lane blockers need no such test: the max() below
                    // already leaves a faster car ahead alone, because the nav speed is then above ghost speed.
                    if (crossing && m_MovingData.TryGetComponent(blockerVehicle, out Moving blockerMoving)
                        && math.lengthsq(blockerMoving.m_Velocity) > kStationarySpeed * kStationarySpeed)
                    {
                        m_Stats[kSkipCrossing]++;
                        return;
                    }
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
                float currentSpeed = math.length(moving.m_Velocity);
                CarData prefabCar = m_PrefabCarData[prefabRef.m_Prefab];
                float desired = math.min(m_GhostSpeed, currentSpeed + prefabCar.m_Acceleration * kTimeStep);

                // The nav target is ~1 m ahead for a blocked car. Push it out to what `desired` needs, along the
                // current lane only (never across a lane change or past the lane end), then apply vanilla's own
                // no-overshoot clamp against wherever the target ended up.
                float3 position = transform.m_Position;
                float distance = math.distance(position, navigation.m_TargetPosition);
                float need = desired * kTimeStep + kTargetMargin;
                bool retargeted = false;
                if (distance < need && lane.m_ChangeLane == Entity.Null)
                    retargeted = AdvanceTarget(ref navigation, ref lane, prefabRef, position,
                        math.forward(transform.m_Rotation), need, ref distance);

                float target = math.min(desired, distance / kTimeStep);
                if (target <= navigation.m_MaxSpeed)
                    return;

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
                m_Stats[spaceRule ? kPushedSpaceRule : crossing ? kPushedCrossing : kPushed]++;
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
        }
    }
}
