using Game.Net;
using Game.Objects;
using Game.Prefabs;
using Game.Vehicles;
using Unity.Entities;
using Unity.Mathematics;

namespace EmergencyPriority
{
    // Slow traffic pulls over — the civilian directly ahead of a responder in the same lane brakes to a stop for
    // it, so the responder scoots past car after car to the head of the queue. Same-lane only, only for civilians
    // already in slow traffic, only with the responder close behind, and brakes at the civilian's own braking rate.
    // The civilian resumes on its own once the responder is ahead of it (it is then simply following the responder).
    public partial class EmergencyGhostSystem
    {
        // Pull-over: a civilian counts as "in slow traffic" below this nav speed (8 m/s ≈ 29 km/h), and stops when a
        // responder is within this many metres behind it in the same lane.
        private const float kSlowTrafficSpeed = 8f;
        private const float kPullOverDistance = 15f;

        private partial struct GhostJob
        {
            // Civilian side. Returns true if `navigation` was lowered (caller writes it back).
            private bool TryPullOver(Entity entity, CarCurrentLane lane, bool driving, ref CarNavigation navigation, Moving moving, PrefabRef prefabRef)
            {
                if (!(m_PullOver && driving && navigation.m_MaxSpeed < kSlowTrafficSpeed
                    && ResponderCloseBehind(entity, lane)))
                    return false;

                // Brake to a stop at the civilian's own braking rate — vanilla's speed range floor
                // (VehicleUtils.CalculateSpeedRange) — so it looks like a stop, not a freeze.
                float braked = math.max(0f, math.length(moving.m_Velocity)
                    - m_PrefabCarData[prefabRef.m_Prefab].m_Braking * kTimeStep);
                if (braked >= navigation.m_MaxSpeed)
                    return false;

                navigation.m_MaxSpeed = braked;
                m_Stats[kPulledOver]++;
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
    }
}
