using Game.Vehicles;
using Unity.Entities;

namespace EmergencyPriority
{
    // Lights on to get through traffic. An ambulance only carries CarFlags.Emergency while dispatched to a
    // healthcare request or while its patient is flagged Critical (AmbulanceAISystem.ResetPath, :742-760); a
    // routine transport back to hospital drives as ordinary traffic and queues like everyone else. Real crews put
    // the lights on for the jam and off again once through. So: an ambulance with a patient aboard
    // (AmbulanceFlags.Transporting) that is held below kLightsOnSpeed by traffic gets the Emergency flag set here —
    // which is everything at once: priority 108, the doubled speed-limit factor, the warning lights (CarMoveSystem
    // raises TransformFlags.WarningLights from the flag), the green wave, and every feature in this system — and
    // gets it cleared again kLightsOffDelay after it was last held up. The AI only rewrites the flag when a new path
    // lands, when it parks, or when it takes a dispatch (:347, :697, :742-760), so the two never fight: if the AI
    // does reset it mid-jam, the next tick simply lights it up again.
    public partial class EmergencyGhostSystem
    {
        // A transporting ambulance held below kLightsOnSpeed by a vehicle lights up; it stays lit while held below
        // kLightsOffSpeed and goes dark kLightsOffDelayFrames (~5 s at 60 sim frames/s) after it was last held up,
        // so a stop-start queue does not strobe it.
        private const float kLightsOnSpeed = 3f;
        private const float kLightsOffSpeed = 6f;
        private const uint kLightsOffDelayFrames = 300;

        // An ambulance we lit up, keyed by entity: when it was last held up by traffic.
        public struct LitState
        {
            public uint m_LastBlockedFrame;
        }

        private partial struct GhostJob
        {
            // Civilian side: a transporting ambulance held up by traffic gets the lights. Returns true if `car` was
            // changed (caller writes it back and moves on — everything else follows from the flag on its next tick).
            private bool TryLightUp(Entity entity, ref Car car, Blocker held, bool driving, CarNavigation navigation)
            {
                if (!m_LightsInTraffic || !m_AmbulanceData.TryGetComponent(entity, out Game.Vehicles.Ambulance ambulance))
                    return false;

                bool transporting = driving && (ambulance.m_State & AmbulanceFlags.Transporting) != 0
                    && (ambulance.m_State & (AmbulanceFlags.AtTarget | AmbulanceFlags.Disembarking | AmbulanceFlags.Disabled)) == 0;
                if (transporting && held.m_Blocker != Entity.Null && navigation.m_MaxSpeed < kLightsOnSpeed
                    && (held.m_Type == BlockerType.Continuing || held.m_Type == BlockerType.Crossing))
                {
                    car.m_Flags |= CarFlags.Emergency;
                    m_Lit[entity] = new LitState { m_LastBlockedFrame = m_Frame };
                    m_Stats[kLitUp]++;
                    return true;
                }
                // Not (or no longer) a candidate: forget any entry left behind by an AI flag reset.
                m_Lit.Remove(entity);
                return false;
            }

            // Responder side: an ambulance we lit up for a jam keeps the lights while it is still held up, and loses
            // them once it has been clear for the delay or has stopped transporting. Returns true if the lights were
            // just dropped (caller writes `car` back and moves on).
            private bool TryDropLights(Entity entity, ref Car car, bool driving, CarNavigation navigation, Blocker blocker)
            {
                if (!m_Lit.TryGetValue(entity, out LitState litState))
                    return false;

                bool stillTransporting = driving
                    && m_AmbulanceData.TryGetComponent(entity, out Game.Vehicles.Ambulance litAmbulance)
                    && (litAmbulance.m_State & AmbulanceFlags.Transporting) != 0;
                bool heldUp = blocker.m_Blocker != Entity.Null && navigation.m_MaxSpeed < kLightsOffSpeed
                    && (blocker.m_Type == BlockerType.Continuing || blocker.m_Type == BlockerType.Crossing);
                if (stillTransporting && heldUp)
                {
                    litState.m_LastBlockedFrame = m_Frame;
                    m_Lit[entity] = litState;
                    return false;
                }
                if (stillTransporting && m_Frame - litState.m_LastBlockedFrame < kLightsOffDelayFrames)
                    return false;

                car.m_Flags &= ~CarFlags.Emergency;
                m_Lit.Remove(entity);
                m_Passes.Remove(entity);
                m_Stats[kLitOff]++;
                return true;
            }
        }
    }
}
