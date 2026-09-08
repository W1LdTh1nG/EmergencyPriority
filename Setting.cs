using Colossal.IO.AssetDatabase;
using Game.Modding;
using Game.Settings;

namespace EmergencyPriority
{
    // NOTE: the class SIMPLE NAME must stay unique across all of this author's mods. Setting.ApplyAndSave() saves via
    // AssetDatabase.SaveSpecificSetting(GetType().Name), which resolves by simple name and breaks on the first match —
    // so two mods both named `Setting` write into whichever one wins the race. Nothing on disk derives from the class
    // name; the [FileLocation] value and the LoadSettings(...) name argument are the on-disk identity, not this.
    [FileLocation("EmergencyPriority_Ghost")]
    public class EmergencyPrioritySetting : ModSetting
    {
        public const string Section = "Main";
        public const string Group = "Emergency";
        public const string GroupLights = "Lights";
        public const string GroupGhost = "Ghost";
        public const string GroupJunctions = "Junctions";
        public const string GroupGeneral = "General";

        public EmergencyPrioritySetting(IMod mod) : base(mod) { }

        // NOTE: initializers double as the settings-migration failsafe (missing keys in an old .coc keep these
        // values instead of defaulting to 0/false).

        // Master switch. OFF = pure vanilla (stuck responders get deleted, routes never re-evaluated).
        [SettingsUISection(Section, Group)]
        public bool Enabled { get; set; } = true;

        // Vanilla deletes a responding vehicle outright once the stuck detector flags it. The guard converts that
        // give-up into a fresh pathfind instead, so the unit keeps responding.
        [SettingsUISection(Section, Group)]
        public bool DespawnGuard { get; set; } = true;

        // Re-route a responder that has been sitting behind a blocker for a while — vanilla only prices congestion
        // at dispatch time and never re-evaluates the route en route.
        [SettingsUISection(Section, Group)]
        public bool AutoReroute { get; set; } = true;

        // Seconds a responder must be continuously blocked before it looks for a new route (sim-time seconds).
        [SettingsUISlider(min = 2f, max = 30f, step = 1f, unit = "integer")]
        [SettingsUISection(Section, Group)]
        public int RerouteAfterSeconds { get; set; } = 5;

        // Ask the traffic lights on the route ahead for green, and hold it until the responder is through. The green
        // is for the QUEUE in front of the responder — a responder ignores reds itself. See GreenLightPrioritySystem.
        [SettingsUISection(Section, GroupLights)]
        public bool GreenLightPriority { get; set; } = true;

        // How far ahead a responder is seen by the lights on its route, in metres. Generous by design: a petition is
        // served at the junction's next phase change, so the point is for the light to be green BEFORE it arrives.
        [SettingsUISlider(min = 40f, max = 300f, step = 10f, unit = "integer")]
        [SettingsUISection(Section, GroupLights)]
        public int GreenLightDistance { get; set; } = 120;

        // Reserve the lanes ahead of a responder at emergency priority so cross traffic stops short of them and
        // pedestrians wait at the kerb — the game's own right-of-way mechanism, asked further ahead than vanilla
        // does. Works at unsignalled junctions and roundabouts too. See JunctionClearSystem.
        [SettingsUISection(Section, GroupJunctions)]
        public bool JunctionClear { get; set; } = true;

        // How far ahead of the responder lanes are reserved, in metres. Long enough to cover the junction it is
        // approaching; too long holds several junctions at once.
        [SettingsUISlider(min = 20f, max = 150f, step = 10f, unit = "integer")]
        [SettingsUISection(Section, GroupJunctions)]
        public int JunctionClearDistance { get; set; } = 60;

        // A responder boxed in by STOPPED traffic in its own lane creeps through the car in front at walking pace,
        // as if the queue had pulled aside. Only same-lane vehicle blockers; never cross traffic, pedestrians or
        // signals. See EmergencyGhostSystem.
        [SettingsUISection(Section, GroupGhost)]
        public bool GhostThroughJams { get; set; } = true;

        // Speed while passing through, in m/s. Above ~3.5 m/s EmergencyGhostSystem also has to move the nav target
        // itself (the nav job only looks ~1 m ahead for a blocked car); 3 m/s is vanilla's own drive-through floor.
        [SettingsUISlider(min = 1f, max = 10f, step = 0.5f, unit = "floatSingleFraction")]
        [SettingsUISection(Section, GroupGhost)]
        public float GhostCrawlSpeed { get; set; } = 6f;

        // In slow traffic, the car directly ahead of a responder in the same lane brakes to a stop as if it had
        // pulled aside, so the responder passes car after car to the head of the queue. See EmergencyGhostSystem.
        [SettingsUISection(Section, GroupGhost)]
        public bool TrafficPullsOver { get; set; } = true;

        // A responder queued behind traffic moves into an empty neighbouring lane (either side — works for left or
        // right turns and for left- or right-hand traffic), passes the queue there, and cuts back into its own lane
        // just before the junction. See EmergencyGhostSystem.
        [SettingsUISection(Section, GroupGhost)]
        public bool GhostUsesFreeLane { get; set; } = true;

        // An ambulance carrying a patient without sirens (vanilla only lights a critical patient) puts them on when
        // traffic holds it up, and off again once through. See EmergencyGhostSystem.
        [SettingsUISection(Section, GroupGhost)]
        public bool LightsInTraffic { get; set; } = true;

        public override void SetDefaults()
        {
            Enabled = true;
            DespawnGuard = true;
            AutoReroute = true;
            RerouteAfterSeconds = 5;
            GreenLightPriority = true;
            GreenLightDistance = 120;
            JunctionClear = true;
            JunctionClearDistance = 60;
            GhostThroughJams = true;
            GhostCrawlSpeed = 6f;
            TrafficPullsOver = true;
            GhostUsesFreeLane = true;
            LightsInTraffic = true;
        }
    }
}
