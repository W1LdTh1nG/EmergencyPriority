using Colossal.IO.AssetDatabase;
using Game.Modding;
using Game.Settings;

namespace EmergencyPriority
{
    // NOTE: the class SIMPLE NAME must stay unique across all of this author's mods. Setting.ApplyAndSave() saves via
    // AssetDatabase.SaveSpecificSetting(GetType().Name), which resolves by simple name and breaks on the first match —
    // so two mods both named `Setting` write into whichever one wins the race. Nothing on disk derives from the class
    // name; the [FileLocation] value and the LoadSettings(...) name argument are the on-disk identity, not this.
    [FileLocation(nameof(EmergencyPriority))]
    public class EmergencyPrioritySetting : ModSetting
    {
        public const string Section = "Main";
        public const string Group = "Emergency";
        public const string GroupLights = "Lights";
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

        public override void SetDefaults()
        {
            Enabled = true;
            DespawnGuard = true;
            AutoReroute = true;
            RerouteAfterSeconds = 5;
            GreenLightPriority = true;
            GreenLightDistance = 120;
        }
    }
}
