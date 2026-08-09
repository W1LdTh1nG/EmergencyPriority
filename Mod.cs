using System.Collections.Generic;
using Colossal;
using Colossal.IO.AssetDatabase;
using Colossal.Logging;
using Game;
using Game.Modding;
using Game.SceneFlow;
using Game.Simulation;

namespace EmergencyPriority
{
    public class Mod : IMod
    {
        public static ILog log = LogManager.GetLogger($"{nameof(EmergencyPriority)}.{nameof(Mod)}").SetShowsErrorsInUI(false);
        public static EmergencyPrioritySetting ActiveSetting;

        public void OnLoad(UpdateSystem updateSystem)
        {
            log.Info(nameof(OnLoad));

            ActiveSetting = new EmergencyPrioritySetting(this);
            ActiveSetting.RegisterInOptionsUI();
            var lm = GameManager.instance.localizationManager;
            foreach (var locale in lm.GetSupportedLocales())
                lm.AddSource(locale, new LocaleEn(ActiveSetting, locale));
            AssetDatabase.global.LoadSettings(nameof(EmergencyPriority), ActiveSetting, new EmergencyPrioritySetting(this));

            // After the stuck detector so a freshly raised Stuck flag is converted to a repath before the vehicle
            // AI systems (which run in the same phase) can take their delete branch.
            updateSystem.UpdateAfter<EmergencyRepathSystem, StuckMovingObjectSystem>(SystemUpdatePhase.GameSimulation);

            // Before the light system, so each petition is still standing when it reads. TrafficLightSystem consumes
            // what it reads (m_Priority is reset to m_Default), so this must re-assert every time it runs — hence the
            // matching update interval in GreenLightPrioritySystem.
            updateSystem.UpdateBefore<GreenLightPrioritySystem, TrafficLightSystem>(SystemUpdatePhase.GameSimulation);

            log.Info("[SelfTest] EmergencyPriority loaded (despawn guard + auto re-route + green-light priority).");
        }

        public void OnDispose()
        {
            log.Info(nameof(OnDispose));
            if (ActiveSetting != null)
            {
                ActiveSetting.UnregisterInOptionsUI();
                ActiveSetting = null;
            }
        }
    }

    // Minimal English locale (full localization once mechanics are proven, same pipeline as EconomyTweaks).
    public class LocaleEn : IDictionarySource
    {
        private readonly EmergencyPrioritySetting m_S;
        private readonly string m_L;
        public LocaleEn(EmergencyPrioritySetting setting, string locale) { m_S = setting; m_L = locale; }
        private string T(string k) => Translations.Get(k, m_L);

        public IEnumerable<KeyValuePair<string, string>> ReadEntries(IList<IDictionaryEntryError> errors, Dictionary<string, int> indexCounts)
        {
            return new Dictionary<string, string>
            {
                { m_S.GetSettingsLocaleID(), "Emergency Priority" },
                { m_S.GetOptionTabLocaleID(EmergencyPrioritySetting.Section), "Main" },
                { m_S.GetOptionGroupLocaleID(EmergencyPrioritySetting.Group), T("grp.Group") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.Enabled)), T("opt.Enabled.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.Enabled)), T("opt.Enabled.D") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.DespawnGuard)), T("opt.DespawnGuard.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.DespawnGuard)), T("opt.DespawnGuard.D") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.AutoReroute)), T("opt.AutoReroute.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.AutoReroute)), T("opt.AutoReroute.D") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.RerouteAfterSeconds)), T("opt.RerouteAfterSeconds.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.RerouteAfterSeconds)), T("opt.RerouteAfterSeconds.D") },

                { m_S.GetOptionGroupLocaleID(EmergencyPrioritySetting.GroupLights), T("grp.GroupLights") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.GreenLightPriority)), T("opt.GreenLightPriority.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.GreenLightPriority)), T("opt.GreenLightPriority.D") },
                { m_S.GetOptionLabelLocaleID(nameof(EmergencyPrioritySetting.GreenLightDistance)), T("opt.GreenLightDistance.L") },
                { m_S.GetOptionDescLocaleID(nameof(EmergencyPrioritySetting.GreenLightDistance)), T("opt.GreenLightDistance.D") },

                { m_S.GetOptionGroupLocaleID(EmergencyPrioritySetting.GroupGeneral), T("grp.GroupGeneral") },
            };
        }

        public void Unload() { }
    }
}
