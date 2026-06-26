using System.Diagnostics;
using JetBrains.Annotations;
using Robust.Server.Physics;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Systems;

namespace Robust.Server.GameObjects
{
    [UsedImplicitly]
    public sealed partial class PhysicsSystem : SharedPhysicsSystem
    {
        [Dependency] private IConfigurationManager _configurationManager = default!;

        public override void Initialize()
        {
            base.Initialize();
            LoadMetricCVar();

            Subs.CVar(_configurationManager, CVars.MetricsEnabled, _ => LoadMetricCVar());
        }

        private void LoadMetricCVar()
        {
            MetricsEnabled = _configurationManager.GetCVar(CVars.MetricsEnabled);
        }

        private long _solveTickCount;
        private long _solveAccumulatedTicks;
        private const int ProfileInterval = 100; // log every 100 frames

        /// <inheritdoc />
        public override void Update(float frameTime)
        {
            var sw = Stopwatch.StartNew();
            SimulateWorld(frameTime, false);
            sw.Stop();
            _solveAccumulatedTicks += sw.ElapsedTicks;
            if (++_solveTickCount >= ProfileInterval)
            {
                double avgMs = (_solveAccumulatedTicks / (double)Stopwatch.Frequency) * 1000.0 / _solveTickCount;
                Log.Info($"Solve average: {avgMs:F4} ms over {_solveTickCount} frames");
                _solveTickCount = 0;
                _solveAccumulatedTicks = 0;
            }
        }
    }
}
