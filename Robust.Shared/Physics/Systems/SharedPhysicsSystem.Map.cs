using Robust.Shared.GameObjects;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public partial class SharedPhysicsSystem
{
    #region AddRemove

    private readonly System.Threading.Lock _awakeBodiesLock = new(); // SS220 make awake bodies thread safe

    internal void AddAwakeBody(Entity<PhysicsComponent, TransformComponent> ent)
    {
        var body = ent.Comp1;

        if (!body.CanCollide)
        {
            Log.Error($"Tried to add non-colliding {ToPrettyString(ent)} as an awake body to map!");
            DebugTools.Assert(false);
            return;
        }

        if (body.BodyType == BodyType.Static)
        {
            Log.Error($"Tried to add static body {ToPrettyString(ent)} as an awake body to map!");
            DebugTools.Assert(false);
            return;
        }

        DebugTools.Assert(body.Awake);
        // SS220-make-awake-bodies-thread-safe-begin
        lock (_awakeBodiesLock)
        {
            DebugTools.Assert(!AwakeBodies.Contains(ent));
            AwakeBodies.Add(ent);
        }
        // SS220-make-awake-body-thread-safe-end
    }

    internal void RemoveSleepBody(Entity<PhysicsComponent, TransformComponent> ent)
    {
        DebugTools.Assert(!ent.Comp1.Awake);
        // SS220-make-awake-bodies-thread-safe-begin
        lock (_awakeBodiesLock)
        {
            DebugTools.Assert(AwakeBodies.Contains(ent));
            AwakeBodies.Remove(ent);
        }
        // SS220-make-awake-body-thread-safe-end
    }


    #endregion
}
