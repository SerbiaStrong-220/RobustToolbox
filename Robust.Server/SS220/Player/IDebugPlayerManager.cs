#if DEBUG

using Robust.Shared.Player;

namespace Robust.Server.SS220.Player;

public interface IDebugPlayerManager
{
    ICommonSession AddDebugSession(string name);
    bool RemoveDebugSession(ICommonSession session);
}

#endif
