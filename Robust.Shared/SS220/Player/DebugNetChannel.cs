#if DEBUG

using System;
using System.Collections.Immutable;
using System.Net;
using Lidgren.Network;
using Robust.Shared.Network;

namespace Robust.Shared.SS220.Player;

public sealed class DebugNetChannel : INetChannel
{
    public DebugNetChannel(INetManager netPeer, NetUserId userId, string userName)
    {
        NetPeer = netPeer;
        UserId = userId;
        UserName = userName;
        UserData = new NetUserData(userId, userName)
        {
            HWId = ImmutableArray<byte>.Empty
        };
    }

    public INetManager NetPeer { get; }
    public long ConnectionId => 0;
    public IPEndPoint RemoteEndPoint { get; } = new(IPAddress.Loopback, 0);
    public NetUserId UserId { get; }
    public string UserName { get; }
    public LoginType AuthType => LoginType.GuestAssigned;
    public TimeSpan RemoteTimeOffset => TimeSpan.Zero;
    public TimeSpan RemoteTime => TimeSpan.Zero;
    public short Ping => 0;
    public bool IsConnected => true;
    public NetUserData UserData { get; }
    public bool IsHandshakeComplete => true;
    public int CurrentMtu => 0;

    public T CreateNetMessage<T>() where T : NetMessage, new()
    {
        throw new NotSupportedException();
    }

    public void SendMessage(NetMessage message)
    {
    }

    public void Disconnect(string reason)
    {
    }

    public void Disconnect(string reason, bool sendBye)
    {
    }

    public bool CanSendImmediately(NetDeliveryMethod method, int sequenceChannel)
    {
        return true;
    }
}

#endif
