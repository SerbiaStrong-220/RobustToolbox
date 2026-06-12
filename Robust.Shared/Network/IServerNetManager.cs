using System;
using System.Threading.Tasks;

namespace Robust.Shared.Network
{
    /// <summary>
    /// The server version of the INetManager.
    /// </summary>
    [NotContentImplementable]
    public interface IServerNetManager : INetManager
    {
        public delegate Task<NetApproval> NetApprovalDelegate(NetApprovalEventArgs eventArgs);

        byte[]? CryptoPublicKey { get; }
        AuthMode Auth { get;  }
        Func<string, Task<NetUserId?>>? AssignUserIdCallback { get; set; }
        NetApprovalDelegate? HandleApprovalCallback { get; set; }

        /// <summary>
        ///     Disconnects this channel from the remote peer.
        /// </summary>
        /// <param name="channel">NetChannel to disconnect.</param>
        /// <param name="reason">Reason why it was disconnected.</param>
        void DisconnectChannel(INetChannel channel, string reason);

        /// <summary>
        /// SS220: Handshake complete event before handling OnConnected
        /// </summary>
        event Func<NetChannelArgs, Task> InitialHandshakeComplete;

        /// <summary>
        /// SS220: Hack to correctly handle new channel data if it was changed after initial setup
        /// </summary>
        /// <param name="netChannel"></param>
        /// <param name="newData"></param>
        void ReSetupChannel(INetChannel netChannel, NetUserData newData);
    }
}
