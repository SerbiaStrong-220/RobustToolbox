# Release notes for RobustToolbox.

Changes to RobustToolbox made by the SS220 team.

<!-- If last documented version if release candidate and you're willing to create another release candidate or even release itself you should be modifying the last block instead of creating a new one  -->

## c1.1.0-rc0

Available upstream versions: v277.0.0

This is a release candidate version and may differ from the final version.

### New features

* Added `IServerNetManager.InitialHandshakeCompleted` event which raises after NetChannel set up and serialization manager finishes its handshake, right before `Connected` event.
* Added `IServerNetManager.ReSetupChannel(INetChannel, NetUserData, LoginType)` hack for overwriting NetChannel data by content. Highly not recommended for use outside handshake context. Use it only if you know what you are doing.

### Other

* Awakening bodies now is thread-safe

## c1.0.1

Available upstream versions: v272.0.0, v277.0.0

*Changes weren't documented*

## c1.0.0

Available upstream versions: v272.0.0

*Changes weren't documented*
