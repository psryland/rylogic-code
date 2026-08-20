# Rylogic.D3D12

Lifetime-safe managed ownership contracts for shared Direct3D 12 interfaces.

`DeviceLease` owns exactly one COM reference to an `ID3D12Device`. Clones own independent references, and friend packages may pin a lease only for the duration
of a native call that takes its own reference. No public raw device pointer is exposed, so a producing renderer or compute engine may be disposed while a lease
remains alive.
