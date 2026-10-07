# Rylogic.D3D12

Lifetime-safe managed ownership contracts for shared Direct3D 12 interfaces.

`ComLease` owns exactly one COM reference to a Direct3D 12 interface. `DeviceLease`, `FenceLease`, and `ResourceLease` name the `ID3D12Device`,
`ID3D12Fence`, and `ID3D12Resource` interfaces. Clones own independent references, and friend packages may pin a lease only for the duration of a native
call that takes its own reference. No public raw pointer is exposed, so a producing renderer or compute engine may be disposed while a lease remains alive.
