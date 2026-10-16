# Reconnection Check: 2026-10-09

No explicit motion, mode, power, recovery or load-setting command was issued.

1. The initial left-arm readback succeeded while the controller was idle.
   The SDK active toolset returned mass 0 kg, center of mass [0,0,0] m,
   zero inertia, and zero tool/reference transforms. The returned tools list
   contained g_tool_0 through g_tool_15 and tool0; all returned zero payload
   and zero transforms. This does not inspect unloaded RL projects, and is
   not a verification of whether collision detection is enabled.
2. A new capture/planning attempt stopped before producing any snapshot or
   trajectory: `robot moved during capture; discard this burst`.
   The original failure did not record the joint index or magnitude.
   Its log is `../full_plan/run_4e8TR7Ti/planning.log`.
3. A subsequent readback attempt returned `robot not idle; no further queries`.
   It produced no JSON result. No additional camera or robot query followed.

The capture diagnostic was enhanced to report joint index, reference/current
angles and delta on a future stationary-check failure. The 0.0005 rad threshold
was not relaxed. The camera executable was explicitly recompiled with make -B
because cached generated build files had future timestamps. The host reported
NTP synchronization; no clock or timestamp settings were changed.

Next hardware work requires an idle robot and reconciliation of the SDK zero
payload with the reported pendant configuration. A photo of the configured
tool name, mass and center of mass was requested. No historical target or plan
is authorized for automatic resumption.
