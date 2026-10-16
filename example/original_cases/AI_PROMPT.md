# AI Prompt - Original SDK Cases

You are reading the original official xCoreSDK v0.5.1 example cases.

Use this folder as the baseline for correct SDK API usage:
- Robot connection and lifecycle.
- Non-realtime motion commands.
- Realtime motion control.
- xMateModel and FollowPosition usage.
- Force control, keypad state, Modbus/RS485 examples.

Rules:
- Treat files here as reference material from the SDK, not as user task code.
- Do not rewrite or simplify official examples unless the user explicitly asks.
- When generating new runnable programs, copy patterns from here into `../function_config` or into a separate generated task file.
- Preserve the user's robot network convention when adapting examples:
  - left arm: `192.168.2.161`
  - right arm: `192.168.2.160`
  - local PC: `192.168.2.6`
- Prefer `xMateRobot` for the user's XMC18 setup unless a specific example requires another robot class.

Build behavior:
- Complete `.cpp` files in this folder are automatically scanned by `../CMakeLists.txt`.
- If a file uses xMateModel or FollowPosition, configure CMake with `-DXCORE_USE_XMATE_MODEL=ON`.
