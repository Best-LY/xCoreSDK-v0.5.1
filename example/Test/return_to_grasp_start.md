# Return to the Saved Grasp Start

From any working directory:

```bash
/home/arm/xCoreSDK-v0.5.1/example/Test/return_to_grasp_start.sh --execute
```

Read-only planning:

```bash
/home/arm/xCoreSDK-v0.5.1/example/Test/return_to_grasp_start.sh --plan
```

No arguments or `--help` prints help without connecting to a robot.
The launcher builds `return_observation` before running it.

The fixed reference is `../function_config/recorded_poses/20261006_143947_before_manual/left.yml`.
It is the earlier left-arm observation/grasp-start pose, not home, and is never
overwritten by the launcher. Joint angles (radians) are approximately:

```text
[-0.607991887, -1.587736628, -0.504887486, -3.233608012, 2.317595219, -0.923861741]
```

- Moves only the left arm (192.168.2.161), using MoveAbsJ at 1% joint speed.
- No camera capture, recognition, or camera connection is required.
- Both robots must be reachable and idle. The right arm is read-only monitored.
- The executable checks soft limits, sampled joint-path workspace, actual path,
  speed, controller errors, interruption, timeout, and final joint position.
- It stops, powers down the left arm, and returns it to manual mode on completion;
  on execution errors or SIGINT/SIGTERM it attempts stop and cleanup.
- A sampled joint path is NOT an environment or self-collision certificate.
  Confirm tool disengagement, cable slack, full swept-space clearance, and onsite
  emergency-stop supervision before each execution. Do not run other controllers
  concurrently. An emergency stop remains necessary for loss of host/network control.
- Returning does not automatically start perception, approach, or gripping.
