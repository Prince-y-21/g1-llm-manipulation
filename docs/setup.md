# Setup and reproduction levels

## What this release reproduces

The package supports reading the control implementation, checking source/data structure and tracing selected historical experiments. Hardware execution requires a new live configuration and the documented checks. There is no unified launch command for all historical programs.

The command below performs local static inspection only:

```bash
python tools/check_release.py
```

Do not use `import` to explore arbitrary historical scripts: some perform work at module scope. The checker parses syntax without importing those modules.

## Machines and dependencies

Historical work used Windows for tool orchestration and an Ubuntu robot server for C++ control/camera programs, often on ARM64. `robot-g1` is an SSH alias to configure locally, not an included host or credential. Confirm host identity against your own trusted reference.

| Component | Dependencies / assumptions |
|---|---|
| Body state and control | Unitree SDK2 C++ headers/library, Cyclone DDS C/C++ libraries, C++17, pthreads, correct DDS interface/domain |
| Dex1-1 | Appropriate Unitree motor SDK, serial headers/library, device mapping and permissions, correct motor-side/output-side units |
| Head RGB-D | librealsense2, correctly identified RGB-D device and stream formats |
| Wrist images | OpenCV with V4L2 support; historical camera device indexes must be re-enumerated |
| Python planning | NumPy/SciPy; Pillow for selected image tools; some historical programs additionally use geometry/robotics packages |
| Geometry | Correct G1/hand/tool assets and installation transforms, obtained separately; public approximate descriptions do not constitute physical calibration |

An exact cross-platform dependency lock was not captured for every historical version. Do not invent one from a current Windows environment or treat it as the robot-server environment. Inspect per-task build scripts and record the SDK ABI, compiler and package versions used on your installation.

## Historical build examples

`robot_tools/pour_build_control.py` records one body build using the SDK2 static library and DDS libraries, and one gripper build using the ARM64 motor SDK. It also generates task-specific source and writes to server directories. Read it as a historical build recipe; it is not a generic installer or read-only checker.

The later paper snapshot includes all local headers used by its body controller and the original compile helper under `experiments/paper_partial_placement/tools_snapshot/remote_compile_v4.py`. That helper expects other run artifacts and installed dependencies; running it is a deployment operation. Source completeness for an individual binary is distinct from complete reproduction of its supervisor, workspace and model files.

The paper supervisor references a pre-existing observer under an earlier task directory. That complete prior server workspace is not bundled, and no compatibility claim is made for substituting a similarly named observer. Resolve that dependency explicitly before hardware reproduction. The bottle source snapshot is retained separately rather than silently merging the two control generations.

## Hardware execution conditions

Use the prompt library to establish one boot check per boot, current mode/control ownership, support conditions, calibrated/approximate geometry status, temperature/battery margins and a task-specific stop/return plan. Historical low-level trials relied on fully load-bearing suspension, debug-position mode and an on-site emergency-stop operator. Free-standing/locomotion behavior is outside that tested scope.

`configs/robot.example.json` is a descriptive checklist. Historical scripts do not automatically read it; it documents the values that a future unified runtime must expose. It contains no live targets or authority to move.
