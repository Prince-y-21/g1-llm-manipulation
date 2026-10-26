# Source guide

| Location | Role |
|---|---|
| `robot_tools/task_plan*.py`, `task_execute.py`, `task_step.py` | Bottle-era planning and staged execution; task-specific geometry, directories and command formats |
| `robot_tools/task_guard*.cpp`, `task_gripper.cpp` | Body/gripper control versions; inspect startup, holding, thermal and exit logic |
| `robot_tools/pour_*` | Pouring-task development, geometry, payload/thermal investigation and recovery; not evidence of completed pouring |
| `robot_tools/cap_*`, `bimanual_*` | Cap-task geometry and limited unloaded bimanual development; not a validated cap-opening skill |
| `experiments/bottle_to_mousepad/source_snapshot/` | Archived server-side source, including gripper variants absent from the shared directory |
| `experiments/paper_partial_placement/source_snapshot/` | Later body, gripper, camera, supervisor and local-header execution snapshots |
| `experiments/paper_partial_placement/tools_snapshot/` | Phase preparation, compilation, orchestration, verification and offline thermal-contract checks |
| `prompts/` | Operational knowledge, task templates and explicit validation gaps |

Task names do not establish success. Likewise, a source file's presence does not establish which exact binary ran. `source_provenance.csv` preserves original/public hashes and source roles; public transformations affect identity strings and documentation, so the public hash may differ from the original executed source.

Shared and archived copies are deliberately separated. Model assets, live working directories, installed vendor libraries and full raw logs are excluded. No historical trajectory is installed as a default active plan.

This release has not refactored hundreds of historical scripts into a single runtime. That work should preserve tested behavior, introduce explicit configuration and receive separate offline and hardware validation before being advertised as a supported deployment path.
