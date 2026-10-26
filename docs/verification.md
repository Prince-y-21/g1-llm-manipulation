# Release verification

Local preparation date: October 26, 2026. Checks concern the public source candidate, not a new physical robot experiment.

- Python files were parsed without importing controller programs: passed.
- JSON syntax and Markdown file links: passed.
- Public prompt-library export regenerated from the public sources; its existing structural/hash validator: passed.
- Existing offline thermal-contract compiler suite: 10 tests passed.
- Existing moving-preview/stale-observation suite: 2 tests passed, using temporary synthetic files rather than robot I/O.
- Selected demonstration videos previously completed full decode checks; bottle and paper frame review used 18 and 12 output times respectively. Both clips are silent and retain acceleration/outcome captions.
- Original/public source hashes and a distributable file manifest are included.

No C++ recompilation was completed on this preparation machine: the attempted local WSL `g++` invocation reported that the compiler was unavailable. No compiler was installed. Hardware tests, model recalibration and full dependency restoration were not performed. Historical execution is not new validation of the sanitized public copy.

Run the standard-library static checker from the repository root:

```bash
python tools/check_release.py
```

Run the two included offline Python suites from their snapshot directory:

```bash
cd experiments/paper_partial_placement/tools_snapshot
python test_contract_compiler_v4.py
python test_preview_v4.py
```

The C++ thermal contract test is independent of the vendor motor/DDS libraries. On a Linux installation with a C++17 compiler, it can be built against the included local header. That build remains unverified for this public package:

```bash
mkdir -p build
c++ -std=c++17 -O2 -I experiments/paper_partial_placement/source_snapshot experiments/paper_partial_placement/tools_snapshot/test_thermal_v4.cpp -o build/test_thermal_v4
./build/test_thermal_v4
```

Passing synthetic/contract tests does not validate physical thermal recovery, collision freedom, camera/tool calibration or operation without a load-bearing suspension rig.
