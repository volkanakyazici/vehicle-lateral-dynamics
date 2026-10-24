# Validation record

**Latest recorded run:** 24 October 2026  
**MATLAB/Simulink:** 26.2.0.3386108 (R2026b)  
**Primary evidence:** [`results/simulink/execution_status.json`](../results/simulink/execution_status.json)

## Local MATLAB/Simulink run

The recorded command was:

```matlab
run_project
```

The execution status reports a completed `PASS` suite containing **21 MATLAB mathematical assertions** and **12 Simulink scenario runs**. `run_project.m` runs the math checks, loads `matlab/lateral_bicycle.slx`, executes all cases, exports CSV and JSON results, and configures the model for nominal 20 m/s tracking.

The latest recorded run uses the repository layout with `matlab/lateral_bicycle.slx` loaded directly. Its start and completion timestamps are stored in `execution_status.json`. The results below refer to that local MATLAB/Simulink execution.

### Mathematical checks (21/21 PASS)

At 10, 20, and 30 m/s, `test_math.m` checks seven properties per speed:

1. Stable continuous lateral two-state subsystem poles.
2. Converged discrete Riccati solution and closed-loop spectral radius below one.
3. Agreement between direct Newton/tire equations and the state-space matrix form.
4. Agreement with the analytical steady-state yaw gain.
5. Zero-input/zero-state consistency.
6. Input sign symmetry for the linear open-loop model.
7. Sample-period refinement for the tracked lateral position (0.01 s versus 0.005 s).

These are mathematical and numerical consistency checks for the *chosen model*, not measured tire/vehicle validation.

## Simulink verification: individual cases

Each `run_case.m` execution compares Simulink-logged `state_log`, `request_log`, and `steering_log` with MATLAB's separate sampled implementation, `reference_loop.m`. It also verifies sample count and time grid, finite outputs, steering angle/rate limits, and case-specific behavior. The source-based acceptance tolerances are:

- Maximum state-component difference: **less than 1e-8**, in the component's SI unit.
- Maximum requested steering difference: **less than 1e-9 rad**.
- Maximum applied steering difference: **less than 1e-9 rad**.
- For `track` and `offset` (not `limited`): full-run lateral-position RMSE **less than 0.15 m** and final absolute error **less than 0.02 m**.

| Case | Status | Max state-component difference (SI unit) | Max applied-steering difference | Small-angle screen |
|---|---|---:|---:|---|
| `step_20` | PASS | 0 | 0 rad | YES |
| `sine_20` | PASS | 0 | 0 rad | YES |
| `step_10` | PASS | 0 | 0 rad | YES |
| `step_30` | PASS | 0 | 0 rad | YES |
| `uncontrolled_10` | PASS | 0 | 0 rad | YES |
| `track_10` | PASS | 1.08e-19 | 4.34e-19 rad | YES |
| `uncontrolled_20` | PASS | 0 | 0 rad | YES |
| `track_20` | PASS | 4.44e-16 | 7.01e-17 rad | YES |
| `uncontrolled_30` | PASS | 0 | 0 rad | YES |
| `track_30` | PASS | 2.78e-16 | 1.73e-17 rad | YES |
| `offset_20` | PASS | 5.55e-17 | 1.04e-17 rad | YES |
| `limited_20` | PASS | 8.67e-19 | 4.34e-19 rad | YES |

The largest recorded state-component mismatch was **4.44e-16**, largest requested-steering mismatch **7.01e-17 rad**, and largest applied-steering mismatch **7.01e-17 rad**. Differences this small are consistent with floating-point round-off, but cannot establish accuracy of the physical model.

## Range and constraint checks

`run_case.m` enforces the configured angle and steering-rate limits. The `limited_20` case specifically verifies that steering saturation is activated.

The case reports separately record a modeling screen:

- Maximum absolute front/rear tire slip angle ≤ 3°.
- Maximum absolute body heading ≤ 10°.
- Maximum lateral acceleration ≤ 0.3 g.

The recorded screen passed for **all 12 cases**. This is an internal applicability screen based on chosen project thresholds, **not** an industry certification of linear-tire validity. The restricted steering case still performs poorly in tracking despite passing the screen.

## Python analysis and evidence

Source: `python/analyze.py`. The script reads **12 saved Simulink CSV histories**; checks execution provenance; calculates metrics; and generates five figures plus `results/simulink/metrics.csv`.

```bash
python3 -m pip install -r requirements.txt
python3 python/analyze.py --input results/simulink
```

The optional `tests/check_numerics.py` runs a Python-only numerical preflight and writes outputs under ignored `results/preview/`. Those Python preview outputs are deliberately excluded from the saved Simulink evidence.

Each individual CSV and JSON report can be found at `results/simulink/<case>_<speed>.csv` and `.json`. Figures live in `results/simulink/figures/`.

## Reproducibility notes

- Place `matlab/` in MATLAB's Current Folder and run `run_project`.
- Ensure Simulink is available and no older `lateral_bicycle` model with the same name is already loaded from a different folder.
- The committed `.slx` is the model source; no model-generation script is required.
- Only MATLAB/Simulink R2026b has been reported as tested.
- Re-running the full suite overwrites generated `results/simulink/` case exports and status logs.
- The case suite was automated by `run_project`; the logged scenarios should not be misrepresented as manual, independent retuning experiments.

## What was **not** validated

There was no track test, physical test bench, measured vehicle identification, sensor-noise test, actuator-delay test, friction-limit identification, tire combined-slip model, or hardware-in-the-loop run.

Simulink and MATLAB use the same physical assumptions and state equations. Their agreement establishes implementation consistency, not real-world predictive accuracy. Steering limits are represented as ideal discrete constraints and not as a steering system model.
