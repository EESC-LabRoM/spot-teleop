# Held-drill model fit — offline, not deployable (2026-11-21)

The current v3 held-object models explain part of the premature obstacle
avoidance while carrying the drill, but the data does not yet support a safe
smaller replacement. No live files or robot state were changed.

First-pass persisted measurements from `held_fit_results.json` and the
053830/050322 recorded runs:

| Run | Model | Cloud coverage | 12 mm margin coverage | Support-column coverage | Lowest sphere at grasp | Side overhang |
|---|---|---:|---:|---:|---:|---:|
| 053830 | live v3 | 99.52% | 96.97% | 23.08% | -34.7 mm | 60.8 mm |
| 050322 | live v3 | 98.36% | 96.55% | 64.3% | -14.8 mm | 49.8 mm |
| 053830 | first 4-sphere solve, 5 mm support floor | 100% | 99.73% | 66.67% | +0.5 mm | 104.6 mm |

A later single solve reported by the optimizer worker had 100% cloud coverage
and 99.6% with 12 mm margin, but only 61.5% of hidden base samples covered
(10.5 mm gap) and 101 mm side overhang. That candidate was not persisted in
the result JSON and its tests were not rerun; treat it as provisional. Both
nonpenetrating candidates are rejected because they miss part of the support
column and have more lateral overhang than v3. Other exact MILP cases timed
out or were infeasible; all attempted table-clear fits for 050322 were
infeasible. A synthetic exact-box unit test in the optimization branch failed;
no passing held-fit test suite is claimed.

Conclusion: keep the deployed held model and 5 cm MPC activation distance
unchanged. A replacement needs a corrected objective and acceptance gate
requiring full visible-cloud coverage, an explicitly modelled support column
through the known table-contact area, nonpenetration at grasp, and full-body
swept replay. The current mesh-only replay cannot certify unknown space. This
document is a diagnostic result, not an instruction to test the current grasp
on the robot.
