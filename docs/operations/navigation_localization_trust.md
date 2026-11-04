# Offline localization evidence ledger

The ledger combines exact-capture-stamp localization output, the frozen prior,
and scan/map coverage diagnostics. It never selects a controller pose or returns
physical acceptance. Missing query results remain visible. No nearest-time join,
future packet consumption, localization reseeding, or query-frame deletion is
allowed. An explicit unavailable receipt blocks consumption; legacy missing
receipts permit offline geometric inspection with a timing blocker.

Coverage categories come from the declared frozen prior, never the pose fitted by
the localization being evaluated. They require 20 observed endpoints, 80% known
map coverage, and 70% endpoint support within the existing 0.15 m support band.
This adds a support requirement to the preceding coverage-only diagnostic. Good
coverage can still be ambiguous; the six along-heading hypotheses do not prove
pose uniqueness. These categories are review criteria, not physical truth.

Translation disagreement over 0.35 m and yaw disagreement over 8 degrees use the
existing planar policy as cumulative review triggers. Small particle standard
deviations (both XY <=0.10 m, yaw <=2 degrees) and high endpoint support (>=90%)
are reported alongside conflicts; neither clears them. The prior shares
odometry with map construction and is not independent physical ground truth.
The module preserves the original localization and calibration gates.

`evaluate_saved_localization.py` takes `--coverage-dir` with the prior and two
model coverage files, `--witnesses` with the original exported AMCL results, and
`--output`. It converts the coverage laser-origin pose to the configured axle
using the explicit parallel-axis mount [0.323, 0.22875] m. The converted AMCL
pose must match the original axle trajectory to 1e-9. Covariance comes from the
same exact-stamp original trajectory; aggregate covariance is never expanded
into per-frame evidence. Inputs are hashed and rechecked after evaluation.

The current 132-frame holdout retains all frames for both models. Frozen-prior
categories are 60 with insufficient coverage/support, 17 with along-heading
ambiguity, and 55 with no trigger in the tested hypotheses. The previous
coverage-only count was 59; the additional frame has 91.45% known coverage but
64.47% support. It is a stricter support review, not changed source data.

The default likelihood-field model has 25 translation conflicts, all in the
insufficient-coverage/support category; maximum shared-prior disagreement is
2.732 m. The probability model has 57 conflicts, 49 accompanied by small particle
covariance, and 9 by high endpoint support. Its maximum disagreement is 5.992 m.
In the same 55-frame no-trigger reference subset, maximum disagreements are
0.253 m and 0.404 m, respectively; the probability model still has 3 conflicts.
These observations do not prove the prior is correct. They show why low
covariance and map fit cannot be promoted into trusted localization.

The raw near-envelope overlaps occur outside this 70..105 s holdout. No overlap
is joined to an unrelated nearest localization timestamp. Arrival-reference
timestamps, unvalidated configured geometry, lack of independent pose truth,
and unknown receipts remain explicit blockers for all frames. No hardware or
control topics are used.

The maintained command is `bash scripts/navigation_localization_trust` with
`--coverage-dir`, `--witnesses`, and `--output reports/.../result.json`.
StoragePolicy checks the configured reports destination; existing reports are
never overwritten. The command does not import ROS or start localization.
