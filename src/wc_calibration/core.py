"""Numerical calibration without ROS, hardware access, or invented live transforms.

T_left_right maps right coordinates into left. Each scene is matched in its own
left coordinates; scenes need not have been registered into a global map.
"""

from dataclasses import asdict, dataclass
import hashlib
import itertools
import json
import math
import numpy as np
import scipy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


class CalibrationError(ValueError):
    """An explicit invalid input or unavailable estimate; never silent success."""


@dataclass(frozen=True)
class QualityProfile:
    """Experiment starting values. Persisted with results, not sensor accuracy claims."""

    voxel_sizes: tuple = (0.10, 0.055, 0.025)
    correspondence_distances: tuple = (0.40, 0.22, 0.12)
    normal_neighbors: int = 16
    normal_max_curvature: float = 0.12
    iterations_per_scale: int = 35
    huber_m: float = 0.025
    min_scene_points: int = 80
    min_correspondences: int = 45
    min_overlap_ratio: float = 0.35
    validation_p95_m: float = 0.04
    information_relative_min: float = 2e-4
    normal_direction_min: float = 0.015
    repeat_translation_max_m: float = 0.025
    repeat_rotation_max_rad: float = 0.02

    def __post_init__(self):
        if not self.voxel_sizes or len(self.voxel_sizes) != len(self.correspondence_distances):
            raise CalibrationError("voxel and correspondence scales must be nonempty and equal")
        for value in (*self.voxel_sizes, *self.correspondence_distances, self.huber_m):
            if not np.isfinite(value) or value <= 0:
                raise CalibrationError("scales and loss size must be finite positive metres")
        if self.normal_neighbors < 6 or self.min_scene_points < self.normal_neighbors:
            raise CalibrationError("insufficient neighborhood/scene size")
        if self.min_correspondences < 12 or self.iterations_per_scale < 1:
            raise CalibrationError("invalid correspondence or iteration budget")
        if not 0 < self.min_overlap_ratio <= 1:
            raise CalibrationError("min_overlap_ratio must be in (0,1]")
        for value in (self.normal_max_curvature, self.validation_p95_m,
                      self.information_relative_min, self.normal_direction_min,
                      self.repeat_translation_max_m, self.repeat_rotation_max_rad):
            if not np.isfinite(value) or value <= 0:
                raise CalibrationError("quality limits must be finite and positive")


def _profile(value=None):
    return value if isinstance(value, QualityProfile) else QualityProfile(**(value or {}))


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode("utf-8")).hexdigest()


def validate_transform(value):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise CalibrationError("transform must be finite 4x4")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise CalibrationError("transform bottom row is not homogeneous")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or \
            not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise CalibrationError("transform rotation must belong to SO(3)")
    return transform.copy()


def _points(value, minimum=3):
    points = np.asarray(value, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < minimum:
        raise CalibrationError("point array must be Nx3 with sufficient observations")
    if not np.isfinite(points).all():
        raise CalibrationError("nonfinite points require explicit upstream filtering")
    return points


def _transform(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def _skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def _se3_exp(delta):
    omega, velocity = delta[:3], delta[3:]
    theta = float(np.linalg.norm(omega))
    wx = _skew(omega)
    rotation = Rotation.from_rotvec(omega).as_matrix()
    if theta < 1e-8:
        vmat = np.eye(3) + wx / 2 + wx @ wx / 6
    else:
        vmat = np.eye(3) + (1 - math.cos(theta)) / theta**2 * wx + \
            (theta - math.sin(theta)) / theta**3 * (wx @ wx)
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = vmat @ velocity
    return result


def _voxel(points, size):
    indices = np.floor(points / size).astype(np.int64)
    _, inverse = np.unique(indices, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    return np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts
                            for axis in range(3)])


def _normals(points, profile):
    count = min(profile.normal_neighbors, len(points))
    tree = cKDTree(points)
    _, neighbors = tree.query(points, k=count)
    patches = points[neighbors]
    centered = patches - patches.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / count
    values, vectors = np.linalg.eigh(covariance)
    valid = (values[:, 1] > 1e-10) & \
        (values[:, 0] / np.maximum(values.sum(axis=1), 1e-15) < profile.normal_max_curvature)
    return vectors[:, :, 0], valid, tree


def _scene_digest(left, right):
    # Point order does not create an independent validation scene.
    def ordered(points):
        order = np.lexsort((points[:, 2], points[:, 1], points[:, 0]))
        return np.ascontiguousarray(points[order], dtype="<f8").tobytes()
    return hashlib.sha256(ordered(left) + b"RIGHT" + ordered(right)).hexdigest()


def _dataset(payload, profile):
    if payload.get("schema_version") != 1 or payload.get("source_mode") not in {"synthetic", "real"}:
        raise CalibrationError("schema_version=1 and explicit synthetic|real source_mode required")
    identities = payload.get("sensor_ids", {})
    if not all(isinstance(identities.get(side), str) and identities[side].strip()
               for side in ("left", "right")) or identities["left"] == identities["right"]:
        raise CalibrationError("two nonempty distinct sensor identities are required")
    seen_ids, seen_contents = set(), set()
    output = {}
    for split in ("training", "validation"):
        scenes = payload.get(split, [])
        if not isinstance(scenes, list) or len(scenes) < (2 if split == "training" else 1):
            raise CalibrationError("at least two training scenes and one independent held-out scene required")
        output[split] = []
        for scene in scenes:
            identity = scene.get("id")
            if not isinstance(identity, str) or not identity:
                raise CalibrationError("each scene needs a stable nonempty id")
            left = _points(scene.get("left"), profile.min_scene_points)
            right = _points(scene.get("right"), profile.min_scene_points)
            digest = _scene_digest(left, right)
            if identity in seen_ids or digest in seen_contents:
                raise CalibrationError("repeated scene id/content: training and holdout must be independent")
            seen_ids.add(identity)
            seen_contents.add(digest)
            output[split].append({"id": identity, "left": left, "right": right, "hash": digest})
    if payload.get("axis_conversion_count", 1) != 1:
        raise CalibrationError("axis convention must be converted exactly once")
    return output


def _prepared(scenes, voxel, profile, reverse=False):
    output = []
    for scene in scenes:
        target = _voxel(scene["right" if reverse else "left"], voxel)
        source = _voxel(scene["left" if reverse else "right"], voxel)
        if len(target) < profile.normal_neighbors or len(source) < profile.normal_neighbors:
            raise CalibrationError("downsampling leaves insufficient geometry")
        normals, valid, tree = _normals(target, profile)
        output.append((scene["id"], target, source, normals, valid, tree))
    return output


def _correspondences(prepared, transform, max_distance):
    rows, residuals, all_normals, all_q, statistics = [], [], [], [], []
    for identity, target, source, normals, valid, tree in prepared:
        q = _transform(source, transform)
        distances, indices = tree.query(q, k=1)
        keep = (distances <= max_distance) & valid[indices]
        q, p, normal = q[keep], target[indices[keep]], normals[indices[keep]]
        r = np.einsum("ij,ij->i", normal, q - p)
        rows.append(np.column_stack((np.cross(q, normal), normal)))
        residuals.append(r)
        all_normals.append(normal)
        all_q.append(q)
        statistics.append({"scene_id": identity, "source_points": len(source),
                           "correspondences": int(keep.sum()), "overlap_ratio": float(keep.mean()),
                           "abs_residual_quantiles_m": _quantiles(np.abs(r)),
                           "euclidean_distance_quantiles_m": _quantiles(distances[keep])})
    return (np.concatenate(rows), np.concatenate(residuals), np.concatenate(all_normals),
            np.concatenate(all_q), statistics)


def _quantiles(values):
    if not len(values):
        return {"p50": None, "p90": None, "p95": None, "p99": None}
    return dict(zip(("p50", "p90", "p95", "p99"),
                    map(float, np.quantile(values, (0.5, 0.9, 0.95, 0.99)))))


def _optimize(prepared, initial, distance, profile, iterations=None):
    transform = initial.copy()
    history = []
    converged = False
    for _ in range(iterations or profile.iterations_per_scale):
        jacobian, residual, _, _, _ = _correspondences(prepared, transform, distance)
        if len(residual) < 12:
            return transform, {"converged": False, "reason": "INSUFFICIENT_CORRESPONDENCES", "iterations": history}
        weights = np.minimum(1.0, profile.huber_m / np.maximum(np.abs(residual), 1e-12))
        weighted_j = jacobian * np.sqrt(weights[:, None])
        delta, _, rank, _ = np.linalg.lstsq(weighted_j, -residual * np.sqrt(weights), rcond=1e-10)
        # Bounded steps prevent a poor initial association from making a large jump.
        scale = max(1.0, np.linalg.norm(delta[:3]) / 0.15, np.linalg.norm(delta[3:]) / 0.20)
        delta /= scale
        transform = _se3_exp(delta) @ transform
        history.append({"rmse_m": float(np.sqrt(np.mean(residual**2))),
                        "correspondences": len(residual), "rank": int(rank),
                        "rotation_step_rad": float(np.linalg.norm(delta[:3])),
                        "translation_step_m": float(np.linalg.norm(delta[3:]))})
        if np.linalg.norm(delta[:3]) < 1e-7 and np.linalg.norm(delta[3:]) < 1e-7:
            converged = True
            break
    return transform, {"converged": converged, "iterations": history}


def _coarse_initial(scenes, profile):
    """Explicit geometric PCA hypotheses, not guaranteed global place recognition."""
    left, right = scenes[0]["left"], scenes[0]["right"]
    lc, rc = np.median(left, axis=0), np.median(right, axis=0)
    _, lvec = np.linalg.eigh(np.cov((left - lc).T))
    _, rvec = np.linalg.eigh(np.cov((right - rc).T))
    hypotheses = [np.eye(4)]
    hypotheses[0][:3, 3] = lc - rc
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            basis = np.eye(3)[:, permutation] @ np.diag(signs)
            rotation = lvec @ basis @ rvec.T
            if np.linalg.det(rotation) < 0:
                continue
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, lc - rotation @ rc
            hypotheses.append(transform)
    prepared = _prepared(scenes, profile.voxel_sizes[0], profile)
    def score(transform):
        values = []
        for _, _, source, _, _, tree in prepared:
            distances, _ = tree.query(_transform(source, transform), k=1)
            values.append(float(np.mean(np.sort(distances)[:max(1, int(len(distances) * 0.7))])))
        return float(np.mean(values))
    ranked = sorted(hypotheses, key=score)[:4]
    refined = [_optimize(prepared, candidate, profile.correspondence_distances[0], profile, 15)[0]
               for candidate in ranked]
    refined.sort(key=score)
    scores = [score(item) for item in refined]
    ambiguous = False
    for candidate, candidate_score in zip(refined[1:], scores[1:]):
        difference = candidate @ np.linalg.inv(refined[0])
        distinct = np.linalg.norm(difference[:3, 3]) > 0.10 or \
            Rotation.from_matrix(difference[:3, :3]).magnitude() > 0.08
        if distinct and candidate_score <= scores[0] + 0.03:
            ambiguous = True
    return refined[0], {"method": "PCA_proper_rotation_hypotheses_then_joint_ICP",
                        "hypothesis_count": len(hypotheses),
                        "scores_m": scores, "ambiguous_distinct_hypotheses": bool(ambiguous),
                        "global_uniqueness_validated": False}


def _information(jacobian, normals, q, residual, profile):
    if len(jacobian) < 12:
        return {"observable": False, "reason": "INSUFFICIENT_CORRESPONDENCES",
                "eigenvalues": [], "weak_directions": []}
    radius = max(0.1, float(np.sqrt(np.mean(np.sum((q - q.mean(axis=0))**2, axis=1)))))
    normalized = jacobian.copy()
    normalized[:, :3] /= radius
    weights = np.minimum(1.0, profile.huber_m / np.maximum(np.abs(residual), 1e-12))
    matrix = (normalized.T * weights) @ normalized / len(normalized)
    values, vectors = np.linalg.eigh(matrix)
    ratio = float(max(0.0, values[0]) / max(values[-1], 1e-15))
    normal_values = np.linalg.eigvalsh(normals.T @ normals / len(normals))
    observable = ratio >= profile.information_relative_min and normal_values[0] >= profile.normal_direction_min
    weak = np.flatnonzero(values < values[-1] * profile.information_relative_min)
    return {"observable": bool(observable), "normalization_radius_m": radius,
            "parameter_order": ["radius*rx", "radius*ry", "radius*rz", "tx", "ty", "tz"],
            "matrix": matrix.tolist(), "eigenvalues": values.tolist(), "relative_min": ratio,
            "weak_directions": vectors[:, weak].T.tolist(),
            "normal_direction_eigenvalues": normal_values.tolist(),
            "covariance": None, "covariance_status": "UNAVAILABLE_not_a_sensor_noise_model"}


def _evaluate(dataset, transform, profile):
    diagnostics, reasons = {}, []
    finest = profile.voxel_sizes[-1]
    distance = profile.correspondence_distances[-1]
    for split in ("training", "validation"):
        forward = _correspondences(_prepared(dataset[split], finest, profile), transform, distance)
        backward = _correspondences(_prepared(dataset[split], finest, profile, reverse=True),
                                    np.linalg.inv(transform), distance)
        info = _information(forward[0], forward[2], forward[3], forward[1], profile)
        diagnostics[split] = {"scene_ids": [scene["id"] for scene in dataset[split]],
                              "scene_hashes": [scene["hash"] for scene in dataset[split]],
                              "forward": forward[4], "backward": backward[4], "information": info}
        for direction, result in (("forward", forward), ("backward", backward)):
            for scene in result[4]:
                if scene["correspondences"] < profile.min_correspondences or \
                        scene["overlap_ratio"] < profile.min_overlap_ratio:
                    reasons.append(f"{split}:{scene['scene_id']}:{direction}:LOW_OVERLAP")
                p95 = scene["abs_residual_quantiles_m"]["p95"]
                if p95 is None or p95 > profile.validation_p95_m:
                    reasons.append(f"{split}:{scene['scene_id']}:{direction}:RESIDUAL_LIMIT")
        if not info["observable"]:
            reasons.append(f"{split}:DEGENERATE_GEOMETRY")
    return diagnostics, reasons


def _result(payload, dataset, transform, profile, diagnostics, reasons):
    provenance = payload.get("provenance", {})
    required = ("installation_id", "coordinate_mode", "firmware_versions", "sdk_version", "static_scene_evidence")
    static_evidence = provenance.get("static_scene_evidence", {})
    scenes_confirmed = isinstance(static_evidence, dict) and all(
        isinstance(static_evidence.get(scene["id"]), str) and static_evidence[scene["id"]].strip()
        for split in ("training", "validation") for scene in dataset[split])
    provenance_ok = all(provenance.get(item) for item in required) and scenes_confirmed
    # A geometric fit only validates extrinsics; it never validates time, ground, or navigation.
    live_eligible = not reasons and payload["source_mode"] == "real" and provenance_ok
    status = "UNVALIDATED" if reasons else ("VALIDATED" if live_eligible else "CANDIDATE")
    calibration_id = _hash({"T_left_right": transform.tolist(), "sensor_ids": payload["sensor_ids"],
                            "source_mode": payload["source_mode"], "input_hash": _hash(payload),
                            "provenance": provenance, "quality_profile": asdict(profile)})
    return {"schema_version": 1, "calibration_id": calibration_id, "status": status, "source_mode": payload["source_mode"],
            "sensor_ids": payload["sensor_ids"], "T_left_right": transform.tolist(),
            "transform_definition": "p_left = T_left_right * p_right; metres; radians; xyzw",
            "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
            "T_right_left": np.linalg.inv(transform).tolist(), "live_eligible": live_eligible,
            "validation_scope": "fixed_dual_lidar_extrinsic_geometry_only",
            "provenance": provenance, "missing_live_provenance": [key for key in required if not provenance.get(key)],
            "all_static_scenes_have_evidence": bool(scenes_confirmed),
            "diagnostics": diagnostics, "rejection_reasons": sorted(set(reasons)),
            "input_hash": _hash(payload), "quality_profile": asdict(profile),
            "quality_profile_role": "explicit_experimental_thresholds_not_accuracy_claims",
            "versions": {"algorithm": "wc_calibration/0.1.0", "numpy": np.__version__, "scipy": scipy.__version__},
            "time_validated": False, "ground_validated": False, "navigation_validated": False}


def validate_calibration(payload, transform, profile=None):
    """Evaluate a frozen T on independent held-out scenes without optimizing it."""
    profile = _profile(profile)
    dataset = _dataset(payload, profile)
    transform = validate_transform(transform)
    diagnostics, reasons = _evaluate(dataset, transform, profile)
    diagnostics["operation"] = "frozen_transform_holdout_validation"
    return _result(payload, dataset, transform, profile, diagnostics, reasons)


def calibrate(payload, profile=None):
    profile = _profile(profile)
    dataset = _dataset(payload, profile)
    if payload.get("initial_T_left_right") is None:
        initial, initialization = _coarse_initial(dataset["training"], profile)
    else:
        initial = validate_transform(payload["initial_T_left_right"])
        initialization = {"method": "provided_initial_T_left_right"}
    transform, stages = initial.copy(), []
    for voxel, distance in zip(profile.voxel_sizes, profile.correspondence_distances):
        prepared = _prepared(dataset["training"], voxel, profile)
        transform, detail = _optimize(prepared, transform, distance, profile)
        stages.append({"voxel_m": voxel, "max_correspondence_m": distance, **detail})
    diagnostics, reasons = _evaluate(dataset, transform, profile)
    if initialization.get("ambiguous_distinct_hypotheses"):
        reasons.append("COARSE_INITIALIZATION_AMBIGUOUS_DISTINCT_POSES")
    # Independent small initial perturbations test local repeatability, not global uniqueness.
    repeats = []
    prepared = _prepared(dataset["training"], profile.voxel_sizes[-1], profile)
    for perturbation in ([0.012, -0.009, 0.008, 0.014, -0.011, 0.010],
                         [-0.010, 0.011, -0.009, -0.013, 0.012, -0.008]):
        repeated, _ = _optimize(prepared, _se3_exp(np.asarray(perturbation)) @ transform,
                                profile.correspondence_distances[-1], profile, 20)
        difference = repeated @ np.linalg.inv(transform)
        repeats.append({"translation_difference_m": float(np.linalg.norm(difference[:3, 3])),
                        "rotation_difference_rad": float(Rotation.from_matrix(difference[:3, :3]).magnitude())})
    if any(item["translation_difference_m"] > profile.repeat_translation_max_m or
           item["rotation_difference_rad"] > profile.repeat_rotation_max_rad for item in repeats):
        reasons.append("LOCAL_REPEAT_SOLVES_UNSTABLE")
    change = transform @ np.linalg.inv(initial)
    diagnostics.update({"initialization": initialization, "stages": stages,
                        "local_repeat_solve_dispersion": repeats,
                        "global_uniqueness_validated": False,
                        "initial_to_result_translation_m": float(np.linalg.norm(change[:3, 3])),
                        "initial_to_result_rotation_rad": float(Rotation.from_matrix(change[:3, :3]).magnitude()),
                        "scene_motion_assumption": "each_input_pair_is_static; externally_confirmed_for_real_data"})
    return _result(payload, dataset, transform, profile, diagnostics, reasons)


def fit_ground_plane(points, roi, *, trusted_roi=False, expected_up=None,
                     expected_height_m=None, threshold_m=0.025, max_tilt_rad=0.35,
                     min_inlier_ratio=0.65, iterations=250, seed=20260911, source_mode="synthetic"):
    """RANSAC only inside a caller-supplied trusted ground ROI; no largest-plane shortcut."""
    if source_mode not in {"synthetic", "real"}:
        raise CalibrationError("explicit ground source_mode required")
    if not trusted_roi:
        raise CalibrationError("trusted ground ROI acknowledgement is required")
    points = _points(points, 20)
    if not isinstance(roi, dict) or "min" not in roi or "max" not in roi:
        raise CalibrationError("ground ROI requires finite min/max xyz bounds")
    lower, upper = np.asarray(roi["min"], float), np.asarray(roi["max"], float)
    if lower.shape != (3,) or upper.shape != (3,) or not np.isfinite([lower, upper]).all() or np.any(lower >= upper):
        raise CalibrationError("invalid ground ROI")
    if threshold_m <= 0 or iterations < 1 or not 0 < min_inlier_ratio <= 1:
        raise CalibrationError("invalid RANSAC bounds")
    selected = points[np.all((points >= lower) & (points <= upper), axis=1)]
    if len(selected) < 20:
        raise CalibrationError("insufficient points inside trusted ground ROI")
    up = None if expected_up is None else np.asarray(expected_up, float)
    if up is not None and (up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-10):
        raise CalibrationError("expected_up must be a finite nonzero vector")
    if up is not None:
        up = up / np.linalg.norm(up)
    generator = np.random.default_rng(seed)
    best = np.zeros(len(selected), dtype=bool)
    for _ in range(iterations):
        a, b, c = selected[generator.choice(len(selected), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        size = np.linalg.norm(normal)
        if size < 1e-10:
            continue
        normal /= size
        if up is not None and abs(float(normal @ up)) < math.cos(max_tilt_rad):
            continue
        residuals = np.abs((selected - a) @ normal)
        mask = residuals <= threshold_m
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < min_inlier_ratio:
        raise CalibrationError("no ground plane satisfies trusted ROI and support constraints")
    support = selected[best]
    centroid = support.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((support - centroid).T))
    if values[1] < 1e-5 or values[1] / max(values[2], 1e-15) < 0.01:
        raise CalibrationError("ground support is a degenerate line or point")
    normal = vectors[:, 0]
    reference = up if up is not None else np.array([0., 0., 1.])
    if normal @ reference < 0:
        normal = -normal
    offset = -float(normal @ centroid)
    if up is not None and normal @ up < math.cos(max_tilt_rad):
        raise CalibrationError("refined ground normal violates independent up constraint")
    height = offset  # Plane below sensor: n*p + positive offset = 0.
    height_consistent = expected_height_m is not None and abs(height - expected_height_m) <= 3 * threshold_m
    if expected_height_m is not None and not height_consistent:
        raise CalibrationError("ground height disagrees with supplied independent height")
    # Plane tilt fixes roll/pitch; chosen x axis is a reference convention, not measured yaw.
    x = np.array([1., 0., 0.])
    x -= normal * (normal @ x)
    if np.linalg.norm(x) < 0.1:
        x = np.array([0., 1., 0.]) - normal * normal[1]
    x /= np.linalg.norm(x)
    y = np.cross(normal, x)
    transform = np.eye(4)
    transform[:3, :3] = np.vstack((x, y, normal))
    transform[2, 3] = offset
    eligible = source_mode == "real" and up is not None and height_consistent
    return {"schema_version": 1, "source_mode": source_mode, "status": "VALIDATED" if eligible else "CANDIDATE",
            "normal_rig": normal.tolist(), "plane_offset_m": offset, "height_m": height,
            "T_ground_rig": transform.tolist(), "inlier_ratio": float(best.mean()),
            "residual_quantiles_m": _quantiles(np.abs(support @ normal + offset)),
            "roi": roi, "trusted_roi": True, "seed": seed, "live_eligible": eligible,
            "expected_up_available": up is not None, "independent_height_consistent": bool(height_consistent),
            "unobservable_dofs": ["horizontal_translation_x", "horizontal_translation_y", "yaw"],
            "reference_convention": "projected rig x; zero horizontal translation; all map products must share this transform",
            "input_hash": _hash(points.tolist())}


def fit_clock_model(device_ns, host_receive_ns, *, stream_epochs=None,
                    measurement_semantics_verified=False, transport_bias_bound_ns=None,
                    oscillator_extrapolation_bound_ppm=None, max_extrapolation_ns=0):
    """Fit arrival=a*device+b using integer-centered ns.

    Intercept includes unknown fixed transport delay. Small regression residuals
    alone never verify exposure time. Unknown uncertainty is None, never zero.
    """
    device, host = list(device_ns), list(host_receive_ns)
    if len(device) != len(host) or len(device) < 5:
        raise CalibrationError("clock fitting needs at least five paired samples")
    if any(isinstance(value, bool) or not isinstance(value, (int, np.integer)) for value in device + host):
        raise CalibrationError("clock samples must be integer nanoseconds")
    if stream_epochs is not None and (len(stream_epochs) != len(device) or len(set(stream_epochs)) != 1):
        raise CalibrationError("fit each stream epoch separately; never mix reboot/wrap epochs")
    if any(b <= a for a, b in zip(device, device[1:])) or any(b <= a for a, b in zip(host, host[1:])):
        raise CalibrationError("nonmonotonic clock: split resets/wraps or order paired samples explicitly")
    if max_extrapolation_ns < 0:
        raise CalibrationError("negative extrapolation horizon")
    for bound in (transport_bias_bound_ns, oscillator_extrapolation_bound_ppm):
        if bound is not None and (not np.isfinite(bound) or bound < 0):
            raise CalibrationError("clock uncertainty bounds must be nonnegative and evidence supplied")
    d0, h0 = int(device[len(device)//2]), int(host[len(host)//2])
    x = np.array([int(value) - d0 for value in device], dtype=float) * 1e-9
    y = np.array([int(value) - h0 for value in host], dtype=float) * 1e-9
    if np.ptp(x) < 1e-3:
        raise CalibrationError("clock observation span insufficient to estimate drift")
    design = np.column_stack((x, np.ones(len(x))))
    parameters = np.linalg.lstsq(design, y, rcond=None)[0]
    for _ in range(15):
        residual = y - design @ parameters
        scale = max(1e-9, float(1.4826 * np.median(np.abs(residual - np.median(residual)))))
        weights = np.minimum(1., 1.5 * scale / np.maximum(np.abs(residual), 1e-15))
        parameters = np.linalg.lstsq(design * np.sqrt(weights[:, None]), y * np.sqrt(weights), rcond=None)[0]
    residual_ns = (y - design @ parameters) * 1e9
    a, centered_offset = map(float, parameters)
    empirical_jitter = float(np.max(np.abs(residual_ns)))
    uncertainty_valid = bool(measurement_semantics_verified and transport_bias_bound_ns is not None and
                             (max_extrapolation_ns == 0 or oscillator_extrapolation_bound_ppm is not None))
    extrapolation = (max_extrapolation_ns * oscillator_extrapolation_bound_ppm * 1e-6
                     if oscillator_extrapolation_bound_ppm is not None else 0.)
    uncertainty = empirical_jitter + transport_bias_bound_ns + extrapolation if uncertainty_valid else None
    result = {"schema_version": 1, "model": "host_origin_ns + a*(device_ns-device_origin_ns) + offset_ns",
              "device_origin_ns": d0, "host_origin_ns": h0, "a": a,
              "offset_ns": centered_offset * 1e9, "drift_ppm": (a - 1.) * 1e6,
              "stream_epoch": None if stream_epochs is None else stream_epochs[0],
              "valid_device_interval_ns": [int(device[0]), int(device[-1])],
              "max_extrapolation_ns": int(max_extrapolation_ns),
              "arrival_fit_residual_abs_ns": _quantiles(np.abs(residual_ns)),
              "empirical_max_arrival_residual_ns": empirical_jitter,
              "fixed_transport_bias_ns": None,
              "transport_bias_bound_ns": transport_bias_bound_ns,
              "measurement_semantics_verified": bool(measurement_semantics_verified),
              "uncertainty_valid": uncertainty_valid, "uncertainty_bound_ns": uncertainty,
              "time_valid": uncertainty_valid,
              "status": "BOUNDED_MEASUREMENT_MODEL" if uncertainty_valid else "ARRIVAL_MODEL_ONLY",
              "limitations": ["intercept includes unidentifiable fixed transport delay",
                              "empirical residual is not a proof of exposure synchronization",
                              "real relative exposure timing requires independent dynamic holdout"],
              "input_hash": _hash({"device_ns": [int(v) for v in device], "host_receive_ns": [int(v) for v in host]})}
    result["clock_model_id"] = _hash(result)
    return result
