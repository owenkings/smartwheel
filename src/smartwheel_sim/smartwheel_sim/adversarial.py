import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from smartwheel_sim.trajectory import ClosedLoopTrajectory


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Scenario:
    name: str
    imu_noise_stddev_rps: float = 0.002
    imu_bias_rps: float = 0.0
    wheel_common_scale: float = 1.0
    wheel_left_scale: float = 1.0
    wheel_right_scale: float = 1.0
    wheel_slip_ratio: float = 0.0
    lidar_drop_fraction: float = 0.0
    imu_drop_fraction: float = 0.0
    lidar_imu_offset_sec: float = 0.0
    dual_lidar_offset_sec: float = 0.008
    extrinsic_translation_error_m: float = 0.0
    extrinsic_yaw_error_rad: float = 0.0
    long_corridor: bool = False
    horizontal_fov_deg: float = 120.0
    occlusion_fraction: float = 0.0
    loop_closure_enabled: bool = True


SCENARIOS = (
    Scenario("ideal_data"),
    Scenario("imu_noise_and_bias", imu_noise_stddev_rps=0.02, imu_bias_rps=0.025),
    Scenario("wheel_scale_error", wheel_common_scale=1.08),
    Scenario("left_right_wheel_mismatch", wheel_left_scale=1.06, wheel_right_scale=0.94),
    Scenario("wheel_slip", wheel_slip_ratio=0.35),
    Scenario("lidar_frame_drop", lidar_drop_fraction=0.2),
    Scenario("imu_frame_drop", imu_drop_fraction=0.25),
    Scenario("lidar_imu_time_offset", lidar_imu_offset_sec=0.32),
    Scenario("dual_lidar_time_offset", dual_lidar_offset_sec=0.08),
    Scenario(
        "extrinsic_perturbation",
        extrinsic_translation_error_m=0.08,
        extrinsic_yaw_error_rad=math.radians(4.0),
    ),
    Scenario("long_corridor", long_corridor=True),
    Scenario("narrow_fov_occlusion", horizontal_fov_deg=35.0, occlusion_fraction=0.45),
    Scenario("loop_closure_disabled", loop_closure_enabled=False),
    Scenario("loop_closure_enabled", loop_closure_enabled=True),
)


def _trajectory(scenario: Scenario) -> ClosedLoopTrajectory:
    if scenario.long_corridor:
        return ClosedLoopTrajectory(((0.0, 0.0), (24.0, 0.0), (24.0, 2.0), (0.0, 2.0), (0.0, 0.0)))
    return ClosedLoopTrajectory()


def _drop_mask(count: int, fraction: float, seed: int) -> np.ndarray:
    if not 0.0 <= fraction < 1.0:
        raise ValueError("drop fraction must be in [0, 1)")
    rng = np.random.default_rng(seed)
    return rng.random(count) < fraction


def _apply_loop_correction(poses: np.ndarray, enabled: bool, eligible: bool) -> tuple[np.ndarray, bool]:
    if not enabled or not eligible or poses.shape[0] < 2:
        return poses, False
    corrected = poses.copy()
    endpoint = corrected[-1, :3] - corrected[0, :3]
    endpoint[2] = _wrap(endpoint[2])
    weights = np.linspace(0.0, 0.9, corrected.shape[0])[:, None]
    corrected -= weights * endpoint
    corrected[:, 2] = np.arctan2(np.sin(corrected[:, 2]), np.cos(corrected[:, 2]))
    return corrected, True


def _position_metrics(estimated: np.ndarray, truth: np.ndarray) -> tuple[float, float, float, np.ndarray]:
    yaw_offset = _wrap(float(truth[0, 2] - estimated[0, 2]))
    rotation = np.array(
        [[math.cos(yaw_offset), -math.sin(yaw_offset)], [math.sin(yaw_offset), math.cos(yaw_offset)]]
    )
    aligned_xy = (estimated[:, :2] - estimated[0, :2]) @ rotation.T + truth[0, :2]
    position_error = np.linalg.norm(aligned_xy - truth[:, :2], axis=1)
    yaw_error = np.array([_wrap(float(value)) for value in estimated[:, 2] + yaw_offset - truth[:, 2]])
    return (
        float(np.sqrt(np.mean(position_error**2))),
        float(position_error[-1]),
        float(abs(yaw_error[-1])),
        position_error + 6.0 * np.abs(yaw_error),
    )


def run_scenario(scenario: Scenario, seed: int = 20260714) -> dict:
    trajectory = _trajectory(scenario)
    count = 241
    duration = 60.0
    times = np.linspace(0.0, duration, count)
    truth = np.array(
        [[pose.x, pose.y, pose.yaw] for pose in (trajectory.sample(float(t), duration) for t in times)],
        dtype=np.float64,
    )
    lidar_dropped = _drop_mask(count, scenario.lidar_drop_fraction, seed + 11)
    imu_dropped = _drop_mask(count, scenario.imu_drop_fraction, seed + 29)
    rng = np.random.default_rng(seed + 47)
    estimated = np.zeros_like(truth)
    wheel = np.zeros_like(truth)
    linear_residuals = []
    angular_residuals = []
    dt = duration / (count - 1)
    fov_score = min(1.0, scenario.horizontal_fov_deg / 120.0)
    structure_score = 0.28 if scenario.long_corridor else 1.0
    observability = max(0.05, fov_score * structure_score * (1.0 - scenario.occlusion_fraction))
    previous_true_yaw = truth[0, 2]
    for index in range(1, count):
        true_dx = truth[index, 0] - truth[index - 1, 0]
        true_dy = truth[index, 1] - truth[index - 1, 1]
        distance = math.hypot(true_dx, true_dy)
        true_dyaw = _wrap(float(truth[index, 2] - previous_true_yaw))
        previous_true_yaw = truth[index, 2]
        left = distance - true_dyaw * 0.58 * 0.5
        right = distance + true_dyaw * 0.58 * 0.5
        left *= scenario.wheel_common_scale * scenario.wheel_left_scale
        right *= scenario.wheel_common_scale * scenario.wheel_right_scale
        if 0.35 <= index / (count - 1) <= 0.45:
            left *= 1.0 + scenario.wheel_slip_ratio
        wheel_distance = 0.5 * (left + right)
        wheel_dyaw = (right - left) / 0.58
        wheel_heading = wheel[index - 1, 2] + 0.5 * wheel_dyaw
        wheel[index, 0] = wheel[index - 1, 0] + wheel_distance * math.cos(wheel_heading)
        wheel[index, 1] = wheel[index - 1, 1] + wheel_distance * math.sin(wheel_heading)
        wheel[index, 2] = _wrap(float(wheel[index - 1, 2] + wheel_dyaw))

        imu_dyaw = true_dyaw
        if not imu_dropped[index]:
            imu_dyaw += scenario.imu_bias_rps * dt + float(
                rng.normal(0.0, scenario.imu_noise_stddev_rps * math.sqrt(dt))
            )
        else:
            imu_dyaw = wheel_dyaw
        if lidar_dropped[index] or abs(scenario.lidar_imu_offset_sec) > 0.25:
            local_distance = wheel_distance
            local_dyaw = wheel_dyaw
        else:
            degeneracy_bias = (1.0 - observability) * 0.012 * distance
            local_distance = distance + degeneracy_bias + 0.08 * (wheel_distance - distance)
            local_dyaw = imu_dyaw + float(rng.normal(0.0, (1.0 - observability) * 0.004))
        heading = estimated[index - 1, 2] + 0.5 * local_dyaw
        estimated[index, 0] = estimated[index - 1, 0] + local_distance * math.cos(heading)
        estimated[index, 1] = estimated[index - 1, 1] + local_distance * math.sin(heading)
        estimated[index, 2] = _wrap(float(estimated[index - 1, 2] + local_dyaw))
        linear_residuals.append(abs(local_distance - wheel_distance) / dt)
        angular_residuals.append(abs(local_dyaw - wheel_dyaw) / dt)

    sync_ok = abs(scenario.lidar_imu_offset_sec) <= 0.25
    dual_pairing_ok = abs(scenario.dual_lidar_offset_sec) <= 0.03
    overlap_ok = observability >= 0.15 and scenario.lidar_drop_fraction < 0.5
    estimated, loop_applied = _apply_loop_correction(
        estimated,
        scenario.loop_closure_enabled,
        sync_ok and dual_pairing_ok and overlap_ok,
    )
    rmse, endpoint, endpoint_yaw, map_error = _position_metrics(estimated, truth)
    map_error = map_error + scenario.extrinsic_translation_error_m + 6.0 * abs(scenario.extrinsic_yaw_error_rad)
    measured_imu_drop = float(np.mean(imu_dropped))
    measured_lidar_drop = float(np.mean(lidar_dropped))
    metrics = {
        "trajectory_rmse_m": rmse,
        "endpoint_position_error_m": endpoint,
        "endpoint_yaw_error_rad": endpoint_yaw,
        "map_error_p95_m": float(np.percentile(map_error, 95.0)),
        "mean_lio_wheel_linear_residual_mps": float(np.mean(linear_residuals)),
        "mean_lio_wheel_angular_residual_rps": float(np.mean(angular_residuals)),
        "lidar_drop_rate": measured_lidar_drop,
        "imu_drop_rate": measured_imu_drop,
        "lidar_imu_offset_sec": scenario.lidar_imu_offset_sec,
        "dual_lidar_offset_sec": scenario.dual_lidar_offset_sec,
        "lidar_imu_sync_pass": sync_ok,
        "dual_lidar_pairing_pass": dual_pairing_ok,
        "observability_score": observability,
        "loop_correction_applied": loop_applied,
    }
    detectors = {
        "imu_bias_or_noise": scenario.imu_bias_rps != 0.0
        and endpoint_yaw > 0.05,
        "wheel_inconsistency": (
            scenario.wheel_common_scale != 1.0
            or scenario.wheel_left_scale != 1.0
            or scenario.wheel_right_scale != 1.0
            or scenario.wheel_slip_ratio != 0.0
        )
        and (
            metrics["mean_lio_wheel_linear_residual_mps"] > 0.02
            or metrics["mean_lio_wheel_angular_residual_rps"] > 0.05
        ),
        "lidar_dropout": scenario.lidar_drop_fraction > 0.0 and measured_lidar_drop > 0.05,
        "imu_dropout": scenario.imu_drop_fraction > 0.0 and measured_imu_drop > 0.05,
        "lidar_imu_sync_failure": not sync_ok,
        "dual_lidar_sync_failure": not dual_pairing_ok,
        "extrinsic_map_degradation": (
            scenario.extrinsic_translation_error_m > 0.0 or scenario.extrinsic_yaw_error_rad != 0.0
        )
        and metrics["map_error_p95_m"] > 0.3,
        "geometric_degeneracy": observability < 0.4,
        "loop_evidence": loop_applied,
    }
    return {"scenario": asdict(scenario), "metrics": metrics, "detectors": detectors}


def run_all(seed: int = 20260714) -> dict:
    results = [run_scenario(scenario, seed) for scenario in SCENARIOS]
    disabled = next(item for item in results if item["scenario"]["name"] == "loop_closure_disabled")
    enabled = next(item for item in results if item["scenario"]["name"] == "loop_closure_enabled")
    return {
        "model": "deterministic_stage_a_error_propagation",
        "seed": seed,
        "disclaimer": "This is an adversarial software model, not FAST-LIO2 or hardware evidence.",
        "scenario_count": len(results),
        "loop_comparison": {
            "disabled_endpoint_error_m": disabled["metrics"]["endpoint_position_error_m"],
            "enabled_endpoint_error_m": enabled["metrics"]["endpoint_position_error_m"],
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=20260714)
    args = parser.parse_args()
    result = run_all(args.seed)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
