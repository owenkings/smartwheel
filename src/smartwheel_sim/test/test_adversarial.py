from smartwheel_sim.adversarial import SCENARIOS, run_all


def by_name(result, name):
    return next(item for item in result["results"] if item["scenario"]["name"] == name)


def test_all_required_adversarial_scenarios_are_executed():
    result = run_all()
    assert len(SCENARIOS) == 14
    assert result["scenario_count"] == 14


def test_time_offsets_are_rejected_by_configured_gates():
    result = run_all()
    assert not by_name(result, "lidar_imu_time_offset")["metrics"]["lidar_imu_sync_pass"]
    assert not by_name(result, "dual_lidar_time_offset")["metrics"]["dual_lidar_pairing_pass"]


def test_wheel_and_extrinsic_faults_are_quantitatively_detected():
    result = run_all()
    assert by_name(result, "wheel_scale_error")["detectors"]["wheel_inconsistency"]
    assert by_name(result, "left_right_wheel_mismatch")["detectors"]["wheel_inconsistency"]
    assert by_name(result, "wheel_slip")["detectors"]["wheel_inconsistency"]
    assert by_name(result, "extrinsic_perturbation")["detectors"]["extrinsic_map_degradation"]


def test_degenerate_geometry_and_dropouts_are_detected():
    result = run_all()
    assert by_name(result, "long_corridor")["detectors"]["geometric_degeneracy"]
    assert by_name(result, "narrow_fov_occlusion")["detectors"]["geometric_degeneracy"]
    assert by_name(result, "lidar_frame_drop")["detectors"]["lidar_dropout"]
    assert by_name(result, "imu_frame_drop")["detectors"]["imu_dropout"]


def test_modeled_loop_correction_reduces_endpoint_error():
    result = run_all()
    comparison = result["loop_comparison"]
    assert comparison["enabled_endpoint_error_m"] < comparison["disabled_endpoint_error_m"]
