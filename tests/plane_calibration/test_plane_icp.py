"""Synthetic ICP checks with correspondences isolated by board pose."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from wc_calibration.plane_calibration import solve_planes
from wc_calibration.plane_icp import refine_planes_icp, _rotation_exp


NORMALS = [[-1, 0, 0], [-0.8, 0.6, 0], [-0.8, 0, 0.6], [-0.75, -0.45, -0.48],
           [-0.6, 0.6, -0.52], [-0.65, -0.3, 0.7]]


def build_scene(holdout=True, noise=0.0, normals=NORMALS):
    rng = np.random.default_rng(301)
    rotation = _rotation_exp(np.array([0.05, -0.03, 0.14]))
    translation = np.array([0.03, -0.40, 0.02])
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = translation
    observations, clouds = [], {}
    for i, normal in enumerate(normals + (normals[:3] if holdout else [])):
        normal = np.asarray(normal, dtype=float); normal /= np.linalg.norm(normal)
        d = 1.7 + 0.055 * i
        centroid = -d * normal
        a = np.cross(normal, [0, 0, 1]); a /= np.linalg.norm(a)
        b = np.cross(normal, a)
        # Fixed full 2D board extent makes six-DOF JTJ observable.
        uv = rng.uniform(-0.35, 0.35, (900, 2))
        left = centroid + uv[:, :1] * a + uv[:, 1:] * b
        right = (left - translation) @ rotation
        if noise:
            left += rng.normal(0, noise, (len(left), 1)) * normal
            right += rng.normal(0, noise, (len(right), 1)) * (rotation.T @ normal)
        nr = rotation.T @ normal
        dr = d + float(normal @ translation)
        oid = f"pose_{i}"
        observations.append({"id": oid, "capture_id": f"capture_{i}",
                             "role": "train" if i < len(normals) else "holdout",
                             "same_face_confirmed": True,
                             "left": {"normal": normal.tolist(), "d_m": d, "centroid": centroid.tolist(), "status": "PASS"},
                             "right": {"normal": nr.tolist(), "d_m": dr, "centroid": (-dr * nr).tolist(), "status": "PASS"}})
        clouds[oid] = {"left": left, "right": right}
    return observations, clouds, transform


class PlaneICPTests(unittest.TestCase):
    def test_refines_known_small_initial_error_and_holdout(self):
        observations, clouds, truth = build_scene()
        initial = truth.copy()
        initial[:3, :3] = _rotation_exp(np.radians([0.4, -0.3, 0.6])) @ initial[:3, :3]
        initial[:3, 3] += [0.012, -0.010, 0.008]
        result = refine_planes_icp(observations, clouds, initial)
        self.assertEqual(result["status"], "CANDIDATE", result)
        np.testing.assert_allclose(result["refined_T_left_right"], truth, atol=2e-7)
        self.assertTrue(result["independent_validation_performed"])
        self.assertTrue(result["independent_validation_passed"])
        self.assertFalse(result["live_eligible"])
        self.assertLess(result["refined_train_summary"]["point_to_plane_rmse_m"],
                        result["initial_train_summary"]["point_to_plane_rmse_m"])
        json.dumps(result, allow_nan=False)

    def test_single_plane_degeneracy_refused_before_icp(self):
        observations, clouds, truth = build_scene()
        result = refine_planes_icp(observations[:1], clouds, truth)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIn("PLANE_INITIALIZATION_NOT_CANDIDATE", result["reasons"])

    def test_cloud_correspondences_cannot_cross_different_poses(self):
        observations, clouds, truth = build_scene(holdout=False)
        # Another pose may contain identical-looking board points, but this
        # pose's target has moved far *within its plane*: no cross-pose rescue.
        normal = np.asarray(observations[0]["left"]["normal"])
        axis = np.cross(normal, [0, 0, 1]); axis /= np.linalg.norm(axis)
        clouds["pose_0"]["left"] += 3.0 * axis
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("pose_0:INSUFFICIENT_ICP_OVERLAP" in r for r in result["reasons"]), result)

    def test_holdout_is_not_optimized(self):
        observations, clouds, truth = build_scene()
        baseline = refine_planes_icp(observations, clouds, truth)
        for i in range(6, 9):
            nr = np.asarray(observations[i]["right"]["normal"])
            clouds[f"pose_{i}"]["right"] += 0.015 * nr
            observations[i]["right"]["d_m"] -= 0.015
            dr = observations[i]["right"]["d_m"]
            observations[i]["right"]["centroid"] = (-dr * nr).tolist()
        changed = refine_planes_icp(observations, clouds, truth)
        np.testing.assert_array_equal(changed["refined_T_left_right"], baseline["refined_T_left_right"])
        self.assertGreater(changed["refined_holdout_summary"]["maximum_abs_offset_m"], 0.014)

    def test_degenerate_point_extent_rejected_by_jtj(self):
        observations, clouds, truth = build_scene(holdout=False)
        for i in range(6):
            centroid = np.asarray(observations[i]["left"]["centroid"])
            left = np.repeat(centroid[None, :], 60, axis=0)
            clouds[f"pose_{i}"] = {"left": left, "right": (left - truth[:3, 3]) @ truth[:3, :3]}
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIn("ICP_NORMAL_EQUATIONS_ILL_CONDITIONED", result["reasons"])

    def test_no_independent_validation_without_holdout(self):
        observations, clouds, truth = build_scene(holdout=False, noise=0.002)
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "CANDIDATE", result)
        self.assertFalse(result["independent_validation_performed"])
        self.assertFalse(result["independent_validation_passed"])

    def test_unfiltered_background_cloud_rejected(self):
        observations, clouds, truth = build_scene(holdout=False)
        clouds["pose_0"]["left"][:200, 0] += 0.3
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("ROI_CLOUD_NOT_PLANE_INLIERS" in r for r in result["reasons"]))

    def test_invalid_initial_rotation_rejected(self):
        observations, clouds, truth = build_scene()
        truth[:3, 0] *= -1
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("INVALID_PROPER_ROTATION" in r for r in result["reasons"]))

    def test_total_translation_and_rotation_bounds_are_enforced(self):
        normals = [[-1, 0, 0], [-1, 0.4, 0], [-1, -0.4, 0],
                   [-1, 0, 0.4], [-1, 0, -0.4], [-1, 0.2, 0.2]]
        observations, clouds, truth = build_scene(holdout=False, normals=normals)
        for translation_error, rotation_error in (([0, 0.11, 0], [0, 0, 0]),
                                                   ([0, 0, 0], [np.radians(6), 0, 0])):
            initial = truth.copy()
            initial[:3, 3] += translation_error
            initial[:3, :3] = _rotation_exp(np.asarray(rotation_error)) @ initial[:3, :3]
            result = refine_planes_icp(observations, clouds, initial)
            self.assertEqual(result["status"], "REJECTED", result)
            self.assertIn("ICP_TOTAL_CHANGE_LIMIT_EXCEEDED", result["reasons"], result)

    def test_holdout_regression_detected_even_inside_absolute_gates(self):
        observations, clouds, truth = build_scene()
        for i in range(6):
            nr = np.asarray(observations[i]["right"]["normal"])
            clouds[f"pose_{i}"]["right"] += 0.015 * nr
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "REJECTED", result)
        self.assertTrue(any("HOLDOUT_REGRESSION" in r for r in result["reasons"]), result)
        self.assertTrue(result["independent_validation_performed"])
        self.assertFalse(result["independent_validation_passed"])

    def test_repeated_roi_cannot_dominate_fitting(self):
        import copy
        observations, clouds, truth = build_scene(holdout=False)
        baseline = refine_planes_icp(observations, clouds, truth)
        for i in range(12):
            repeated = copy.deepcopy(observations[0])
            repeated["id"] = f"repeated_{i}"
            observations.append(repeated)
            nr = np.asarray(repeated["right"]["normal"])
            clouds[repeated["id"]] = {"left": clouds["pose_0"]["left"].copy(),
                                      "right": clouds["pose_0"]["right"] + 0.02 * nr}
        result = refine_planes_icp(observations, clouds, truth)
        self.assertEqual(result["status"], "CANDIDATE", result)
        np.testing.assert_array_equal(result["refined_T_left_right"], baseline["refined_T_left_right"])


if __name__ == "__main__":
    unittest.main()
