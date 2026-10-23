"""Meaningful synthetic tests for offline plane calibration, no hardware."""
import importlib.util
import json
import math
from pathlib import Path
import unittest

import numpy as np


MODULE = Path(__file__).resolve().parents[2] / "src" / "wc_calibration" / "plane_calibration.py"
spec = importlib.util.spec_from_file_location("plane_calibration_under_test", MODULE)
pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pc)


def rotation_xyz(x=0.0, y=0.0, z=0.0):
    cx, cy, cz = np.cos([x, y, z])
    sx, sy, sz = np.sin([x, y, z])
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array(
        [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @ np.array(
        [[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


def fit_descriptor(normal, d):
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    return {"status": "PASS", "normal": normal.tolist(), "d_m": float(d),
            "centroid": (-float(d) * normal).tolist()}


def observation(index, normal, *, rotation, translation, role="train", capture_id=None):
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal)
    # Plane lies in front of both lidar origins and both view the same face.
    dl = 2.0 + 0.07 * index
    nr = rotation.T @ normal
    dr = dl + float(normal @ translation)
    return {"id": f"observation_{index}", "role": role,
            "capture_id": capture_id or f"capture_{index}", "same_face_confirmed": True,
            "left": fit_descriptor(normal, dl), "right": fit_descriptor(nr, dr)}


NORMALS = [[-1, 0, 0], [-0.8, 0.6, 0], [-0.8, 0, 0.6],
           [-0.75, -0.45, -0.48], [-0.6, 0.6, -0.52], [-0.65, -0.3, 0.7]]


class PlaneFitTests(unittest.TestCase):
    def test_known_tilted_plane_noise_and_sparse_outliers(self):
        rng = np.random.default_rng(17)
        coords = rng.uniform(-0.4, 0.4, (1200, 2))
        normal = np.array([-1.0, 0.2, -0.1]); normal /= np.linalg.norm(normal)
        axis1 = np.cross(normal, [0, 0, 1]); axis1 /= np.linalg.norm(axis1)
        axis2 = np.cross(normal, axis1)
        centroid = -1.6 * normal
        xyz = centroid + coords[:, :1] * axis1 + coords[:, 1:] * axis2
        xyz += rng.normal(0, 0.004, (len(xyz), 1)) * normal
        xyz[:25] += 0.25 * normal
        result = pc.fit_plane(xyz, seed=11)
        self.assertEqual(result["status"], "PASS", result)
        self.assertGreater(np.dot(result["normal"], normal), 0.9999)
        self.assertLess(abs(result["d_m"] - 1.6), 0.001)
        self.assertLess(result["rmse_m"], 0.005)
        self.assertLess(result["inlier_fraction"], 0.99)
        self.assertGreater(result["inlier_fraction"], 0.95)
        self.assertTrue(set(range(25)).isdisjoint(result["inlier_indices"]))
        self.assertLess(np.dot(result["normal"], result["centroid"]), 0)
        json.dumps(result, allow_nan=False)

    def test_background_cannot_pass_on_42_percent_support(self):
        rng = np.random.default_rng(23)
        yz = rng.uniform(-0.4, 0.4, (1000, 2))
        xyz = np.column_stack([np.full(1000, 1.5), yz])
        xyz[420:, 0] += rng.uniform(0.15, 1.1, 580)
        result = pc.fit_plane(xyz)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIn("LOW_INLIER_FRACTION", result["reasons"])
        self.assertIn("ALL_POINT_P95_LIMIT_EXCEEDED", result["reasons"])

    def test_small_or_line_roi_rejected(self):
        for yz in (np.zeros((100, 2)), np.column_stack([np.linspace(0, 1, 100), np.zeros(100)])):
            result = pc.fit_plane(np.column_stack([np.ones(100), yz]))
            self.assertEqual(result["status"], "REJECTED")
            self.assertIn("INSUFFICIENT_TWO_DIMENSIONAL_SPAN", result["reasons"])

    def test_inliers_and_all_points_differ(self):
        rng = np.random.default_rng(90)
        xyz = np.column_stack([np.ones(1000), rng.uniform(-0.5, 0.5, (1000, 2))])
        xyz[:100, 0] += 0.2
        result = pc.fit_plane(xyz)
        self.assertEqual(result["status"], "REJECTED")
        self.assertGreater(result["inlier_fraction"], 0.85)
        self.assertLess(result["rmse_m"], 1e-10)
        self.assertGreater(result["all_p95_m"], 0.15)

    def test_input_validation_json_safe(self):
        for xyz in ([[1, 2]], [[1, 2, math.nan]], [[1, 2, 3]] * 20, []):
            result = pc.fit_plane(xyz)
            self.assertEqual(result["status"], "REJECTED")
            json.dumps(result, allow_nan=False)


class PlaneSolveTests(unittest.TestCase):
    def setUp(self):
        self.rotation = rotation_xyz(0.08, -0.04, 0.19)
        self.translation = np.array([0.05, -0.43, 0.03])
        self.train = [observation(i, n, rotation=self.rotation, translation=self.translation)
                      for i, n in enumerate(NORMALS)]

    def test_known_rotation_translation_and_direction(self):
        result = pc.solve_planes(self.train)
        self.assertEqual(result["status"], "CANDIDATE", result)
        transform = np.asarray(result["initial_T_left_right"])
        np.testing.assert_allclose(transform[:3, :3], self.rotation, atol=1e-12)
        np.testing.assert_allclose(transform[:3, 3], self.translation, atol=1e-12)
        self.assertAlmostEqual(result["rotation_determinant"], 1.0)
        self.assertFalse(result["live_eligible"])
        self.assertTrue(result["icp_allowed"])
        self.assertFalse(result["independent_validation_performed"])
        self.assertLess(result["train_summary"]["maximum_abs_offset_m"], 1e-12)
        json.dumps(result, allow_nan=False)

    def test_parallel_and_rank_two_normals_rejected(self):
        for normals in ([[-1, 0, 0]] * 4,
                        [[-1, 0, 0], [-0.8, 0.6, 0], [-0.8, -0.6, 0], [-0.6, 0.8, 0]]):
            obs = [observation(i, n, rotation=self.rotation, translation=self.translation)
                   for i, n in enumerate(normals)]
            result = pc.solve_planes(obs)
            self.assertEqual(result["status"], "REJECTED")
            self.assertIn("TRAIN_NORMALS_RANK_DEFICIENT", result["reasons"])

    def test_rank_three_but_ill_conditioned_rejected(self):
        normals = [[-1, 0, 0], [-1, 0.01, 0], [-1, 0, 0.01], [-1, -0.01, -0.01]]
        obs = [observation(i, n, rotation=self.rotation, translation=self.translation)
               for i, n in enumerate(normals)]
        result = pc.solve_planes(obs)
        self.assertEqual(result["observability"]["rank"], 3)
        self.assertIn("TRAIN_NORMALS_ILL_CONDITIONED", result["reasons"])

    def test_sign_flip_normal_and_d_together_is_handled(self):
        for i, item in enumerate(self.train):
            fit = item["left" if i % 2 else "right"]
            fit["normal"] = (-np.asarray(fit["normal"])).tolist()
            fit["d_m"] *= -1
        result = pc.solve_planes(self.train)
        self.assertEqual(result["status"], "CANDIDATE", result)
        np.testing.assert_allclose(result["translation_left_right_m"], self.translation, atol=1e-12)

    def test_same_face_must_be_confirmed(self):
        self.train[0].pop("same_face_confirmed")
        result = pc.solve_planes(self.train)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("SAME_FACE_CONFIRMATION_REQUIRED" in r for r in result["reasons"]))

    def test_repeated_frames_do_not_count_as_independent_poses(self):
        obs = [observation(i, [-1, 0.1, 0.2], rotation=self.rotation,
                           translation=self.translation, capture_id="one_capture") for i in range(3)]
        result = pc.solve_planes(obs)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIn("TOO_FEW_INDEPENDENT_TRAIN_CAPTURES", result["reasons"])

    def test_train_holdout_capture_leakage_rejected(self):
        hold = observation(12, NORMALS[2], rotation=self.rotation, translation=self.translation,
                           role="holdout", capture_id="capture_0")
        result = pc.solve_planes(self.train + [hold])
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("CAPTURE_ROLE_LEAKAGE" in r for r in result["reasons"]))

    def test_independent_holdout_coverage_required_and_fitting_unchanged(self):
        hold = [observation(10 + i, n, rotation=self.rotation, translation=self.translation,
                            role="holdout") for i, n in enumerate(NORMALS[:3])]
        baseline = pc.solve_planes(self.train)
        two = pc.solve_planes(self.train + hold[:2])
        self.assertFalse(two["independent_validation_performed"])
        result = pc.solve_planes(self.train + hold)
        self.assertEqual(result["status"], "CANDIDATE", result)
        self.assertTrue(result["independent_validation_performed"])
        np.testing.assert_array_equal(result["initial_T_left_right"], baseline["initial_T_left_right"])
        self.assertFalse(result["live_eligible"])

    def test_failed_holdout_preserved_and_rejects_candidate(self):
        hold = [observation(10 + i, n, rotation=self.rotation, translation=self.translation,
                            role="holdout") for i, n in enumerate(NORMALS[:3])]
        hold[1]["right"]["d_m"] += 0.12
        n = np.asarray(hold[1]["right"]["normal"])
        hold[1]["right"]["centroid"] = (-hold[1]["right"]["d_m"] * n).tolist()
        result = pc.solve_planes(self.train + hold)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(result["independent_validation_performed"])
        self.assertFalse(result["holdout_summary"]["passed"])
        self.assertFalse(result["icp_allowed"])
        self.assertEqual(len(result["per_pose_metrics"]), 9)

    def test_invalid_plane_not_silently_dropped(self):
        self.train[0]["left"]["status"] = "REJECTED"
        result = pc.solve_planes(self.train)
        self.assertEqual(result["status"], "REJECTED")
        self.assertIsNone(result["initial_T_left_right"])

    def test_invalid_plane_object_returns_diagnostic_instead_of_crashing(self):
        self.train[0]["left"] = None
        result = pc.solve_planes(self.train)
        self.assertEqual(result["status"], "REJECTED")
        self.assertTrue(any("INVALID_PLANE_OBJECT" in reason for reason in result["reasons"]))

    def test_opposite_face_geometry_cannot_become_same_face_candidate(self):
        # A board between the sensor origins is viewed from opposite faces.
        # Its oriented normals disagree physically; a false checkbox must not
        # hide the residuals by silently flipping each plane independently.
        opposite = [observation(i, n, rotation=np.eye(3), translation=np.array([4.0, 0, 0]))
                    for i, n in enumerate(NORMALS)]
        result = pc.solve_planes(opposite)
        self.assertEqual(result["status"], "REJECTED", result)
        self.assertTrue(any("PLANE_RESIDUAL_LIMIT_EXCEEDED" in reason for reason in result["reasons"]))

    def test_solve_from_actual_noisy_plane_fits(self):
        rng = np.random.default_rng(127)
        observations = []
        for i, n in enumerate(NORMALS):
            normal = np.asarray(n, float); normal /= np.linalg.norm(normal)
            a = np.cross(normal, [0, 0, 1]); a /= np.linalg.norm(a)
            b = np.cross(normal, a)
            uv = rng.uniform(-0.4, 0.4, (1800, 2))
            left = -(1.8 + 0.04 * i) * normal + uv[:, :1] * a + uv[:, 1:] * b
            right = (left - self.translation) @ self.rotation
            left += rng.normal(0, 0.003, (len(left), 1)) * normal
            right += rng.normal(0, 0.003, (len(right), 1)) * (self.rotation.T @ normal)
            fit_l, fit_r = pc.fit_plane(left, seed=i), pc.fit_plane(right, seed=i+9)
            self.assertEqual(fit_l["status"], "PASS", fit_l)
            self.assertEqual(fit_r["status"], "PASS", fit_r)
            observations.append({"id": str(i), "role": "train", "capture_id": f"capture_{i}",
                                 "same_face_confirmed": True, "left": fit_l, "right": fit_r})
        result = pc.solve_planes(observations)
        self.assertEqual(result["status"], "CANDIDATE", result)
        np.testing.assert_allclose(result["rotation_left_right"], self.rotation, atol=0.002)
        np.testing.assert_allclose(result["translation_left_right_m"], self.translation, atol=0.003)


if __name__ == "__main__":
    unittest.main()
