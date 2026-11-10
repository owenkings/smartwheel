"""Live 3D obstacle input, pure parts: the faster witness, pose history, cycle pairing, pose gate and processing."""
import math

import numpy as np
import pytest

from wc_navigation.perception import live_obstacles as lo
from wc_navigation.perception.consensus import BoxWitness, RecentFrames, Witness, move, planar_relative
from wc_navigation.perception.nav2_obstacles import CLOUD_FRAMES
from wc_navigation.perception.obstacle_cloud import ObstacleFilter, body_outline, filter_frame

LEFT = np.array([0.323, 0.22875, 0.5915])
RIGHT = np.array([0.41173, -0.23331, 0.55265])
MEASURED = [(0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425)]
RUNTIME = {"mounts": {"left": {"t_axle_lidar_m": LEFT.tolist(),
                               "R_axle_lidar": [[0.9999703, -0.006982, 0.0032561], [0.0069805, 0.9999755, 0.0004714],
                                                [-0.0032593, -0.0004487, 0.9999946]]},
                      "right": {"t_axle_lidar_m": RIGHT.tolist(),
                                "R_axle_lidar": [[0.9969843, 0.0727125, 0.027115], [-0.0725535, 0.9973413, -0.0068022],
                                                 [-0.0275375, 0.0048144, 0.9996091]]}},
           "sensor_ids": {"left": "XT-LEFT", "right": "XT-RIGHT"}}


def block(x0, x1, y0, y1, h0, h1, lever, step=0.03):
    """Box surface samples given in base_link heights above the floor, returned in that lidar's frame."""
    xs, ys, hs = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step), np.arange(h0, h1 + 1e-9, step)
    grid = np.array(np.meshgrid(xs, ys, hs, indexing="ij")).reshape(3, -1).T
    return np.column_stack([grid[:, 0], grid[:, 1], grid[:, 2] - 0.185]) - lever


def scene(lever, rng, ghost=True):
    """Floor, a wall, a box and (optionally) a floating cluster straight ahead, seen from one lidar."""
    parts = [block(1.6, 3.4, -1.0, 1.2, 0.0, 0.0, lever, step=0.06), block(3.5, 3.52, -1.2, 1.2, 0.0, 1.5, lever, 0.06),
             block(1.8, 1.95, -0.6, -0.45, 0.0, 0.5, lever)]
    if ghost:
        parts.append(block(2.2, 2.32, 0.15, 0.30, 0.85, 0.94, lever))
    cloud = np.vstack(parts)
    return (cloud + rng.normal(0.0, 0.003, cloud.shape)).astype(np.float32)


# -- faster witness ----------------------------------------------------------------------------------
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_box_witness_gives_the_same_shares_as_the_witness(seed):
    rng = np.random.default_rng(seed)
    other = RIGHT
    returns = np.vstack([scene(other, rng, ghost=bool(seed % 2)) + other,
                         other + rng.uniform(-4, 4, (300, 3)) * [1, 1, 0.3]])          # stray returns all around
    axis = np.array([[other[0] + d, other[1], other[2]] for d in np.arange(0.5, 4.0, 0.2)])  # rays along the x axis
    returns = np.vstack([returns, axis])
    slow, fast = Witness(returns, other), BoxWitness(returns, other)
    clusters = [rng.uniform([0.5, -1.5, 0.0], [4.0, 1.5, 1.6], (int(rng.integers(1, 40)), 3)) for _ in range(60)]
    clusters += [axis[3:6] + [0.0, 0.0, 0.01], axis[-2:], np.zeros((0, 3)), returns[:5]]
    for points in clusters:
        assert fast.shares(points) == slow.shares(points)
    assert BoxWitness(np.zeros((0, 3)), other).shares(clusters[0]) == (0.0, 0.0)


def test_filter_frame_is_the_same_with_either_witness():
    rng = np.random.default_rng(7)
    cloud = scene(LEFT, rng)
    other = scene(RIGHT, rng, ghost=False).astype(float) + RIGHT
    cfg = ObstacleFilter()
    body = body_outline(MEASURED)
    a = filter_frame(cloud, LEFT, cfg, body, Witness(other, RIGHT), clearing=True, scan=True)
    b = filter_frame(cloud, LEFT, cfg, body, BoxWitness(other, RIGHT), clearing=True, scan=True)
    assert a["stats"] == b["stats"] and a["stats"]["removed_contradicted"] > 0
    for key in ("points", "rays", "scan"):
        np.testing.assert_array_equal(a[key], b[key])


# -- configuration, plan and geometry -------------------------------------------------------------------
def test_config_limits():
    assert lo.LiveObstacleConfig().rate_hz == 5.0
    for bad in ({"rate_hz": 0}, {"rate_hz": 20}, {"pair_window_s": 0.2}, {"pose_wait_s": 2.0},
                {"odom_history_s": 0.5}, {"rate_hz": True}):
        with pytest.raises(ValueError):
            lo.LiveObstacleConfig(**bad)


def test_output_plan_follows_the_obstacle_options_and_stays_off_by_default():
    combo = lo.output_plan("scan", ("left", "right"), "filtered", "cloud")
    assert combo["published"] == ["left", "right"] and combo["read"] == ["left", "right"]
    assert combo["filtered_scan"] and not combo["clearing"] and combo["clearing_topics"] == {}
    assert combo["source_topics"]["right"] == "/wc_mapping/lidar_right/source_frame_filtered"
    cloud = lo.output_plan("cloud", ("left", "right"))
    assert cloud["clearing"] and set(cloud["clearing_topics"]) == {"left", "right"} and not cloud["filtered_scan"]
    scan2d = lo.output_plan("scan", ("left",), "filtered")
    assert scan2d["published"] == ["left"] and scan2d["read"] == ["left", "right"]      # right: witness only
    assert lo.output_plan("scan", ("left",), "filtered", witness=False)["read"] == ["left"]
    with pytest.raises(ValueError):
        lo.output_plan()                                                            # the default: off


def test_obstacle_frames_are_not_the_lidars_own_frames():
    assert CLOUD_FRAMES == {"left": "obstacles_lidar_left", "right": "obstacles_lidar_right"}


def test_each_lidar_uses_its_own_mount_and_sensor_id():
    left, right = lo.side_geometry(RUNTIME, "left"), lo.side_geometry(RUNTIME, "right")
    assert right["lever"] == RIGHT.tolist() and right["sensor_id"] == "XT-RIGHT"
    assert right["source_frame_id"] == "lidar_right"
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = RUNTIME["mounts"]["right"]["R_axle_lidar"], RIGHT
    np.testing.assert_allclose(right["T_base_axle"], np.linalg.inv(T))
    np.testing.assert_allclose(right["R_reference_base"], np.linalg.inv(T)[:3, :3].T)
    live = {"T_base_axle": left["T_base_axle"], "R_reference_base": left["R_reference_base"],
            "lever_xy": LEFT[:2].tolist(), "identities": {"lidar_left": "XT-LEFT"}}
    lo.check_left_geometry(left, live)
    with pytest.raises(ValueError):
        lo.check_left_geometry(right, live)
    with pytest.raises(ValueError):
        lo.check_left_geometry(left, dict(live, identities={"lidar_left": "XT-OTHER"}))
    broken = {"mounts": {"right": {"t_axle_lidar_m": [0, 0, 0], "R_axle_lidar": np.diag([1, 1, 2]).tolist()}},
              "sensor_ids": {"right": "XT-RIGHT"}}
    with pytest.raises(ValueError):
        lo.side_geometry(broken, "right")
    with pytest.raises(ValueError):
        lo.side_geometry({"mounts": {}}, "left")


def test_messages_are_stamped_like_the_scan():
    from wc_navigation.live.scan_adapter import scan_stamp_ns
    stamp = 1_794_228_777_100_408_000
    assert lo.published_stamp_ns(stamp, 75.0) == scan_stamp_ns(stamp, 75.0) == stamp - 75_000_000
    with pytest.raises(ValueError):
        lo.offset_ns(300.0)


# -- pose history ---------------------------------------------------------------------------------------
def test_pose_history_interpolates_unwrapped_and_does_not_extrapolate():
    h = lo.PoseHistory(history_s=1.0)
    for k, yaw in enumerate((3.10, -3.10, -3.00)):                       # across +-pi
        h.add(1_000_000_000 + 20_000_000 * k, 0.1 * k, 0.0, yaw)
    x, y, yaw = h.at(1_010_000_000)
    assert x == pytest.approx(0.05) and abs(abs(yaw) - math.pi) < 1e-6   # half way from 3.10 to -3.10 (+0.083 rad)
    assert h.at(1_050_000_000) is None and not h.covers(1_050_000_000) and h.covers(1_040_000_000)
    assert h.at(999_000_000) is None
    h.add(1_030_000_000, 9, 9, 0)                                        # older than the newest: ignored
    assert h.counts["out_of_order"] == 1
    h.add(3_000_000_000, 1.0, 0.0, 0.0)                                  # 1 s window: the old samples go
    assert h.at(1_000_000_000) is None and h.newest_ns == 3_000_000_000
    h.add(500_000_000, 0.0, 0.0, 0.0)                                    # clock went back: start over
    assert h.counts["resets"] == 1 and h.newest_ns == 500_000_000
    h.add(600_000_000, float("nan"), 0, 0)
    assert h.counts["rejected"] == 1


# -- cycle selection ----------------------------------------------------------------------------------------
def frames_10hz(seconds, right_offsets_ms, start_ns=10 ** 12):
    """Arrival order of both lidars' frames at 10 Hz; the right frame comes before or after the left one."""
    events = []
    for k in range(int(seconds * 10)):
        left = start_ns + k * 100_000_000
        right = left + int(right_offsets_ms[k % len(right_offsets_ms)] * 1e6)
        events += sorted([(left, "left"), (right, "right")])
    return events


def test_cycles_pair_both_lidars_in_either_order_and_keep_five_per_second():
    selector = lo.CycleSelector(("left", "right"))
    kept = []
    for stamp, side in frames_10hz(4.0, (5, -6)):
        kept += selector.offer(side, stamp, stamp + 2_000_000, payload=(side, stamp))
    kept += selector.poll(10 ** 13)
    assert len(kept) == 20 and all(c["paired"] for c in kept)
    gaps = np.diff([c["frames"]["left"]["stamp_ns"] for c in kept])
    assert set(gaps) == {200_000_000}
    for c in kept:
        assert abs(c["frames"]["left"]["stamp_ns"] - c["frames"]["right"]["stamp_ns"]) <= 6_000_000
        assert c["frames"]["left"]["payload"] == ("left", c["frames"]["left"]["stamp_ns"])
    assert selector.counts["left"]["skipped_by_rate"] == 20 and selector.counts["right"]["unpaired"] == 0
    full = lo.CycleSelector(("left", "right"), lo.LiveObstacleConfig(rate_hz=10.0))
    every = [c for s, side in frames_10hz(1.0, (5,)) for c in full.offer(side, s, s)]
    assert len(every) == 10


def test_a_frame_without_partner_goes_alone_after_the_wait_and_one_lidar_needs_no_wait():
    selector = lo.CycleSelector(("left", "right"))
    out = selector.offer("left", 10 ** 12, 10 ** 12)
    assert out == [] and selector.poll(10 ** 12 + 40_000_000) == []
    alone = selector.poll(10 ** 12 + 60_000_000)
    assert len(alone) == 1 and not alone[0]["paired"] and list(alone[0]["frames"]) == ["left"]
    assert selector.counts["left"]["unpaired"] == 1
    # frames of different cycles never pair: the older one goes alone first
    out = selector.offer("right", 10 ** 12 + 100_000_000, 10 ** 12 + 100_000_000)
    out += selector.offer("left", 10 ** 12 + 300_000_000, 10 ** 12 + 300_000_000)
    assert [list(c["frames"]) for c in out] == [["right"]]
    single = lo.CycleSelector(("left",))
    assert len(single.offer("left", 10 ** 12, 10 ** 12)) == 1
    assert single.offer("left", 10 ** 12, 10 ** 12) == [] and single.counts["left"]["out_of_order"] == 1
    with pytest.raises(ValueError):
        single.offer("right", 10 ** 12, 10 ** 12)


# -- pose gate --------------------------------------------------------------------------------------------
def cycle(stamps):
    return {"cycle": 1, "paired": len(stamps) == 2,
            "frames": {side: {"stamp_ns": s, "arrival_ns": s, "payload": None} for side, s in stamps.items()}}


def test_pose_gate_waits_for_the_odometry_and_releases_without_a_pose_after_the_wait():
    gate = lo.PoseGate(75.0)
    history = lo.PoseHistory()
    t = 2_000_000_000
    gate.add(cycle({"left": t, "right": t + 5_000_000}), now_ns=t)
    history.add(t - 100_000_000, 0.0, 0.0, 0.0)
    history.add(t - 75_000_000 + 1_000_000, 0.01, 0.0, 0.0)               # covers the left frame only
    assert gate.release(history, t + 10_000_000) == []
    history.add(t, 0.1, 0.0, 0.0)
    released = gate.release(history, t + 20_000_000)
    assert len(released) == 1 and not released[0]["pose_timeout"]
    assert released[0]["measured_ns"] == {"left": t - 75_000_000, "right": t - 70_000_000}
    assert released[0]["poses"]["right"][0] == pytest.approx(0.01 + 0.09 * 4 / 74)   # between t-74 ms and t
    gate.add(cycle({"left": t + 200_000_000}), now_ns=t + 200_000_000)
    assert gate.release(history, t + 400_000_000) == []
    late = gate.release(history, t + 500_000_000)
    assert late[0]["pose_timeout"] and late[0]["poses"] == {"left": None}
    assert gate.counts["pose_timeouts"] == 1 and gate.counts["frames_without_pose"] == 1


# -- processing ---------------------------------------------------------------------------------------------
def released_cycle(t_ns, poses, frames=("left", "right"), offset_ms=75.0):
    stamps = {s: t_ns + (5_000_000 if s == "right" else 0) for s in frames}
    return {"cycle": {"cycle": 1, "paired": len(frames) == 2,
                      "frames": {s: {"stamp_ns": v, "arrival_ns": v, "payload": None} for s, v in stamps.items()}},
            "poses": poses, "measured_ns": {s: v - lo.offset_ns(offset_ms) for s, v in stamps.items()},
            "pose_timeout": any(p is None for p in poses.values()), "waited_ns": 0}


def geometry():
    return {"left": lo.side_geometry(RUNTIME, "left"), "right": lo.side_geometry(RUNTIME, "right")}


def test_a_cycle_is_filtered_as_in_the_shadow_runs():
    rng = np.random.default_rng(3)
    clouds = {"left": scene(LEFT, rng), "right": scene(RIGHT, rng, ghost=False)}
    g = geometry()
    plan = lo.output_plan("scan", ("left", "right"), "filtered", "cloud")
    cfg = ObstacleFilter(persist_min_age_s=0.6)
    body = body_outline(MEASURED)
    histories = lo.persistence_histories(plan["published"], cfg)
    reference = {s: RecentFrames(cfg.persist_min_age_s, cfg.persist_max_age_s, cfg.cluster_voxel_m) for s in ("left", "right")}
    t0 = 5_000_000_000
    for k in range(6):                                    # 1.2 s of a chair driving 0.3 m/s along x at 5 Hz
        t = t0 + k * 200_000_000
        poses = {"left": (0.06 * k, 0.0, 0.0), "right": (0.06 * k + 0.0015, 0.0, 0.0)}
        released = released_cycle(t, poses)
        out = lo.process_cycle(released, clouds, g, plan, cfg, body, histories)
        for side, other in (("left", "right"), ("right", "left")):
            rel = planar_relative(poses[other], poses[side])
            ol = np.asarray(g[other]["lever"])
            witness = Witness(move(clouds[other].astype(float) + ol, rel), move(ol[None, :], rel)[0])
            t_s = released["measured_ns"][side] * 1e-9
            recent = reference[side].recent(t_s, poses[side])
            expected = filter_frame(clouds[side], np.asarray(g[side]["lever"]), cfg, body, witness, clearing=False,
                                    scan=side == "left", recent=recent)
            reference[side].add(t_s, poses[side], expected["candidates"])
            np.testing.assert_array_equal(out[side]["points"], expected["points"])
            assert out[side]["stats"] == expected["stats"] and out[side]["witness"]
            assert out[side]["recent"] == (k >= 3) and out[side]["rays"] is None
            assert out[side]["partner_stamp_ns"] == released["cycle"]["frames"][other]["stamp_ns"]
            if side == "left":
                np.testing.assert_array_equal(out[side]["scan"], expected["scan"])
            else:
                assert out[side]["scan"] is None


def test_a_frame_without_pose_keeps_its_points_and_skips_witness_and_persistence():
    rng = np.random.default_rng(4)
    clouds = {"left": scene(LEFT, rng), "right": scene(RIGHT, rng, ghost=False)}
    g = geometry()
    plan = lo.output_plan("cloud", ("left", "right"))
    cfg = ObstacleFilter(persist_min_age_s=0.6)
    histories = lo.persistence_histories(plan["published"], cfg)
    out = lo.process_cycle(released_cycle(10 ** 10, {"left": None, "right": (0.0, 0.0, 0.0)}), clouds, g, plan, cfg,
                           (), histories)
    alone = filter_frame(clouds["left"], np.asarray(g["left"]["lever"]), cfg, (), None, clearing=True)
    np.testing.assert_array_equal(out["left"]["points"], alone["points"])
    assert not out["left"]["witness"] and not out["left"]["recent"] and out["left"]["rays"] is not None
    assert histories["left"].entries == [] and len(histories["right"].entries) == 1
    assert not out["right"]["witness"]                     # its partner has no pose either
    missing = lo.process_cycle(released_cycle(10 ** 10, {"left": (0, 0, 0)}, frames=("left",)), {}, g, plan, cfg, ())
    assert missing == {}                                   # no cloud (rejected frame): nothing published


def test_records_carry_hashes_poses_and_stats():
    rng = np.random.default_rng(5)
    clouds = {"left": scene(LEFT, rng), "right": scene(RIGHT, rng, ghost=False)}
    g = geometry()
    plan = lo.output_plan("scan", ("left",), "filtered")
    released = released_cycle(10 ** 10, {"left": (0.1, 0.2, 0.3), "right": (0.1, 0.2, 0.3)})
    products = lo.process_cycle(released, clouds, g, plan, ObstacleFilter(), ())
    lines = lo.frame_records(released, clouds, products, 75.0, timing_ms={"left": {"filter": 12.345}},
                             rejected={"right": None})
    left, right = (next(x for x in lines if x["side"] == s) for s in ("left", "right"))
    assert left["published"] and not right["published"] and right["input_sha256"] == lo.array_sha256(clouds["right"])
    assert left["output_sha256"] == lo.array_sha256(products["left"]["points"]) and left["scan_sha256"]
    assert left["pose"] == [0.1, 0.2, 0.3] and left["published_stamp_ns"] == 10 ** 10 - 75_000_000
    assert left["timing_ms"] == {"filter": 12.35} and "rays" not in left
    assert all(isinstance(v, int) for v in left["stats"].values())


def test_status_reports_stale_inputs_and_failures():
    status = lo.LiveStatus(("left", "right"), lo.LiveObstacleConfig(), 75.0, {"source": "CONFIGURATION"})
    assert status.status(10.0)["state"] == "STALE"
    for name in ("left", "right", "odom"):
        status.mark(name, 10.0)
    report = status.status(10.5)
    assert report["state"] == "RUNNING" and report["role"] == "obstacles" and not report["commands_subscribed"]
    assert status.status(11.2)["stale_inputs"] == ["left", "right", "odom"]
    status.timing["filter_ms"].extend([10.0, 20.0, 30.0])
    status.reject("right", "host filter left no points")
    status.fail("processing", RuntimeError("boom"))
    report = status.status(10.5)
    assert report["state"] == "FAILED" and report["timing_ms"]["filter_ms"]["median"] == 20.0
    assert report["rejections"] == {"right: host filter left no points": 1}


# -- offline check (live_check) ------------------------------------------------------------------------------
def entries(plan):
    g = geometry()
    return {"plan": plan, "config": lo.LiveObstacleConfig().as_dict(),
            "filter": ObstacleFilter(persist_min_age_s=0.6).as_dict(), "body": [list(p) for p in MEASURED],
            "geometry": {s: g[s] for s in plan["read"]}, "lidar_time_offset_ms": 75.0}


def simulated_node(plan, seconds=2.0, timeout_at=None, reject=None):
    """Run the pure pipeline like the node does and keep its log lines and the clouds it saw."""
    rng = np.random.default_rng(11)
    e = entries(plan)
    cfg = ObstacleFilter(**e["filter"])
    body = [tuple(p) for p in e["body"]]
    selector, gate, history = lo.CycleSelector(plan["read"]), lo.PoseGate(75.0), lo.PoseHistory()
    histories = lo.persistence_histories(plan["published"], cfg)
    seen, lines = {}, []
    t0 = 10 ** 12
    for k in range(int(seconds * 10)):
        for side, delay in (("left", 0), ("right", 5_000_000)):
            stamp = t0 + k * 100_000_000 + delay
            for c in selector.offer(side, stamp, stamp + 1_000_000):
                c["clouds"], c["rejected"] = {}, {}
                for s, f in c["frames"].items():
                    if reject and (s, f["stamp_ns"]) == reject:
                        c["rejected"][s] = "host filter left no points"
                        continue
                    cloud = scene(LEFT if s == "left" else RIGHT, rng, ghost=s == "left")
                    seen[(s, f["stamp_ns"])] = cloud
                    c["clouds"][s] = cloud
                gate.add(c, stamp + 2_000_000)
        now = t0 + k * 100_000_000 + 30_000_000
        if timeout_at is None or not timeout_at[0] <= k < timeout_at[1]:
            history.add(now, 0.03 * k, 0.0, 0.0)
        for released in gate.release(history, now if timeout_at is None or not timeout_at[0] <= k < timeout_at[1]
                                     else now + 400_000_000):
            products = lo.process_cycle(released, released["cycle"]["clouds"], e["geometry"], plan, cfg, body,
                                        histories)
            lines += lo.frame_records(released, released["cycle"]["clouds"], products, 75.0,
                                      rejected=released["cycle"]["rejected"])
    return e, lines, seen


def test_the_recomputation_repeats_the_node_exactly():
    from wc_navigation.perception import live_check
    plan = lo.output_plan("scan", ("left", "right"), "filtered", "cloud")
    e, lines, seen = simulated_node(plan, timeout_at=(6, 8), reject=("right", 10 ** 12 + 1_200_000_000 + 5_000_000))
    assert any(x["pose"] is None for x in lines) and any(x["rejected"] for x in lines)
    result = live_check.recompute(lines, lambda side, stamp: seen.get((side, stamp)), e)
    assert result["status"] == "PASS" and result["frames"] == sum(x["published"] for x in lines)
    tampered = [dict(x) for x in lines]
    tampered[3]["output_sha256"] = "0" * 64
    assert live_check.recompute(tampered, lambda side, stamp: seen.get((side, stamp)), e)["status"] == "FAIL"
    missing = live_check.recompute(lines, lambda side, stamp: None, e)
    assert missing["status"] == "FAIL" and missing["missing_inputs"]


def test_inputs_are_compared_with_the_frozen_cells_where_they_have_the_frame():
    from wc_navigation.perception import live_check
    plan = lo.output_plan("scan", ("left", "right"), "filtered", "cloud")
    _, lines, _ = simulated_node(plan, seconds=1.0)
    left = [x for x in lines if x["side"] == "left"]
    frozen = {"left": {left[0]["stamp_ns"]: left[0]["input_sha256"], left[1]["stamp_ns"]: "f" * 64}}
    result = live_check.compare_inputs(lines, frozen)
    assert result["left"]["checked"] == 2 and result["left"]["identical"] == 1 and result["left"]["status"] == "FAIL"
    assert result["right"]["status"] == "NOT_CHECKED"
    frozen["left"] = {left[0]["stamp_ns"]: left[0]["input_sha256"]}
    assert live_check.compare_inputs(lines, frozen)["left"]["status"] == "PASS"


def test_summary_and_verdict():
    from wc_navigation.perception import live_check
    plan = lo.output_plan("scan", ("left", "right"), "filtered", "cloud")
    e, lines, _ = simulated_node(plan, seconds=6.0)
    summary = live_check.summarize(lines, e)
    left = summary["sides"]["left"]
    assert left["rate_hz"] == pytest.approx(5.0, abs=0.01) and left["paired_share"] == 1.0
    # the first cycle comes before the first odometry sample: no pose, so no witness
    assert left["witness_share"] == pytest.approx(29 / 30, abs=0.001) and left["without_pose"] == 1
    assert left["per_frame"]["voxels"] > 0
    assert live_check.evaluate(summary)["status"] == "PASS"
    assert "the node failed" in live_check.evaluate(summary, states=["RUNNING", "FAILED"])["reasons"][0]
    slow = live_check.summarize(lines[::4], e)
    assert any("below 0.8" in r for r in live_check.evaluate(slow)["reasons"])
    bad_inputs = {"left": {"status": "NOT_CHECKED"}}
    assert live_check.evaluate(summary, inputs=bad_inputs)["status"] == "FAIL"


# -- command line (window D's live/cli.py with C's options) ------------------------------------------------------
def live_cell(tmp_path):
    cell = tmp_path / "cell"
    cell.mkdir()
    (cell / "runtime_config.json").write_text(__import__("json").dumps(RUNTIME))
    return cell


def test_the_live_obstacle_input_is_off_by_default_and_the_options_need_it(tmp_path):
    from wc_navigation.live.cli import arguments
    from wc_navigation.perception.shadow_cli_options import live_entries, live_profile_fields
    common = ["inputs", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
              "--bag", "x"]
    args = arguments(common)
    assert args.obstacle_live is None and live_entries(args, "c", {}, 75.0, {}) is None
    assert live_profile_fields(args) == {}
    for extra in (["--obstacle-sides", "left"], ["--obstacle-rate-hz", "10"], ["--compare-right-cell", "r"],
                  ["--obstacle-filter", '{"persist_min_age_s": 0.6}']):
        with pytest.raises(SystemExit):
            arguments(common + extra)
    shadow = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
              "--map", "m.yaml", "--initial-pose", "0", "0", "0", "--goal", "1", "2", "0"]
    with pytest.raises(SystemExit):                    # not on the vehicle before the relay forwards the right lidar
        arguments(shadow + ["--source", "vehicle", "--sensor-domain", "83", "--obstacle-monitor", "cloud"])
    args = arguments(shadow + ["--source", "bag", "--bag", "x", "--monitor-base-shift", "off"])
    assert args.obstacle_live is None and live_profile_fields(args) == {"monitor_base_shift": False}


def test_the_combination_gets_its_plan_profile_and_run_config_entry(tmp_path):
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.live.cli import arguments
    from wc_navigation.perception.shadow_cli_options import live_entries, live_profile_fields
    cell = live_cell(tmp_path)
    left = lo.side_geometry(RUNTIME, "left")
    live = {"T_base_axle": left["T_base_axle"], "R_reference_base": left["R_reference_base"],
            "lever_xy": LEFT[:2].tolist(), "identities": {"lidar_left": "XT-LEFT"}}
    shadow = ["shadow", "--project-root", str(tmp_path), "--cell", str(cell), "--hardware-setup", "h",
              "--gyro-bias", "b", "--map", "m.yaml", "--initial-pose", "0", "0", "0", "--goal", "1", "2", "0",
              "--source", "bag", "--bag", "x", "--obstacle-input", "scan", "--obstacle-scan", "filtered",
              "--obstacle-monitor", "cloud", "--obstacle-filter", '{"persist_min_age_s": 0.6}',
              "--footprint", *map(str, [v for p in MEASURED for v in p]), "--footprint-source", "MEASURED_20261107"]
    args = arguments(shadow)
    fields = live_profile_fields(args)
    profile = nav2_config.NavProfile(footprint=args.footprint, **fields)
    assert profile.obstacle_clouds == ("left", "right") and profile.monitor_input == "cloud"
    assert profile.obstacle_scan == "filtered" and profile.obstacle_input == "scan"
    source = {"source": "CONFIGURATION", "sha256": "x" * 64}
    entry = live_entries(args, cell, live, 75.0, source)
    assert entry["lidar_time_offset_ms"] == 75.0 and entry["lidar_time_offset_source"] is source
    assert entry["geometry"]["right"]["sensor_id"] == "XT-RIGHT" and entry["geometry"]["right"]["lever"] == RIGHT.tolist()
    assert entry["filter"]["persist_min_age_s"] == 0.6 and entry["body_source"] == "POLYGON_MEASURED_20261107"
    assert entry["plan"]["filtered_scan"] and entry["config"]["rate_hz"] == 5.0
    with pytest.raises(ValueError):                    # a left mount other than the live chain's is refused
        live_entries(args, cell, dict(live, lever_xy=[0.0, 0.0]), 75.0, source)
    inputs = ["inputs", "--project-root", str(tmp_path), "--cell", str(cell), "--hardware-setup", "h",
              "--gyro-bias", "b", "--bag", "x", "--obstacle-scan", "filtered"]
    args = arguments(inputs)
    assert args.obstacle_live["plan"]["published"] == ["left"] and args.obstacle_live["plan"]["read"] == ["left", "right"]
    assert live_entries(args, cell, live, 0.0, {"source": "COMMAND_LINE"})["body_source"] == "CIRCLE_ASSUMED_NOT_MEASURED"


def test_the_monitor_base_shift_option_reaches_the_collision_monitor_and_its_readback():
    from wc_navigation.autonomy import nav2_config
    default = nav2_config.NavProfile()
    off = nav2_config.NavProfile(monitor_base_shift=False)
    assert "monitor_base_shift" not in default.as_dict() and off.as_dict()["monitor_base_shift"] is False
    params = nav2_config.navigation_parameters(off, mode="navigate", bt_to_pose_path="a", bt_through_poses_path="b")
    assert params["collision_monitor"]["ros__parameters"]["base_shift_correction"] is False
    base = nav2_config.navigation_parameters(default, mode="navigate", bt_to_pose_path="a", bt_through_poses_path="b")
    assert base["collision_monitor"]["ros__parameters"]["base_shift_correction"] is True
    assert nav2_config.readback_expectations(off, "navigate")["collision_monitor"]["base_shift_correction"] is False
    assert "base_shift_correction" not in nav2_config.readback_expectations(default, "navigate")["collision_monitor"]
    with pytest.raises(ValueError):
        nav2_config.NavProfile(monitor_base_shift=0)
