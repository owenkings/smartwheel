"""相机现场标定的合成回归：棋盘格 PDF、内参判定（经真实 CameraArchive 格式）、平面法外参。"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

cv2 = pytest.importorskip('cv2')

from wc_calibration.field import camera, checkerboard  # noqa: E402

K_TRUE = np.array([[520.0, 0, 322.0], [0, 518.0, 238.0], [0, 0, 1]])
DIST_TRUE = np.array([-0.12, 0.05, 0, 0, 0])


def test_checkerboard_pdf_fits_a4(tmp_path):
    spec = checkerboard.board_spec()
    assert (spec['cols'], spec['rows']) == (10, 7) and spec['width_mm'] == 250
    out = tmp_path / 'board.pdf'
    checkerboard.write_pdf(out, spec)
    data = out.read_bytes()
    assert data.startswith(b'%PDF-1.4') and data.rstrip().endswith(b'%%EOF')
    assert data.count(b' re f') == 35 + 11 + 1  # 35 个黑格、11 个刻度、1 条比例尺
    with pytest.raises(ValueError):
        checkerboard.board_spec(square_mm=40)


def render_view(rvec, tvec, size=(640, 480)):
    """把 25 mm 棋盘格（带白边）按真实内参透视投影成一帧灰度图。"""
    s = 40  # 每个方格 40 像素的平面纹理
    cols, rows = 10, 7
    tex = np.full(((rows + 2) * s, (cols + 2) * s), 255, np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                tex[(r + 1) * s:(r + 2) * s, (c + 1) * s:(c + 2) * s] = 0
    sq = camera.SQUARE_M
    plane = np.array([[-2 * sq, -2 * sq, 0], [(cols) * sq, -2 * sq, 0], [(cols) * sq, (rows) * sq, 0],
                      [-2 * sq, (rows) * sq, 0]], np.float32)
    img_pts, _ = cv2.projectPoints(plane, rvec, tvec, K_TRUE, np.zeros(5))
    src = np.float32([[0, 0], [tex.shape[1], 0], [tex.shape[1], tex.shape[0]], [0, tex.shape[0]]])
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    img = cv2.warpPerspective(tex, H, size, borderValue=200)
    return cv2.GaussianBlur(img, (3, 3), 0)


def synthetic_views(n=24, seed=0):
    rng = np.random.default_rng(seed)
    views = []
    for k in range(n):
        tilt = np.radians(rng.uniform(-40, 40, 2))
        R = Rotation.from_euler('xyz', [tilt[0], tilt[1], rng.uniform(-0.2, 0.2)]).as_matrix()
        depth = rng.uniform(0.45, 0.8)
        centre = np.array([rng.uniform(-0.12, 0.12) * depth / 0.6, rng.uniform(-0.08, 0.08) * depth / 0.6, depth])
        board_centre = R @ np.array([0.1125, 0.075, 0])
        views.append((cv2.Rodrigues(R)[0], (centre - board_centre).reshape(3, 1)))
    return views


def test_detect_and_calibrate_from_rendered_views():
    corner_sets = []
    for rvec, tvec in synthetic_views():
        corners = camera.detect(render_view(rvec, tvec))
        if corners is not None:
            corner_sets.append(corners)
    assert len(corner_sets) >= 15
    result = camera.calibrate_intrinsics(corner_sets, (640, 480))
    K = np.asarray(result['K'])
    assert result['rms_px'] < 0.5
    assert abs(K[0, 0] - 520) < 8 and abs(K[1, 2] - 238) < 8


def test_analyze_intrinsics_reads_camera_archive(tmp_path):
    from wc_runtime.source_archive import CameraArchive
    root = tmp_path / 'cap' / 'sources' / 'cameras' / 'left_front' / 'stream'
    root.parent.mkdir(parents=True)
    archive = CameraArchive(root, 'cam', 'epoch')
    for k, (rvec, tvec) in enumerate(synthetic_views(30, seed=1)):
        bgr = cv2.cvtColor(render_view(rvec, tvec), cv2.COLOR_GRAY2BGR)
        archive.append(bgr.tobytes(), sequence=k + 1, host_receive_ns=k + 1, host_monotonic_ns=10**9 + k * 10**8,
                       read_start_monotonic_ns=10**9 + k * 10**8, width=640, height=480, device='test',
                       preview_rotation_deg=0)
    archive.close()
    fails, info = camera.analyze_intrinsics(tmp_path / 'cap', 'left_front', stride=1)
    assert info['detected_frames'] >= 15
    assert info['rms_px'] < 0.6
    assert not [f for f in fails if '重投影' in f or '不合理' in f]


def test_plane_extrinsic_recovers_transform():
    rng = np.random.default_rng(2)
    R = Rotation.from_euler('xyz', [-90 + 1.5, 0.8, -90 - 2.0], degrees=True).as_matrix()
    t = np.array([0.01, 0.06, -0.02])
    cam, lid = [], []
    # 现场要求的 5 个姿态：正对、左右转约 30°、上下倾约 25°，再加一个重复正对
    for tilt in ((0, 0), (0, 30), (0, -30), (25, 0), (-25, 0), (3, 2)):
        n = Rotation.from_euler('xy', tilt, degrees=True).as_matrix() @ [0, 0, -1.0]
        d = rng.uniform(1.0, 2.0)
        nl = R @ n
        dl = d - nl @ t
        cam.append((n, d))
        lid.append((nl + rng.normal(0, 0.002, 3), dl + rng.normal(0, 0.003)))
    lid = [(n / np.linalg.norm(n), d) for n, d in lid]
    prior = t + np.array([0.02, -0.02, 0.01])  # 机械名义值本身有几厘米误差
    Rs, ts, angle, dist, cond, t_std = camera.solve_plane_extrinsic(cam, lid, t_prior=prior)
    assert np.degrees(Rotation.from_matrix(Rs.T @ R).magnitude()) < 0.5
    assert np.linalg.norm(ts - t) < 0.02
    assert max(angle) < 0.5 and cond > 0.3 and max(t_std) < 0.03
