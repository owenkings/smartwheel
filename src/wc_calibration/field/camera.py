"""前方相机现场标定检查：内参（手持棋盘格）和相机到雷达外参（棋盘格贴在木板上）。

棋盘格默认 9×6 内角点、方格 25 mm（checkerboard.py 生成的 A4）。
相机光学系：x 右、y 下、z 前；雷达 FLU。外参约定 p_lidar = R @ p_camera + t。
判据是现场经验门限，不是精度证书。
"""
import json
import warnings
from pathlib import Path

import numpy as np

PATTERN = (9, 6)
SQUARE_M = 0.025
CAMERA_LIDAR = {'left_front': 'left', 'right_front': 'right'}
# 光学系 -> FLU 的轴变换：x_flu = z_opt, y_flu = -x_opt, z_flu = -y_opt
R_FLU_OPTICAL = np.array([[0, 0, 1.0], [-1, 0, 0], [0, -1, 0]])
# CAD 外壳面中心：前相机在同侧雷达外侧 52.75 mm、同高同前（只作选点初值）
NOMINAL_T_LIDAR_CAMERA_M = {'left_front': [0.0, 0.05275, 0.0], 'right_front': [0.0, -0.05275, 0.0]}


def _cv2():
    import cv2
    return cv2


def board_object_points(pattern=PATTERN, square=SQUARE_M):
    cols, rows = pattern
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return grid


def camera_index(capture_dir, role):
    """返回 (archive 目录, 帧记录列表)；帧按 host_monotonic_ns 排序。"""
    root = Path(capture_dir) / 'sources' / 'cameras' / role
    indexes = sorted(root.rglob('frames.jsonl'))
    if len(indexes) != 1:
        raise ValueError('%s 相机应有且只有一个帧索引，找到 %d 个：%s' % (role, len(indexes), root))
    rows = [json.loads(line) for line in indexes[0].open(encoding='utf-8') if line.strip()]
    rows.sort(key=lambda r: r['host_monotonic_ns'])
    return indexes[0].parent, rows


def read_gray(directory, row):
    from wc_runtime.source_archive import read_camera_frame
    data = read_camera_frame(directory, row)
    image = np.frombuffer(data, np.uint8).reshape(row['height'], row['width'], 3)
    return _cv2().cvtColor(image, _cv2().COLOR_BGR2GRAY)


def detect(gray, pattern=PATTERN, allow_small=False):
    """近处（方格 ≥15 像素）用快速检测加亚像素细化。

    allow_small=True 时，快速检测失败再用 findChessboardCornersSB：1–2 m 外的 A4 棋盘格每格只有约 6 像素，
    FAST_CHECK 会全部漏检；SB 自带亚像素精度，不再用 5×5 窗口细化（窗口会跨到相邻方格）。
    """
    cv2 = _cv2()
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if ok:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria).reshape(-1, 2)
        square = np.median(np.linalg.norm(np.diff(refined.reshape(pattern[1], pattern[0], 2), axis=1), axis=2))
        if square >= 15 or not allow_small:
            return refined
    if not allow_small:
        return None
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return corners.reshape(-1, 2) if ok else None


def _sharpness(gray, corners):
    x0, y0 = np.floor(corners.min(0)).astype(int)
    x1, y1 = np.ceil(corners.max(0)).astype(int)
    patch = gray[max(y0, 0):y1, max(x0, 0):x1]
    return float(_cv2().Laplacian(patch, _cv2().CV_64F).var()) if patch.size else 0.0


# ------------------------------------------------------------------ 内参
def select_views(detections, size, max_views=40):
    """去掉相近视角：按角点中心、尺度、透视形变分桶，每桶保留最清晰的一帧。"""
    w, h = size
    buckets = {}
    for k, (corners, sharp) in enumerate(detections):
        c = corners.mean(0)
        span = np.ptp(corners, axis=0)
        cols = corners.reshape(PATTERN[1], PATTERN[0], 2)
        skew = (np.linalg.norm(cols[0, -1] - cols[0, 0]) / max(np.linalg.norm(cols[-1, -1] - cols[-1, 0]), 1e-6),
                np.linalg.norm(cols[-1, 0] - cols[0, 0]) / max(np.linalg.norm(cols[-1, -1] - cols[0, -1]), 1e-6))
        key = (int(4 * c[0] / w), int(4 * c[1] / h), int(4 * span[0] / w),
               int(np.clip((skew[0] - 1) * 8, -3, 3)), int(np.clip((skew[1] - 1) * 8, -3, 3)))
        if key not in buckets or sharp > buckets[key][1]:
            buckets[key] = (k, sharp)
    chosen = sorted(buckets.values(), key=lambda v: -v[1])[:max_views]
    return sorted(k for k, _ in chosen)


def coverage(corner_sets, size, grid=3):
    w, h = size
    cells = set()
    for corners in corner_sets:
        for x, y in corners:
            cells.add((min(int(grid * x / w), grid - 1), min(int(grid * y / h), grid - 1)))
    return len(cells) / grid ** 2


def calibrate_intrinsics(corner_sets, size, pattern=PATTERN, square=SQUARE_M):
    cv2 = _cv2()
    obj = board_object_points(pattern, square)
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera([obj] * len(corner_sets),
                                                     [c.astype(np.float32).reshape(-1, 1, 2) for c in corner_sets],
                                                     size, None, None)
    per_view = []
    tilts = []
    for corners, r, t in zip(corner_sets, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, r, t, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners) ** 2, 1)))))
        R, _ = cv2.Rodrigues(r)
        tilts.append(float(np.degrees(np.arccos(abs(R[2, 2])))))
    return dict(rms_px=float(rms), K=K.tolist(), dist=dist.ravel().tolist(), per_view_rms_px=per_view,
                tilt_deg=tilts)


def _detections(capture_dir, role, stride):
    directory, rows = camera_index(capture_dir, role)
    found = []
    for row in rows[::stride]:
        gray = read_gray(directory, row)
        corners = detect(gray)
        if corners is not None:
            found.append((corners, _sharpness(gray, corners)))
    return rows, found


def analyze_intrinsics(capture_dir, role, stride=3, earlier_captures=()):
    """手持棋盘格在相机前换角度和位置，判断能否得到可信内参。

    earlier_captures：同一相机此前的内参录制，检测结果与本次合并（同一相机、同一分辨率），
    这样可以分几次把画面各区域补齐，不必一次录全。
    """
    fails, info = [], dict(role=role)
    try:
        rows, detections = _detections(capture_dir, role, stride)
    except (OSError, ValueError) as error:
        return ['读不到 %s 相机图像：%s' % (role, error)], info
    size = (rows[0]['width'], rows[0]['height'])
    info.update(frames=len(rows), resolution=list(size), detected_this_capture=len(detections))
    if size[0] < 640:
        fails.append('分辨率只有 %d×%d，标定需要 640×480：录制时选 detail_640' % size)
    pooled = []
    for earlier in earlier_captures:
        try:
            earlier_rows, found = _detections(earlier, role, stride)
        except (OSError, ValueError):
            continue
        if found and (earlier_rows[0]['width'], earlier_rows[0]['height']) == size:
            detections += found
            pooled.append(str(earlier))
    info['pooled_captures'] = pooled
    info['detected_frames'] = len(detections)
    if len(detections) < 15:
        fails.append('只在 %d 帧里检测到完整棋盘格（要求 ≥15）：整张纸都要在画面里，别反光、别太快' % len(detections))
        return fails, info
    views = select_views(detections, size)
    corner_sets = [detections[k][0] for k in views]
    info['distinct_views'] = len(views)
    info['coverage_3x3'] = round(coverage(corner_sets, size), 2)
    grid = np.zeros((3, 3), int)
    for corners in corner_sets:
        for x, y in corners:
            grid[min(int(3 * y / size[1]), 2), min(int(3 * x / size[0]), 2)] += 1
    info['coverage_cells'] = grid.tolist()  # 行：上中下；列：左中右；值为角点数
    names = [['左上', '上中', '右上'], ['左中', '中央', '右中'], ['左下', '下中', '右下']]
    info['missing_cells'] = [names[r][c] for r in range(3) for c in range(3) if grid[r, c] == 0]
    if len(views) < 12:
        fails.append('不同视角只有 %d 个（要求 ≥12）：把棋盘格移到画面四角和中间，并且前后、左右倾斜' % len(views))
    if info['coverage_3x3'] < 7 / 9:
        fails.append('角点只覆盖了画面 %d/9 个区域（要求 ≥7），缺：%s。把棋盘格移到这些位置'
                     % (round(9 * info['coverage_3x3']), '、'.join(info['missing_cells'])))
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        result = calibrate_intrinsics(corner_sets, size)
    info.update(rms_px=round(result['rms_px'], 3), fx=round(result['K'][0][0], 1), fy=round(result['K'][1][1], 1),
                cx=round(result['K'][0][2], 1), cy=round(result['K'][1][2], 1),
                dist=[round(v, 4) for v in result['dist']], max_tilt_deg=round(max(result['tilt_deg']), 1),
                K=result['K'], dist_full=result['dist'])
    if result['rms_px'] > 0.5:
        fails.append('重投影误差 %.2f 像素（要求 ≤0.5）：画面模糊或纸不平，拿稳、贴平、动作放慢再录' % result['rms_px'])
    if len(views) < 20:
        fails.append('不同视角只有 %d 个（要求 ≥20）：多换位置和角度' % len(views))
    if max(result['tilt_deg']) < 30:
        fails.append('棋盘格最大倾角只有 %.0f°（要求 ≥30°）：多做前后、左右倾斜' % max(result['tilt_deg']))
    if not 0.8 < info['fx'] / info['fy'] < 1.25 or not (0.3 * size[0] < info['cx'] < 0.7 * size[0]):
        fails.append('内参不合理（fx/fy=%.2f，cx=%.0f）：数据不足以约束，重录' % (info['fx'] / info['fy'], info['cx']))
    return fails, info


# ------------------------------------------------------------------ 相机到雷达
def board_poses(capture_dir, role, K, dist, stride=2, min_still_s=1.0, max_step_m=0.006, max_step_deg=3.0):
    """找出木板静止的时段（相邻检测的棋盘格位姿几乎不变），每段返回平均位姿与时间范围。"""
    cv2 = _cv2()
    directory, rows = camera_index(capture_dir, role)
    K, dist = np.asarray(K), np.asarray(dist)
    obj = board_object_points()
    track = []
    for row in rows[::stride]:
        corners = detect(read_gray(directory, row), allow_small=True)
        if corners is None:
            track.append(None)
            continue
        ok, r, t = cv2.solvePnP(obj, corners.astype(np.float32), K, dist)
        track.append((row['host_monotonic_ns'], cv2.Rodrigues(r)[0], t.ravel()) if ok else None)
    # 静止判断用滑动窗口而不是相邻帧差：PnP 单帧法向抖动可达 2°，相邻帧比较会把一段静止切成碎片。
    # 某帧前后 ±window_s 内，位置偏离窗口中位数 ≤max_step_m、法向偏离窗口平均 ≤max_step_deg，就算静止。
    window_s = 0.4
    valid = [item for item in track if item is not None]
    times = np.array([v[0] for v in valid], np.int64)
    pos = np.array([v[2] for v in valid]) if valid else np.zeros((0, 3))
    nor = np.array([v[1][:, 2] if v[1][:, 2] @ v[2] < 0 else -v[1][:, 2] for v in valid]) if valid else pos
    static = np.zeros(len(valid), bool)
    for i in range(len(valid)):
        m = np.abs(times - times[i]) <= window_s * 1e9
        if m.sum() < 5:
            continue
        mean_n = nor[m].mean(0)
        mean_n /= np.linalg.norm(mean_n)
        static[i] = (np.linalg.norm(pos[m] - np.median(pos[m], 0), axis=1).max() <= max_step_m
                     and np.degrees(np.arccos(np.clip(nor[m] @ mean_n, -1, 1))).max() <= max_step_deg)
    segments, current = [], []
    for i, item in enumerate(valid):
        gap = current and (item[0] - current[-1][0]) * 1e-9 > 0.5
        if not static[i] or gap:
            if len(current) >= 5 and (current[-1][0] - current[0][0]) * 1e-9 >= min_still_s:
                segments.append(current)
            current = []
        if static[i]:
            current.append(item)
    if len(current) >= 5 and (current[-1][0] - current[0][0]) * 1e-9 >= min_still_s:
        segments.append(current)
    out = []
    for seg in segments:
        R = seg[len(seg) // 2][1]
        t = np.median([s[2] for s in seg], axis=0)
        # 远处小棋盘格单帧法向抖动约 1°，整段取平均法向和中位距离
        normals = np.array([s[1][:, 2] if s[1][:, 2] @ s[2] < 0 else -s[1][:, 2] for s in seg])
        n = normals.mean(0)
        n /= np.linalg.norm(n)
        d = float(np.median([-(n @ s[2]) for s in seg]))
        out.append(dict(t0_ns=seg[0][0], t1_ns=seg[-1][0], R=R, t=t, n=n, d=d, samples=len(seg)))
    return out


def board_plane_camera(pose, pattern=PATTERN, square=SQUARE_M):
    """棋盘格平面（相机系）：法向朝相机，n·p + d = 0，d>0；以及棋盘格外框四角。"""
    if 'n' in pose:
        n, d = pose['n'], pose['d']
    else:
        n = pose['R'][:, 2]
        d = -float(n @ pose['t'])
    if d < 0:
        n, d = -n, -d
    w, h = (pattern[0] + 1) * square, (pattern[1] + 1) * square
    corners = np.array([[-square, -square, 0], [w - square, -square, 0], [w - square, h - square, 0],
                        [-square, h - square, 0]])
    return n, d, corners @ pose['R'].T + pose['t']


def lidar_board_plane(cloud_median, corners_lidar_guess, search_radius=0.45, plane_band=0.2,
                      board_radius=0.18, max_normal_deg=20.0, tolerance=0.015, iterations=800, seed=0):
    """在雷达点云里找木板平面。返回 (n, d, 点数, rms) 或 None。

    不取最大平面：木板下方常垫着箱子，箱子正面和木板几乎平行而且更大。这里只接受法向与
    名义值相差 ≤max_normal_deg 的平面，并只保留离棋盘格中心 ≤board_radius（50 cm 木板）的内点。
    """
    from .geometry import fit_plane
    p = cloud_median[np.isfinite(cloud_median).all(1)]
    n0, d0 = fit_plane(corners_lidar_guess)
    centre = corners_lidar_guess.mean(0)
    if n0 @ centre > 0:
        n0, d0 = -n0, -d0
    near = (np.linalg.norm(p - centre, axis=1) < search_radius) & (np.abs(p @ n0 + d0) < plane_band)
    candidates = p[near]
    if len(candidates) < 60:
        return None
    rng = np.random.default_rng(seed)
    cos_limit = np.cos(np.radians(max_normal_deg))
    best, best_score = None, 0
    for _ in range(iterations):
        s = candidates[rng.choice(len(candidates), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if abs(n @ n0) < cos_limit:
            continue
        inliers = np.abs(candidates @ n - n @ s[0]) < tolerance
        # 平面上的中心：内点中离名义中心最近的那团
        if inliers.sum() < 40:
            continue
        q = candidates[inliers]
        local = q[np.linalg.norm(q - np.median(q, axis=0), axis=1) < board_radius]
        score = len(local) - 0.5 * float(np.linalg.norm(np.median(q, axis=0) - centre)) * len(local)
        if score > best_score:
            best_score, best = score, local
    if best is None or len(best) < 40:
        return None
    n, d = fit_plane(best)
    for _ in range(3):
        keep = np.abs(candidates @ n + d) < tolerance
        q = candidates[keep]
        q = q[np.linalg.norm(q - np.median(best, axis=0), axis=1) < board_radius]
        if len(q) < 40:
            break
        best = q
        n, d = fit_plane(best)
    if cloud_median.shape == (9600, 3):
        refined = _organized_board(cloud_median, n, d, corners_lidar_guess, tolerance)
        if refined is not None:
            best = refined
            n, d = fit_plane(best)
    residual = best @ n + d
    return n, float(d), int(len(best)), float(np.sqrt(np.mean(residual ** 2)))


def _organized_board(cloud, n, d, corners_guess, tolerance, paper_margin=0.04, min_points=150):
    """在 60×160 雷达像素网格上只保留木板本身：

    1. 去掉棋盘格纸及其周边：黑白方格反射率不同，ToF 测距会凹凸不平，不能代表板面；
    2. 只留与板中心连通的最大一块，去掉下面箱子、手和背景里恰好共面的零散点。
    名义外参有几厘米误差，纸面范围外扩 paper_margin。
    """
    from scipy import ndimage
    from .geometry import fit_plane
    finite = np.isfinite(cloud).all(1)
    p = np.where(finite[:, None], cloud, 0.0)
    on_plane = finite & (np.abs(p @ n + d) < tolerance)
    # 平面内二维坐标系，判断点是否落在纸面矩形里
    centre = corners_guess.mean(0)
    u = corners_guess[1] - corners_guess[0]
    u = u - (u @ n) * n
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    rel = p - centre
    pu, pv = rel @ u, rel @ v
    cu, cv = (corners_guess - centre) @ u, (corners_guess - centre) @ v
    paper = ((pu > cu.min() - paper_margin) & (pu < cu.max() + paper_margin)
             & (pv > cv.min() - paper_margin) & (pv < cv.max() + paper_margin))
    near = np.linalg.norm(rel, axis=1) < 0.45
    mask = (on_plane & near).reshape(60, 160)
    labels, count = ndimage.label(mask)
    if count == 0:
        return None
    # 取包含纸面周围点最多的连通块（板子一定围着纸）
    ring = (on_plane & near & paper).reshape(60, 160)
    scores = ndimage.sum(ring, labels, range(1, count + 1)) + 1e-3 * ndimage.sum(mask, labels, range(1, count + 1))
    board = (labels == int(np.argmax(scores)) + 1).reshape(-1) & ~paper
    if board.sum() < min_points:
        return None
    return cloud[board]


def solve_plane_extrinsic(camera_planes, lidar_planes, t_prior=None, sigma_distance_m=0.01, sigma_prior_m=0.03):
    """多组对应平面求 p_lidar = R p_camera + t：法向 Kabsch 定 R，距离线性最小二乘定 t。

    板的朝向只能在有限角度内变化，平行于平均法向的平移约束强、侧向弱。给出 t_prior（机械名义值）时
    作为弱先验（σ=sigma_prior_m）加入，避免侧向被噪声带偏；返回值里的 t_std 是各轴后验标准差。
    """
    Nc = np.array([n for n, _ in camera_planes])
    Nl = np.array([n for n, _ in lidar_planes])
    U, S, Vt = np.linalg.svd(Nl.T @ Nc)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    # n_l·p_l + d_l = 0 且 p_l = R p_c + t  =>  d_c = d_l + n_l·t
    A = Nl
    b = np.array([dc - dl for (_, dc), (_, dl) in zip(camera_planes, lidar_planes)])
    H = A.T @ A / sigma_distance_m ** 2
    g = A.T @ b / sigma_distance_m ** 2
    if t_prior is not None:
        H = H + np.eye(3) / sigma_prior_m ** 2
        g = g + np.asarray(t_prior, float) / sigma_prior_m ** 2
    t = np.linalg.solve(H, g)
    t_std = np.sqrt(np.diag(np.linalg.inv(H)))
    angle = [float(np.degrees(np.arccos(np.clip(nl @ (R @ nc), -1, 1)))) for nc, nl in zip(Nc, Nl)]
    dist = [float(dc - dl - nl @ t) for (_, dc), (nl, dl) in zip(camera_planes, lidar_planes)]
    condition = float(np.linalg.svd(Nl, compute_uv=False)[-1])
    return R, t, angle, dist, condition, t_std


def analyze_board_extrinsic(capture, capture_dir, role, intrinsics, earlier_pairs=()):
    """棋盘格贴木板、多个静止姿态：逐姿态检查，再联合求相机到雷达外参并看残差。"""
    from .geometry import describe
    side = CAMERA_LIDAR[role]
    fails, info = [], dict(role=role, lidar=side)
    poses = board_poses(capture_dir, role, intrinsics['K'], intrinsics['dist_full'])
    info['static_board_poses'] = len(poses)
    R0 = R_FLU_OPTICAL
    t0 = np.array(NOMINAL_T_LIDAR_CAMERA_M[role])
    clouds, mono = capture['lidar'][side]['clouds'], capture['lidar'][side]['mono']
    pairs, rows = [], []
    for k, pose in enumerate(poses):
        nc, dc, corners = board_plane_camera(pose)
        sel = np.where((mono >= pose['t0_ns']) & (mono <= pose['t1_ns']))[0]
        row = dict(pose=k, duration_s=round((pose['t1_ns'] - pose['t0_ns']) * 1e-9, 1),
                   distance_m=round(float(np.linalg.norm(pose['t'])), 2),
                   tilt_deg=round(float(np.degrees(np.arccos(abs(nc[2])))), 1))
        if len(sel) < 5:
            row['status'] = '雷达帧不足'
            rows.append(row)
            continue
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            median = np.nanmedian(clouds[sel], axis=0)
        found = lidar_board_plane(median, corners @ R0.T + t0)
        if found is None or found[2] < 80 or found[3] > 0.015:
            row['status'] = '雷达没找到足够大的平整板面（要 ≥80 点、RMS ≤1.5 cm）'
            if found:
                row.update(lidar_points=found[2], lidar_rms_m=round(found[3], 4))
            rows.append(row)
            continue
        nl, dl = found[0], found[1]
        if nl @ (R0 @ nc) < 0:
            nl, dl = -nl, -dl
        row.update(status='OK', lidar_points=found[2], lidar_rms_m=round(found[3], 4))
        rows.append(row)
        pairs.append(((nc, dc), (nl, dl)))
    info['poses'] = rows
    # 保存本次的平面对应，供之后同一相机的录制合并（不用重读录包）
    info['plane_pairs'] = [[nc.tolist(), dc, nl.tolist(), dl] for (nc, dc), (nl, dl) in pairs]
    fx = float(np.asarray(intrinsics['K'])[0][0])
    square_px = [fx * SQUARE_M / r['distance_m'] for r in rows if r.get('status') == 'OK']
    info['median_square_px'] = round(float(np.median(square_px)), 1) if square_px else None
    if square_px and np.median(square_px) < 10:
        fails.append('板离相机太远：相机看到的每个方格只有 %.0f 像素，单姿态法向误差会到 2–4°。'
                     '把板移到相机前 0.8–1.0 米再录' % np.median(square_px))
    tilts = [r['tilt_deg'] for r in rows if r.get('status') == 'OK']
    if tilts and max(tilts) < 30:
        fails.append('最大倾角只有 %.0f°（要求 ≥30°）：左右转、上下倾都要到 35° 左右' % max(tilts))
    pairs = pairs + [((np.array(a), b), (np.array(c), d)) for a, b, c, d in earlier_pairs]
    info['pooled_pairs'] = len(earlier_pairs)
    if len(pairs) < 4:
        fails.append('可用姿态只有 %d 个（要求 ≥4）：板静止 5 秒以上，正对、左右转、上下倾各摆一次；'
                     '板要同时在相机画面里和雷达视野中央' % len(pairs))
        return fails, info
    # 单姿态法向残差约 1–2.5°（相机小目标 + ToF 板面），属于测量噪声；只剔除明显离群的姿态
    # （残差 > max(3°, 2.5×中位数)），再用自助法估计旋转的不确定度作为判据。
    active = list(range(len(pairs)))
    dropped = []
    while True:
        R, t, angle, dist, condition, t_std = solve_plane_extrinsic(
            [pairs[i][0] for i in active], [pairs[i][1] for i in active], t_prior=t0)
        worst = int(np.argmax(angle))
        if len(active) <= 5 or angle[worst] <= max(3.0, 2.5 * float(np.median(angle))):
            break
        dropped.append(active.pop(worst))
    rng = np.random.default_rng(0)
    boot = []
    for _ in range(200):
        pick = [active[i] for i in rng.integers(0, len(active), len(active))]
        Rb = solve_plane_extrinsic([pairs[i][0] for i in pick], [pairs[i][1] for i in pick], t_prior=t0)[0]
        boot.append(np.degrees(np.arccos(np.clip((np.trace(R.T @ Rb) - 1) / 2, -1, 1))))
    rotation_std = float(np.sqrt(np.mean(np.square(boot))))
    info.update(poses_used=len(active), poses_dropped=len(dropped),
                normal_residual_deg_median=round(float(np.median(angle)), 2),
                rotation_std_deg=round(rotation_std, 2))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    delta = R0.T @ R
    info.update(T_lidar_camera=T.tolist(), summary=describe(T @ np.block([[R0.T, np.zeros((3, 1))], [0, 0, 0, 1]])),
                normal_residual_deg_max=round(max(angle), 2), distance_residual_m_max=round(max(map(abs, dist)), 4),
                normal_condition=round(condition, 3), translation_std_m=[round(float(v), 4) for v in t_std],
                rotation_from_nominal_deg=round(float(np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1)))), 2),
                translation_from_nominal_m=round(float(np.linalg.norm(t - t0)), 3))
    if condition < 0.3:
        fails.append('板的朝向变化不够（法向条件 %.2f，要求 ≥0.3）：再加一个上下倾斜或左右转得更多的姿态' % condition)
    if len(active) < 5:
        fails.append('剔除离群后只剩 %d 个姿态（要求 ≥5）：再录一次补几个姿态，结果会合并' % len(active))
    # 平移由全部姿态共同决定，单个姿态 2–3 cm 的距离残差是 ToF 测距噪声，看中位数
    dist_median = float(np.median(np.abs(dist)))
    info['distance_residual_m_median'] = round(dist_median, 4)
    # 距离残差中位 ≤2 cm：ToF 板面测距有 1–2 cm 的偏差（推墙实验），多姿态平均后平移误差 <0.5 cm
    if rotation_std > 1.0 or np.median(angle) > 2.5 or dist_median > 0.02:
        fails.append('结果还不够稳（旋转不确定度 %.2f°、法向残差中位 %.2f°、距离残差中位 %.1f cm，'
                     '要求 ≤1.0°/2.5°/2.0 cm）：再录一次，多摆几个大角度姿态，每个姿态摆好后静止 3 秒以上'
                     % (rotation_std, float(np.median(angle)), 100 * dist_median))
    if info['translation_from_nominal_m'] > 0.15 or info['rotation_from_nominal_deg'] > 10:
        fails.append('结果离机械名义值太远（%.0f cm / %.1f°），可能配错了左右相机或雷达' % (
            100 * info['translation_from_nominal_m'], info['rotation_from_nominal_deg']))
    return fails, info
