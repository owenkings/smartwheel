"""现场外参用的几何工具：平面拟合、竖直方向、找平旋转、左右点到面联合配准。

约定：点云为米制 FLU；`T_left_right` 把右雷达点变换到左雷达坐标，p_left = R @ p_right + t。
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

NAMES6 = ('tx_m', 'ty_m', 'tz_m', 'roll_deg', 'pitch_deg', 'yaw_deg')


def fit_plane(points):
    """TLS 平面：返回单位法向 n 与 d，满足 n·p + d = 0。"""
    centre = points.mean(0)
    _, _, vt = np.linalg.svd(points - centre, full_matrices=False)
    normal = vt[2]
    return normal, -normal @ centre


def ransac_plane(points, want, iterations=1500, tolerance=0.03, seed=0):
    """want='h' 只找近水平面（|n_z|≥0.9），'v' 只找近竖直面（|n_z|≤0.35）。

    返回 (n, d, inliers, rms)，法向翻到朝向原点（d>0）；找不到返回 None。
    """
    rng = np.random.default_rng(seed)
    best, best_count = None, 0
    if len(points) < 3:
        return None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal /= norm
        if (want == 'h' and abs(normal[2]) < 0.9) or (want == 'v' and abs(normal[2]) > 0.35):
            continue
        inliers = np.abs(points @ normal - normal @ sample[0]) < tolerance
        if inliers.sum() > best_count:
            best_count, best = inliers.sum(), inliers
    if best is None:
        return None
    normal, d = fit_plane(points[best])
    for _ in range(3):
        inliers = np.abs(points @ normal + d) < tolerance
        normal, d = fit_plane(points[inliers])
    if d < 0:
        normal, d = -normal, -d
    residual = points[inliers] @ normal + d
    return normal, float(d), inliers, float(np.sqrt(np.mean(residual ** 2)))


def vertical_plane_normals(points, max_planes=2, iterations=300, tolerance=0.03,
                           min_inliers=400, min_height_span=0.6, seed=0):
    """一帧中依次剔除的近竖直大平面（墙）的法向，墙要有足够竖直跨度。"""
    rng = np.random.default_rng(seed)
    found = []
    for _ in range(max_planes):
        if len(points) < min_inliers:
            break
        best, best_count = None, 0
        for _ in range(iterations):
            sample = points[rng.choice(len(points), 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            if np.linalg.norm(normal) < 1e-6:
                continue
            normal /= np.linalg.norm(normal)
            if abs(normal[2]) > 0.35:
                continue
            inliers = np.abs(points @ normal - normal @ sample[0]) < tolerance
            if inliers.sum() > best_count:
                best_count, best = inliers.sum(), inliers
        if best is None or best_count < min_inliers:
            break
        normal, d = fit_plane(points[best])
        inliers = np.abs(points @ normal + d) < tolerance
        normal, _ = fit_plane(points[inliers])
        if np.ptp(points[inliers][:, 2]) > min_height_span:
            found.append(normal)
        points = points[~inliers]
    return found


def up_from_wall_normals(normals, bootstrap=100, seed=0):
    """墙面竖直：向上方向是所有墙法向矩阵的最小特征向量。返回 (up, 68% 角度不确定度°)。"""
    normals = np.asarray(normals, dtype=float)
    if len(normals) < 20:
        raise ValueError('墙面数量不足（%d），无法确定竖直方向' % len(normals))

    def solve(rows):
        _, vectors = np.linalg.eigh(rows.T @ rows)
        up = vectors[:, 0]
        return up if up[2] > 0 else -up

    up = solve(normals)
    rng = np.random.default_rng(seed)
    angles = [np.degrees(np.arccos(np.clip(solve(normals[rng.integers(0, len(normals), len(normals))]) @ up, -1, 1)))
              for _ in range(bootstrap)]
    return up, float(np.percentile(angles, 68))


def roll_pitch_deg(up):
    """雷达坐标中的向上方向对应的横滚、俯仰（度）。"""
    return (float(np.degrees(np.arctan2(up[1], up[2]))), float(np.degrees(-np.arctan2(up[0], up[2]))))


def level_rotation(up):
    """把雷达坐标中的 up 转到 +z 的最小旋转矩阵 R（R @ up = z）。"""
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    axis = np.cross(up, [0, 0, 1.0])
    s, c = np.linalg.norm(axis), up[2]
    if s < 1e-12:
        return np.eye(3)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / s ** 2)


def describe(T):
    euler = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return dict(zip(NAMES6, [round(float(v), 4) for v in (*T[:3, 3], *euler)]))


def clean_cloud(points):
    """去掉无效点、过近/过远点和车前低处（脚踏、脚）。"""
    points = points[np.isfinite(points).all(1)]
    r = np.linalg.norm(points, axis=1)
    return points[(r > 0.7) & (r < 8.0) & ~((points[:, 0] < 1.4) & (points[:, 2] < -0.3))]


class CloudPair:
    """同一时刻左右两帧：左帧作目标（建 KD 树与法向），右帧作源。"""

    def __init__(self, left, right, tag='', max_source=5000):
        self.tag = tag
        target = clean_cloud(np.asarray(left, dtype=float))
        source = clean_cloud(np.asarray(right, dtype=float))
        if len(target) < 200 or len(source) < 200:
            raise ValueError('点数不足')
        self.target = target
        self.tree = cKDTree(target)
        neighbours = self.tree.query(target, k=15)[1]
        patch = target[neighbours] - target[neighbours].mean(1, keepdims=True)
        values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', patch, patch) / 15)
        self.normals = vectors[:, :, 0]
        self.flat = values[:, 0] / np.maximum(values.sum(1), 1e-12) < 0.05
        if len(source) > max_source:
            source = source[np.linspace(0, len(source) - 1, max_source).astype(int)]
        self.source = source

    def residuals(self, T, max_distance):
        moved = self.source @ T[:3, :3].T + T[:3, 3]
        distance, index = self.tree.query(moved, k=1)
        keep = (distance < max_distance) & self.flat[index]
        normal = self.normals[index[keep]]
        residual = np.einsum('ij,ij->i', normal, moved[keep] - self.target[index[keep]])
        return moved[keep], normal, residual


def _normal_equations(pairs, T, max_distance, huber=0.03):
    H, g = np.zeros((6, 6)), np.zeros(6)
    for pair in pairs:
        p, n, r = pair.residuals(T, max_distance)
        if len(r) < 100:
            continue
        w = np.where(np.abs(r) < huber, 1.0, huber / np.maximum(np.abs(r), 1e-12)) / len(r)  # 每对等权
        A = np.column_stack([np.cross(p, n), n])
        H += A.T @ (A * w[:, None])
        g += A.T @ (w * r)
    return H, g


def _schedule(iteration):
    return 0.3 if iteration < 10 else (0.15 if iteration < 20 else 0.08)


def solve_left_right_4dof(pairs, up_left, up_right, initial_yaw_rad, initial_t, iterations=30):
    """固定两台雷达各自的竖直方向，只求绕竖直轴的偏航和三维平移。

    T = [R_lev_left^T · Rz(yaw) · R_lev_right | t]。全部停顿的对应关系进同一个加权最小二乘。
    """
    RL, RR = level_rotation(up_left), level_rotation(up_right)
    up_in_left = RL.T @ np.array([0, 0, 1.0])
    J = np.zeros((6, 4))
    J[:3, 0] = up_in_left
    J[3:, 1:] = np.eye(3)
    x = np.array([initial_yaw_rad, *initial_t], dtype=float)

    def make(state):
        T = np.eye(4)
        T[:3, :3] = RL.T @ Rotation.from_euler('z', state[0]).as_matrix() @ RR
        T[:3, 3] = state[1:]
        return T

    for iteration in range(iterations):
        H, g = _normal_equations(pairs, make(x), _schedule(iteration))
        reduced = J.T @ H @ J
        if np.linalg.matrix_rank(reduced) < 4:
            raise ValueError('配准退化：停顿处共同可见的立体结构不足')
        step = -np.linalg.solve(reduced, J.T @ g)
        x = x + step
        if np.linalg.norm(step) < 1e-7:
            break
    return make(x)


def overlap_residual_mm(pairs, T):
    """重叠区点到面距离：每对取中位数，再取全部对的中位数（毫米）。越小两台越贴合。"""
    values = []
    for pair in pairs:
        _, _, r = pair.residuals(T, 0.10)
        if len(r) >= 100:
            values.append(np.median(np.abs(r)))
    return round(1000 * float(np.median(values)), 2) if values else None
