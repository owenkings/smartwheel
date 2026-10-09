"""Local result comparison. Point-cloud interaction needs no browser or CDN.

Points are sampled deterministically from the actual export, never reconstructed
from a screenshot. The file parser checks the complete declared PLY payload.
"""
import csv
import io
import lzma
import math
import struct
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .ui_common import BackgroundTask, PanelScrollArea, button, combo, heading, hint, local_open, result_label


DISPLAY_POINTS = 180000
MAX_POINTS = 50000000
MAX_DECOMPRESSED_BYTES = 4 * 1024**3
XZ_DECODER_MEMORY_LIMIT = 128 * 1024**2


class _BoundedXZReader(io.RawIOBase):
    """Strict concatenated XZ reader with bounded decoder and output memory.

    liblzma's memlimit covers the dictionary (unlike counting decoded bytes).
    Each stream and its checksum must finish; only standard four-byte-aligned
    zero padding is accepted between/after streams, never arbitrary tail data.
    """
    def __init__(self, path):
        super().__init__()
        self.source = open(path, 'rb')
        self.decoder = None
        self.pending = b''
        self.streams = 0
        self.finished = False
        self.decoded = 0

    def readable(self):
        return True

    def _begin_stream(self):
        padding = 0
        while True:
            data = self.pending or self.source.read(65536)
            self.pending = b''
            if not data:
                if not self.streams or padding % 4:
                    raise ValueError('XZ 文件为空、截断或尾部填充不完整')
                self.finished = True
                return False
            if self.streams:
                tail = data.lstrip(b'\0')
                padding += len(data) - len(tail)
                data = tail
            if data:
                if padding % 4:
                    raise ValueError('XZ 流间填充必须是四字节对齐的零字节')
                self.decoder = lzma.LZMADecompressor(format=lzma.FORMAT_XZ,
                                                    memlimit=XZ_DECODER_MEMORY_LIMIT)
                self.pending = data
                return True

    def readinto(self, target):
        if not len(target):
            return 0
        while not self.finished:
            if self.decoder is None and not self._begin_stream():
                break
            data = b''
            if self.decoder.needs_input:
                data = self.pending or self.source.read(65536)
                self.pending = b''
                if not data:
                    raise ValueError('XZ 压缩流截断，校验未完成')
            try:
                block = self.decoder.decompress(data, max_length=min(len(target), 65536))
            except lzma.LZMAError as error:
                raise ValueError('XZ 数据损坏、含无效尾部或解码器内存超过128 MiB') from error
            self.decoded += len(block)
            if self.decoded > MAX_DECOMPRESSED_BYTES:
                raise ValueError('XZ 解码数据超过读取上限')
            if self.decoder.eof:
                self.pending = self.decoder.unused_data
                self.streams += 1
                self.decoder = None
            if block:
                target[:len(block)] = block
                return len(block)
        return 0

    def close(self):
        if not self.closed:
            self.source.close()
        super().close()


def _open_xz(path, mode='rb'):
    if mode != 'rb':
        raise ValueError('点云压缩归档仅支持只读模式')
    return io.BufferedReader(_BoundedXZReader(path), buffer_size=65536)


class _BoundedCloudStream:
    """Count decoded bytes, including non-vertex payload and trailing content."""
    def __init__(self, stream, format_name="PLY"):
        self.stream, self.position, self.format_name = stream, 0, format_name

    def checked(self, data):
        self.position += len(data)
        if self.position > MAX_DECOMPRESSED_BYTES:
            raise ValueError("{} 解码数据超过读取上限".format(self.format_name))
        return data

    def read(self, size):
        return self.checked(self.stream.read(size))

    def readline(self, size):
        return self.checked(self.stream.readline(size))

    def tell(self):
        return self.position


def _exact(stream, size):
    raw = stream.read(size)
    if len(raw) != size:
        raise ValueError("压缩 PLY 的声明数据不完整")
    return raw


def _stream_ply_blocks(stream, form, elements, chunk_size=65536):
    """Payload counterpart of map_viewer.ply_blocks for a forward-only stream.

    The existing strict header/schema parser is shared. Every declared payload
    byte is consumed and checked; no seek, extraction file, or whole-cloud
    allocation is used for compressed archives.
    """
    from wc_runtime.map_viewer import SCALARS
    endian = ">" if form == "binary_big_endian" else "<"
    for element in elements:
        props, count = element["properties"], element["count"]
        names = [prop[0] for prop in props]
        vertex = element["name"] == "vertex"
        if vertex and (not all(axis in names for axis in ("x", "y", "z")) or any(len(prop) != 2 for prop in props)):
            raise ValueError("PLY 顶点缺少标量 XYZ 坐标")
        if not count:
            continue
        if not props:
            raise ValueError("非空 PLY 元素缺少属性")
        if form != "ascii" and all(len(prop) == 2 for prop in props):
            dtype = np.dtype([(prop[0], endian + SCALARS[prop[1]]) for prop in props])
            if count*dtype.itemsize > MAX_DECOMPRESSED_BYTES:
                raise ValueError("PLY 元素声明超过读取上限")
            binary_chunk = min(chunk_size, max(1, 4*1024**2 // dtype.itemsize))
            for start in range(0, count, binary_chunk):
                n = min(binary_chunk, count-start)
                block = np.frombuffer(_exact(stream, n*dtype.itemsize), dtype=dtype)
                if vertex:
                    yield np.column_stack([block[axis] for axis in ("x", "y", "z")]).astype(np.float64)
        else:
            pending = []
            for _ in range(count):
                tokens = None
                if form == "ascii":
                    line = stream.readline(1024**2)
                    if not line or len(line) >= 1024**2:
                        raise ValueError("压缩 PLY 文本数据损坏或过长")
                    tokens = iter(line.split())
                values = []
                try:
                    for prop in props:
                        dtype = np.dtype(endian + SCALARS[prop[1]])
                        if len(prop) == 2:
                            value = float(next(tokens)) if tokens is not None else np.frombuffer(_exact(stream, dtype.itemsize), dtype=dtype)[0]
                            values.append(value)
                        else:
                            count_type = np.dtype(endian + SCALARS[prop[2]])
                            n = int(next(tokens)) if tokens is not None else int(np.frombuffer(_exact(stream, count_type.itemsize), dtype=count_type)[0])
                            if not 0 <= n <= 1000000:
                                raise ValueError("PLY 列表长度超出上限")
                            if tokens is not None:
                                for _ in range(n):
                                    float(next(tokens))
                            else:
                                remaining = n*dtype.itemsize
                                while remaining:
                                    take = min(65536, remaining)
                                    _exact(stream, take)
                                    remaining -= take
                except StopIteration as error:
                    raise ValueError("压缩 PLY 文本数据字段不完整") from error
                if tokens is not None and next(tokens, None) is not None:
                    raise ValueError("压缩 PLY 文本行含多余字段")
                if vertex:
                    pending.append([values[names.index(axis)] for axis in ("x", "y", "z")])
                    if len(pending) == chunk_size:
                        yield np.asarray(pending, dtype=float)
                        pending = []
            if pending:
                yield np.asarray(pending, dtype=float)
    if form == "ascii":
        for extra in iter(lambda: stream.read(65536), b""):
            if extra.strip():
                raise ValueError("压缩 PLY 含未声明的尾部数据")
    elif stream.read(1):
        raise ValueError("压缩 PLY 含未声明的尾部数据")


def _sample_cloud_blocks(blocks, total, maximum):
    stride = max(1, math.ceil(total / maximum))
    selected_blocks, offset = [], 0
    for block in blocks:
        selected = block[(-offset) % stride::stride]
        if len(selected):
            selected_blocks.append(selected)
        offset += len(block)
    if offset != total:
        raise ValueError("PLY 实际顶点数与文件头不一致")
    return np.concatenate(selected_blocks) if selected_blocks else np.empty((0, 3))


def _lzf_chunks(stream, compressed_size, expected_size):
    """Decode PCD LZF with an 8 KiB history and bounded streaming buffers.

    Format: https://pointclouds.org/documentation/tutorials/pcd_file_format.html
    Back references may overlap; no full decoded cloud is allocated.
    """
    pending, cursor, remaining = b"", 0, compressed_size
    history, emitted, produced = bytearray(), 0, 0

    def take(size):
        nonlocal pending, cursor, remaining
        if len(pending) - cursor < size:
            pending = pending[cursor:]
            cursor = 0
            block = stream.read(min(65536, remaining))
            remaining -= len(block)
            pending += block
        if len(pending) - cursor < size:
            raise ValueError("PCD LZF 压缩数据截断")
        value = pending[cursor:cursor + size]
        cursor += size
        return value

    consumed = 0
    while consumed < compressed_size:
        control = take(1)[0]
        consumed += 1
        if control < 32:
            length = control + 1
            block = take(length)
            consumed += length
        else:
            length = control >> 5
            if length == 7:
                length += take(1)[0]
                consumed += 1
            distance = ((control & 31) << 8) + take(1)[0] + 1
            consumed += 1
            length += 2
            if distance > len(history):
                raise ValueError("PCD LZF 回引用越界")
            pattern = bytes(history[-distance:])
            block = (pattern * math.ceil(length / distance))[:length]
        if consumed > compressed_size or produced + length > expected_size:
            raise ValueError("PCD LZF 数据超过声明长度")
        history.extend(block)
        produced += length
        if len(history) - emitted >= 65536:
            yield bytes(history[emitted:])
            history = history[-8192:]
            emitted = len(history)
    if produced != expected_size:
        raise ValueError("PCD LZF 解码长度与声明不一致")
    if len(history) > emitted:
        yield bytes(history[emitted:])


def _compressed_pcd_xyz(stream, names, sizes, kinds, counts, n, stride):
    raw_sizes = stream.read(8)
    if len(raw_sizes) != 8:
        raise ValueError("PCD 压缩长度头不完整")
    packed, unpacked = struct.unpack("<II", raw_sizes)
    expected = n * sum(size * count for size, count in zip(sizes, counts))
    if unpacked != expected or not packed or stream.tell() + max(packed, unpacked) > MAX_DECOMPRESSED_BYTES:
        raise ValueError("PCD 压缩长度与字段不符或超过读取上限")
    chunks = iter(_lzf_chunks(stream, packed, unpacked))
    pending = bytearray()
    points = np.empty((math.ceil(n / stride), 3), dtype=np.float64)
    # PCL binary_compressed stores each complete field consecutively (SoA),
    # including COUNT>1 fields. Consume and validate non-XYZ fields as well.
    for name, size, kind, count in zip(names, sizes, kinds, counts):
        step = size * count
        block_points = max(1, 65536 // step)
        for start in range(0, n, block_points):
            end = min(n, start + block_points)
            required = (end - start) * step
            while len(pending) < required:
                try:
                    pending.extend(next(chunks))
                except StopIteration as error:
                    raise ValueError("PCD 压缩字段数据不完整") from error
            if name in ("x", "y", "z"):
                dtype = np.dtype("<" + {"F": "f", "I": "i", "U": "u"}[kind] + str(size))
                values = np.frombuffer(bytes(pending[:required]), dtype=dtype)
                first = (-start) % stride
                points[(start + first) // stride:math.ceil(end / stride), ("x", "y", "z").index(name)] = values[first::stride]
            del pending[:required]
    # Exhaust the decoder so its final length check cannot be skipped.
    if pending or next(chunks, None) is not None or stream.read(1):
        raise ValueError("PCD 压缩数据含未声明的尾部数据")
    return points


def load_pcd(path, maximum=DISPLAY_POINTS):
    """Validate and sample standard PCD from plain or XZ streams without extraction."""
    size = path.stat().st_size
    if not 0 < size <= MAX_DECOMPRESSED_BYTES:
        raise ValueError("PCD 文件过大或为空")
    opener = _open_xz if path.name.lower().endswith(".pcd.xz") else open
    with opener(path, "rb") as source:
        stream = _BoundedCloudStream(source, "PCD")
        header = {}
        for _ in range(100):
            line = stream.readline(8192)
            if not line or len(line) >= 8192:
                raise ValueError("PCD 文件头损坏")
            fields = line.decode("ascii").strip().split()
            if not fields or fields[0].startswith("#"):
                continue
            key = fields[0].upper()
            if key in header:
                raise ValueError("PCD 文件头含重复字段")
            header[key] = fields[1:]
            if fields[0].upper() == "DATA":
                break
        names = header.get("FIELDS", header.get("FIELD", []))
        if len(set(names)) != len(names):
            raise ValueError("PCD 坐标字段重名")
        if not all(axis in names for axis in ("x", "y", "z")):
            raise ValueError("PCD 缺少 XYZ 坐标")
        sizes = list(map(int, header.get("SIZE", [])))
        kinds = header.get("TYPE", [])
        counts = list(map(int, header.get("COUNT", ["1"] * len(names))))
        if not len(names) == len(sizes) == len(kinds) == len(counts):
            raise ValueError("PCD 字段描述不一致")
        if any(not 0 < count <= 1024 for count in counts):
            raise ValueError("PCD 字段维数不合法")
        if any(counts[names.index(axis)] != 1 for axis in ("x", "y", "z")):
            raise ValueError("PCD XYZ 必须为标量")
        types = []
        for name, length, kind, count in zip(names, sizes, kinds, counts):
            if kind not in ("F", "I", "U") or length not in (1, 2, 4, 8) or (kind == "F" and length not in (4, 8)):
                raise ValueError("不支持的 PCD 数值类型")
            scalar = "<" + {"F": "f", "I": "i", "U": "u"}[kind] + str(length)
            types.append((name, scalar, (count,)) if count > 1 else (name, scalar))
        def integer_field(key, default):
            values = header.get(key, [str(default)])
            if len(values) != 1:
                raise ValueError('PCD ' + key + ' 必须为单个整数')
            return int(values[0])
        width, height = integer_field('WIDTH', 0), integer_field('HEIGHT', 1)
        if ('WIDTH' in header and width <= 0) or ('HEIGHT' in header and height <= 0):
            raise ValueError('PCD 宽高必须为正整数')
        n = integer_field('POINTS', width * height)
        if not 0 < n <= MAX_POINTS:
            raise ValueError("PCD 点数超出显示器读取范围")
        if "WIDTH" in header and width * height != n:
            raise ValueError("PCD 宽高与点数不一致")
        stride = max(1, math.ceil(n / maximum))
        if len(header.get('DATA', [])) != 1:
            raise ValueError('PCD DATA 必须声明单个存储格式')
        data = header['DATA'][0]
        blocks = []
        if data == "binary":
            dtype = np.dtype(types)
            if stream.tell() + n * dtype.itemsize > MAX_DECOMPRESSED_BYTES:
                raise ValueError("PCD 声明的数据超过读取上限")
            chunk_size = min(65536, max(1, 4 * 1024**2 // dtype.itemsize))
            for start in range(0, n, chunk_size):
                count = min(chunk_size, n - start)
                raw = stream.read(count * dtype.itemsize)
                if len(raw) != count * dtype.itemsize:
                    raise ValueError("PCD 二进制数据长度与文件头不一致")
                block = np.frombuffer(raw, dtype=dtype)
                take = np.arange((-start) % stride, count, stride)
                if len(take):
                    blocks.append(np.column_stack([block[axis][take] for axis in ("x", "y", "z")]))
            if stream.read(1):
                raise ValueError("PCD 二进制数据含未声明的尾部数据")
        elif data == "binary_compressed":
            blocks.append(_compressed_pcd_xyz(stream, names, sizes, kinds, counts, n, stride))
        elif data == "ascii":
            offsets = np.cumsum([0] + counts[:-1])
            xyz_columns = [int(offsets[names.index(axis)]) for axis in ("x", "y", "z")]
            selected = []
            for index in range(n):
                line = stream.readline(1024 * 1024)
                values = line.split()
                if not line or len(line) >= 1024 * 1024 or len(values) != sum(counts):
                    raise ValueError("PCD 文本数据行损坏")
                # Validate every row, including rows omitted by display sampling.
                numeric = [float(value) for value in values]
                if index % stride == 0:
                    selected.append([numeric[c] for c in xyz_columns])
            for extra in iter(lambda: stream.read(65536), b""):
                if extra.strip():
                    raise ValueError("PCD 含未声明的点数据")
            blocks.append(np.asarray(selected, dtype=float))
        else:
            raise ValueError("PCD DATA 必须为 ascii、binary 或 binary_compressed")
    return np.concatenate(blocks) if blocks else np.empty((0, 3)), n


def load_cloud(value, maximum=DISPLAY_POINTS):
    path = Path(value)
    if maximum <= 0:
        raise ValueError("显示点数上限必须大于零")
    if path.name.lower().endswith(".ply.xz"):
        from wc_runtime.map_viewer import ply_header
        if not 0 < path.stat().st_size <= MAX_DECOMPRESSED_BYTES:
            raise ValueError("压缩 PLY 文件为空或超过读取上限")
        with _open_xz(path, "rb") as compressed:
            stream = _BoundedCloudStream(compressed)
            form, elements, _ = ply_header(stream)
            total = next(element["count"] for element in elements if element["name"] == "vertex")
            points = _sample_cloud_blocks(_stream_ply_blocks(stream, form, elements), total, maximum)
    elif path.suffix.lower() == ".pcd" or path.name.lower().endswith(".pcd.xz"):
        points, total = load_pcd(path, maximum)
    elif path.suffix.lower() == ".ply":
        from wc_runtime.map_viewer import ply_header, ply_blocks
        with path.open("rb") as stream:
            _, elements, _ = ply_header(stream)
        total = next(element["count"] for element in elements if element["name"] == "vertex")
        points = _sample_cloud_blocks(ply_blocks(path), total, maximum)
    else:
        raise ValueError("请选择 PLY、PLY.XZ、PCD 或 PCD.XZ 三维点云导出文件")
    finite = np.isfinite(points).all(axis=1)
    points = points[finite].astype(np.float64)
    if not len(points):
        raise ValueError("点云没有有限的 XYZ 坐标")
    return {"points": points, "total": total, "shown": len(points), "path": str(path)}


def load_trajectory(value):
    path = Path(value)
    if path.stat().st_size > 300 * 1024**2:
        raise ValueError("路线文件超过读取范围")
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        x_name = next((n for n in ("x", "x_m", "position_x", "tx", "pose_x") if n in fields), None)
        y_name = next((n for n in ("y", "y_m", "position_y", "ty", "pose_y") if n in fields), None)
        if not x_name or not y_name:
            raise ValueError("路线 CSV 缺少可识别的 x、y 坐标列")
        data = []
        for row in reader:
            point = (float(row[x_name]), float(row[y_name]))
            if all(math.isfinite(v) for v in point):
                data.append(point)
            if len(data) > 2000000:
                raise ValueError("路线点数超出读取范围")
    if not data:
        raise ValueError("路线没有有效坐标")
    return np.asarray(data)


class CloudCanvas(QtWidgets.QWidget):
    """Software orthographic renderer with orbit, pan and zoom interactions."""
    viewChanged = QtCore.pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(340, 270)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setProperty("consumesWheel", True)
        self.points = None
        self.colors = None
        self.yaw, self.pitch = -35.0, 55.0
        self.zoom = 1.0
        self.pan = np.zeros(2)
        self.radius = 1.0
        self.center = np.zeros(3)
        self.last_pos = None
        self.message = "选择一个点云文件"
        self.cached = None

    def set_cloud(self, points):
        self.points = np.asarray(points, dtype=float)
        low, high = self.points.min(axis=0), self.points.max(axis=0)
        self.center = (low + high) / 2
        self.radius = max(float(np.linalg.norm(high-low) / 2), .05)
        z = self.points[:, 2]
        t = np.clip((z-z.min()) / max(float(np.ptp(z)), .01), 0, 1)
        self.colors = np.column_stack((55 + 190 * t, 190 - 60 * t, 240 - 185 * t)).astype(np.uint8)
        self.message = ""
        self.reset_view()

    def state(self):
        return {"yaw": self.yaw, "pitch": self.pitch, "zoom": self.zoom, "pan": self.pan.tolist()}

    def set_state(self, state):
        self.yaw, self.pitch = float(state["yaw"]), float(state["pitch"])
        self.zoom, self.pan = float(state["zoom"]), np.asarray(state["pan"], dtype=float)
        self.invalidate(False)

    def reset_view(self):
        self.yaw, self.pitch, self.zoom = -35.0, 55.0, 1.0
        self.pan = np.zeros(2)
        self.invalidate()

    def top_view(self):
        self.yaw, self.pitch = 0.0, 90.0
        self.invalidate()

    def invalidate(self, notify=True):
        self.cached = None
        self.update()
        if notify:
            self.viewChanged.emit(self.state())

    def projection(self):
        yaw, pitch = math.radians(self.yaw), math.radians(self.pitch)
        # Camera basis: horizontal, vertical and depth. Z is always physical file Z.
        return np.array([[math.cos(yaw), -math.sin(yaw), 0],
                         [math.sin(yaw)*math.sin(pitch), math.cos(yaw)*math.sin(pitch), -math.cos(pitch)],
                         [math.sin(yaw)*math.cos(pitch), math.cos(yaw)*math.cos(pitch), math.sin(pitch)]])

    def render_points(self):
        width, height = max(self.width(), 1), max(self.height(), 1)
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (19, 29, 43)
        if self.points is not None:
            scale = min(width, height) * .43 * self.zoom / self.radius
            camera = (self.points - self.center).dot(self.projection().T)
            # Pan stored as normalized viewport fractions, so linked panes stay aligned.
            x = np.round(camera[:, 0]*scale + width/2 + self.pan[0]*width).astype(np.int64)
            y = np.round(camera[:, 1]*scale + height/2 + self.pan[1]*height).astype(np.int64)
            good = (x >= 1) & (x < width-1) & (y >= 1) & (y < height-1)
            indices = np.flatnonzero(good)
            if len(indices):
                indices = indices[np.argsort(camera[indices, 2])]
                xx, yy, color = x[indices], y[indices], self.colors[indices]
                canvas[yy, xx] = color
                canvas[yy+1, xx] = color
                canvas[yy, xx+1] = color
        return QtGui.QImage(canvas.data, width, height, width*3, QtGui.QImage.Format_RGB888).copy()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        if self.cached is None:
            self.cached = self.render_points()
        painter.drawImage(0, 0, self.cached)
        painter.setPen(QtGui.QColor("#cedbe9"))
        if self.message:
            painter.drawText(self.rect().adjusted(18, 18, -18, -18), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.message)
        if self.points is not None:
            painter.drawText(14, self.height()-14, "左键旋转 · 右键 / 中键平移 · 滚轮缩放 · 双击复位")
            basis = self.projection()
            origin = QtCore.QPointF(42, 46)
            for axis, color, label in ((0, "#f57273", "X"), (1, "#7dde97", "Y"), (2, "#73b2ff", "Z")):
                endpoint = origin + QtCore.QPointF(float(basis[0, axis])*24, float(basis[1, axis])*24)
                painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
                painter.drawLine(origin, endpoint)
                painter.drawText(endpoint + QtCore.QPointF(3, -3), label)
        painter.end()

    def resizeEvent(self, event):
        self.invalidate(False)
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        self.last_pos = event.pos()
        self.setCursor(QtCore.Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self.last_pos is None:
            return
        delta = event.pos()-self.last_pos
        self.last_pos = event.pos()
        if event.buttons() & (QtCore.Qt.RightButton | QtCore.Qt.MiddleButton) or event.modifiers() & QtCore.Qt.ShiftModifier:
            self.pan += np.array([delta.x()/max(self.width(), 1), delta.y()/max(self.height(), 1)])
        elif event.buttons() & QtCore.Qt.LeftButton:
            self.yaw += delta.x()*.45
            self.pitch = min(179.5, max(-179.5, self.pitch+delta.y()*.45))
        self.invalidate()

    def mouseReleaseEvent(self, event):
        self.last_pos = None
        self.unsetCursor()

    def mouseDoubleClickEvent(self, event):
        self.reset_view()

    def wheelEvent(self, event):
        self.zoom = min(250.0, max(.015, self.zoom * math.exp(event.angleDelta().y()/700)))
        self.invalidate()
        event.accept()


class ImageView(QtWidgets.QGraphicsView):
    """Image inspection with independent pan/zoom, including inside scroll pages."""
    viewChanged = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty("consumesWheel", True)
        self.viewport().setProperty("consumesWheel", True)
        self.setScene(QtWidgets.QGraphicsScene(self))
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.NoAnchor)
        self.setResizeAnchor(QtWidgets.QGraphicsView.NoAnchor)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setMinimumHeight(270)
        self.setBackgroundBrush(QtGui.QColor("#f0f4f8"))
        self.setStyleSheet("QGraphicsView { border: none; background: #f0f4f8; }")
        self.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        self.has_image = False
        self.fit_mode = True
        self.image_rect = QtCore.QRectF()
        self.message = "该结果没有可用图片"

    def sizeHint(self):
        # QGraphicsView derives its default hint from the scene and desktop.
        # Large exports plus navigation padding otherwise enlarge the whole
        # result card beyond the visible window, even in fit mode.
        return QtCore.QSize(520, 320)

    def minimumSizeHint(self):
        return QtCore.QSize(160, 200)

    def clear_image(self, message="该结果没有可用图片"):
        self.scene().clear()
        self.has_image = False
        self.image_rect = QtCore.QRectF()
        self.message = message
        self.resetTransform()
        self.viewport().update()
        self.viewChanged.emit()

    def set_image(self, path):
        image = QtGui.QImage(str(path))
        if image.isNull():
            self.clear_image("图片读取失败")
            raise ValueError("无法读取图片：" + str(path))
        self.scene().clear()
        self.scene().addPixmap(QtGui.QPixmap.fromImage(image))
        self.image_rect = QtCore.QRectF(0, 0, image.width(), image.height())
        # Navigation room on every side keeps the cursor's image coordinate
        # fixed during zoom, even when the image initially fits the viewport.
        padding = max(image.width(), image.height()) * 4
        self.scene().setSceneRect(self.image_rect.adjusted(-padding, -padding, padding, padding))
        self.has_image = True
        self.message = ""
        self.reset_view()

    def reset_view(self):
        if self.has_image:
            self.fit_mode = True
            self.fitInView(self.image_rect, QtCore.Qt.KeepAspectRatio)
            self.centerOn(self.image_rect.center())
            self.viewChanged.emit()

    def zoom_by(self, factor, anchor=None):
        if not self.has_image:
            return
        anchor = self.viewport().rect().center() if anchor is None else anchor
        before = self.mapToScene(anchor)
        scale = self.transform().m11()
        target = min(500.0, max(.003, scale * factor))
        if target == scale:
            return
        self.fit_mode = False
        self.scale(target / scale, target / scale)
        after = self.mapToScene(anchor)
        viewport_center = QtCore.QPointF(self.viewport().width()/2, self.viewport().height()/2)
        center = self.viewportTransform().inverted()[0].map(viewport_center)
        self.centerOn(center + before - after)
        self.viewChanged.emit()

    def zoom_in(self):
        self.zoom_by(1.25)

    def zoom_out(self):
        self.zoom_by(.8)

    def wheelEvent(self, event):
        delta = event.pixelDelta().y() or event.angleDelta().y()
        self.zoom_by(math.exp(max(-1400, min(1400, delta))/700), event.pos())
        event.accept()

    def resizeEvent(self, event):
        old_center = self.mapToScene(QtCore.QPoint(event.oldSize().width()//2, event.oldSize().height()//2))
        super().resizeEvent(event)
        if self.fit_mode:
            self.reset_view()
        elif self.has_image:
            self.centerOn(old_center)

    def mousePressEvent(self, event):
        if self.has_image and event.button() == QtCore.Qt.LeftButton:
            self.fit_mode = False
            self.viewChanged.emit()
        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        self.reset_view()
        event.accept()

    def drawForeground(self, painter, rect):
        if self.has_image or not self.message:
            return
        painter.save()
        painter.resetTransform()
        painter.setPen(QtGui.QColor("#74849a"))
        painter.drawText(self.viewport().rect().adjusted(20, 20, -20, -20),
                         QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.message)
        painter.restore()


class CanvasFrame(QtWidgets.QFrame):
    """One rounded surface around each renderer, with no native sunken frame."""
    def __init__(self, canvas):
        super().__init__()
        self.setObjectName("resultCanvasFrame")
        self.setStyleSheet("QFrame#resultCanvasFrame { background: #f0f4f8; border: 1px solid #dce4ed; border-radius: 13px; }")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(1, 1, 1, 1)
        layout.addWidget(canvas)
        self.canvas = canvas
        canvas.installEventFilter(self)

    def eventFilter(self, widget, event):
        if widget is self.canvas and event.type() in (QtCore.QEvent.Resize, QtCore.QEvent.Show):
            path = QtGui.QPainterPath()
            path.addRoundedRect(QtCore.QRectF(widget.rect()), 12, 12)
            widget.setMask(QtGui.QRegion(path.toFillPolygon().toPolygon()))
        return super().eventFilter(widget, event)


class ImageTools(QtWidgets.QWidget):
    def __init__(self, view):
        super().__init__()
        row = QtWidgets.QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        self.minus = button("−", view.zoom_out)
        self.plus = button("＋", view.zoom_in)
        for item, label in ((self.minus, "缩小图片"), (self.plus, "放大图片")):
            item.setAccessibleName(label)
            item.setToolTip(label)
            item.setFixedWidth(38)
            item.setStyleSheet("QPushButton { padding: 4px; min-height: 24px; font-size: 18px; border-radius: 8px; }")
        self.fit = button("适应窗口", view.reset_view)
        self.fit.setStyleSheet("QPushButton { padding: 5px 12px; min-height: 22px; border-radius: 8px; }")
        self.percent = QtWidgets.QLabel()
        self.percent.setObjectName("hint")
        self.percent.setMinimumWidth(90)
        row.addWidget(self.minus)
        row.addWidget(self.plus)
        row.addWidget(self.fit)
        row.addWidget(self.percent)
        row.addStretch()
        self.view = view
        view.viewChanged.connect(self.refresh)
        self.refresh()

    def refresh(self):
        for item in (self.minus, self.plus, self.fit):
            item.setEnabled(self.view.has_image)
        percent = self.view.transform().m11() * 100
        number = "{:.0f}%".format(percent) if percent >= 10 else "{:.1f}%".format(percent)
        self.percent.setText(("适应 · " if self.view.fit_mode else "") + number if self.view.has_image else "暂无图片")


class TrajectoryCanvas(QtWidgets.QWidget):
    geometryChanged = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(270)
        self.points = None
        self.message = "没有可用路线文件"
        self.shared_bounds = None
        self.shared_viewport = None

    def set_shared_view(self, bounds, viewport):
        self.shared_bounds, self.shared_viewport = bounds, viewport
        self.update()

    def projected_points(self):
        low, high = self.shared_bounds if self.shared_bounds is not None else (self.points.min(axis=0), self.points.max(axis=0))
        span = np.maximum(high-low, .01)
        viewport = self.shared_viewport or (max(self.width()-80, 1), max(self.height()-80, 1))
        scale = min(viewport[0]/span[0], viewport[1]/span[1])
        points = (self.points-(low+high)/2)*[scale, -scale]+[self.width()/2, self.height()/2]
        rect = QtCore.QRectF((self.width()-viewport[0])/2, (self.height()-viewport[1])/2, *viewport)
        return points, rect, span

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.geometryChanged.emit()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if self.points is None:
            painter.setPen(QtGui.QColor("#74849a"))
            painter.drawText(self.rect().adjusted(10, 10, -10, -10), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.message)
            return
        points, plot_rect, span = self.projected_points()
        painter.setPen(QtGui.QPen(QtGui.QColor("#e2e8ef"), 1))
        painter.drawRect(plot_rect)
        painter.save()
        painter.setClipRect(plot_rect.adjusted(-6, -6, 6, 6))
        path = QtGui.QPainterPath(QtCore.QPointF(*points[0]))
        stride = max(1, math.ceil(len(points)/50000))
        for point in points[stride::stride]:
            path.lineTo(QtCore.QPointF(*point))
        path.lineTo(QtCore.QPointF(*points[-1]))
        painter.setPen(QtGui.QPen(QtGui.QColor("#307fb5"), 2))
        painter.drawPath(path)
        painter.setBrush(QtGui.QColor("#3fa375"))
        painter.drawEllipse(QtCore.QPointF(*points[0]), 5, 5)
        painter.setBrush(QtGui.QColor("#d66b4e"))
        painter.drawEllipse(QtCore.QPointF(*points[-1]), 5, 5)
        painter.restore()
        painter.setPen(QtGui.QColor("#5c6b7d"))
        painter.drawText(12, self.height()-12, "绿：起点  橙：终点  ·  X {:.2f} m / Y {:.2f} m".format(*span))


def path_string(value):
    return str(value.get("path", value.get("file", ""))) if isinstance(value, dict) else str(value)


class ResultCard(QtWidgets.QFrame):
    routeChanged = QtCore.pyqtSignal()

    def __init__(self, result, parent=None):
        super().__init__(parent)
        self.setObjectName("resultCard")
        self.setStyleSheet("QFrame#resultCard { background: white; border: 1px solid #dce4ed; border-radius: 16px; }")
        self.result = result
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(18, 16, 18, 16)
        self.layout.setSpacing(12)
        title_row = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(result_label(result))
        title.setWordWrap(True)
        title.setStyleSheet("font-size: 16px; font-weight: 600; color: #203c57;")
        title_row.addWidget(title, 1)
        folder = button("打开目录", lambda: local_open(self.result["path"]))
        folder.setToolTip(str(result["path"]))
        folder.setStyleSheet("QPushButton { padding: 5px 10px; min-height: 20px; border-radius: 8px; }")
        title_row.addWidget(folder)
        self.layout.addLayout(title_row)
        path = Path(result["path"])
        path_label = hint("目录：" + str(Path(path.parent.name) / path.name))
        path_label.setToolTip(str(path))
        self.layout.addWidget(path_label)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setObjectName("resultModes")
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setDrawBase(False)
        self.tabs.setStyleSheet("""
            QTabWidget#resultModes::pane { border: none; background: transparent; }
            QTabBar { background: #eef3f8; border: none; border-radius: 11px; }
            QTabBar::tab { background: transparent; color: #62768b; border: none;
                border-radius: 8px; padding: 8px 20px; margin: 4px 3px; min-width: 55px; }
            QTabBar::tab:selected { background: #ffffff; color: #2167a7; font-weight: 600; }
            QTabBar::tab:hover:!selected { background: #e0ebf5; color: #294f73; }
            QTabBar::tab:focus { color: #174f81; }
        """)
        self.layout.addWidget(self.tabs, 1)
        self.cloud_canvas = CloudCanvas()
        self.images = [path_string(p) for p in result.get("images", [])]
        self.trajectories = [path_string(p) for p in result.get("trajectories", [])]
        self.image_roles = {role: path_string(value) for role, value in result.get("image_roles", {}).items() if value}
        # A route screenshot may be listed as an image by older result indexes.
        route_images = [p for p in self.images if any(token in Path(p).stem.lower() for token in ("trajectory", "route", "path", "轨迹", "路线"))]
        if self.image_roles.get("trajectory"):
            route_images.append(self.image_roles["trajectory"])
        self.trajectories = sorted(dict.fromkeys(self.trajectories + route_images), key=lambda p: (Path(p).suffix.lower() != ".csv", p))
        prioritized_maps = [self.image_roles[role] for role in ("grid", "top") if self.image_roles.get(role)]
        self.map_images = list(dict.fromkeys(prioritized_maps + [p for p in self.images if p not in route_images and p != self.image_roles.get("3d")]))
        self.map_page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.map_page)
        layout.setContentsMargins(0, 12, 0, 0)
        layout.setSpacing(10)
        self.map_select = combo([(("栅格地图 · " if p == self.image_roles.get("grid") else "俯视点云 · " if p == self.image_roles.get("top") else "") + Path(p).name, p) for p in self.map_images])
        layout.addWidget(self.map_select)
        self.map_view = ImageView()
        self.map_tools = ImageTools(self.map_view)
        layout.addWidget(self.map_tools)
        layout.addWidget(CanvasFrame(self.map_view), 1)
        self.map_status = hint("双击适应窗口，拖动平移，滚轮缩放。" if self.map_images else "该结果没有导出的 2D 地图图片。")
        layout.addWidget(self.map_status)
        self.tabs.addTab(self.map_page, "2D 地图")
        self.route_page = QtWidgets.QWidget()
        route_layout = QtWidgets.QVBoxLayout(self.route_page)
        route_layout.setContentsMargins(0, 12, 0, 0)
        route_layout.setSpacing(10)
        self.route_select = combo([(Path(p).name, p) for p in self.trajectories])
        route_layout.addWidget(self.route_select)
        self.route_stack = QtWidgets.QStackedWidget()
        self.route_image, self.route_canvas = ImageView(), TrajectoryCanvas()
        self.route_stack.addWidget(self.route_image)
        self.route_stack.addWidget(self.route_canvas)
        self.route_stack.setCurrentWidget(self.route_canvas)
        self.route_tools = ImageTools(self.route_image)
        self.route_tools.hide()
        route_layout.addWidget(self.route_tools)
        route_layout.addWidget(CanvasFrame(self.route_stack), 1)
        self.route_status = hint("路线来自所选结果的导出文件。")
        route_layout.addWidget(self.route_status)
        self.tabs.addTab(self.route_page, "路线")
        self.cloud_page = QtWidgets.QWidget()
        cloud_layout = QtWidgets.QVBoxLayout(self.cloud_page)
        cloud_layout.setContentsMargins(0, 12, 0, 0)
        cloud_layout.setSpacing(10)
        self.cloud_select = combo([(Path(path_string(p)).name, path_string(p)) for p in result.get("clouds", [])])
        cloud_layout.addWidget(self.cloud_select)
        cloud_layout.addWidget(CanvasFrame(self.cloud_canvas), 1)
        self.cloud_status = hint("大点云采用确定性抽样显示，原始导出文件保持不变。")
        cloud_layout.addWidget(self.cloud_status)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(button("复位视角", self.cloud_canvas.reset_view))
        controls.addWidget(button("俯视", self.cloud_canvas.top_view))
        controls.addStretch()
        cloud_layout.addLayout(controls)
        self.tabs.addTab(self.cloud_page, "3D 点云")
        self.map_select.currentIndexChanged.connect(self.load_map)
        self.route_select.currentIndexChanged.connect(self.load_route)
        self.cloud_select.currentIndexChanged.connect(self.load_selected_cloud)
        self.tabs.currentChanged.connect(self.tab_changed)
        self.cloud_loaded = None
        self.cloud_generation = 0
        self.route_generation = 0
        self.load_map()
        self.load_route()

    def load_map(self, *_):
        path = self.map_select.currentData()
        if path:
            try:
                self.map_view.set_image(path)
                self.map_status.setText(Path(path).name + " · 双击复位，拖动平移，滚轮缩放")
            except ValueError as error:
                self.map_status.setText(str(error))

    def load_route(self, *_):
        path = self.route_select.currentData()
        if not path:
            return
        self.route_generation += 1
        generation = self.route_generation
        if Path(path).suffix.lower() in (".png", ".jpg", ".jpeg", ".pgm", ".bmp", ".tif", ".tiff"):
            self.route_canvas.points = None
            self.routeChanged.emit()
            self.route_stack.setCurrentWidget(self.route_image)
            self.route_tools.show()
            try:
                self.route_image.set_image(path)
                self.route_status.setText(Path(path).name + " · 路线图片不含米制坐标，不能保证与其他结果同尺度；优先选择 CSV。")
            except ValueError as error:
                self.route_status.setText(str(error))
        else:
            self.route_tools.hide()
            self.route_stack.setCurrentWidget(self.route_canvas)
            self.route_canvas.points = None
            self.routeChanged.emit()
            self.route_canvas.message = "正在读取路线…"
            self.route_canvas.update()
            BackgroundTask(self, lambda: load_trajectory(path), lambda points: self.route_ready(generation, points),
                           lambda error: self.route_error(generation, error))

    def route_ready(self, generation, points):
        if generation != self.route_generation:
            return
        self.route_canvas.points = points
        self.route_canvas.message = ""
        self.route_canvas.update()
        self.route_status.setText("{} 个路线位置 · 等比例 XY 坐标".format(len(points)))
        self.routeChanged.emit()

    def route_error(self, generation, error):
        if generation == self.route_generation:
            self.route_canvas.message = str(error)
            self.route_canvas.update()
            self.route_status.setText("路线读取失败")
            self.routeChanged.emit()

    def tab_changed(self, index):
        if self.tabs.widget(index) == self.cloud_page:
            self.load_selected_cloud()

    def load_selected_cloud(self, *_):
        path = self.cloud_select.currentData()
        if not path or path == self.cloud_loaded:
            return
        self.cloud_loaded = path
        self.cloud_generation += 1
        generation = self.cloud_generation
        self.cloud_canvas.points = None
        self.cloud_canvas.message = "正在读取真实点云文件…"
        self.cloud_canvas.invalidate(False)
        self.cloud_status.setText("读取：" + str(path))
        BackgroundTask(self, lambda: load_cloud(path), lambda result: self.cloud_ready(generation, result),
                       lambda error: self.cloud_error(generation, error))

    def cloud_ready(self, generation, result):
        if generation != self.cloud_generation:
            return
        self.cloud_canvas.set_cloud(result["points"])
        self.cloud_status.setText("原始 {:,} 点 · 当前显示 {:,} 个真实采样点".format(result["total"], result["shown"]))

    def cloud_error(self, generation, error):
        if generation != self.cloud_generation:
            return
        self.cloud_loaded = None
        self.cloud_canvas.message = str(error)
        self.cloud_canvas.invalidate(False)
        self.cloud_status.setText("点云读取失败")


class ResultsViewer(QtWidgets.QMainWindow):
    def __init__(self, results, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("融合结果对比")
        self.resize(1480, 940)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(22, 20, 22, 20)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(heading("融合结果对比"))
        row.addStretch()
        row.addWidget(QtWidgets.QLabel("统一切换"))
        self.mode = combo([("2D 地图", 0), ("路线", 1), ("3D 点云", 2)])
        self.mode.setFixedWidth(165)
        self.mode.currentIndexChanged.connect(self.change_all_tabs)
        row.addWidget(self.mode)
        self.link = QtWidgets.QCheckBox("联动三维视角")
        row.addWidget(self.link)
        self.uniform_routes = QtWidgets.QCheckBox("路线统一米制比例")
        self.uniform_routes.setChecked(True)
        self.uniform_routes.toggled.connect(self.sync_route_views)
        row.addWidget(self.uniform_routes)
        layout.addLayout(row)
        layout.addWidget(hint("每张卡片可独立查看；联动视角同步旋转、平移和缩放，不代表不同结果已经完成空间配准。"))
        scroll = PanelScrollArea()
        self.scroll = scroll
        content = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(content)
        grid.setContentsMargins(0, 2, 4, 4)
        grid.setSpacing(16)
        self.cards = []
        columns = 1 if len(results) == 1 else 2
        for column in range(columns):
            grid.setColumnStretch(column, 1)
        for index, result in enumerate(results):
            card = ResultCard(result)
            card.setMinimumHeight(530)
            card.cloud_canvas.viewChanged.connect(lambda state, source=card: self.sync_views(source, state))
            card.routeChanged.connect(self.sync_route_views)
            card.route_canvas.geometryChanged.connect(self.sync_route_views)
            grid.addWidget(card, index//columns, index % columns)
            self.cards.append(card)
        scroll.setWidget(content)
        layout.addWidget(scroll, 1)

    def change_all_tabs(self, *_):
        for card in self.cards:
            card.tabs.setCurrentIndex(self.mode.currentData())
        self.sync_route_views()

    def sync_route_views(self, *_):
        canvases = [card.route_canvas for card in self.cards if card.route_canvas.points is not None]
        bounds, viewport = None, None
        if self.uniform_routes.isChecked() and canvases:
            bounds = (np.min([canvas.points.min(axis=0) for canvas in canvases], axis=0),
                      np.max([canvas.points.max(axis=0) for canvas in canvases], axis=0))
            # The shared plot rectangle is identical in pixels even when card
            # labels make surrounding widgets different sizes. One metre has
            # the same scale and the same XY window in every selected route.
            viewport = (max(1, min(canvas.width()-80 for canvas in canvases)),
                        max(1, min(canvas.height()-80 for canvas in canvases)))
        for card in self.cards:
            card.route_canvas.set_shared_view(bounds, viewport)
            if card.route_canvas.points is not None:
                card.route_status.setText("{} 个路线位置 · {}".format(len(card.route_canvas.points),
                    "统一米制比例及 XY 视域" if bounds is not None else "本图独立适应窗口"))

    def sync_views(self, source, state):
        if self.link.isChecked():
            for card in self.cards:
                if card is not source:
                    card.cloud_canvas.set_state(state)
