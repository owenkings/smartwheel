"""Explicit offline sensor inputs. ROS imports are lazy and no node is started.

Humble API references are recorded in handoffs/calibration.md. Cloud parsing
preserves organized row padding and endian layout; it does not infer units,
physical extrinsics, synchronization, or static-scene validity.
"""

from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .core import CalibrationError, _voxel


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _ns(value):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise CalibrationError("timestamps and bounds must be integer nanoseconds")
    return int(value)


def _stamp(value):
    sec, nanosec = _ns(value.sec), _ns(value.nanosec)
    if not 0 <= nanosec < 1_000_000_000:
        raise CalibrationError("ROS Time nanosec is not normalized")
    return sec * 1_000_000_000 + nanosec


def _unit_scale(units):
    if units == "m":
        return 1.0
    if units == "mm":
        return 0.001
    raise CalibrationError("explicit supported cloud units are m or mm; unknown units cannot be inferred")


def _clean_cloud(points, units, metadata):
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    valid = np.isfinite(values).all(axis=1)
    cleaned = values[valid] * _unit_scale(units)
    if len(cleaned) < 3:
        raise CalibrationError("cloud has fewer than three finite xyz points")
    return {"points": cleaned, "metadata": {**metadata, "original_units": units, "output_units": "m",
            "raw_count": len(values), "finite_count": len(cleaned),
            "discarded_nonfinite_count": int((~valid).sum()), "finite_indices": np.flatnonzero(valid).tolist()}}


def _read_pcd(stream, max_points):
    header = {}
    for _ in range(1000):
        line = stream.readline()
        if not line:
            raise CalibrationError("truncated PCD header")
        parts = line.strip().split()
        if not parts or parts[0].startswith("#"):
            continue
        key = parts[0].upper()
        if key in header:
            raise CalibrationError("duplicate PCD header entry")
        header[key] = parts[1:]
        if key == "DATA":
            break
    else:
        raise CalibrationError("PCD header exceeds bounded size")
    if header.get("DATA") != ["ascii"]:
        raise CalibrationError("only PCD DATA ascii is supported; binary is never interpreted as text")
    fields = header.get("FIELDS", [])
    if len(set(fields)) != len(fields) or not all(name in fields for name in ("x", "y", "z")):
        raise CalibrationError("PCD needs unique xyz fields")
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    if len(counts) != len(fields) or len(sizes) != len(fields) or len(types) != len(fields) or \
            any(value <= 0 for value in counts) or any(value not in (1, 2, 4, 8) for value in sizes) or \
            any(value not in {"F", "I", "U"} for value in types):
        raise CalibrationError("invalid PCD field sizes/types/counts")
    if any(counts[fields.index(name)] != 1 for name in ("x", "y", "z")):
        raise CalibrationError("PCD xyz must each be scalar")
    width, height = int(header["WIDTH"][0]), int(header["HEIGHT"][0])
    total = int(header.get("POINTS", [str(width * height)])[0])
    if width < 1 or height < 1 or total != width * height or not 3 <= total <= max_points:
        raise CalibrationError("PCD dimensions/count exceed bounds or disagree")
    starts = np.cumsum([0] + counts[:-1])
    xyz_columns = [int(starts[fields.index(name)]) for name in ("x", "y", "z")]
    points = []
    for line in stream:
        parts = line.strip().split()
        if not parts or parts[0].startswith("#"):
            continue
        if len(parts) != sum(counts) or len(points) >= total:
            raise CalibrationError("PCD data row/count disagrees with header")
        points.append([float(parts[index]) for index in xyz_columns])
    if len(points) != total:
        raise CalibrationError("truncated PCD data")
    return points, {"format": "pcd_ascii", "fields": fields, "width": width, "height": height,
                    "viewpoint_raw": header.get("VIEWPOINT"), "viewpoint_applied": False}


_PLY_SCALARS = {"char", "uchar", "short", "ushort", "int", "uint", "float", "double",
                "int8", "uint8", "int16", "uint16", "int32", "uint32", "float32", "float64"}


def _read_ply(stream, max_points):
    if stream.readline().strip() != "ply":
        raise CalibrationError("missing PLY signature")
    elements, encoding = [], None
    for _ in range(1000):
        line = stream.readline()
        if not line:
            raise CalibrationError("truncated PLY header")
        parts = line.strip().split()
        if not parts or parts[0] in {"comment", "obj_info"}:
            continue
        if parts[0] == "end_header":
            break
        if parts[0] == "format":
            encoding = parts[1:]
        elif parts[0] == "element" and len(parts) == 3:
            count = int(parts[2])
            if count < 0 or count > max_points:
                raise CalibrationError("PLY element exceeds bounded count")
            elements.append({"name": parts[1], "count": count, "properties": []})
        elif parts[0] == "property" and elements:
            if len(parts) == 3 and parts[1] in _PLY_SCALARS:
                prop = {"name": parts[2], "type": parts[1], "list": False}
            elif len(parts) == 5 and parts[1] == "list" and all(t in _PLY_SCALARS for t in parts[2:4]):
                prop = {"name": parts[4], "type": parts[3], "list": True}
            else:
                raise CalibrationError("unsupported PLY property declaration")
            if any(old["name"] == prop["name"] for old in elements[-1]["properties"]):
                raise CalibrationError("duplicate PLY property")
            elements[-1]["properties"].append(prop)
        else:
            raise CalibrationError("unsupported or misplaced PLY header entry")
    else:
        raise CalibrationError("PLY header exceeds bounded size")
    if encoding != ["ascii", "1.0"]:
        raise CalibrationError("only PLY format ascii 1.0 is supported")
    vertices = [element for element in elements if element["name"] == "vertex"]
    if len(vertices) != 1 or vertices[0]["count"] < 3:
        raise CalibrationError("PLY needs exactly one nonempty vertex element")
    xyz = {prop["name"] for prop in vertices[0]["properties"] if not prop["list"]}
    if not all(name in xyz for name in ("x", "y", "z")):
        raise CalibrationError("PLY needs scalar vertex xyz properties")
    points = []
    for element in elements:
        for _ in range(element["count"]):
            line = stream.readline()
            if not line:
                raise CalibrationError("truncated PLY element data")
            fields, cursor, values = line.split(), 0, {}
            for prop in element["properties"]:
                if cursor >= len(fields):
                    raise CalibrationError("truncated PLY property data")
                if prop["list"]:
                    length = int(fields[cursor])
                    if length < 0 or length > max_points or cursor + length + 1 > len(fields):
                        raise CalibrationError("invalid PLY list property length")
                    cursor += length + 1
                else:
                    if element["name"] == "vertex" and prop["name"] in {"x", "y", "z"}:
                        values[prop["name"]] = float(fields[cursor])
                    cursor += 1
            if cursor != len(fields):
                raise CalibrationError("PLY row has unexpected trailing values")
            if element["name"] == "vertex":
                points.append([values[name] for name in ("x", "y", "z")])
    if any(line.strip() for line in stream):
        raise CalibrationError("PLY contains undeclared trailing data")
    return points, {"format": "ply_ascii", "elements": elements}


def read_ascii_cloud(path, *, units, coordinate_convention, max_points=200_000, max_bytes=128_000_000):
    """Read a named file only. The supplied axis convention is preserved, never inferred."""
    path = Path(path)
    if not isinstance(coordinate_convention, str) or not coordinate_convention.strip():
        raise CalibrationError("explicit cloud axis convention is required")
    if not path.is_file() or path.stat().st_size > max_bytes:
        raise CalibrationError("cloud file missing or exceeds bounded file size")
    try:
        with path.open("r", encoding="ascii") as stream:
            if path.suffix.lower() == ".pcd":
                points, metadata = _read_pcd(stream, max_points)
            elif path.suffix.lower() == ".ply":
                points, metadata = _read_ply(stream, max_points)
            else:
                raise CalibrationError("only explicitly named ASCII .pcd/.ply inputs supported")
    except (UnicodeError, KeyError, IndexError) as error:
        raise CalibrationError(f"invalid ASCII cloud: {error}") from error
    return _clean_cloud(points, units, {**metadata, "path": str(path.resolve()),
                         "sha256": _sha256(path), "coordinate_convention": coordinate_convention,
                         "static_validated": False, "time_validated": False})


def prepare_file_dataset(manifest, base_dir):
    """Materialize explicit paired file scenes into the frozen numerical JSON schema."""
    if manifest.get("schema_version") != 1 or manifest.get("source_mode") not in {"real", "synthetic"}:
        raise CalibrationError("file manifest needs v1 and explicit source_mode")
    output = {key: manifest[key] for key in ("schema_version", "source_mode", "sensor_ids")}
    output["initial_T_left_right"] = manifest.get("initial_T_left_right")
    output["input_preparation"] = {"kind": "explicit_ascii_files", "status": "PREPARED_NOT_VALIDATED",
                                    "static_review_required": True, "automatic_static_evidence": False}
    for split in ("training", "validation"):
        output[split] = []
        for entry in manifest.get(split, []):
            scene = {"id": entry["id"], "input_files": {}}
            conventions = []
            for side in ("left", "right"):
                spec = entry[side]
                path = Path(spec["path"])
                if not path.is_absolute():
                    path = Path(base_dir) / path
                loaded = read_ascii_cloud(path, units=spec["units"], coordinate_convention=spec["coordinate_convention"])
                scene[side] = loaded["points"].tolist()
                scene["input_files"][side] = loaded["metadata"]
                conventions.append(spec["coordinate_convention"])
            if conventions[0] != conventions[1]:
                raise CalibrationError("left/right axis conventions differ; apply a reviewed explicit axis conversion first")
            output[split].append(scene)
    # Do not copy user assertions into automatic evidence. A separate review may
    # add real static provenance before calibration promotion.
    return output


def pointcloud2_xyz(cloud, *, max_points=200_000):
    """PointCloud2 xyz with explicit offsets, row_step and endian handling; duck-typed for unit tests."""
    width, height, step, row = map(int, (cloud.width, cloud.height, cloud.point_step, cloud.row_step))
    if width < 1 or height < 1 or not 3 <= width * height <= max_points or step < 1 or row < width * step:
        raise CalibrationError("invalid PointCloud2 dimensions/stride")
    payload = memoryview(bytes(cloud.data))
    if len(payload) != row * height:
        raise CalibrationError("PointCloud2 data length disagrees with row_step*height")
    fields = {field.name: field for field in cloud.fields}
    if len(fields) != len(cloud.fields) or not all(name in fields for name in ("x", "y", "z")):
        raise CalibrationError("PointCloud2 needs unique xyz fields")
    datatype = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    axes = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.count != 1 or field.datatype not in datatype:
            raise CalibrationError("PointCloud2 xyz field must be a supported scalar")
        dtype = np.dtype((">" if cloud.is_bigendian else "<") + datatype[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > step:
            raise CalibrationError("PointCloud2 field extends outside point_step")
        array = np.ndarray((height, width), dtype=dtype, buffer=payload, offset=int(field.offset), strides=(row, step))
        axes.append(array.reshape(-1).astype(float))
    return np.column_stack(axes)


def source_frame_record(message, bag_record_time_ns, *, expected_side=None):
    """Freeze every SourceFrame metadata field required to interpret the selected cloud."""
    side = str(message.side)
    if side not in {"left", "right"} or (expected_side is not None and side != expected_side):
        raise CalibrationError("SourceFrame side disagrees with selected topic")
    for key in ("session_id", "sensor_id", "stream_epoch", "source_config_hash", "coordinate_convention"):
        if not str(getattr(message, key)).strip():
            raise CalibrationError(f"SourceFrame lacks authoritative {key}")
    raw_key = [str(message.session_id), str(message.sensor_id), str(message.stream_epoch), int(message.frame_sequence)]
    raw_points = pointcloud2_xyz(message.cloud)
    loaded = _clean_cloud(raw_points, str(message.units), {})
    return {"raw_key": raw_key, "side": side, "session_id": str(message.session_id),
            "sensor_id": str(message.sensor_id), "stream_epoch": str(message.stream_epoch),
            "frame_sequence": int(message.frame_sequence), "points": loaded["points"],
            "source_config_hash": str(message.source_config_hash),
            "coordinate_convention": str(message.coordinate_convention), "units": "m",
            "source_units": str(message.units), "header_frame_id": str(message.header.frame_id),
            "header_stamp_ns": _stamp(message.header.stamp), "cloud_frame_id": str(message.cloud.header.frame_id),
            "cloud_stamp_ns": _stamp(message.cloud.header.stamp),
            "device_timestamp_valid": bool(message.device_timestamp_valid),
            "device_timestamp_raw": int(message.device_timestamp_raw),
            "device_timestamp_unit": str(message.device_timestamp_unit),
            "device_time_type": str(message.device_time_type), "device_sync_state": str(message.device_sync_state),
            "host_receive_time_ns": _stamp(message.host_receive_time), "host_monotonic_ns": int(message.host_monotonic_ns),
            "common_time_valid": bool(message.common_time_valid), "common_time_ns": int(message.common_time_ns),
            "clock_model_id": str(message.clock_model_id), "time_source": str(message.time_source),
            "uncertainty_valid": bool(message.uncertainty_valid),
            "uncertainty_ns": int(message.uncertainty_ns) if message.uncertainty_valid else None,
            "reported_raw_count": int(message.raw_count), "reported_valid_count": int(message.valid_count),
            "cloud_data_sha256": hashlib.sha256(bytes(message.cloud.data)).hexdigest(),
            "diagnostic_flags": list(message.diagnostic_flags), "filtering": loaded["metadata"],
            "bag_record_time_ns": _ns(bag_record_time_ns)}


def iter_rosbag_source_records(bag_path, topics, *, storage_id="sqlite3", start_ns=None, end_ns=None, max_frames=1200):
    """Read only two explicitly whitelisted SourceFrame topics using verified Humble API."""
    if set(topics) != {"left", "right"} or topics["left"] == topics["right"]:
        raise CalibrationError("two distinct explicitly selected left/right topics required")
    if max_frames < 1:
        raise CalibrationError("positive bounded max_frames required")
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except (ImportError, OSError) as error:
        raise CalibrationError("rosbag2/rclpy/type support unavailable; source verified target Humble environment") from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    available = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    for topic in topics.values():
        if topic not in available or available[topic].type != "wc_interfaces/msg/SourceFrame":
            raise CalibrationError(f"selected topic missing or not authoritative SourceFrame: {topic}")
        if getattr(available[topic], "serialization_format", "cdr") != "cdr":
            raise CalibrationError("selected topic serialization is not CDR")
    message_type = get_message("wc_interfaces/msg/SourceFrame")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics.values())))
    if start_ns is not None:
        reader.seek(_ns(start_ns))
    sides = {topic: side for side, topic in topics.items()}
    count = 0
    while reader.has_next():
        topic, serialized, record_time = reader.read_next()
        if topic not in sides:
            raise CalibrationError("rosbag storage filter returned a nonwhitelisted topic")
        if end_ns is not None and record_time > _ns(end_ns):
            break
        if start_ns is not None and record_time < _ns(start_ns):
            continue
        count += 1
        if count > max_frames:
            raise CalibrationError("bounded bag frame budget exceeded; select shorter intervals or raise explicit budget")
        yield source_frame_record(deserialize_message(serialized, message_type), int(record_time), expected_side=sides[topic])


def iter_paired_records(records, *, max_pair_ns=50_000_000, time_basis="common", max_queue=12,
                        max_frames=100_000, diagnostics=None):
    """Exactly-once bounded offline pairing. Host arrival is explicit exploratory use only."""
    if time_basis not in {"common", "host_receive"} or _ns(max_pair_ns) < 0 or _ns(max_queue) < 1 or _ns(max_frames) < 1:
        raise CalibrationError("invalid pairing policy")
    counters = diagnostics if diagnostics is not None else Counter()
    queues, active, old_epochs, seen, session = {"left": [], "right": []}, {}, {"left": set(), "right": set()}, set(), None
    for frame_index, frame in enumerate(records):
        if frame_index >= max_frames:
            raise CalibrationError("bounded offline pairing frame budget exceeded")
        side = frame["side"]
        if side not in queues:
            raise CalibrationError("invalid source side")
        if session is None:
            session = frame["session_id"]
        if frame["session_id"] != session:
            raise CalibrationError("do not pair across recording sessions")
        key = tuple(frame["raw_key"])
        if key in seen:
            raise CalibrationError("duplicate authoritative raw frame key")
        seen.add(key)
        stream = (frame["stream_epoch"], frame["clock_model_id"], frame["source_config_hash"])
        if stream in old_epochs[side]:
            counters["old_stream_late"] += 1
            continue
        if side in active and stream != active[side]:
            old_epochs[side].add(active[side])
            counters["stream_boundary_flush"] += len(queues["left"]) + len(queues["right"])
            queues = {"left": [], "right": []}
        active[side] = stream
        if time_basis == "common":
            if not frame["common_time_valid"] or not frame["clock_model_id"]:
                counters["invalid_common_time"] += 1
                continue
            stamp = _ns(frame["common_time_ns"])
        else:
            stamp = _ns(frame["host_receive_time_ns"])
        frame = {**frame, "pairing_stamp_ns": stamp}
        queues[side].append(frame)
        queues[side].sort(key=lambda value: (value["pairing_stamp_ns"], value["frame_sequence"]))
        if len(queues[side]) > max_queue:
            queues[side].pop(0)
            counters["queue_overflow"] += 1
        while queues["left"] and queues["right"]:
            left, right = queues["left"][0], queues["right"][0]
            delta = left["pairing_stamp_ns"] - right["pairing_stamp_ns"]
            if abs(delta) > max_pair_ns:
                queues["left" if delta < 0 else "right"].pop(0)
                counters["outside_pair_window"] += 1
                continue
            if left["sensor_id"] == right["sensor_id"]:
                raise CalibrationError("paired sources share the same device identity")
            if left["coordinate_convention"] != right["coordinate_convention"]:
                raise CalibrationError("paired cloud axis conventions differ")
            queues["left"].pop(0)
            queues["right"].pop(0)
            counters["paired"] += 1
            yield {"left": left, "right": right, "t_ref_ns": max(left["pairing_stamp_ns"], right["pairing_stamp_ns"]),
                   "pair_dt_ns": abs(delta), "time_basis": time_basis,
                   "time_quality": "metadata_common_time" if time_basis == "common" else "ARRIVAL_ONLY_NOT_SYNCHRONIZED",
                   "static_validated": False}
    counters["unmatched_end"] += len(queues["left"]) + len(queues["right"])


def _cloud_change(a, b, voxel_m, sample_limit):
    clouds = [_voxel(np.asarray(value, float), voxel_m) for value in (a, b)]
    clouds = [value[np.linspace(0, len(value) - 1, min(sample_limit, len(value)), dtype=int)] for value in clouds]
    if min(map(len, clouds)) < 10:
        return None
    ab, _ = cKDTree(clouds[1]).query(clouds[0])
    ba, _ = cKDTree(clouds[0]).query(clouds[1])
    return max(float(np.quantile(ab, .90)), float(np.quantile(ba, .90)))


def suggest_static_segments(pairs, *, min_pairs=5, min_duration_ns=1_000_000_000,
                            max_gap_ns=400_000_000, change_p90_m=0.04, voxel_m=0.035,
                            sample_limit=1200, max_pairs=200):
    """Conservative shape-stability suggestions, never proof of zero six-DoF motion."""
    if min_pairs < 2 or _ns(min_duration_ns) < 0 or _ns(max_gap_ns) <= 0 or change_p90_m <= 0 or voxel_m <= 0 or sample_limit < 10:
        raise CalibrationError("invalid static suggestion policy")
    pairs = list(pairs)
    if len(pairs) > max_pairs:
        raise CalibrationError("static suggestion exceeds explicit pair budget")
    segments, links, start = [], [], 0
    def finish(stop):
        if stop - start >= min_pairs and pairs[stop-1]["t_ref_ns"] - pairs[start]["t_ref_ns"] >= min_duration_ns:
            segments.append({"start_pair_index": start, "end_pair_index": stop-1, "pair_count": stop-start,
                             "start_ns": pairs[start]["t_ref_ns"], "end_ns": pairs[stop-1]["t_ref_ns"],
                             "status": "STATIC_CANDIDATE_REQUIRES_REVIEW", "confidence_probability": None})
    for index in range(1, len(pairs)):
        earlier, current, anchor = pairs[index-1], pairs[index], pairs[start]
        gap = current["t_ref_ns"] - earlier["t_ref_ns"]
        same_stream = all(current[side]["raw_key"][:3] == earlier[side]["raw_key"][:3] and
                          current[side]["source_config_hash"] == earlier[side]["source_config_hash"] and
                          current[side]["clock_model_id"] == earlier[side]["clock_model_id"]
                          for side in ("left", "right"))
        changes = {"left": None, "right": None}
        if same_stream and 0 < gap <= max_gap_ns:
            for side in ("left", "right"):
                values = [_cloud_change(earlier[side]["points"], current[side]["points"], voxel_m, sample_limit),
                          _cloud_change(anchor[side]["points"], current[side]["points"], voxel_m, sample_limit)]
                changes[side] = max(values) if all(value is not None for value in values) else None
        stable = same_stream and 0 < gap <= max_gap_ns and all(value is not None and value <= change_p90_m for value in changes.values())
        links.append({"pair_index": index, "stable_candidate": bool(stable), "shape_change_p90_m": changes})
        if not stable:
            finish(index)
            start = index
    if pairs:
        finish(len(pairs))
    return {"status": "SUGGESTION_ONLY", "static_validated": False, "segments": segments, "links": links,
            "policy": {"min_pairs": min_pairs, "min_duration_ns": min_duration_ns, "max_gap_ns": max_gap_ns,
                       "change_p90_m": change_p90_m, "voxel_m": voxel_m, "sample_limit": sample_limit, "max_pairs": max_pairs},
            "limitations": ["shape stability does not prove zero motion in unobservable or repeated geometry",
                            "arrival pairing does not establish exposure synchronization",
                            "an operator must review scene stillness and independent training/holdout captures"]}


def prepare_bag_dataset(selection, *, record_iterator=None):
    """Use explicit nonoverlapping bag-record windows; output one authoritative pair per scene."""
    if selection.get("schema_version") != 1 or selection.get("source_mode") not in {"real", "synthetic"}:
        raise CalibrationError("bag selection requires v1 and explicit source_mode")
    windows = []
    for split in ("training", "validation"):
        for entry in selection.get(split, []):
            lower, upper = _ns(entry["start_record_ns"]), _ns(entry["end_record_ns"])
            if lower >= upper:
                raise CalibrationError("scene window start must be before end")
            windows.append({"split": split, "id": entry["id"], "start": lower, "end": upper, "pairs": []})
    windows.sort(key=lambda value: value["start"])
    if not windows or len({value["id"] for value in windows}) != len(windows) or \
            any(a["end"] >= b["start"] for a, b in zip(windows, windows[1:])):
        raise CalibrationError("training/validation scene IDs and bag windows must be independent and disjoint")
    if record_iterator is None:
        record_iterator = iter_rosbag_source_records(selection["bag_uri"], selection["topics"],
            storage_id=selection.get("storage_id", "sqlite3"), start_ns=windows[0]["start"], end_ns=windows[-1]["end"],
            max_frames=int(selection.get("max_frames", 1200)))
    counts = Counter()
    max_pairs = int(selection.get("max_pairs_per_scene", 40))
    max_retained_points = int(selection.get("max_retained_points", 4_000_000))
    if max_pairs < 1 or max_retained_points < 1:
        raise CalibrationError("explicit positive retained-data budgets required")
    retained = 0
    for pair in iter_paired_records(record_iterator, max_pair_ns=selection.get("max_pair_ns", 50_000_000),
                                    time_basis=selection.get("time_basis", "common"), diagnostics=counts):
        for window in windows:
            if all(window["start"] <= pair[side]["bag_record_time_ns"] <= window["end"] for side in ("left", "right")):
                if any(pair[side]["sensor_id"] != selection["sensor_ids"][side] for side in ("left", "right")):
                    raise CalibrationError("bag device identity disagrees with explicit selection")
                retained += sum(len(pair[side]["points"]) for side in ("left", "right"))
                if len(window["pairs"]) >= max_pairs or retained > max_retained_points:
                    raise CalibrationError("selected scenes exceed retained-data budget; narrow intervals explicitly")
                window["pairs"].append(pair)
                break
    output = {key: selection[key] for key in ("schema_version", "source_mode", "sensor_ids")}
    output.update(training=[], validation=[], initial_T_left_right=selection.get("initial_T_left_right"))
    for window in windows:
        if not window["pairs"]:
            raise CalibrationError(f"no authoritative pair in selected scene: {window['id']}")
        report = suggest_static_segments(window["pairs"], **selection.get("static_suggestion_policy", {}))
        center = (window["start"] + window["end"]) // 2
        pair = min(window["pairs"], key=lambda value: abs(max(value[side]["bag_record_time_ns"] for side in ("left", "right")) - center))
        output[window["split"]].append({"id": window["id"], "left": pair["left"]["points"].tolist(),
            "right": pair["right"]["points"].tolist(), "selected_raw_frames": {
                side: {key: value for key, value in pair[side].items() if key != "points"} for side in ("left", "right")},
            "bag_record_window_ns": [window["start"], window["end"]], "pair_dt_ns": pair["pair_dt_ns"],
            "time_quality": pair["time_quality"], "static_suggestion": report})
    output["input_preparation"] = {"kind": "rosbag2_source_frame", "status": "PREPARED_NOT_VALIDATED",
        "bag_uri": selection.get("bag_uri"), "topics": selection.get("topics"), "pairing_diagnostics": dict(counts),
        "static_review_required": True, "automatic_static_evidence": False,
        "time_basis": selection.get("time_basis", "common"),
        "selection_sha256": hashlib.sha256(json.dumps(selection, sort_keys=True).encode()).hexdigest()}
    return output
