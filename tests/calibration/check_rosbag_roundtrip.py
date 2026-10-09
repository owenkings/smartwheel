"""Explicit SYNTHETIC ROS bag/type-support integration test, never hardware.

Run after wc_interfaces is built and the verified target Humble setup is sourced:
python3 tests/calibration/check_rosbag_roundtrip.py --output-dir <new-test-directory>
No skip path: missing ROS/type support or any failed assertion exits nonzero.
"""

import argparse
import gc
import hashlib
import json
from pathlib import Path
import struct

from wc_calibration.importers import iter_paired_records, iter_rosbag_source_records


def run(output_dir):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from wc_interfaces.msg import SourceFrame

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    bag = output_dir / "synthetic_source_frames"
    topics = {side: f"/wc_mapping/lidar_{side}/source_frame" for side in ("left", "right")}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    for topic in topics.values():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="wc_interfaces/msg/SourceFrame", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/synthetic/unrelated_text", type="std_msgs/msg/String", serialization_format="cdr"))
    expected = {}
    for sequence in range(8):
        for side in ("left", "right"):
            stamp = 5_000_000_000 + sequence*250_000_000 + (1_000_000 if side == "right" else 0)
            message = SourceFrame()
            message.header.frame_id = f"lidar_{side}"
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            message.session_id = "SYNTHETIC-CALIBRATION-READER"
            message.side, message.sensor_id, message.stream_epoch = side, "SYN-"+side, "SYN-boot0"
            message.frame_sequence = sequence
            cloud = PointCloud2()
            cloud.header = message.header
            cloud.width, cloud.height, cloud.point_step, cloud.row_step = 2, 2, 16, 40
            cloud.is_bigendian = side == "right"
            cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate("xyz")]
            payload = bytearray()
            for row in range(2):
                for column in range(2):
                    value = float(row*2+column+1)
                    payload.extend(struct.pack((">" if cloud.is_bigendian else "<") + "ffff", value, value+1, value+2, 99.))
                payload.extend(b"padding!")
            cloud.data = bytes(payload)
            message.cloud = cloud
            message.source_config_hash, message.coordinate_convention, message.units = "SYN-cfg-"+side, "FLU", "m"
            message.device_timestamp_valid = False
            message.device_timestamp_unit, message.device_time_type, message.device_sync_state = "UNKNOWN", "UNKNOWN", "UNKNOWN"
            receive = stamp + 2_000_000
            message.host_receive_time.sec, message.host_receive_time.nanosec = divmod(receive, 1_000_000_000)
            message.host_monotonic_ns = sequence*250_000_000 + 1
            message.common_time_valid, message.common_time_ns = True, stamp
            message.clock_model_id, message.time_source = "SYN-clock-"+side, "synthetic_truth"
            message.uncertainty_valid, message.uncertainty_ns = False, 0
            message.raw_count, message.valid_count = 4, 4
            message.diagnostic_flags = ["SYNTHETIC_TEST_ONLY"]
            expected[(side, sequence)] = stamp
            writer.write(topics[side], serialize_message(message), receive+1_000_000)
        text = String()
        text.data = "This unrelated topic must not be deserialized by calibration input."
        writer.write("/synthetic/unrelated_text", serialize_message(text), 5_100_000_000+sequence*250_000_000)
    del writer
    gc.collect()
    def hashes():
        return {file.name: hashlib.sha256(file.read_bytes()).hexdigest() for file in bag.iterdir()
                if file.is_file() and file.suffix in {".db3", ".yaml"}}
    before = hashes()
    records = list(iter_rosbag_source_records(bag, topics, max_frames=20))
    assert len(records) == 16
    for record in records:
        assert record["common_time_ns"] == expected[(record["side"], record["frame_sequence"])]
        assert record["points"].tolist() == [[1., 2., 3.], [2., 3., 4.], [3., 4., 5.], [4., 5., 6.]]
        assert record["uncertainty_ns"] is None
        assert record["diagnostic_flags"] == ["SYNTHETIC_TEST_ONLY"]
    pairs = list(iter_paired_records(records))
    assert len(pairs) == 8 and all(pair["pair_dt_ns"] == 1_000_000 for pair in pairs)
    assert before == hashes(), "read-only bag import changed existing database/metadata bytes"
    evidence = {"level": "SYNTHETIC", "status": "PASS", "test": "real_Humble_SourceFrame_CDR_rosbag2_roundtrip",
                "bag_path": str(bag.resolve()), "source_records": len(records), "dual_pairs": len(pairs),
                "data_hashes": before, "physical_hardware_used": False, "navigation_started": False}
    (output_dir / "evidence.json").write_text(json.dumps(evidence, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(evidence))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    run(parser.parse_args().output_dir)
