import copy
import struct
import unittest
import numpy as np

from wc_sensors.pointcloud import validate_pointcloud2, decode_pointcloud2, copy_sdk_points


def cloud(big=False, rows=1):
    data = bytearray()
    for row in range(rows):
        for col in range(2):
            data.extend(struct.pack((">" if big else "<") + "ffff", row + 1, col + 2, 3., 42.))
        data.extend(b"PAD!")
    return {"width": 2, "height": rows, "point_step": 16, "row_step": 36,
            "is_bigendian": big, "data": data,
            "fields": [{"name": n, "offset": i * 4, "datatype": 7, "count": 1}
                       for i, n in enumerate(("x", "y", "z", "intensity"))]}


class PointCloudTests(unittest.TestCase):
    def test_big_and_little_endian_and_row_padding(self):
        for endian in (False, True):
            message = cloud(endian, 2)
            points = decode_pointcloud2(message)
            np.testing.assert_allclose(points, [[1, 2, 3], [1, 3, 3], [2, 2, 3], [2, 3, 3]])
            self.assertTrue(points.flags.owndata)
            self.assertEqual(validate_pointcloud2(message)["point_count"], 4)

    def test_callback_storage_reuse(self):
        message = cloud()
        points = decode_pointcloud2(message)
        message["data"][:] = b"\0" * len(message["data"])
        np.testing.assert_allclose(points[0], [1, 2, 3])

    def test_duplicate_or_missing_xyz_rejected(self):
        for modify in (lambda m: m["fields"].append(copy.deepcopy(m["fields"][0])),
                       lambda m: m["fields"].pop(0)):
            message = cloud()
            modify(message)
            with self.assertRaises(ValueError):
                validate_pointcloud2(message)

    def test_overlapping_fields_and_point_overrun(self):
        for offset in (2, 16):
            message = cloud()
            message["fields"][-1]["offset"] = offset
            with self.assertRaises(ValueError):
                validate_pointcloud2(message)

    def test_truncated_data_or_too_short_stride(self):
        message = cloud()
        message["data"].pop()
        with self.assertRaises(ValueError):
            validate_pointcloud2(message)
        message = cloud()
        message["row_step"] = 8
        with self.assertRaises(ValueError):
            validate_pointcloud2(message)

    def test_xyz_requires_scalar_float(self):
        for datatype, count in ((5, 1), (7, 0), (7, 2), (99, 1)):
            message = cloud()
            message["fields"][0].update(datatype=datatype, count=count)
            with self.assertRaises(ValueError):
                validate_pointcloud2(message)

    def test_invalid_numeric_types(self):
        for name, value in (("width", True), ("height", 1.0), ("is_bigendian", 0)):
            message = cloud()
            message[name] = value
            with self.assertRaises(TypeError):
                validate_pointcloud2(message)

    def test_empty_cloud_is_well_formed(self):
        message = cloud()
        message.update(width=0, row_step=0, data=b"")
        self.assertEqual(decode_pointcloud2(message).shape, (0, 3))

    def test_non_contiguous_sdk_view_becomes_owning_copy(self):
        storage = np.arange(30).reshape(5, 6)
        snapshot = copy_sdk_points(storage[:, ::2])
        storage[:] = 0
        np.testing.assert_array_equal(snapshot[1], [6, 8, 10])
        self.assertTrue(snapshot.flags.c_contiguous)
        self.assertTrue(snapshot.flags.owndata)


if __name__ == "__main__":
    unittest.main()
