"""PCD binary_compressed wire-format and bounded-reader regressions."""
import io
import base64
import binascii
import lzma
import os
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
from wc_panel.results_viewer import _lzf_chunks, load_cloud


def literals(raw):
    return b"".join(bytes([len(raw[i:i+32])-1])+raw[i:i+32] for i in range(0, len(raw), 32))


def header(n):
    return ("VERSION .7\nFIELDS z descriptor x y\nSIZE 4 2 8 4\nTYPE F U F I\n"
            "COUNT 1 3 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary_compressed\n" % (n,n)).encode()


class PcdLzfTests(unittest.TestCase):
    def test_target_pcl_library_independent_compressed_vector(self):
        # Produced by Orin libpcl_io.so pcl::lzfCompress on 2026-10-08.
        packed=base64.b64decode(
            "HwABAgMEBQYHCAkKCwwNDg8QERITFBUWFxgZGhscHR4fHyAhIiMkJSYnKCkqKywtLi8wMTIzNDU2Nzg5Ojs8PT4/H0BBQkNERUZHSElKS0xNTk9QUVJTVFVWV1hZWltcXV5fH2BhYmNkZWZnaGlqa2xtbm9wcXJzdHV2d3h5ent8fX5/H4CBgoOEhYaHiImKi4yNjo+QkZKTlJWWl5iZmpucnZ6fH6ChoqOkpaanqKmqq6ytrq+wsbKztLW2t7i5uru8vb6/H8DBwsPExcbHyMnKy8zNzs/Q0dLT1NXW19jZ2tvc3d7fH+Dh4uPk5ebn6Onq6+zt7u/w8fLz9PX29/j5+vv8/f7/AADg///h///i///j///k///l///m///n///o///p///q///r///s///t///u///v///w///x///y///z///0///1///2///3///4///5///6///7///8///9///+//////8GAQIDBAUGB//////////f/wBh4P8A4H8AAWJj4P8C4f8H4ToHAWJj")
        expected=bytes(range(256))*37+b"a"*400+b"abcabc"*100
        self.assertEqual(b"".join(_lzf_chunks(io.BytesIO(packed),len(packed),len(expected))),expected)

    def test_lzf_overlapping_extended_and_max_distance_references(self):
        # Literal 'ab' then a distance-two overlapping match of nine bytes.
        packed = b"\x01ab\xe0\x00\x01"
        self.assertEqual(b"".join(_lzf_chunks(io.BytesIO(packed), len(packed), 11)), b"abababababa")
        prefix = bytes(range(256))*32
        packed = literals(prefix)+b"\xff\xff\xff"  # distance 8192, length 264
        self.assertEqual(b"".join(_lzf_chunks(io.BytesIO(packed),len(packed),8456)),prefix+prefix[:264])

    def test_lzf_history_survives_stream_chunk_flush(self):
        raw = bytes(range(256))*257
        packed = literals(raw)+b"\xff\xff\xff"
        expected = raw+raw[-8192:-8192+264]
        self.assertEqual(b"".join(_lzf_chunks(io.BytesIO(packed),len(packed),len(expected))),expected)

    def test_compressed_soa_multi_count_mixed_types_sampling_plain_and_xz(self):
        n=17003
        x=np.arange(n,dtype="<f8")*.125
        y=-np.arange(n,dtype="<i4")
        z=np.arange(n,dtype="<f4")*.5
        fields=z.tobytes()+np.zeros((n,3),dtype="<u2").tobytes()+x.tobytes()+y.tobytes()
        packed=literals(fields)
        data=header(n)+struct.pack("<II",len(packed),len(fields))+packed
        with tempfile.TemporaryDirectory() as temp:
            paths=[Path(temp)/"cloud.pcd",Path(temp)/"cloud.pcd.xz"]
            paths[0].write_bytes(data);paths[1].write_bytes(lzma.compress(data))
            for path in paths:
                with self.subTest(path=path.name):
                    actual=load_cloud(path,maximum=83)
                    self.assertEqual(actual["total"],n)
                    stride=int(np.ceil(n/83))
                    np.testing.assert_allclose(actual["points"],np.column_stack([x,y,z])[::stride])
            self.assertEqual(set(Path(temp).iterdir()),set(paths))

    def test_malformed_lengths_references_tail_and_limit_are_rejected(self):
        raw=b"\x00"*22
        packed=literals(raw)
        good=header(1)+struct.pack("<II",len(packed),len(raw))+packed
        cases=[good[:-1],good+b"tail",header(1)+b"bad",
               header(1)+struct.pack("<II",2,22)+b"\x20\x00",
               header(1)+struct.pack("<II",len(packed),23)+packed,
               header(1)+struct.pack("<II",len(packed)+1,22)+packed+b"\x00"]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"cloud.pcd"
            for payload in cases:
                path.write_bytes(payload)
                with self.subTest(payload=payload[-20:]), self.assertRaises(ValueError):
                    load_cloud(path)
            path.write_bytes(good)
            with patch("wc_panel.results_viewer.MAX_DECOMPRESSED_BYTES",len(good)-1):
                with self.assertRaises(ValueError):load_cloud(path)

    def test_xz_concatenated_streams_padding_and_physical_tail_validation(self):
        raw=np.array([1.,2.,3.],dtype='<f4').tobytes()
        packed=literals(raw)
        pcd=(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n'
             +struct.pack('<II',len(packed),len(raw))+packed)
        ply=b'ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n1 2 3\n'
        with tempfile.TemporaryDirectory() as temp:
            for suffix,payload in [('pcd',pcd),('ply',ply)]:
                path=Path(temp)/('cloud.'+suffix+'.xz')
                middle=len(payload)//2
                valid=lzma.compress(payload[:middle])+b'\0'*4+lzma.compress(payload[middle:])+b'\0'*8
                path.write_bytes(valid)
                np.testing.assert_allclose(load_cloud(path)['points'],[[1.,2.,3.]])
                for bad in [valid+b'junk',valid+b'\0',valid[:-9],valid+lzma.compress(b'')[:8]]:
                    path.write_bytes(bad)
                    with self.subTest(suffix=suffix,tail=bad[-16:]), self.assertRaises(ValueError):
                        load_cloud(path)

    def test_xz_large_declared_dictionary_is_rejected_before_large_allocation(self):
        payload=(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n')
        encoded=bytearray(lzma.compress(payload))
        start=12
        block_header_size=(encoded[start]+1)*4
        end=start+block_header_size
        # Preserve the stream data, but declare the LZMA2 dictionary as 256 MiB.
        # Update the block-header CRC, so this exercises the memory limit rather
        # than rejecting a corrupt header. No huge compressor allocation needed.
        property_offset=bytes(encoded[start:end-4]).index(b'\x21\x01')+start+2
        encoded[property_offset]=32
        encoded[end-4:end]=struct.pack('<I',binascii.crc32(encoded[start:end-4])&0xffffffff)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'large_dictionary.pcd.xz';path.write_bytes(encoded)
            with self.assertRaisesRegex(ValueError,'128 MiB'):
                load_cloud(path)

    def test_pcd_dimensions_and_scalar_header_fields_must_be_valid(self):
        good=b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n'
        malformed=[good.replace(b'WIDTH 1',b'WIDTH -1').replace(b'HEIGHT 1',b'HEIGHT -1'),
                   good.replace(b'HEIGHT 1',b'HEIGHT 0'),good.replace(b'POINTS 1',b'POINTS'),
                   good.replace(b'POINTS 1',b'POINTS 1 2'),good.replace(b'DATA ascii',b'DATA')]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'bad.pcd'
            for data in malformed:
                path.write_bytes(data)
                with self.subTest(data=data), self.assertRaises(ValueError):load_cloud(path)


if __name__ == "__main__":
    unittest.main()
