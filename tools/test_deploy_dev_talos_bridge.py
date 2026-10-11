import importlib.util
import mmap
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("deploy_dev_talos_bridge.py")
SPEC = importlib.util.spec_from_file_location("deploy_dev_talos_bridge", MODULE_PATH)
bridge = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)


class TalosTransportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)
        meta = bytearray(bridge.META_SIZE)
        struct.pack_into("<IIQQII32x", meta, 0, bridge.SHM_MAGIC, bridge.SHM_VERSION,
                         1, 1, bridge.IMAGE_W, bridge.IMAGE_H)
        offsets = (bridge.IMAGE_TB_OFFSET, *bridge.POSE_TB_OFFSETS,
                   bridge.GIMBAL_CMD_TB_OFFSET)
        for offset in offsets:
            meta[offset], meta[offset + 1], meta[offset + 2] = 1, 0, 2
        struct.pack_into(
            "<Q4d5dII24x", meta, bridge.CAMERA_INFO_OFFSET,
            1, 1303.0, 1303.0, 720.0, 540.0,
            0.0, 0.0, 0.0, 0.0, 0.0, bridge.IMAGE_W, bridge.IMAGE_H
        )
        (self.path / "talos_ipc_meta").write_bytes(meta)
        (self.path / "talos_ipc_image_pool").write_bytes(bytes(bridge.IMAGE_POOL_SIZE))

    def tearDown(self):
        self.temp.cleanup()

    def publish_bundle(self, seq=7, timestamp=123456):
        with open(self.path / "talos_ipc_meta", "r+b", buffering=0) as stream:
            meta = mmap.mmap(stream.fileno(), bridge.META_SIZE, access=mmap.ACCESS_WRITE)
            struct.pack_into("<QQIIBB6x", meta, bridge.IMAGE_SLOT_OFFSET, seq, timestamp,
                             bridge.IMAGE_W, bridge.IMAGE_H, 0, 0)
            for index, offset in enumerate(bridge.POSE_TB_OFFSETS):
                position = (10.0, 20.0, 30.0) if index == 1 else (0.0, 0.0, 0.0)
                struct.pack_into("<Q3f4f4xQ16x", meta, offset + 64, seq, *position,
                                 1.0, 0.0, 0.0, 0.0, timestamp)
                meta[offset] = bridge.FLAG_NEW
            meta[bridge.IMAGE_TB_OFFSET] = bridge.FLAG_NEW
            meta.close()

    def test_consumes_coherent_bundle_and_preserves_wxyz(self):
        self.publish_bundle()
        with bridge.TalosShm(self.path) as talos:
            bundle = talos.read_bundle()
            self.assertIsNotNone(bundle)
            self.assertEqual(bundle.frame_seq, 7)
            np.testing.assert_allclose(bundle.gimbal.quaternion_wxyz, [1, 0, 0, 0])
            np.testing.assert_allclose(bundle.odom.position, [10, 20, 30])
            self.assertIsNone(talos.read_bundle())
            for offset in (bridge.IMAGE_TB_OFFSET, *bridge.POSE_TB_OFFSETS):
                self.assertEqual(talos.meta[offset] & bridge.FLAG_NEW, 0)
                self.assertEqual(talos.meta[offset + 2], 0)

    def test_writes_rust_gimbal_triple_buffer_protocol(self):
        with bridge.TalosShm(self.path) as talos:
            talos.write_gimbal_command(12.5, -3.0, 4.2, False)
            self.assertEqual(talos.meta[bridge.GIMBAL_CMD_TB_OFFSET], bridge.FLAG_NEW)
            self.assertEqual(talos.meta[bridge.GIMBAL_CMD_TB_OFFSET + 1], 1)
            slot = bridge.GIMBAL_CMD_TB_OFFSET + 64
            _, yaw, pitch, distance, fire = struct.unpack_from("<QfffB", talos.meta, slot)
            self.assertAlmostEqual(yaw, 12.5)
            self.assertAlmostEqual(pitch, -3.0)
            self.assertAlmostEqual(distance, 4.2, places=5)
            self.assertEqual(fire, 0)


class MathTest(unittest.TestCase):
    def test_identity_quaternion_faces_positive_x(self):
        rotation = bridge.quaternion_matrix_wxyz(np.array([1.0, 0.0, 0.0, 0.0]))
        yaw, pitch = bridge.yaw_pitch_from_rotation(rotation)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(pitch, 0.0)

    def test_ballistic_solution_is_finite(self):
        result = bridge.solve_trajectory(23.0, 5.0, 0.0)
        self.assertIsNotNone(result)
        pitch, fly_time = result
        self.assertGreater(pitch, 0.0)
        self.assertGreater(fly_time, 0.0)


if __name__ == "__main__":
    unittest.main()
