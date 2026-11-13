"""
Pruebas sin robot del protocolo y del freno de seguridad.
  python3 -m unittest discover tests -v      (desde la raiz del repositorio)
"""
import math
import unittest

import numpy as np

from tb4 import protocol as P

TOKEN = P.make_token("grupo3")


def ring(n=360, dist=3.0):
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return angles, np.full(n, dist)


class ProtocolTests(unittest.TestCase):
    def test_other_group_rejected(self):
        data = P.pack_cmd(P.make_token("grupo5"), 1, 0.1, 0.0)
        with self.assertRaises(P.ProtocolError):
            P.unpack(data, TOKEN)

    def test_cmd_roundtrip(self):
        kind, payload = P.unpack(P.pack_cmd(TOKEN, 7, 0.15, -0.5), TOKEN)
        self.assertEqual(kind, P.CMD)
        seq, v, w = P.unpack_cmd(payload)
        self.assertEqual(seq, 7)
        self.assertAlmostEqual(v, 0.15, places=5)
        self.assertAlmostEqual(w, -0.5, places=5)

    def test_cmd_nan_rejected(self):
        _, payload = P.unpack(P.pack_cmd(TOKEN, 1, float("nan"), 0.0), TOKEN)
        with self.assertRaises(P.ProtocolError):
            P.unpack_cmd(payload)

    def test_scan_roundtrip_special_values(self):
        ranges = [0.5, float("inf"), float("nan"), 0.05, 20.0, 1.234]
        data = P.pack_scan(TOKEN, 12.5, -1.0, 0.01, 0.15, 12.0, ranges)
        _, payload = P.unpack(data, TOKEN)
        stamp, angles, r, rmax = P.unpack_scan(payload)
        self.assertEqual(stamp, 12.5)
        self.assertAlmostEqual(float(angles[1]), -0.99, places=5)
        self.assertAlmostEqual(float(r[0]), 0.5, places=3)
        self.assertTrue(math.isinf(r[1]))
        self.assertTrue(math.isnan(r[2]))      # nan
        self.assertTrue(math.isnan(r[3]))      # menor que range_min
        self.assertTrue(math.isinf(r[4]))      # mayor que range_max
        self.assertAlmostEqual(float(r[5]), 1.234, places=3)

    def test_scan_is_compact(self):
        data = P.pack_scan(TOKEN, 0.0, 0.0, 0.01, 0.15, 12.0, np.ones(1080))
        self.assertLess(len(data), 2 * 1080 + 64)

    def test_image_chunks_reassemble(self):
        jpeg = bytes(range(256)) * 20
        parts = {}
        for pkt in P.pack_img_chunks(TOKEN, 3, jpeg):
            _, payload = P.unpack(pkt, TOKEN)
            req, idx, total, chunk = P.unpack_img(payload)
            self.assertEqual(req, 3)
            self.assertLessEqual(len(pkt), P.IMG_CHUNK + 64)
            parts[idx] = chunk
        self.assertEqual(b"".join(parts[i] for i in range(total)), jpeg)


    def test_photo_preview_and_hd(self):
        _, payload = P.unpack(P.pack_photo(TOKEN, 9, 85), TOKEN)
        self.assertEqual(P.unpack_photo(payload), (9, 85, P.PREVIEW))
        self.assertEqual(len(payload), 5)                          # formato original, para puentes viejos
        _, payload = P.unpack(P.pack_photo(TOKEN, 10, 70, P.HD), TOKEN)
        self.assertEqual(P.unpack_photo(payload), (10, 70, P.HD))
        self.assertEqual(P._PHOTO.unpack_from(payload), (10, 70))  # un puente viejo lo lee como preview


class GeometryTests(unittest.TestCase):
    def test_lidar_offset_maps_scan_left_to_front(self):
        # El LiDAR del TB4 esta girado +90: su angulo -90 (su derecha) es el frente del robot.
        self.assertAlmostEqual(P.scan_angles_to_base(-math.pi / 2, 90.0), 0.0)

    def test_corridor_front_obstacle(self):
        a, r = ring()
        r[np.abs(a) < math.radians(3)] = 0.40           # caja justo delante
        front, rear = P.corridor_clearance(a, r, 0.20)
        self.assertAlmostEqual(front, 0.40, places=2)
        self.assertGreater(rear, 2.9)

    def test_corridor_ignores_side_obstacle(self):
        a, r = ring()
        r[np.abs(a - math.pi / 2) < math.radians(5)] = 0.30   # pared a la izquierda
        front, _ = P.corridor_clearance(a, r, 0.20)
        self.assertGreater(front, 2.9)

    def test_corridor_rear_obstacle(self):
        a, r = ring()
        r[np.abs(np.abs(a) - math.pi) < math.radians(3)] = 0.35
        _, rear = P.corridor_clearance(a, r, 0.20)
        self.assertAlmostEqual(rear, 0.35, places=2)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.f = P.SafetyFilter(P.SafetyConfig())

    def test_free_path_passes_and_clamps(self):
        v, w, flags = self.f.apply(1.0, 5.0, math.inf, math.inf, 0.1)
        self.assertEqual((v, w, flags), (0.31, 1.9, 0))

    def test_stale_scan_blocks_translation_not_rotation(self):
        v, w, flags = self.f.apply(0.2, 0.5, math.inf, math.inf, None)
        self.assertEqual((v, w), (0.0, 0.5))
        self.assertTrue(flags & P.FLAG_SCAN_STALE)

    def test_front_block_with_hysteresis(self):
        v, _, flags = self.f.apply(0.2, 0.0, 0.25, math.inf, 0.1)
        self.assertEqual(v, 0.0)
        self.assertTrue(flags & P.FLAG_FRONT_BLOCKED)
        v, _, _ = self.f.apply(0.2, 0.0, 0.33, math.inf, 0.1)   # entre stop y clear: sigue bloqueado
        self.assertEqual(v, 0.0)
        v, _, flags = self.f.apply(0.2, 0.0, 0.45, math.inf, 0.1)
        self.assertGreater(v, 0.0)
        self.assertFalse(flags & P.FLAG_FRONT_BLOCKED)

    def test_front_block_allows_reverse(self):
        v, _, _ = self.f.apply(-0.1, 0.0, 0.20, math.inf, 0.1)
        self.assertEqual(v, -0.1)

    def test_slow_zone_scales_speed(self):
        v, _, _ = self.f.apply(0.31, 0.0, 0.44, math.inf, 0.1)
        self.assertLess(v, 0.31)
        self.assertGreaterEqual(v, 0.05)


if __name__ == "__main__":
    unittest.main()
