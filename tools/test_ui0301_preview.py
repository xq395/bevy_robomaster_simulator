import struct
import tempfile
import unittest
from pathlib import Path

import ui0301_preview as p


def graphic(name, op, kind, layer, color, a0, a1, width, x0, y0, r=0, x1=0, y1=0, value=None):
    w1 = op | kind << 3 | layer << 6 | color << 10 | a0 << 14 | a1 << 23
    w2 = width | x0 << 10 | y0 << 21
    tail = struct.pack("<i", value) if value is not None else struct.pack("<I", r | x1 << 10 | y1 << 21)
    return name.encode().ljust(3, b"\x00") + struct.pack("<II", w1, w2) + tail


def one(g, sender=3, receiver=0x0103):
    return p.frame(struct.pack("<HHH", p.SUB_ONE, sender, receiver) + g)


class PreviewTest(unittest.TestCase):
    def test_crc_matches_referee_reference(self):
        # Same CRC8/CRC16 tables as BSP_CRC.c (checked against frames from the host-built UI_print.c).
        self.assertEqual(p.crc8(bytes([0xA5, 0x15, 0x00, 0x01])), 0xE2)

    def test_roundtrip_line_and_int(self):
        buf = one(graphic("c01", 1, 0, 0, 1, 0, 0, 2, 940, 540, x1=980, y1=540))
        buf += one(graphic("i01", 1, 6, 2, 8, 20, 0, 2, 1520, 245, value=-42))
        frames, skipped = p.scan(b"\x00\x11" + buf)
        self.assertEqual(skipped, 2)
        self.assertEqual([f.issues for f in frames], [[], []])
        line, num = frames[0].graphics[0], frames[1].graphics[0]
        self.assertEqual((line.x0, line.y0, line.x1, line.y1, line.width), (940, 540, 980, 540, 2))
        self.assertEqual(num.value, -42)

    def test_flags_bad_crc_receiver_and_range(self):
        bad = bytearray(one(graphic("x", 1, 0, 12, 9, 0, 0, 0, 1000, 500), sender=3, receiver=0x0105))
        bad[-1] ^= 0xFF
        issues = " ".join(p.scan(bytes(bad))[0][0].issues)
        for word in ("CRC16", "receiver", "图层", "颜色", "线宽"):
            self.assertIn(word, issues)

    def test_scene_modify_without_add_and_delete_layer(self):
        sc = p.Scene()
        for fr in p.scan(one(graphic("a", 2, 0, 1, 1, 0, 0, 2, 1, 1)) + one(graphic("b", 1, 0, 1, 1, 0, 0, 2, 1, 1))
                         + p.frame(struct.pack("<HHHBB", p.SUB_DELETE, 3, 0x0103, 1, 1)))[0]:
            sc.apply(fr)
        self.assertEqual(sc.items, {})
        self.assertTrue(any("从未增加" in m for m in sc.log))

    def test_char_length_field_shorter_than_text(self):
        g = graphic("s01", 1, 7, 3, 8, 20, 3, 2, 100, 800)
        payload = struct.pack("<HHH", p.SUB_CHAR, 3, 0x0103) + g + b"HELLO".ljust(30, b"\x00")
        fr = p.scan(p.frame(payload))[0][0]
        self.assertEqual(fr.graphics[0].text, "HEL")
        self.assertTrue(any("截断" in m for m in fr.issues))

    def test_render_writes_png(self):
        sc = p.Scene()
        sc.apply(p.scan(one(graphic("c", 1, 4, 0, 0, 30, 150, 4, 960, 540, x1=100, y1=100)))[0][0])
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "ui.png"
            p.render(sc, "blue", out)
            self.assertGreater(out.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
