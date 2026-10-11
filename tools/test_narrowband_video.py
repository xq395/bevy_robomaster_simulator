import unittest
import av
from PIL import Image, ImageDraw
from narrowband_video import packetize, Receiver, encoder, PACKET_SIZE, NarrowbandVideo


class CodecTest(unittest.TestCase):
    def units(self):
        codec = encoder(320, 240, 12, 80)
        result = []
        for i in range(60):
            image = Image.new('RGB', (320, 240), '#182b38')
            ImageDraw.Draw(image).rectangle((i*3 % 250, 70, i*3 % 250+40, 110), fill='red')
            frame = av.VideoFrame.from_image(image)
            frame.pts = i
            for packet in codec.encode(frame):
                result.append(packetize(bytes(packet), len(result), packet.is_keyframe))
        return result

    def test_roundtrip_and_recovery_after_fragment_loss(self):
        units = self.units()
        receiver = Receiver()
        decoded = sum(len(receiver.push(p)) for unit in units for p in unit)
        self.assertEqual(decoded, 60)
        self.assertTrue(all(len(p) == PACKET_SIZE for u in units for p in u))
        receiver = Receiver()
        before = []
        after = []
        for index, unit in enumerate(units):
            for j, packet in enumerate(unit):
                if index == 5 and j == 0:
                    continue
                frames = receiver.push(packet)
                (before if index < 5 else after).extend(frames)
        self.assertEqual(len(before), 5)
        self.assertTrue(after, 'must resume at a later IDR')
        self.assertGreater(receiver.losses, 0)

    def test_bad_packet_and_budget_rejected(self):
        with self.assertRaises(ValueError):
            Receiver().push(b'bad')
        with self.assertRaises(ValueError):
            NarrowbandVideo().configure(dict(bitrate=100, packet_hz=40))


if __name__ == '__main__':
    unittest.main()