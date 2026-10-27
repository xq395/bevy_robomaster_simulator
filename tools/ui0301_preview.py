"""0x0301 custom-UI previewer: decode referee-serial frames and draw them the way the player client lays them out.

Input is raw bytes sent toward the referee system (a host-built UI_print.c dump, or a USB-serial capture of the
C-board's UART6 TX). Every 0xA5 frame with cmd 0x0301 and a client-UI sub-id is checked (CRC8 / CRC16 / lengths /
field ranges), applied to a scene keyed by graphic name, and rendered onto a 1920x1080 canvas (origin bottom-left).

    python tools/ui0301_preview.py frames.bin -o ui.png [--bg screenshot.png] [--rate 10]
"""
from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

W, H = 1920, 1080
SOF = 0xA5
CMD_ROBOT_INTERACTION = 0x0301
SUB_DELETE, SUB_ONE, SUB_TWO, SUB_FIVE, SUB_SEVEN, SUB_CHAR = 0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0110
GRAPH_COUNT = {SUB_ONE: 1, SUB_TWO: 2, SUB_FIVE: 5, SUB_SEVEN: 7}
DATA_LEN = {SUB_DELETE: 8, SUB_ONE: 21, SUB_TWO: 36, SUB_FIVE: 81, SUB_SEVEN: 111, SUB_CHAR: 51}
TYPES = ["line", "rect", "circle", "ellipse", "arc", "float", "int", "char"]
COLORS = {1: (255, 230, 0), 2: (0, 220, 80), 3: (255, 140, 0), 4: (200, 0, 150), 5: (255, 120, 190),
          6: (0, 230, 230), 7: (0, 0, 0), 8: (255, 255, 255)}
TEAM = {"red": (255, 60, 60), "blue": (60, 140, 255)}


def crc8(data: bytes, init: int = 0xFF) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
    return c


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c


def frame(payload: bytes, seq: int = 0) -> bytes:
    """Wrap a 0x0301 payload (sub-id + sender + receiver + content) into a referee frame."""
    head = struct.pack("<BHB", SOF, len(payload), seq & 0xFF)
    head += bytes([crc8(head)])
    body = head + struct.pack("<H", CMD_ROBOT_INTERACTION) + payload
    return body + struct.pack("<H", crc16(body))


@dataclass
class Graphic:
    name: str
    op: int
    kind: int
    layer: int
    color: int
    a0: int  # start_angle / font size
    a1: int  # end_angle / char length / float digits
    width: int
    x0: int
    y0: int
    r: int = 0
    x1: int = 0
    y1: int = 0
    value: int = 0
    text: str = ""


def parse_graphic(raw: bytes) -> Graphic:
    name = raw[:3].decode("ascii", "replace").rstrip("\x00")
    w1, w2, w3 = struct.unpack("<III", raw[3:15])
    g = Graphic(name=name, op=w1 & 7, kind=(w1 >> 3) & 7, layer=(w1 >> 6) & 15, color=(w1 >> 10) & 15,
                a0=(w1 >> 14) & 0x1FF, a1=(w1 >> 23) & 0x1FF,
                width=w2 & 0x3FF, x0=(w2 >> 10) & 0x7FF, y0=(w2 >> 21) & 0x7FF)
    if g.kind in (5, 6):
        g.value = struct.unpack("<i", raw[11:15])[0]
    else:
        g.r, g.x1, g.y1 = w3 & 0x3FF, (w3 >> 10) & 0x7FF, (w3 >> 21) & 0x7FF
    return g


@dataclass
class Frame:
    offset: int
    seq: int
    sub: int
    sender: int
    receiver: int
    graphics: list = field(default_factory=list)
    delete: tuple | None = None
    issues: list = field(default_factory=list)
    size: int = 0


def scan(buf: bytes) -> tuple[list[Frame], int]:
    """Find every 0x0301 frame in a byte stream. Returns (frames, skipped_bytes)."""
    frames, i, skipped = [], 0, 0
    while i + 9 <= len(buf):
        if buf[i] != SOF or crc8(buf[i:i + 4]) != buf[i + 4]:
            i += 1
            skipped += 1
            continue
        length, seq = struct.unpack_from("<HB", buf, i + 1)
        end = i + 7 + length + 2
        if end > len(buf):
            break
        cmd = struct.unpack_from("<H", buf, i + 5)[0]
        f_issues = []
        if struct.unpack_from("<H", buf, end - 2)[0] != crc16(buf[i:end - 2]):
            f_issues.append("CRC16 错")
        if cmd != CMD_ROBOT_INTERACTION or length < 6:
            i = end
            continue
        sub, sender, receiver = struct.unpack_from("<HHH", buf, i + 7)
        fr = Frame(offset=i, seq=seq, sub=sub, sender=sender, receiver=receiver, issues=f_issues, size=end - i)
        data = buf[i + 13:end - 2]
        if sub in DATA_LEN and DATA_LEN[sub] != length:
            fr.issues.append(f"Data_Length={length}，该子命令应为 {DATA_LEN[sub]}")
        if receiver != 0x8080 and not (0x0100 < receiver < 0x0180):
            fr.issues.append(f"receiver 0x{receiver:04X} 不是选手端 ID")
        elif receiver != 0x8080 and receiver - 0x0100 != sender:
            fr.issues.append(f"receiver 0x{receiver:04X} 与 sender {sender} 不对应")
        if sub == SUB_DELETE and len(data) >= 2:
            fr.delete = (data[0], data[1])
        elif sub in GRAPH_COUNT:
            fr.graphics = [parse_graphic(data[k * 15:k * 15 + 15]) for k in range(GRAPH_COUNT[sub]) if len(data) >= k * 15 + 15]
        elif sub == SUB_CHAR and len(data) >= 45:
            g = parse_graphic(data[:15])
            raw = data[15:45]
            g.text = raw[:min(g.a1, 30)].decode("ascii", "replace")
            if g.kind != 7:
                fr.issues.append(f"字符帧里的图形类型是 {g.kind}，应为 7")
            if g.a1 > 30:
                fr.issues.append(f"字符长度 {g.a1} 超过 30")
            tail = raw[g.a1:].rstrip(b"\x00") if g.a1 <= 30 else b""
            if tail:
                fr.issues.append(f"字符长度字段 {g.a1} 小于实际内容（截断了 {tail!r}）")
            fr.graphics = [g]
        for g in fr.graphics:
            fr.issues += check_graphic(g)
        frames.append(fr)
        i = end
    return frames, skipped


def check_graphic(g: Graphic) -> list[str]:
    out = []
    if g.op > 3:
        out.append(f"{g.name}: 操作 {g.op} 无效")
    if g.layer > 9:
        out.append(f"{g.name}: 图层 {g.layer} 超出 0-9")
    if g.color > 8:
        out.append(f"{g.name}: 颜色 {g.color} 超出 0-8")
    if g.x0 >= W or g.y0 >= H:
        out.append(f"{g.name}: 起点 ({g.x0},{g.y0}) 在屏幕外")
    if g.kind in (0, 1) and (g.x1 >= W or g.y1 >= H):
        out.append(f"{g.name}: 终点 ({g.x1},{g.y1}) 在屏幕外")
    if g.kind in (3, 4) and (g.a0 > 360 or g.a1 > 360):
        out.append(f"{g.name}: 角度 {g.a0}-{g.a1} 超出 0-360")
    if g.kind in (5, 6, 7) and g.a0 == 0:
        out.append(f"{g.name}: 字号为 0")
    if g.width == 0 and g.kind < 5:
        out.append(f"{g.name}: 线宽为 0，不可见")
    return out


class Scene:
    def __init__(self):
        self.items: dict[str, Graphic] = {}
        self.log: list[str] = []

    def apply(self, fr: Frame):
        if fr.delete:
            op, layer = fr.delete
            if op == 1:
                self.items = {k: v for k, v in self.items.items() if v.layer != layer}
            elif op == 2:
                self.items.clear()
            return
        for g in fr.graphics:
            if g.op == 1:
                if g.name in self.items:
                    self.log.append(f"{g.name}: 重复“增加”（客户端会忽略或覆盖，建议改成修改）")
                self.items[g.name] = g
            elif g.op == 2:
                if g.name not in self.items:
                    self.log.append(f"{g.name}: “修改”了一个从未增加过的图形，客户端上不会显示")
                    continue
                self.items[g.name] = g
            elif g.op == 3:
                self.items.pop(g.name, None)


def team_of(sender: int) -> str:
    return "blue" if sender >= 100 else "red"


def render(scene: Scene, team: str, out: Path, bg: Path | None = None):
    from PIL import Image, ImageDraw, ImageFont

    img = Image.open(bg).convert("RGB").resize((W, H)) if bg else Image.new("RGB", (W, H), (40, 44, 52))
    d = ImageDraw.Draw(img)
    fonts: dict[int, object] = {}

    def font(size):
        if size not in fonts:
            for name in ("consola.ttf", "DejaVuSansMono.ttf", "arial.ttf"):
                try:
                    fonts[size] = ImageFont.truetype(name, size)
                    break
                except OSError:
                    continue
            else:
                fonts[size] = ImageFont.load_default()
        return fonts[size]

    def Y(y):
        return H - y

    for g in sorted(scene.items.values(), key=lambda g: g.layer):
        col = TEAM[team] if g.color == 0 else COLORS.get(g.color, (128, 128, 128))
        w = max(1, g.width)
        if g.kind == 0:
            d.line([(g.x0, Y(g.y0)), (g.x1, Y(g.y1))], fill=col, width=w)
        elif g.kind == 1:
            d.rectangle([min(g.x0, g.x1), Y(max(g.y0, g.y1)), max(g.x0, g.x1), Y(min(g.y0, g.y1))], outline=col, width=w)
        elif g.kind == 2:
            d.ellipse([g.x0 - g.r, Y(g.y0) - g.r, g.x0 + g.r, Y(g.y0) + g.r], outline=col, width=w)
        elif g.kind in (3, 4):
            box = [g.x0 - g.x1, Y(g.y0) - g.y1, g.x0 + g.x1, Y(g.y0) + g.y1]
            if g.kind == 3:
                d.ellipse(box, outline=col, width=w)
            else:  # protocol: 0 deg at 12 o'clock, clockwise
                d.arc(box, g.a0 - 90, g.a1 - 90, fill=col, width=w)
        elif g.kind in (5, 6, 7):
            if g.kind == 6:
                text = str(g.value)
            elif g.kind == 5:
                text = f"{g.value / 1000:.{min(g.a1, 6)}f}" if g.a1 else f"{g.value / 1000:g}"
            else:
                text = g.text
            d.text((g.x0, Y(g.y0)), text, fill=col, font=font(max(8, g.a0)))
    img.save(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="raw bytes sent to the referee system")
    ap.add_argument("-o", "--out", type=Path, default=Path("ui0301.png"))
    ap.add_argument("--bg", type=Path, help="optional 1920x1080 screenshot to draw over")
    ap.add_argument("--team", choices=["red", "blue"], help="colour 0 (team colour); default from sender id")
    ap.add_argument("--rate", type=float, default=10.0, help="frames/s the firmware sends at, for the budget line")
    a = ap.parse_args(argv)

    frames, skipped = scan(a.input.read_bytes())
    scene, bad = Scene(), 0
    for fr in frames:
        what = (f"删除 op={fr.delete[0]} layer={fr.delete[1]}" if fr.delete else
                ", ".join(f"{g.name}:{TYPES[g.kind]}/op{g.op}/L{g.layer}" + (f"={g.value}" if g.kind in (5, 6) else "")
                          + (f"=\"{g.text}\"" if g.kind == 7 else "") for g in fr.graphics))
        print(f"@{fr.offset:5d} seq={fr.seq:3d} 0x{fr.sub:04X} {fr.sender}->0x{fr.receiver:04X} {fr.size:3d}B  {what}")
        for msg in fr.issues:
            print(f"        ! {msg}")
        bad += bool(fr.issues)
        scene.apply(fr)
    for msg in scene.log:
        print(f"  ! {msg}")
    team = a.team or (team_of(frames[0].sender) if frames else "red")
    render(scene, team, a.out, a.bg)
    total = sum(fr.size for fr in frames)
    avg = total / len(frames) if frames else 0
    print(f"\n{len(frames)} 帧，{bad} 帧有问题，跳过 {skipped} 字节；场景中 {len(scene.items)} 个图形 -> {a.out}")
    if frames:
        print(f"平均 {avg:.0f} B/帧，按 {a.rate:g} 帧/s 约 {avg * a.rate:.0f} B/s"
              f"（0x0301 上限 30 帧/s，内容 ≤112 B，所有子命令共用）")
    return 1 if bad or scene.log else 0


if __name__ == "__main__":
    sys.exit(main())
