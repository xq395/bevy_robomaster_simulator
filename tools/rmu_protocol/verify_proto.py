"""校验 rm_custom_client.proto 与手册 V2.0.0 原文逐字段一致，并做线上字节回环测试。

用法：
    python verify_proto.py <手册文本.txt> <生成的 pb2 所在目录>
"""
import re
import sys

MANUAL_TXT = sys.argv[1]
PB2_DIR = sys.argv[2]
sys.path.insert(0, PB2_DIR)

import rm_custom_client_pb2 as pb  # noqa: E402
from google.protobuf.descriptor import FieldDescriptor as FD  # noqa: E402

TYPE_NAMES = {
    FD.TYPE_INT32: "int32", FD.TYPE_UINT32: "uint32", FD.TYPE_UINT64: "uint64",
    FD.TYPE_BOOL: "bool", FD.TYPE_FLOAT: "float", FD.TYPE_BYTES: "bytes",
    FD.TYPE_STRING: "string", FD.TYPE_MESSAGE: "message",
}

# 已知且有意的差异（见 proto 文件头说明）
KNOWN_DEVIATIONS = {"RadarInfoToClient"}


def is_repeated(field):
    # protobuf 6 起 label 已弃用，优先用 is_repeated
    if hasattr(field, "is_repeated"):
        return field.is_repeated
    return field.label == FD.LABEL_REPEATED


def parse_manual(path):
    """从手册文本里抽出每个 message 块的 (编号, 类型, 字段名)。"""
    text = open(path, encoding="utf-8").read()
    # 去掉分页页脚，避免把字段拆开
    text = re.sub(r"\n\s*\n\s*\d+ © 2026 大疆 版权所有\s*\n", "\n", text)
    blocks = {}
    for m in re.finditer(r"message\s+(\w+)\s*\{(.*?)\}", text, re.S):
        name, body = m.group(1), m.group(2)
        fields = []
        for f in re.finditer(
            r"(optional|repeated)?\s*(u?int32|uint64|bool|float|bytes|string)\s+(\w+)\s*=\s*(\d+)",
            body,
        ):
            fields.append((int(f.group(4)), f.group(2), f.group(3), f.group(1) == "repeated"))
        blocks[name] = fields
    return blocks


def check_fields():
    manual = parse_manual(MANUAL_TXT)
    ours = pb.DESCRIPTOR.message_types_by_name
    errors = []

    missing = sorted(set(manual) - set(ours))
    extra = sorted(set(ours) - set(manual))
    if missing:
        errors.append(f"手册里有、proto 里缺：{missing}")
    if extra:
        errors.append(f"proto 里多出、手册里没有：{extra}")

    for name, mfields in manual.items():
        if name not in ours or name in KNOWN_DEVIATIONS:
            continue
        desc = ours[name]
        ofields = {
            f.number: (TYPE_NAMES[f.type], f.name, is_repeated(f))
            for f in desc.fields
        }
        mmap = {n: (t, fn, rep) for n, t, fn, rep in mfields}
        if set(mmap) != set(ofields):
            errors.append(f"{name}: 编号集合不一致 手册={sorted(mmap)} 我们={sorted(ofields)}")
            continue
        for n in mmap:
            if mmap[n] != ofields[n]:
                errors.append(f"{name}.#{n}: 手册={mmap[n]} 我们={ofields[n]}")
    return manual, errors


def check_wire():
    """手工拼字节，验证编号、类型编码、proto3 optional 的存在性语义。"""
    errors = []

    # KeyboardMouseControl：W+Shift，鼠标左移 5，左键按下
    kmc = pb.KeyboardMouseControl(mouse_x=-5, left_button_down=True, keyboard_value=(1 << 0) | (1 << 4))
    raw = kmc.SerializeToString()
    # field1 varint -5 (int32 负数占 10 字节), field4 bool true, field6 varint 17
    expect = bytes([0x08]) + bytes([0xFB] + [0xFF] * 8 + [0x01]) + bytes([0x20, 0x01, 0x30, 0x11])
    if raw != expect:
        errors.append(f"KeyboardMouseControl 字节不符: {raw.hex()} != {expect.hex()}")

    # proto3 optional：显式写 0 也要能区分“有值”和“没发”
    gs = pb.GameStatus(current_stage=0)
    if not pb.GameStatus.FromString(gs.SerializeToString()).HasField("current_stage"):
        errors.append("GameStatus.current_stage=0 回环后丢失存在性")
    if pb.GameStatus().HasField("current_stage"):
        errors.append("空 GameStatus 不应有 current_stage")

    # GlobalUnitStatus.robot_health：proto3 repeated 标量默认 packed
    gus = pb.GlobalUnitStatus(robot_health=[200, 0, 150, 150, 400, 200, 0, 150, 150, 400])
    back = pb.GlobalUnitStatus.FromString(gus.SerializeToString())
    if list(back.robot_health) != list(gus.robot_health):
        errors.append("GlobalUnitStatus.robot_health 回环不一致")

    # CustomControl：30 字节上限由调用方保证，这里只验证回环
    cc = pb.CustomControl(data=bytes(range(30)))
    if pb.CustomControl.FromString(cc.SerializeToString()).data != bytes(range(30)):
        errors.append("CustomControl 回环不一致")

    # MapClickCmd：robot_id 固定 7 字节
    mc = pb.MapClickCmd(is_send_all=0, robot_id=bytes([1, 2, 0, 0, 0, 0, 0]), mode=1, type=1, map_x=12.5, map_y=3.0)
    back = pb.MapClickCmd.FromString(mc.SerializeToString())
    if back.robot_id != mc.robot_id or abs(back.map_x - 12.5) > 1e-6:
        errors.append("MapClickCmd 回环不一致")

    # RadarInfoToClient：12 组
    r = pb.RadarInfoToClient(robot_info=[pb.RadarSingleRobotInfo(target_pos_x=i * 100, target_pos_y=i, is_high_light=i % 3) for i in range(12)])
    if len(pb.RadarInfoToClient.FromString(r.SerializeToString()).robot_info) != 12:
        errors.append("RadarInfoToClient 回环不一致")

    return errors


def main():
    manual, field_errors = check_fields()
    wire_errors = check_wire()
    total_fields = sum(len(v) for v in manual.values())
    print(f"手册 message 数={len(manual)} 字段数={total_fields}")
    print(f"proto message 数={len(pb.DESCRIPTOR.message_types_by_name)}")
    print(f"有意差异（未逐字段比对）：{sorted(KNOWN_DEVIATIONS)}")
    for e in field_errors + wire_errors:
        print("FAIL", e)
    if field_errors or wire_errors:
        sys.exit(1)
    print("PASS 字段逐一一致，线上字节回环通过")


if __name__ == "__main__":
    main()
