#!/usr/bin/env python3
"""PC-side codec for IS-06 Buddy 1 MQTT frames (mirror of app_program/app_data.h).

  selftest                      check the golden vectors shared with the C tests
  cmd abort|pause|resume SEQ    print the 3-byte command; --out FILE writes it
  decode HEX [HEX ...]          decode 9-byte telemetry frames
  decode --stdin                decode lines from mosquitto_sub, e.g.
      mosquitto_sub -h BROKER -t robot/is06/state -F "%U %x" | \
          python3 tools/wifi_telemetry_tool.py decode --stdin
    (each line: optional unix timestamp, then the payload as hex)

Send a command (binary payload, QoS 0, never retained):
  python3 tools/wifi_telemetry_tool.py cmd abort 12 --out /tmp/c.bin
  mosquitto_pub -h BROKER -t robot/is06/cmd -f /tmp/c.bin
"""
import sys

PROTO_VERSION = 1
STATES = ["INITIALIZING", "DRIVING", "HUMP_CROSSING", "OBSTACLE_BYPASS",
          "LINE_SEARCH", "PAUSED", "ERROR", "MISSION_COMPLETE"]
FAULTS = ["WIFI_LINK", "MQTT_LINK", "SOCK_TIMEOUT", "CMD_MALFORMED",
          "TELEM_OVERFLOW", "PROTO_ERROR", "ESTOP_LATCHED", "COLLISION",
          "IMU_I2C", "LINE_LOST", "LINE_NOT_RECOVERED", "MOTOR"]
CMDS = {"abort": 1, "pause": 2, "resume": 3}


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_cmd(name, seq):
    if name not in CMDS or not 0 <= seq <= 255:
        raise ValueError("command must be abort|pause|resume and seq 0..255")
    b0 = PROTO_VERSION | (CMDS[name] << 2)
    return bytes([b0, seq, crc8(bytes([b0, seq]))])


def decode_telem(f):
    """Strict decode, same checks as app_telem_decode(). Raises ValueError."""
    if len(f) != 9:
        raise ValueError("length %d != 9" % len(f))
    if f[0] & 0x03 != PROTO_VERSION:
        raise ValueError("unsupported version %d" % (f[0] & 0x03))
    if f[5] & 0x80 or f[6] & 0x80 or f[8] & 0xF0:
        raise ValueError("reserved bit set")
    speed = f[4] - 256 if f[4] >= 128 else f[4]
    obst = f[6] & 0x7F
    if not (-100 <= speed <= 100 or speed == -128) or not (obst <= 100 or obst == 127):
        raise ValueError("field out of range")
    hump = f[5] & 0x3F
    faults = f[7] | (f[8] << 8)
    return {
        "version": f[0] & 0x03,
        "state": STATES[(f[0] >> 2) & 0x07],
        "retry": (f[0] >> 5) & 0x07,
        "seq": f[1],
        "t_ms": (f[2] | (f[3] << 8)) * 10,
        "speed_cm_s": None if speed == -128 else speed,
        "hump_mm": None if hump == 63 else hump,
        "obst_cm": None if obst == 127 else obst,
        "backlog": bool(f[5] & 0x40),
        "faults": [n for i, n in enumerate(FAULTS) if faults & (1 << i)],
    }


def selftest():
    gold = bytes.fromhex("45343412FB1928A500")	# same vector as S1-H07 / S1-U07
    d = decode_telem(gold)
    ok = (d["state"] == "DRIVING" and d["retry"] == 2 and d["seq"] == 0x34 and
          d["t_ms"] == 0x1234 * 10 and d["speed_cm_s"] == -5 and d["hump_mm"] == 25 and
          d["obst_cm"] == 40 and not d["backlog"] and
          d["faults"] == ["WIFI_LINK", "SOCK_TIMEOUT", "PROTO_ERROR", "COLLISION"])
    ok &= encode_cmd("abort", 7) == bytes([0x05, 0x07, 0x54])
    ok &= crc8(b"123456789") == 0xF4
    for bad in ("45343412FB1928A510", "46343412FB1928A500", "45343412FB19"):
        try:
            decode_telem(bytes.fromhex(bad))
            ok = False
        except ValueError:
            pass
    print("selftest", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    if argv[1] == "selftest":
        return selftest()
    if argv[1] == "cmd" and len(argv) >= 4:
        frame = encode_cmd(argv[2], int(argv[3], 0))
        print(frame.hex(" "))
        if "--out" in argv:
            with open(argv[argv.index("--out") + 1], "wb") as fh:
                fh.write(frame)
        return 0
    if argv[1] == "decode":
        lines = sys.stdin if "--stdin" in argv else argv[2:]
        last_seq = None
        for line in lines:
            parts = line.split()
            if not parts:
                continue
            try:
                d = decode_telem(bytes.fromhex(parts[-1]))
            except ValueError as e:
                print("REJECT", parts[-1], e)
                continue
            gap = "" if last_seq is None or d["backlog"] or (last_seq + 1) & 0xFF == d["seq"] \
                else "  <-- seq gap %d" % ((d["seq"] - last_seq - 1) & 0xFF)
            if not d["backlog"]:
                last_seq = d["seq"]
            stamp = parts[0] + " " if len(parts) > 1 else ""
            print(stamp + str(d) + gap)
            sys.stdout.flush()
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
