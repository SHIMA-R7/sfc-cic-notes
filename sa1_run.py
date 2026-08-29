"""電源投入直後に全64バンクを一気に読む。フラグを外から指定できる版。

■ なぜ新しく書いたのか
sa1_auto.py / sa1_burst.py は「bit2 = Nano-2からカートへクロック供給」を
**ハードコード**している。21.4MHzduinoがカート56番を常時駆動している今の配線では、
Nano-2 D11からも出すと出力同士がぶつかりうる。
どちらが繋がっているか未確認なので、既定では出さない。

■ 判定
CRC32でNo-Introと照合する。加えて sa1_quality.rom_likeness で
「本物のROMらしいか」を必ず見る。0xFF一色・0x00一色の読みは相互一致率が
高く出るので、一致率だけでは3回騙された実績がある。
"""

import argparse
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BAUD, BANK_SIZE  # noqa: E402
from sa1_auto import header_at, power_cycle, wait_port  # noqa: E402
from sa1_quality import rom_likeness  # noqa: E402

import serial  # noqa: E402

TARGETS = {
    "151bd470": "Hoshi no Kirby Super Deluxe (Japan)",
    "dbbcd010": "Hoshi no Kirby Super Deluxe (Japan) (Rev 1)",
    "1f35f230": "Hoshi no Kirby Super Deluxe (Japan) (Rev 2)",
    "5527071e": "Super Mario RPG (Japan)",
}


def burst(port, start_bank, count, timing, flags, clock_ocr, log):
    rd_us, addr_us, pulse_us = timing
    try:
        ser = serial.Serial(port, BAUD, timeout=60)
    except Exception as e:
        log(f"  ポートを開けません: {e}")
        return None
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            if ser.read(1) == b"R":
                break
        else:
            log("  準備完了(R)が来ませんでした")
            return None
        ser.write(bytes([
            start_bank, 0,
            rd_us & 0xFF, (rd_us >> 8) & 0xFF,
            addr_us & 0xFF, (addr_us >> 8) & 0xFF,
            pulse_us & 0xFF, (pulse_us >> 8) & 0xFF,
            flags, clock_ocr, count & 0xFF,
        ]))
        ser.flush()
        want = BANK_SIZE * count
        buf = bytearray()
        while len(buf) < want:
            chunk = ser.read(min(8192, want - len(buf)))
            if not chunk:
                log(f"  タイムアウト ({len(buf)}/{want})")
                return None
            buf += chunk
        return bytes(buf)
    finally:
        ser.close()


def report(rom, start, log):
    """バンクごとの品質を出す。まとめて1本のCRCを見るだけでは何も分からない。"""
    good = []
    for i in range(len(rom) // BANK_SIZE):
        r = rom_likeness(rom[i * BANK_SIZE:(i + 1) * BANK_SIZE])
        if r["is_rom"]:
            good.append(start + i)
    log(f"  ROMとして成立したバンク: {len(good)}/{len(rom)//BANK_SIZE}")
    if good and len(good) <= 24:
        log("    " + " ".join(f"${b:02X}" for b in good))
    return good


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM12")
    p.add_argument("--relay", default="COM17")
    p.add_argument("--start-bank", default="0xC0")
    p.add_argument("--banks", type=int, default=64)
    p.add_argument("--out", default=r"C:\SFC-ROM\KIRBYSDX.sfc")
    p.add_argument("--rounds", type=int, default=1)
    p.add_argument("--off-seconds", type=float, default=8.0)
    p.add_argument("--rd", type=int, default=5)
    p.add_argument("--addr", type=int, default=5)
    p.add_argument("--pulse", type=int, default=3)
    p.add_argument("--prime", type=int, default=1,
                   help="1でバンク$C0の1024バイト空読み(sanniのPrime SA1)を行う")
    p.add_argument("--nano2-clock", type=int, default=0,
                   help="1でNano-2 D11からカートへクロックを出す。"
                        "21.4MHzduinoが56番を駆動している間は0にすること")
    p.add_argument("--clock-ocr", type=int, default=1, help="1=4MHz")
    a = p.parse_args()

    start = int(str(a.start_bank), 0)
    flags = (0x04 if a.nano2_clock else 0x00) | (0x20 if a.prime else 0x00)
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)
    log(f"フラグ 0x{flags:02X} (prime={'有' if a.prime else '無'} / "
        f"Nano-2クロック={'有' if a.nano2_clock else '無'}) / "
        f"タイミング {a.rd}/{a.addr}/{a.pulse}")

    for rnd in range(1, a.rounds + 1):
        log(f"=== 挑戦 {rnd}/{a.rounds} ===")
        if not power_cycle(a.relay, a.off_seconds, log):
            time.sleep(10)
            continue
        if not wait_port(a.port, 30, log):
            continue
        t0 = time.time()
        rom = burst(a.port, start, a.banks, (a.rd, a.addr, a.pulse),
                    flags, a.clock_ocr, log)
        if rom is None:
            continue
        el = time.time() - t0
        crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")
        ff = rom.count(0xFF) / len(rom)
        log(f"  {el:.0f}秒 / CRC {crc} / 0xFF率 {ff:.4f}")
        good = report(rom, start, log)

        h = header_at(rom, 0x7FC0) or header_at(rom, 0xFFC0)
        if h:
            total = sum(rom) & 0xFFFF
            log(f"  ヘッダ『{h[0]}』期待0x{h[1]:04x} 計算0x{total:04x}"
                + ("  ★総和一致" if total == h[1] else ""))
        else:
            log("  ヘッダ検出できず")

        if crc in TARGETS:
            with open(a.out, "wb") as f:
                f.write(rom)
            log(f"★★★ No-Intro一致! 『{TARGETS[crc]}』 保存: {a.out}")
            return 0
        if good:
            path = f"{a.out}.r{rnd}-{crc}.unverified"
            with open(path, "wb") as f:
                f.write(rom)
            log(f"  未検証だが実データを含む。保存: {path}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
