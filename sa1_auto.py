"""電源リレーと組み合わせて、SA-1カートを無人で吸い続ける。

■ なぜこの形なのか
実測で分かっていること:

  ・SA-1は**電源投入直後だけよく開く**。時間が経つと閉じる
    (再投入直後に、60回かけても取れなかった12バンクが1回読みで100%埋まった)
  ・バンクごとに独立して開閉するので、**別々の時間に読んだバンクを混ぜてはいけない**
    (一晩かけて積み上げた内容と、翌朝1回で読んだ内容が6万バイト食い違った)
  ・したがって必要なのは「**単一の時間帯で全64バンクを読み切る**」こと

電源の入切が人手だった間は、この最良の条件を無人で作れなかった。
リレー(Arduino Uno COM17)が入ったので、それが可能になった。

    電源OFF → 8秒待つ → ON → 即座に全64バンク読む → CRC判定 → 繰り返す

高速化により1回3.1分。一晩で150回以上試せる。

■ 判定
CRC32でNo-Introと照合する。**16bitチェックサムはたまたま合うことがある**ので
それだけでは信用しない。あわせて「最頻値の占有率」も見る。施錠状態は
0xFF一色だけでなく0x00一色・0x01一色にもなり、0xFF率だけを見て71回連続で
誤判定した実績があるため。
"""

import argparse
import sys
import time
import zlib

import serial
import serial.tools.list_ports as list_ports

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BAUD, BANK_SIZE  # noqa: E402

# No-Intro。カービィSDXは3リビジョンあるので全部受ける。
TARGETS = {
    "151bd470": "Hoshi no Kirby Super Deluxe (Japan)",
    "dbbcd010": "Hoshi no Kirby Super Deluxe (Japan) (Rev 1)",
    "1f35f230": "Hoshi no Kirby Super Deluxe (Japan) (Rev 2)",
    "5527071e": "Super Mario RPG (Japan)",
}


def power_cycle(relay_port, off_s, log):
    """リレーで電源を入れ直し、リーダーのポートが戻るまで待つ。"""
    try:
        r = serial.Serial(relay_port, 115200, timeout=3)
    except Exception as e:
        log(f"  リレーを開けません: {e}")
        return False
    try:
        time.sleep(2.2)
        r.readline()
        r.write(b"1")
        time.sleep(0.3)
        r.readline()
        time.sleep(off_s)
        r.write(b"0")
        time.sleep(0.3)
        r.readline()
    finally:
        r.close()
    return True


def wait_port(port, timeout_s, log):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if port in [p.device for p in list_ports.comports()]:
            time.sleep(2.0)          # 列挙直後は開けないことがある
            return True
        time.sleep(0.5)
    log(f"  {port} が復帰しませんでした")
    return False


def burst_read(port, start_bank, count, timing, log):
    """1回の接続で count バンクを続けて読む。"""
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
            # bit2 = カートへクロック供給 / **bit5 = prime(慣らし)**
            #
            # ■ primeを立てる理由（2026-08-28、sanniのソースを全部読んで判明）
            # sanni の getCartInfo_SNES() は、ヘッダを読む前に必ずこれをやる:
            #
            #     //Prime SA1 cartridge
            #     PORTL = 192;                       // バンク $C0
            #     for (uint16_t b = 0; b < 1024; b++) {
            #         PORTF = b & 0xFF; PORTK = b >> 8;
            #         NOP x6;                        // 読み捨てる
            #     }
            #
            # **バンク$C0で1024バイトのダミーアクセスをしてからヘッダを読む。**
            # ファーム側には primeMode として実装済みだったが、
            # **ホスト側でフラグを立てていなかった。** 一度も使っていなかった。
            0x04 | 0x20,
            1,                       # clock_ocr = 4MHz
            count & 0xFF,            # 連続して読むバンク数
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


def looks_locked(rom):
    """施錠状態かどうか。判定は sa1_quality に一本化してある。

    かつては各スクリプトが独自に判定を書いていて、**そのせいで3回騙された**
    (0xFF率だけ見る / 0x00一色を見逃す / 「0xFFでない=実データ」とみなす)。
    1箇所直しても他が古いままだったのが原因。ここで共通化する。
    """
    from sa1_quality import rom_likeness
    return not rom_likeness(rom[:BANK_SIZE])["is_rom"]


def header_at(data, off):
    if len(data) < off + 32:
        return None
    h = data[off:off + 32]
    comp = h[28] | (h[29] << 8)
    csum = h[30] | (h[31] << 8)
    if ((csum + comp) & 0xFFFF) == 0xFFFF and csum:
        return h[:21].decode("shift_jis", "replace").strip(), csum
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM12")
    p.add_argument("--relay", default="COM17")
    p.add_argument("--start-bank", default="0xC0")
    p.add_argument("--banks", type=int, default=64)
    p.add_argument("--out", default=r"C:\SFC-ROM\KIRBYSDX.sfc")
    p.add_argument("--rounds", type=int, default=200)
    p.add_argument("--off-seconds", type=float, default=8.0)
    p.add_argument("--rd", type=int, default=5)
    p.add_argument("--addr", type=int, default=5)
    p.add_argument("--pulse", type=int, default=3)
    a = p.parse_args()

    start = int(str(a.start_bank), 0)
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)
    timing = (a.rd, a.addr, a.pulse)
    log(f"タイミング rd={a.rd} addr={a.addr} pulse={a.pulse}")

    best = None
    for rnd in range(1, a.rounds + 1):
        log(f"=== 挑戦 {rnd} ===")
        if not power_cycle(a.relay, a.off_seconds, log):
            time.sleep(10)
            continue
        if not wait_port(a.port, 30, log):
            continue

        t0 = time.time()
        rom = burst_read(a.port, start, a.banks, timing, log)
        if rom is None:
            continue
        el = time.time() - t0
        crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")

        if looks_locked(rom):
            log(f"  {el:.0f}秒 / CRC {crc} / **施錠**（読めていない）")
            continue

        h = header_at(rom, 0x7FC0) or header_at(rom, 0xFFC0)
        total = sum(rom) & 0xFFFF
        ff = rom.count(0xFF) / len(rom)
        hdr = f"『{h[0]}』期待0x{h[1]:04x} 計算0x{total:04x}" if h else "ヘッダ無し"
        match = h and total == h[1]
        log(f"  {el:.0f}秒 / CRC {crc} / 0xFF率{ff:.4f} / {hdr}"
            + ("  ★チェックサム一致" if match else ""))

        if crc in TARGETS:
            with open(a.out, "wb") as f:
                f.write(rom)
            log(f"★★★ No-Intro一致! 『{TARGETS[crc]}』 保存: {a.out}")
            return 0

        # チェックサムが合ったものは残す。CRCが違ってもかなり近い可能性がある。
        if match:
            path = f"{a.out}.csum-{crc}.unverified"
            with open(path, "wb") as f:
                f.write(rom)
            log(f"  チェックサム一致だがCRC不一致。{path} に保存")
        if best is None or ff < best[0]:
            best = (ff, crc)
            with open(f"{a.out}.best.unverified", "wb") as f:
                f.write(rom)
    log(f"上限回数に達しました。最良: {best}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
