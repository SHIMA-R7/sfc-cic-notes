"""SA-1カートを「1回の接続で連続読み」する。

■ なぜ必要か
SA-1は時間とともに提示する内容を変える。実測では、同じバンク($D6)を54秒間
読み続けると、2つ以上の状態を行き来した（相違5万台と数千台を往復。先頭8バイトは
毎回同じなのに、バンク全体が別物になる）。

    多数決は「同じ物を何度も読んだ誤差」を潰す道具であって、
    **別々の状態を混ぜる**のには使えない。67分かけたマージが
    「全バンク未決着0まで収束したのに中身が別物」だったのはこのため。

したがって、正しさは**読み切るまでの時間**に直結する。

■ 何を削ったか
従来はバンクごとにシリアルを開き直し、そのたびDTRでNano-2がリセットされ、
起動処理をやり直していた。バンクあたり約2秒の固定費で、64バンクなら2分強。
このスクリプトは**1回の接続で全バンクを続けて読む**（ファーム側の連続バンク
モード、ヘッダ11バイト目）。

sanniは1バイトあたり375ns(アドレス設定+6NOP)、このリグは約7000ns。
1バイトの差は埋められないが、バンクあたりの固定費は消せる。

■ 答え合わせ
No-Intro: Super Mario RPG (Japan) / 4,194,304 bytes / CRC32 5527071e
"""

import argparse
import os
import sys
import time
import zlib

import serial

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BAUD, BANK_SIZE, TIMING_TIERS  # noqa: E402

TARGET_CRC = "5527071e"


def header_at(data, off):
    if len(data) < off + 32:
        return None
    h = data[off:off + 32]
    comp = h[28] | (h[29] << 8)
    csum = h[30] | (h[31] << 8)
    if ((csum + comp) & 0xFFFF) == 0xFFFF and csum:
        return h[:21].decode("shift_jis", "replace").strip(), csum
    return None


def burst_read(port, start_bank, count, tier, log):
    """1回の接続で count バンクを続けて読む。戻り値は連結したバイト列。

    途中でタイムアウトしたら、そこまでのデータを返さず None にする。
    部分的なデータを混ぜると、どこでずれたのか分からなくなるため。
    """
    rd_us, addr_us, pulse_us, _ = TIMING_TIERS[tier]
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
            log("  Nanoからの準備完了(R)が来ませんでした")
            return None

        hdr = bytes([
            start_bank,
            0,                       # totalBanks(OLED表示用)。使わない
            rd_us & 0xFF, (rd_us >> 8) & 0xFF,
            addr_us & 0xFF, (addr_us >> 8) & 0xFF,
            pulse_us & 0xFF, (pulse_us >> 8) & 0xFF,
            0x04,                    # bit2 = カートへクロック供給
            1,                       # clock_ocr: 1 = 4MHz
            count & 0xFF,            # 11バイト目 = 連続して読むバンク数
        ])
        ser.write(hdr)
        ser.flush()

        want = BANK_SIZE * count
        buf = bytearray()
        last = time.time()
        while len(buf) < want:
            chunk = ser.read(min(8192, want - len(buf)))
            if not chunk:
                log(f"  タイムアウト ({len(buf)}/{want})")
                return None
            buf += chunk
            if time.time() - last > 15:
                log(f"    {len(buf) // BANK_SIZE}/{count} バンク")
                last = time.time()
        return bytes(buf)
    finally:
        ser.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM16")
    p.add_argument("--start-bank", default="0xC0")
    p.add_argument("--banks", type=int, default=64)
    p.add_argument("--tier", type=int, default=1)
    p.add_argument("--out", default=r"C:\SFC-ROM\SUPERMARIORPG.sfc")
    p.add_argument("--rounds", type=int, default=20)
    a = p.parse_args()
    start = int(str(a.start_bank), 0)
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)

    for rnd in range(1, a.rounds + 1):
        log(f"=== 挑戦 {rnd} ===")
        t0 = time.time()
        rom = burst_read(a.port, start, a.banks, a.tier, log)
        if rom is None:
            log("  失敗。10秒待って再試行")
            time.sleep(10)
            continue
        elapsed = time.time() - t0
        crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")
        log(f"  {len(rom)} bytes を {elapsed:.0f}秒で読了 / CRC32 = {crc}")

        # **0xFF率だけでは駄目だった（実測で判明）。**
        # 施錠状態は0xFF一色だけでなく、0x00一色・0x01一色にもなる。
        # 71回通しダンプを回して全部0xFF率0.0000だったが、中身は0x00や0x01の
        # 一色だった。0xFF率しか見ていなかったため、施錠状態を「開いている」と
        # 71回連続で誤判定し続けていた。
        #
        # 最頻値の占有率で見る。本物のROMなら1バイト値が支配的になることはない
        # (256種に近い値が出るはず)。
        import collections
        top = collections.Counter(rom[:65536]).most_common(1)[0][1] / 65536
        uniq = len(set(rom[:65536]))
        log(f"  bank $C0: 異なる値{uniq}種 / 最頻値占有率{top:.4f}"
            + ("  ← 施錠。読めていない" if top > 0.3 or uniq < 50 else ""))

        h = header_at(rom, 0x7FC0) or header_at(rom, 0xFFC0)
        if h:
            total = sum(rom) & 0xFFFF
            log(f"  ヘッダ『{h[0]}』 期待=0x{h[1]:04x} 計算=0x{total:04x}"
                f" -> {'一致' if total == h[1] else '不一致'}")
        else:
            log("  有効なヘッダなし（施錠中の可能性）")

        if crc == TARGET_CRC:
            with open(a.out, "wb") as f:
                f.write(rom)
            log(f"★★★ No-Intro一致! CRC32={crc} 保存: {a.out}")
            return 0
        with open(f"{a.out}.burst-{crc}.unverified", "wb") as f:
            f.write(rom)
    log("上限回数に達しました")
    return 1


if __name__ == "__main__":
    sys.exit(main())
