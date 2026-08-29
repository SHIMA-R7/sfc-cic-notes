"""電源投入直後の「数秒の窓」だけを使って、少しずつSA-1カートを吸う。

■ 実測で分かった窓の長さ
電源投入直後に8バンク読んで、バンクごとの実データ率を測った:

    試行1: 0.55 0.00 0.00 0.00 0.00 0.00 0.00 0.00
    試行2: 0.63 0.00 0.00 0.00 0.00 0.00 0.00 0.00
    試行3: 0.92 0.75 0.00 0.00 0.00 0.00 0.00 0.00

**開いているのは1〜2バンク分だけ。** 1バンク約1.5秒なので、窓は数秒しかない。
64バンクを一度に読む(95秒)のは原理的に不可能だった。

■ だから分割する
1回の電源サイクルで**先頭の数バンクだけ**読む。読みたいバンクを毎回変えれば、
32回ほどのサイクルで全64バンクが揃う。

**重要**: 各バンクが「電源投入直後」という**同一条件**で読まれる。
以前失敗した「別々の時間帯のバンクを混ぜる」問題とは違う。あのときは
一晩かけてバラバラの時刻に読んだものを混ぜたので6万バイト食い違った。

■ 実データ率で採否を決める
0xFF率ではなく「0xFFでないバイトの割合」で見る。施錠状態は0x00一色や
0x01一色にもなるので、最頻値の占有率も併せて確認する。
(0xFF率だけを見て71回連続で誤判定した実績がある)
"""

import argparse
import os
import sys
import time
import zlib

import numpy as np

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BANK_SIZE  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa1_auto import burst_read, header_at, power_cycle, wait_port  # noqa: E402
from sa1_quality import rom_likeness  # noqa: E402

TARGETS = {
    "151bd470": "Hoshi no Kirby Super Deluxe (Japan)",
    "dbbcd010": "Hoshi no Kirby Super Deluxe (Japan) (Rev 1)",
    "1f35f230": "Hoshi no Kirby Super Deluxe (Japan) (Rev 2)",
    "5527071e": "Super Mario RPG (Japan)",
}


def quality(chunk):
    """このバンクの品質。1.0が満点、0.0はROMとして成立していない。

    **以前は「0xFFでないバイトの割合」で測っていたが、これは誤りだった。**
    0x00 や 0x02 で埋まっていても「0xFFではない」ので 1.00 になる。
    実際、8/16/32バンクすべてが満点と出たのに、KIRBYの文字列がROM内に
    一つも存在しないという空振りを掴んだ。

    sa1_quality.rom_likeness は「異なるバイト値の種類数」で見るので、
    どんな値の一色でも弾ける。判定はそちらに一本化する。
    """
    r = rom_likeness(chunk)
    if not r["is_rom"]:
        return 0.0
    return 1.0 - r["ff"]                # ROMと認めた上で、未駆動が少ないほど高得点


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM12")
    p.add_argument("--relay", default="COM17")
    p.add_argument("--start-bank", default="0xC0")
    p.add_argument("--banks", type=int, default=64)
    p.add_argument("--slice", type=int, default=3,
                   help="1サイクルで読むバンク数 (窓は1〜2バンクなので余裕をみて3)")
    p.add_argument("--out", default=r"C:\SFC-ROM\KIRBYSDX.sfc")
    p.add_argument("--min-quality", type=float, default=0.30,
                   help="このデータ率を超えたら採用")
    p.add_argument("--hours", type=float, default=8.0)
    p.add_argument("--off-seconds", type=float, default=8.0)
    a = p.parse_args()

    start = int(str(a.start_bank), 0)
    cache = a.out + ".slice"
    os.makedirs(cache, exist_ok=True)
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)

    banks = [start + i for i in range(a.banks)]
    best = {}
    for b in banks:
        f = os.path.join(cache, f"bank_{b:03d}.bin")
        if os.path.exists(f) and os.path.getsize(f) == BANK_SIZE:
            with open(f, "rb") as fh:
                d = fh.read()
            best[b] = (quality(d), d)
    if best:
        log(f"既存 {len(best)}/{len(banks)} バンクを再利用")

    deadline = time.time() + a.hours * 3600
    cycle = 0
    while time.time() < deadline:
        todo = [b for b in banks if b not in best or best[b][0] < 0.95]
        if not todo:
            break
        cycle += 1
        target = todo[cycle % len(todo)]
        log(f"--- サイクル {cycle} / bank ${target:02X} から{a.slice}本 "
            f"(確定 {sum(1 for b in banks if b in best and best[b][0] >= a.min_quality)}"
            f"/{len(banks)})")

        if not power_cycle(a.relay, a.off_seconds, log):
            time.sleep(10)
            continue
        if not wait_port(a.port, 30, log):
            continue
        rom = burst_read(a.port, target, a.slice, (5, 5, 3), log)
        if rom is None:
            continue

        for i in range(a.slice):
            b = target + i
            if b not in banks:
                continue
            chunk = rom[i * BANK_SIZE:(i + 1) * BANK_SIZE]
            q = quality(chunk)
            if q < a.min_quality:
                continue
            if b not in best or q > best[b][0]:
                best[b] = (q, chunk)
                with open(os.path.join(cache, f"bank_{b:03d}.bin"), "wb") as f:
                    f.write(chunk)
                log(f"    ★ bank ${b:02X} 更新 (データ率 {q:.3f})")

        got = sum(1 for b in banks if b in best and best[b][0] >= a.min_quality)
        if got == len(banks):
            rom = b"".join(best[b][1] for b in banks)
            crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")
            h = header_at(rom, 0x7FC0) or header_at(rom, 0xFFC0)
            total = sum(rom) & 0xFFFF
            log(f"全バンク揃った / CRC {crc} / "
                + (f"期待0x{h[1]:04x} 計算0x{total:04x}" if h else "ヘッダ無し"))
            if crc in TARGETS:
                with open(a.out, "wb") as f:
                    f.write(rom)
                log(f"★★★ No-Intro一致! 『{TARGETS[crc]}』 保存: {a.out}")
                return 0
            with open(f"{a.out}.slice-{crc}.unverified", "wb") as f:
                f.write(rom)
    log("時間切れ")
    return 1


if __name__ == "__main__":
    sys.exit(main())
