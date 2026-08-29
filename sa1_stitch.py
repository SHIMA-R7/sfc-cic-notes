"""SA-1の「途中で力尽きる」バンクを、複数回の読みから繋ぎ合わせる。

■ 何が起きているか（実測）
残った12バンクを調べたら、化けているのではなかった。
**バンクの先頭から順に読めて、途中で力尽きて以降が全部0xFFになる。**
そして到達距離が読むたびに違う。

    $EA を3回読んだときの4KBブロックごとの実データ率:
      1回目: 0.97 0.93 0.10 0.00 0.00 ... (2ブロック目で力尽き)
      2回目: 0.97 0.66 0.00 0.00 0.00 ... (1ブロック目で力尽き)
      3回目: 0.97 0.93 0.95 0.96 0.82 ... (4ブロック目まで到達)

決定的なのは、**重なった領域での一致率が 1.0000** だったこと。
読めた部分は完全に正しい。ランダムな化けではない。

■ だから繋ぎ合わせられる
「0xFFでないバイト」だけを採用して重ねていけば、いつかバンク全体が埋まる。
読むたびに到達距離が変わるので、回数を重ねれば遠い場所もいずれ埋まる。

■ 0xFFが本物のデータだったら？
ROMには本物の0xFFパディングも存在する。それは**永久に埋まらない**ので、
一定回数試して埋まらなかった位置は0xFFのまま確定させる。
実害はない（本物の0xFFなら正解と一致する）。

■ 答え合わせ
No-Intro: Super Mario RPG (Japan) / 4,194,304 bytes / CRC32 5527071e
"""

import argparse
import os
import sys
import time
import zlib

import numpy as np

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BANK_SIZE, TIMING_TIERS, _read_bank_once  # noqa: E402

TARGET_CRC = "5527071e"


def read_once(port, bank, tier):
    d = _read_bank_once(port, bank, TIMING_TIERS[tier],
                        cart_clock=True, clock_ocr=1, log=lambda m: None)
    return np.frombuffer(d, dtype=np.uint8) if d else None


def stitch_bank(port, bank, tier, max_reads, log, seed=None):
    """1バンクを複数回読んで繋ぎ合わせる。戻り値は (データ, 埋まった割合)。

    埋まっていない位置は0xFFのまま。本物の0xFFパディングと区別できないが、
    その場合は0xFFが正解なので実害はない。
    """
    acc = np.full(BANK_SIZE, 255, dtype=np.uint8)
    filled = np.zeros(BANK_SIZE, dtype=bool)
    conflicts = 0

    # 前回の .partial があれば、そこから積み増す。
    # 1バンク25回で81%まで埋まった実績があるので、続きからやれば無駄がない。
    if seed is not None:
        live = seed != 255
        acc[live] = seed[live]
        filled |= live

    for n in range(max_reads):
        d = read_once(port, bank, tier)
        if d is None:
            continue
        live = d != 255
        # 既に埋まっている場所と食い違わないか確認する。
        # 実測では重なった領域の一致率が1.0000だったので、食い違いは異常。
        both = live & filled
        if both.any():
            bad = int((acc[both] != d[both]).sum())
            conflicts += bad
        fresh = live & ~filled
        acc[fresh] = d[fresh]
        filled |= live
        if filled.all():
            break

    return acc, filled.mean(), conflicts, n + 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM16")
    p.add_argument("--banks", nargs="+", required=True,
                   help="繋ぎ合わせたいバンク番号 (16進可)")
    p.add_argument("--tier", type=int, default=1)
    p.add_argument("--cache", default=r"C:\SFC-ROM\SUPERMARIORPG.sfc.harvest")
    p.add_argument("--max-reads", type=int, default=40,
                   help="1バンクあたり最大何回読むか")
    p.add_argument("--min-filled", type=float, default=0.995,
                   help="この割合まで埋まったら確定 (既定0.995)")
    a = p.parse_args()

    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)
    os.makedirs(a.cache, exist_ok=True)
    banks = [int(str(b), 0) for b in a.banks]

    for bank in banks:
        path = os.path.join(a.cache, f"bank_{bank:03d}.bin")
        if os.path.exists(path) and os.path.getsize(path) == BANK_SIZE:
            log(f"bank ${bank:02X}: 収穫済み。飛ばす")
            continue
        log(f"bank ${bank:02X}: 繋ぎ合わせ開始")
        seed = None
        if os.path.exists(path + ".partial"):
            seed = np.fromfile(path + ".partial", dtype=np.uint8)
            log(f"  前回の途中結果を引き継ぐ ({(seed != 255).mean() * 100:.1f}% 済)")
        acc, ratio, conflicts, reads = stitch_bank(a.port, bank, a.tier,
                                                   a.max_reads, log, seed)
        note = f" / 食い違い {conflicts}" if conflicts else ""
        log(f"bank ${bank:02X}: {reads}回で {ratio * 100:.2f}% 埋まった{note}")
        if ratio >= a.min_filled:
            with open(path, "wb") as f:
                f.write(acc.tobytes())
            log(f"  ★ 確定して保存")
        else:
            with open(path + ".partial", "wb") as f:
                f.write(acc.tobytes())
            log(f"  未達。.partial として保存（次回はここから積み増せる）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
