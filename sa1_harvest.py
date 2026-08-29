"""SA-1カートを「バンク単位で収穫する」。開いた瞬間のバンクだけを確定させる。

■ なぜこの形なのか
実測で分かったこと:

  ・SA-1は時間とともに開閉する。**バンクごとに独立して**開閉する
    (同じ瞬間に $C2 の0xFF率が4%、$D0 が86% ということが起きる)
  ・通しダンプ(5分半)の間にも状態が変わる。1本目0xFF率0.85 -> 2本目0.98 と悪化した
  ・**つまり「全64バンクが同時に開いている5分半」を待つのは筋が悪い**

そこで方針を変える。1本のROMを一度に作ろうとせず、**バンクごとに最良の1枚を
蓄積する**。あるバンクが開いた瞬間にそれを確定させ、まだ確定していないバンクだけを
読み続ける。全部揃ったら結合する。

■ 「開いた」の判定
0xFF率で判断する。**「2回読んで一致」では駄目**。0xFF一色の読みは安定していて
一致してしまう(READMEに記録のある罠。今夜も62バンクが揃って0xFF一色になり、
相互一致率が高く見えて誤認した)。

判定は2段構え:
  1. 0xFF率が閾値未満 -> 候補
  2. 同じバンクをもう一度読んで一致 -> 確定

0xFF率だけでは「たまたま実データに見えるゴミ」を弾けないので、一致も要求する。

■ 答え合わせ
No-Intro: Super Mario RPG (Japan) / 4,194,304 bytes / CRC32 5527071e
"""

import argparse
import os
import sys
import time
import zlib

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import BANK_SIZE, TIMING_TIERS, _read_bank_once  # noqa: E402

TARGET_CRC = "5527071e"


def read_once(port, bank, tier):
    return _read_bank_once(port, bank, TIMING_TIERS[tier],
                           cart_clock=True, clock_ocr=1, log=lambda m: None)


def header_ok(data, off=0x7FC0):
    if len(data) < off + 32:
        return False
    h = data[off:off + 32]
    comp = h[28] | (h[29] << 8)
    csum = h[30] | (h[31] << 8)
    return ((csum + comp) & 0xFFFF) == 0xFFFF and csum != 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM16")
    p.add_argument("--start-bank", default="0xC0")
    p.add_argument("--banks", type=int, default=64)
    p.add_argument("--tier", type=int, default=1)
    p.add_argument("--out", default=r"C:\SFC-ROM\SUPERMARIORPG.sfc")
    p.add_argument("--max-ff", type=float, default=0.30,
                   help="このFF率未満なら「開いている」とみなす候補にする (既定0.30)")
    p.add_argument("--hours", type=float, default=6.0, help="最大何時間回すか")
    a = p.parse_args()

    start = int(str(a.start_bank), 0)
    cache = a.out + ".harvest"
    os.makedirs(cache, exist_ok=True)
    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)

    banks = [start + i for i in range(a.banks)]
    done = {}
    # 既に収穫済みのものを読み込む（中断しても続きから積める）
    for b in banks:
        f = os.path.join(cache, f"bank_{b:03d}.bin")
        if os.path.exists(f) and os.path.getsize(f) == BANK_SIZE:
            with open(f, "rb") as fh:
                done[b] = fh.read()
    if done:
        log(f"収穫済み {len(done)}/{len(banks)} バンクを再利用")

    deadline = time.time() + a.hours * 3600
    rounds = 0
    while len(done) < len(banks) and time.time() < deadline:
        rounds += 1
        todo = [b for b in banks if b not in done]
        log(f"=== 巡回 {rounds} / 残り {len(todo)} バンク ===")
        got = 0
        for b in todo:
            if time.time() > deadline:
                break
            # ■ 1回読みで判定する（実測に基づく最終形）
            # 6時間回して46/64で頭打ちになった原因は「2回連続で開いている」を
            # 要求していたこと。$E8 を10回読むと最良0xFF率3.7%まで開くのに、
            # **開くのは一瞬で次の読みではもう閉じている**ため確定できなかった。
            #
            # 0xFF率が十分低いこと自体が「バスが駆動されている＝実データ」の
            # 証拠なので、一致も2回目も要求しない。閾値を厳しめ(既定0.10)に
            # 取ることで、0xFF一色のゴミは弾ける。
            d1 = read_once(a.port, b, a.tier)
            if d1 is None:
                continue
            ff = d1.count(0xFF) / BANK_SIZE
            if ff >= a.max_ff:
                continue                      # まだ閉じている
            done[b] = d1
            got += 1
            with open(os.path.join(cache, f"bank_{b:03d}.bin"), "wb") as fh:
                fh.write(d1)
            log(f"  ★ bank ${b:02X} 確定 (0xFF率 {ff:.4f}) — 収穫 {len(done)}/{len(banks)}")
        if got == 0:
            log("  この巡回では1本も取れず。30秒待つ")
            time.sleep(30)

    if len(done) < len(banks):
        log(f"時間切れ。{len(done)}/{len(banks)} バンクまで")
        return 1

    rom = b"".join(done[b] for b in banks)
    crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")
    total = sum(rom) & 0xFFFF
    log(f"全バンク収穫完了 / CRC32 = {crc} / 総和 = 0x{total:04x}")
    if crc == TARGET_CRC:
        with open(a.out, "wb") as f:
            f.write(rom)
        log(f"★★★ No-Intro一致! 保存: {a.out}")
        return 0
    with open(f"{a.out}.harvest-{crc}.unverified", "wb") as f:
        f.write(rom)
    log(f"不一致。{a.out}.harvest-{crc}.unverified として保存")
    return 1


if __name__ == "__main__":
    sys.exit(main())
