"""
シミュレータから握手の正解表を吸い出す。

これまで実機で総当りして推測していた値が、すべて実測で出る:
    ラウンドごとの開始位置X と 長さ(16-X)
    送信すべきビット列 / 期待すべきビット列
    **1ビットの周期（命令サイクル）**
    **ラウンド境界の待ち時間（命令サイクル）**

命令サイクル → クロックパルスは ×4（CICはクロック/4で1命令）。
"""

import sys

from cic_sim import Pair, ROM_PATH
from sm590 import load_rom

ADDR_BIT_OUT = 0x117      # 主ループでビットをP0へ出す ATR
ADDR_X_SET = 0x13F        # lxa。ここでラウンドの開始位置Xが確定する


def collect(rounds_wanted=17, target_id=0xF):
    rom, _ = load_rom(ROM_PATH)
    p = Pair(rom, target_id=target_id)

    rounds = []          # (X, tx, rx)
    emits = []           # (ラウンド番号, ビット番号, lockのサイクル)
    cur = None
    prev_pc = -1

    for _ in range(3_000_000):
        pc = p.lock.pc
        # ビットを出す瞬間のサイクルを記録
        if pc == ADDR_BIT_OUT and prev_pc != ADDR_BIT_OUT and cur is not None:
            emits.append((len(rounds) - 1, p.lock.cycles))
        was_x = (pc == ADDR_X_SET)
        prev_pc = pc
        p.step()
        if was_x and p.lock.pc != ADDR_X_SET:
            x = p.lock.x
            b0, b1 = p.lock.ram[0:16], p.lock.ram[16:32]
            tx = [b0[i] & 1 for i in range(x, 16)]
            rx = [b1[i] & 1 for i in range(x, 16)]
            # 向き: NEXT STREAM BIT のマスクを決める P3.0。
            # 1ならビット0側(&5)、0ならビット1側(&2)に出す。
            rounds.append((x, tx, rx, p.lock.ports[3] & 1))
            cur = len(rounds) - 1
            if len(rounds) > rounds_wanted:
                break
    return rounds[:rounds_wanted], emits


def timings(rounds, emits):
    """ビット周期とラウンド境界の待ちを、実測サイクルから出す。"""
    by_round = {}
    for r, c in emits:
        by_round.setdefault(r, []).append(c)
    periods = []
    for r, cs in sorted(by_round.items()):
        if len(cs) >= 2:
            periods += [cs[i] - cs[i - 1] for i in range(1, len(cs))]
    gaps = {}
    for r in sorted(by_round):
        nxt = by_round.get(r + 1)
        if nxt and by_round[r]:
            # 最後のビットの送出から、次ラウンド最初のビットの送出まで
            gaps[r] = nxt[0] - by_round[r][-1]
    period = max(set(periods), key=periods.count) if periods else None
    return period, gaps


def as_plan(n=16, target_id=0xF):
    """実機プローブへ渡す形にして返す。 (向き, 送信, 期待, 待ちパルス) の並び。"""
    rounds, emits = collect(n, target_id)
    _period, gaps = timings(rounds, emits)
    out = []
    for r, (x, tx, rx, d) in enumerate(rounds):
        out.append((d, list(tx), list(rx), gaps.get(r, 0)))
    return out


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 17
    rounds, emits = collect(n)
    period, gaps = timings(rounds, emits)
    print(f"1ビットの周期: {period} 命令サイクル = {period * 4} クロックパルス")
    print()
    print(" R  X 長さ 向き 待ち(命令/パルス)   送信(Lock)        期待(Key)")
    for r, (x, tx, rx, d) in enumerate(rounds):
        g = gaps.get(r)
        gs = f"{g:5d}/{g * 4:6d}" if g else "     -/     -"
        print(f"{r:2d}  {x:x} {16 - x:2d} P3.0={d} {gs}   "
              f"{''.join(map(str, tx)):16s}  {''.join(map(str, rx))}")


if __name__ == "__main__":
    main()
