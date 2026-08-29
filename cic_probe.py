"""
第3〜4段: Nano-3にCICの握手を再生させ、返ってきたビットを正解表と突き合わせる。

Nano-3側は何も判断しない。ここで送信ビットと期待ビットの両方を作り、
受信ビットと比較して「何ラウンド目の何ビット目からズレたか」を出す。

    python cic_probe.py --port COM14                 # 既定値で1回
    python cic_probe.py --port COM14 --sweep id      # 初期遅延を掃引
    python cic_probe.py --port COM14 --sweep wiring  # 結線と極性の4通りを総当り
"""

import argparse
import sys
import time

import serial

from cic_model import Lock, ROUND_BITS

BAUD = 115200

# 既定値の根拠:
#   bit-pulses  15命令/ビット × 4クロック/命令 = 60
#   sample-at   元コードは書き込みの2命令後に読む -> 8パルス
#   drive       4命令ぶん駆動して0に戻す -> 16パルス
#   id-delay    元コードの wait(0xba)=562命令 ぶん。× 4 で 2248パルス
DEFAULTS = dict(bit_pulses=60, drive_pulses=16, sample_at=8,
                half_delay=0, id_delay=2248)
# 主ループのビット長はIDのそれとは別物。Key側は約90命令 ≒ 360パルス。


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--rounds", type=int, default=8)
    p.add_argument("--id-delay", type=int, default=DEFAULTS["id_delay"])
    p.add_argument("--bit-pulses", type=int, default=DEFAULTS["bit_pulses"])
    p.add_argument("--drive-pulses", type=int, default=DEFAULTS["drive_pulses"])
    p.add_argument("--sample-at", type=int, default=DEFAULTS["sample_at"])
    p.add_argument("--half-delay", type=int, default=DEFAULTS["half_delay"],
                   help="クロック半周期の水増し。0でおよそ2MHz")
    p.add_argument("--swap", action="store_true", help="A4/A5の役割を入れ替える")
    p.add_argument("--rst-high", action="store_true",
                   help="リセットをアクティブHighとして扱う")
    p.add_argument("--sweep", choices=["id", "wiring", "speed", "bit"],
                   default=None)
    p.add_argument("--invert", action="store_true",
                   help="データ線をアイドルHigh・ビットはLowパルスとして扱う")
    p.add_argument("--listen", action="store_true",
                   help="こちらは駆動せず、Keyが何か出すかだけを見る")
    p.add_argument("--decim", type=int, default=1,
                   help="聴くだけモードで何パルスに1回記録するか")
    return p.parse_args()


LISTEN_BYTES = 400

# 直近セッションで測った「抵抗の向こう側」の電位 [Low時A6, Low時A7, High時A6, High時A7]
PROBE = [0.0, 0.0, 0.0, 0.0]

# データ線の極性。アイドルHigh・ビットはLowパルス、で試すとき True
INVERT = [False]

# 電位測定を行うか。クロックを止めるので握手の本番では切る。
PROBE_ON = [False]

# ID送信後、主ループに入るまでのクロックパルス数
POST_ID = [520]

# ストリームIDの出し方 0=パルス 1=レベル保持 2=送らない
ID_MODE = [0]

# ラウンド境界の待ちのあいだ両線をLowに駆動するか
GAP_BOTH_LOW = [False]

# 向きの切り替えを待ちの前に置くか
SWAP_BEFORE_GAP = [False]

# ラウンド1の頭で入力線を1パルスごとに記録する
TRACE_R1 = [False]

# ラウンド境界の待ちに足す補正（パルス）。mangle以外の固定分。
GAP_ADJ = [0]


def listen(port, id_delay, bit_pulses, half_delay, rst_high, decim):
    """データ線を入力のまま、クロックだけ流して両線を観測する。

    握手を試す前に「相手が起動しているか」だけを切り分けるためのもの。
    こちらが一切駆動しないので、線がぶつかる心配がない。
    """
    ser = serial.Serial(port, BAUD, timeout=10)
    time.sleep(2.0)
    try:
        ser.reset_input_buffer()
        deadline = time.time() + 5
        while time.time() < deadline:
            if ser.read(1) == b"K":
                break
        else:
            return None
        flags = 0x04 | (0x01 if rst_high else 0) | ((decim & 0x1F) << 3)
        ser.write(bytes([0, id_delay & 0xFF, (id_delay >> 8) & 0xFF,
                         bit_pulses & 0xFF, 16, 8, half_delay, flags, 0,
                         (bit_pulses >> 8) & 0xFF,
                         POST_ID[0] & 0xFF, (POST_ID[0] >> 8) & 0xFF, 0]))
        ser.flush()
        buf = bytearray()
        while len(buf) < LISTEN_BYTES:
            chunk = ser.read(LISTEN_BYTES - len(buf))
            if not chunk:
                return None
            buf += chunk
        a, b = [], []
        for byte in buf:
            for s in range(4):
                v = (byte >> (s * 2)) & 3
                a.append(v & 1)
                b.append((v >> 1) & 1)
        return a, b
    finally:
        ser.close()


def describe_listen(a, b, decim):
    lines = []
    for name, sig in (("A4 (カート24)", a), ("A5 (カート55)", b)):
        edges = [i for i in range(1, len(sig)) if sig[i] != sig[i - 1]]
        high = sum(sig) * 100 // len(sig)
        lines.append(f"{name}: High率 {high}%  遷移 {len(edges)}回"
                     f"  ({len(sig)}サンプル / {decim}パルスおき)")
        if edges:
            gaps = [(edges[i] - edges[i - 1]) * decim
                    for i in range(1, min(len(edges), 12))]
            lines.append(f"    最初の遷移まで {edges[0] * decim}パルス"
                         f" / 以降の間隔(パルス) {gaps}")
    if not any(sig[i] != sig[i - 1] for sig in (a, b) for i in range(1, len(sig))):
        lines.append("→ 両線とも完全に静止。Keyは動いていない")
    return "\n".join(lines)


# 実機から返る地域申告ビット。1=411(NTSC)。日本のカートは1のはず。
REGION = [1]

# keyシード[1]へ書き込むニブル。None なら 0x9/0x6（SuperCIC準拠）
REGION_VALUE = [None]

# ラウンドごとの開始レジスタkを外から決めるための関数。実機に選ばせるとき用。
K_OVERRIDE = [None]

# 向き・開始位置の出どころ 'key' か 'lock'
K_SRC = ['lock']
DIR_SRC = ['lock']
SRC = ['key']   # 旧互換（未使用）

# ラウンド1以降の向きを反転して試すためのもの。
# 向きが逆だと「Keyがこちらに期待している線」を誰も駆動せず、浮いてHighになり、
# Key側の「アイドル中は両線Low」検査に引っかかって即死する。
DIR_FLIP = [False]


def plan(rounds):
    """各ラウンドの 向き / 送信ビット / 期待ビット を作る。"""
    lk = Lock(region_bit=REGION[0], region_value=REGION_VALUE[0])
    lk.k_override = K_OVERRIDE[0]
    lk.k_src = K_SRC[0]
    lk.dir_src = DIR_SRC[0]
    out = []
    for _ in range(rounds):
        tx, rx, d, gap = lk.next_round()
        out.append((d, tx, rx, gap))
    if DIR_FLIP[0]:
        out = [out[0]] + [(1 - d, tx, rx, g) for d, tx, rx, g in out[1:]]
    return out


def _pack(bits):
    """最大16ビットを2バイトに詰める。"""
    v = 0
    for i, b in enumerate(bits):
        if b:
            v |= 1 << i
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def _unpack(two, n=ROUND_BITS):
    v = two[0] | (two[1] << 8)
    return [(v >> i) & 1 for i in range(n)]


def run(port, rounds, id_delay, bit_pulses, drive_pulses, sample_at,
        half_delay, swap, rst_high, ser=None):
    """1セッション実行して、受信ビットのリストを返す。"""
    close = ser is None
    if ser is None:
        ser = serial.Serial(port, BAUD, timeout=5)
        time.sleep(2.0)          # DTRリセット明けを待つ
    try:
        # 繋ぎっぱなしで回すときはバッファを消さない。消すと、直前のセッション終了
        # 直後に届いた'K'まで捨ててしまい、次の'K'が来るまで3秒待つことになる。
        if close:
            ser.reset_input_buffer()
        deadline = time.time() + 5
        while time.time() < deadline:
            if ser.read(1) == b"K":
                break
        else:
            return None

        rounds_plan = plan(rounds)
        flags = (0x01 if rst_high else 0) | (0x02 if swap else 0)
        ser.write(bytes([
            rounds,
            id_delay & 0xFF, (id_delay >> 8) & 0xFF,
            bit_pulses & 0xFF, drive_pulses, sample_at, half_delay, flags,
            ((0x01 if INVERT[0] else 0) | (0x02 if PROBE_ON[0] else 0)
             | ((ID_MODE[0] & 3) << 2) | (0x10 if GAP_BOTH_LOW[0] else 0)
             | (0x20 if SWAP_BEFORE_GAP[0] else 0)
             | (0x40 if TRACE_R1[0] else 0)),
            (bit_pulses >> 8) & 0xFF,
            POST_ID[0] & 0xFF, (POST_ID[0] >> 8) & 0xFF,
            0,                     # hdr[12] トレース対象ラウンド（通常は未使用）
        ]))
        for d, tx, _rx, gap in rounds_plan:
            g = min(gap + GAP_ADJ[0], 65535)
            ser.write(bytes([d]) + _pack(tx)
                      + bytes([g & 0xFF, (g >> 8) & 0xFF, len(tx)]))
        ser.flush()

        need = rounds * 2 + 4        # 末尾4バイトは抵抗の向こう側の電位
        buf = bytearray()
        while len(buf) < need:
            chunk = ser.read(need - len(buf))
            if not chunk:
                return None
            buf += chunk
        PROBE[:] = [v * 5.0 / 255.0 for v in buf[rounds * 2:]]
        # ラウンド長は可変なので、送った長さで展開する
        return [_unpack(buf[i * 2:i * 2 + 2], len(rounds_plan[i][1]))
                for i in range(rounds)]
    finally:
        if close:
            ser.close()


def score(got, rounds):
    """一致ビット数と、最初にズレた位置を返す。

    ラウンド0のビット1はKey側の地域申告で、こちらは予測できない。
    比較から外し、値だけ別に報告する。
    """
    exp = [rx for _d, _tx, rx, _g in plan(rounds)]
    match = 0
    total = 0
    first_bad = None
    for r in range(rounds):
        for b in range(ROUND_BITS):
            if r == 0 and b == 1:
                continue
            total += 1
            if got[r][b] == exp[r][b]:
                match += 1
            elif first_bad is None:
                first_bad = (r, b)
    return match, total, first_bad, exp


def describe(got, rounds):
    match, total, first_bad, exp = score(got, rounds)
    lines = [f"一致 {match}/{total} ({match * 100 // max(1, total)}%)"]
    lines.append(f"線の主導権(Key稼働中): こちらLow駆動 -> A6 {PROBE[0]:.2f}V / A7 {PROBE[1]:.2f}V"
                 f"  |  High駆動 -> A6 {PROBE[2]:.2f}V / A7 {PROBE[3]:.2f}V")

    region = got[0][1]
    lines.append(f"地域ビット(R0 b1) = {region} "
                 f"-> {'411 NTSC' if region else '413 PAL'}")

    if all(all(b == 0 for b in r) for r in got):
        lines.append("→ 全ビット0。相手が何も返していないか、線が読めていない")
    elif all(all(b == 1 for b in r) for r in got):
        lines.append("→ 全ビット1。線がHighに張り付いている")

    if first_bad:
        lines.append(f"最初のズレ: ラウンド{first_bad[0]} ビット{first_bad[1]}")

    lines.append("")
    lines.append("R   期待              受信")
    for r in range(rounds):
        e = "".join(str(x) for x in exp[r])
        g = "".join(str(x) for x in got[r])
        mark = "".join("." if e[i] == g[i] else "^" for i in range(ROUND_BITS))
        lines.append(f"{r:2d}  {e}   {g}")
        if mark.strip("."):
            lines.append(f"    {' ' * ROUND_BITS}   {mark}")
    return "\n".join(lines)


def main():
    a = parse_args()
    INVERT[0] = a.invert
    common = dict(port=a.port, rounds=a.rounds, bit_pulses=a.bit_pulses,
                  drive_pulses=a.drive_pulses, sample_at=a.sample_at,
                  half_delay=a.half_delay)

    if a.listen:
        res = listen(a.port, a.id_delay, a.bit_pulses, a.half_delay,
                     a.rst_high, a.decim)
        if res is None:
            print("Nano-3が応答しない")
            return 1
        print(f"■ 聴くだけモード / リセット極性="
              f"{'High' if a.rst_high else 'Low'} / 間引き={a.decim}")
        print(describe_listen(res[0], res[1], a.decim))
        return 0

    if a.sweep == "wiring":
        print("結線2通り × リセット極性2通りを総当り\n")
        best = None
        for swap in (False, True):
            for hi in (False, True):
                got = run(id_delay=a.id_delay, swap=swap, rst_high=hi, **common)
                if got is None:
                    print(f"swap={int(swap)} rst_high={int(hi)}: 応答なし")
                    continue
                m, t, _fb, _e = score(got, a.rounds)
                print(f"swap={int(swap)} rst_high={int(hi)}: 一致 {m}/{t}")
                if best is None or m > best[0]:
                    best = (m, swap, hi, got)
        if best:
            print(f"\n最良: swap={int(best[1])} rst_high={int(best[2])}")
            print(describe(best[3], a.rounds))
        return 0

    if a.sweep == "bit":
        # 1ビットのクロックパルス数と初期遅延の2次元掃引。
        # 「クロック/4が命令速度」はPICの前提であって、カート内のSharp製CICが
        # 同じ分周とは限らない。分周が違えばビット長も初期遅延も比例してずれる。
        print("ビット長 × 初期遅延 の総当り\n")
        ser = serial.Serial(a.port, BAUD, timeout=5)
        time.sleep(2.0)
        best = None
        try:
            for bp in (15, 20, 24, 30, 40, 48, 60, 80, 120, 240):
                top = None
                for swap in (False, True):
                    for d in range(0, 6144, 32):
                        got = run(id_delay=d, swap=swap, rst_high=a.rst_high,
                                  ser=ser, **{**common, "bit_pulses": bp})
                        if got is None:
                            continue
                        m, t, _fb, _e = score(got, a.rounds)
                        if top is None or m > top[0]:
                            top = (m, d, swap, got)
                print(f"  bit={bp:4d}: 最良 {top[0]}/{t} "
                      f"(id_delay={top[1]} swap={int(top[2])})")
                if best is None or top[0] > best[0]:
                    best = (top[0], bp, top[1], top[2], top[3])
        finally:
            ser.close()
        if best:
            print(f"\n総合最良: bit={best[1]} id_delay={best[2]} "
                  f"swap={int(best[3])}\n")
            print(describe(best[4], a.rounds))
        return 0

    if a.sweep == "id":
        print("初期遅延を掃引（Key側の起動タイミングとの位相合わせ）\n")
        # 1セッションごとに開き直すとDTRリセット待ちで2秒かかる。
        # Nano-3は1回終えると自分から'K'を送って次を待つので、繋ぎっぱなしでよい。
        ser = serial.Serial(a.port, BAUD, timeout=5)
        time.sleep(2.0)
        try:
            for swap in ((a.swap,) if a.swap else (False, True)):
                best = None
                for d in range(a.id_delay, a.id_delay + 4096, 16):
                    got = run(id_delay=d, swap=swap, rst_high=a.rst_high,
                              ser=ser, **common)
                    if got is None:
                        continue
                    m, t, _fb, _e = score(got, a.rounds)
                    if best is None or m > best[0]:
                        best = (m, d, got)
                        print(f"  swap={int(swap)} id_delay={d:5d}"
                              f"  一致 {m}/{t}   <= 更新")
                if best:
                    print(f"\n-- swap={int(swap)} 最良 id_delay={best[1]}"
                          f" 一致 {best[0]}\n")
                    print(describe(best[2], a.rounds))
                    print()
        finally:
            ser.close()
        return 0

    if a.sweep == "speed":
        print("クロック速度を掃引（Keyが追従できる範囲を探す）\n")
        for hd in (0, 1, 2, 4, 8, 16, 32, 64):
            got = run(id_delay=a.id_delay, swap=a.swap, rst_high=a.rst_high,
                      **{**common, "half_delay": hd})
            if got is None:
                print(f"  half_delay={hd:3d}: 応答なし")
                continue
            m, t, _fb, _e = score(got, a.rounds)
            print(f"  half_delay={hd:3d}: 一致 {m}/{t}")
        return 0

    got = run(id_delay=a.id_delay, swap=a.swap, rst_high=a.rst_high, **common)
    if got is None:
        print("Nano-3が応答しない")
        return 1
    print(describe(got, a.rounds))
    return 0


if __name__ == "__main__":
    sys.exit(main())
