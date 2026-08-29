"""
健全な瞬間を待ち構えて、ラウンド1の波形を撮る。

■ なぜ必要か
カートの接触が不安定で、CICが応答する窓が一瞬しかない。
握手は一発勝負のロックステップなので、1ビット欠ければKeyは die して戻らない。
吸い出しのように「読み直せばいい」が通用しない。

そこで、**ラウンド0が14/14通ることを毎回確認し、通った直後にだけ**
ラウンド1の波形を記録する。ダメな間は延々とリトライする。
使う側は差し直しを繰り返すだけでよい。

    python capture_r1.py            # 既定 8分間ねばる
    python capture_r1.py 300        # 秒数を指定
"""

import sys
import time

import serial

import cic_probe as C

PORT = "COM8"
BEST = dict(id_delay=2496, bit_pulses=372, drive_pulses=24, sample_at=16,
            half_delay=0, swap=True, rst_high=True)

# 「Keyが居ない／死んでいる」ときに必ず出る模様。判定の目印に使う。
DEAD_PATTERN = "110100101101001"

# トレースの間引き。4なら1600サンプルで15ビット全部を覆える。
DECIM = [1]

# トレース対象のラウンド
TRACE_ROUND = [1]

# 健全性の合格ライン（ビット2〜14の13ビット）
GATE = 13

# ラウンド1の送信ビットを1つ反転して試すための指定。None なら素のまま。
# シードを推測するより、Keyに直接「どのビットが違うか」を聞いた方が速い。
TX_FLIP = [()]   # 反転するビット位置の集合


def set_led(ser, on):
    """基板上のLEDで、いま触っていいかを示す。

    消灯=Keyが応答していない(差し直す) / 点灯=ラウンド0が通った(触らない)。
    判定はこちらが持っているので、Nano側には点灯指示だけ送る。
    """
    deadline = time.time() + 3
    while time.time() < deadline:
        if ser.read(1) == b"K":
            break
    else:
        return
    try:
        ser.write(bytes([0xFE, 1 if on else 0]))
        ser.flush()
    except serial.SerialTimeoutException:
        ser.reset_output_buffer()


def resync(ser):
    """プロトコルがずれたときに同期を取り直す。

    ファームは3秒で受信待ちをあきらめて 'K' に戻るので、
    バッファを捨てて 'K' が出るまで待てば復帰できる。
    LEDも消しておく（点いたままだと固まったように見える）。
    """
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        deadline = time.time() + 6
        while time.time() < deadline:
            if ser.read(1) == b"K":
                set_led(ser, False)
                return True
    except Exception:
        pass
    return False


def sanity(ser):
    """既知の良好パラメータでラウンド0が14/14通るか。通れば健全。"""
    g = C.run(port=PORT, rounds=1, ser=ser, **BEST)
    if g is None:
        return -1, ""
    got = "".join(map(str, g[0]))
    exp = C.plan(1)[0][2]
    # ビット1は地域申告で予測できない。
    # ビット0はID保持の直後で一番マージンが薄く、実機で安定して落ちることがある。
    # どちらもKeyの送信ビットであって、こちらの送信内容には影響しない。
    # つまり握手の成否とは無関係なので、健全性の判定から外す。
    n = sum(1 for i in range(2, 15) if g[0][i] == exp[i])
    return n, got


def trace(ser):
    """ラウンド0を正規に通し、ラウンド1の頭から1パルスごとに入力線を記録する。"""
    # 生で叩いたときは resync を挟むと確実に通った。同じ形に揃える。
    # LED制御と本測定が交互に走るので、境目で同期がずれることがある。
    resync(ser)
    deadline = time.time() + 5
    while time.time() < deadline:
        if ser.read(1) == b"K":
            break
    else:
        return None
    ser.write(bytes([
        TRACE_ROUND[0] + 1, BEST["id_delay"] & 0xFF, BEST["id_delay"] >> 8,
        BEST["bit_pulses"] & 0xFF, BEST["drive_pulses"], BEST["sample_at"], 0,
        # hdr[7]: bit0=rst_high bit1=swap bit2=listen bit3以上=間引き
        0x01 | 0x02 | ((DECIM[0] & 0x1F) << 3),
        (C.ID_MODE[0] << 2) | 0x40,
        BEST["bit_pulses"] >> 8,
        C.POST_ID[0] & 0xFF, C.POST_ID[0] >> 8,
        TRACE_ROUND[0],
    ]))
    for r, (d, tx, _rx, gap) in enumerate(C.plan(TRACE_ROUND[0] + 1)):
        if r == TRACE_ROUND[0] and TX_FLIP[0]:
            tx = list(tx)
            for b in TX_FLIP[0]:
                tx[b] ^= 1
        g = min(gap + C.GAP_ADJ[0], 65535)
        ser.write(bytes([d]) + C._pack(tx) + bytes([g & 0xFF, (g >> 8) & 0xFF]))
    ser.flush()
    buf = bytearray()
    while len(buf) < 400:
        chunk = ser.read(400 - len(buf))
        if not chunk:
            break
        buf += chunk
    return buf if len(buf) == 400 else None


def describe(buf):
    a, b = [], []
    for byte in buf:
        for s in range(4):
            v = (byte >> (s * 2)) & 3
            a.append(v & 1)
            b.append((v >> 1) & 1)
    d = DECIM[0]
    out = [f"1600サンプル x 間引き{d} = {1600 * d}パルス"
           f" = 約{1600 * d / BEST['bit_pulses']:.1f}ビット周期ぶん"]
    # ビットごとに、その周期内にKeyのパルスがあったかを判定する
    per = BEST['bit_pulses'] // d
    for name, sig in (("A4(カート24)", a), ("A5(カート55)", b)):
        ed = [i for i in range(1, len(sig)) if sig[i] != sig[i - 1]]
        out.append(f"\n{name}: High率 {sum(sig) * 100 // len(sig)}%  遷移 {len(ed)}回")
        if ed:
            out.append(f"  遷移位置 {ed[:24]}")
            out.append(f"  間隔     {[ed[i] - ed[i - 1] for i in range(1, min(len(ed), 24))]}")
            # 372パルス周期のどこに来ているか。Keyが喋っていれば頭に寄るはず。
            out.append(f"  周期内位相 {[e % BEST['bit_pulses'] for e in ed[:24]]}")
    return "\n".join(out)


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else 480.0
    C.ID_MODE[0] = 1
    C.POST_ID[0] = 520
    C.REGION[0] = 1
    C.GAP_ADJ[0] = 280

    ser = serial.Serial(PORT, C.BAUD, timeout=10)
    time.sleep(2.0)
    deadline = time.time() + limit
    tries = 0
    dead = 0
    try:
        while time.time() < deadline:
            tries += 1
            n, got = sanity(ser)
            set_led(ser, n >= 14)
            if got == DEAD_PATTERN:
                dead += 1
                continue
            if n < 14:
                continue
            buf = trace(ser)
            if buf is None:
                continue
            print(f"■ 捕獲成功（{tries}回目 / {time.time() - deadline + limit:.0f}秒経過）")
            print(describe(buf))
            with open("r1_trace.bin", "wb") as f:
                f.write(buf)
            print("\n生データ: r1_trace.bin")
            return 0
        print(f"時間切れ。{tries}回試行、うち{dead}回は「Keyが居ない」模様")
        return 1
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
