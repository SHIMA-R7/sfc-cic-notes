"""
測定経路の作り直し。

■ なぜ作り直したか
`sanity()` `set_led()` `trace()` を別々の関数にして組み合わせたところ、
接触が良好（健全性13/13が6回連続）でファームも正常（ヘッダを生で組めば
0.2秒で400バイト返る）なのに、交互に呼ぶと止まるようになった。
'K' の待ち受けが関数をまたいで取り合いになっていたのが原因と思われる。

原因を追うより、**生で通った手順をそのまま1つの関数に閉じ込める**方が速い。
ここでは1回のやりとりを `exchange()` に集約し、
'K' の待ち受け・ヘッダ送信・本体送信・応答受信を分割しない。

■ プロトコル（Nano-3側と対）
    ホスト -> Nano  1バイト 0xFE + 1バイト  = LED制御（握手はしない）
    ホスト -> Nano  13バイトのヘッダ + 各ラウンド6バイト
    Nano -> ホスト  通常は 2*rounds+4 バイト / トレース時は 400 バイト
"""

import time

import serial

import cic_probe as C

PORT = "COM8"
BAUD = C.BAUD

# 実機で確定したパラメータ
# 駆動幅と読み位置。
# シミュレータの実測は「送出を0として 読み+2命令=8パルス / 駆動終了+3命令=12パルス」。
# ただしこちらのクロックはソフトウェアで作っているので、実機で最適な値とは
# 一致しない（実際に12/8にしたらラウンド0の読みまで崩れた）。
# 実機で通っている 24/16 を既定にし、必要なら掃引して選ぶ。
BEST = dict(id_delay=2496, bit_pulses=372, drive_pulses=24, sample_at=16)
GATE = 13          # 健全性の合格ライン（ラウンド0のビット2〜14）
DEAD = "110100101101001"   # Keyが居ない／死んでいるときの模様

# 向きを待ちのどこで切り替えるか。0.0=待ちの直前 / 1.0=待ちの直後（従来）
SWITCH_FRAC = [1.0]


def wait_ready(ser, timeout=6.0):
    """'K' が来るまで待つ。取りこぼしたら3秒後にまた来る。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.read(1) == b"K":
            return True
    return False


def exchange(ser, rounds, trace_round=None, tx_flip=(), decim=1,
             k_src="lock", dir_src="lock", region_value=None, plan=None):
    """1セッションを最初から最後まで、途中で関数を分けずに実行する。

    trace_round を指定すると、そのラウンドを1パルス単位で記録した
    400バイトが返る。指定しなければ各ラウンドの受信ビットが返る。
    """
    if not wait_ready(ser):
        return None

    prev = (C.K_SRC[0], C.DIR_SRC[0], C.K_OVERRIDE[0])
    prev_rv = C.REGION_VALUE[0]
    C.K_SRC[0], C.DIR_SRC[0] = k_src, dir_src
    if region_value is not None:
        C.REGION_VALUE[0] = region_value
    try:
        # plan を渡せばそれを使う。シミュレータ（本家ROM）が出した正解表を
        # そのまま流し込むため。渡さなければ従来のモデルで作る。
        plan = plan[:rounds] if plan else C.plan(rounds)
    finally:
        C.K_SRC[0], C.DIR_SRC[0], C.K_OVERRIDE[0] = prev
        C.REGION_VALUE[0] = prev_rv

    tracing = trace_round is not None
    hdr = bytes([
        rounds,
        BEST["id_delay"] & 0xFF, BEST["id_delay"] >> 8,
        BEST["bit_pulses"] & 0xFF,
        BEST["drive_pulses"], BEST["sample_at"], 0,
        0x01 | 0x02 | ((decim & 0x1F) << 3),      # rst_high | swap | 間引き
        (C.ID_MODE[0] << 2) | (0x40 if tracing else 0),
        BEST["bit_pulses"] >> 8,
        C.POST_ID[0] & 0xFF, C.POST_ID[0] >> 8,
        trace_round if tracing else 0,
    ])
    body = b""
    for r, (d, tx, _rx, gap) in enumerate(plan):
        if tracing and r == trace_round and tx_flip:
            tx = list(tx)
            for b in tx_flip:
                if b < len(tx):
                    tx[b] ^= 1
        g = min(gap + C.GAP_ADJ[0], 65535)
        # 切り替え位置。SWITCH_FRAC=0.0で待ちの直前、1.0で待ちの直後。
        sw = int(g * SWITCH_FRAC[0])
        body += bytes([d]) + C._pack(tx) + bytes([g & 0xFF, (g >> 8) & 0xFF,
                                                  len(tx),
                                                  sw & 0xFF, (sw >> 8) & 0xFF])
    ser.write(hdr + body)
    ser.flush()

    need = 400 if tracing else rounds * 2 + 4
    buf = bytearray()
    t0 = time.time()
    while len(buf) < need and time.time() - t0 < 15:
        chunk = ser.read(need - len(buf))
        if not chunk:
            break
        buf += chunk
    if len(buf) < need:
        return None
    if tracing:
        return bytes(buf), plan
    got = [C._unpack(buf[i * 2:i * 2 + 2], len(plan[i][1]))
           for i in range(rounds)]
    return got, plan


def led(ser, on):
    if not wait_ready(ser, 3.0):
        return
    try:
        ser.write(bytes([0xFE, 1 if on else 0]))
        ser.flush()
    except serial.SerialTimeoutException:
        pass


def check(ser):
    """ラウンド0で健全性を見る。(点数, 受信文字列) を返す。

    ビット0はID保持の直後でマージンが薄く、ビット1は地域申告。
    どちらもKeyの送信ビットで、こちらの送信内容には影響しないので判定から外す。
    """
    r = exchange(ser, 1)   # ラウンド0はkもdirも使わない
    if r is None:
        return -1, ""
    got, plan = r
    exp = plan[0][2]
    n = sum(1 for i in range(2, 15) if got[0][i] == exp[i])
    return n, "".join(map(str, got[0]))


def bit_levels(raw, decim, direction=0, bit_pulses=372, nbits=16):
    """トレースからビットごとの「Highだったサンプル数」を出す。

    0=Keyが0を送信 / 1〜20=パルスで1 / 21以上=そのビットで死亡。

    **監視する線は向きで変わる。** swap=True の配線では
      dir=0 -> こちらはA4を駆動、Keyの線はA5
      dir=1 -> こちらはA5を駆動、Keyの線はA4
    向きを渡さないと自分の出力を記録してしまう（実際にやらかした）。
    """
    shift = 1 if direction == 0 else 0     # 1=A5 / 0=A4
    b = []
    for byte in raw:
        for s in range(4):
            b.append((byte >> (s * 2 + shift)) & 1)
    per = bit_pulses // decim
    return [sum(b[i * per:(i + 1) * per]) for i in range(nbits)]


def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=6, write_timeout=5)
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser
