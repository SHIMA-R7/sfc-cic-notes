"""
CIC認証プロトコルのホスト側モデル（第0段）。

■ 目的
Lock側（＝コンソール役。こちらが作る側）が、
  - 自分が送るべき15ビット
  - Key側（＝カート内CIC）が返すべき15ビット
の両方を、ラウンドごとに算出できることを確かめる。

これができると、実機で観測した応答をビット単位で答え合わせできる。
今までCICの実験が進まなかったのは「成立したかどうか」を電圧のしきい値で
判定しようとしていたからで、正解表があれば判定そのものが要らなくなる。

■ 出典と扱い
アルゴリズムと定数は SuperCIC (ikari_01 / borti4938) の逆アセンブル
supercic-lock.asm に基づく。あちらは GPL。
このファイルは公開リポジトリ（MIT）には含めず、ローカル専用とする。

■ 用語の注意
元アセンブラのルーチン名は紛らわしい。
    mangle_lock  -> 0x31-0x3f (key seed)  を変換する
    mangle_key   -> 0x21-0x2f (lock seed) を変換する
名前は「どちらの変換規則か」を指しており、配列の持ち主とは対応していない。
ここでは配列の持ち主で呼ぶ。
"""

# 0x21-0x2f: こちらが送るビット列の元
LOCK_SEED = [0xb, 0x1, 0x4, 0xf, 0x4, 0xb, 0x5, 0x7, 0xf, 0xd, 0x6, 0x1, 0xe, 0x9, 0x8]

# 0x31-0x3f: Keyが返すビット列の元。
# [1] は Key側の自己申告（地域判定）で埋まる枠。Lockは0で初期化しておき、
# ラウンド0のビット1を受け取った時点で checkkey が書き換える:
#     受信1 -> 411(NTSC) -> 0x9 を書く
#     受信0 -> 413(PAL)  -> 0x6 を書く
# **この書き込みは以降すべてのmangleに参加する。**
# ここを0のまま固定していたため、ラウンド0（mangle前）は一致するのに
# ラウンド1（最初のmangle出力）から全部ずれていた。
KEY_SEED = [0xf, 0x0, 0xa, 0x1, 0x8, 0x5, 0xf, 0x1, 0x1, 0xe, 0x1, 0x0, 0xd, 0xe, 0xc]

ROUND_BITS = 15      # 1ラウンドで交換するビット数（配列長と同じ）
MANGLE_PER_ROUND = 3  # 1ラウンドの終わりにmangleを3回かける


# 元CICのmangleは1周78サイクル（SuperCICはそれに合わせて9個のnopを詰めている）。
# 周回数は桁上がりが消えるまでで、ラウンドごとに変わる。
# こちらはLock役なので、Keyが計算している間ちょうど同じだけ待たなければならない。
MANGLE_CYCLES_PER_ITER = 78
MANGLE_CALL_OVERHEAD = 4      # movf/movwf と return


def mangle(s):
    """15要素の配列を1回変換し、ループを何周したかを返す。

    PICは8bitなので、途中の桁上がりを捨てずに 0xff でマスクし続ける必要がある。
    ニブルだけ見て実装すると合わない（bit4のキャリーが分岐条件そのものなので）。

    周回数を返すのは、Key側がこの計算に使う時間をこちらが待つため。
    """
    iters = 0
    buf = s[14]
    a = buf
    while True:
        iters += 1
        a = (a + 1) & 0xFF
        s[0] = (s[0] + a) & 0xFF

        t40 = s[1]
        s[1] = (s[1] + s[0]) & 0xFF
        s[1] = (s[1] + 1) & 0xFF
        s[1] = (~s[1]) & 0xFF

        t41 = s[2]
        s[2] = s[2] & 0x0F
        s[2] = (s[2] + (t40 & 0x0F)) & 0xFF
        s[2] = (s[2] + 1) & 0xFF

        # bit4への桁上がりの有無で、以降の処理が1要素ぶんずれる
        # （元コードの withskip / withoutskip）
        i0 = 2 if (s[2] & 0x10) else 3

        s[i0] = (s[i0] + t41) & 0xFF

        t40 = s[i0 + 1]
        s[i0 + 1] = (s[i0 + 1] + s[i0]) & 0xFF

        t41 = s[i0 + 2]
        w = ((t40 & 0x0F) + 8) & 0xFF
        if not (w & 0x10):
            w = (w + s[i0 + 2]) & 0xFF
        s[i0 + 2] = w

        w = (t41 + 1) & 0xFF
        s[i0 + 3] = (s[i0 + 3] + w) & 0xFF

        for i in range(i0 + 4, 15):
            w = (s[i - 1] + 1) & 0xFF
            s[i] = (s[i] + w) & 0xFF

        buf = ((buf & 0x0F) + 0x0F) & 0xFF
        a = buf
        if not (buf & 0x10):
            return iters, buf


def mangle_round(lock, key):
    """ラウンド境界の変換。元コードは mangle_lock -> mangle_key の順で呼ぶ。

    (周回数の合計, keyのスクラッチ, lockのスクラッチ) を返す。
    スクラッチ(0x20/0x30)は次ラウンドの先頭ビットになり得るので保持する。
    """
    ik, bk = mangle(key)
    il, bl = mangle(lock)
    return ik + il, bk, bl


class Lock:
    """Lock側の状態。1ラウンドぶんの送信/期待ビットを生成する。"""

    def __init__(self, region_bit=1, region_value=None):
        # region_bit は実機から返ってくる地域申告。日本のカートは1(411/NTSC)。
        # region_value はそのとき keyシード[1] へ書き込むニブル。
        # SuperCICのLockは 411->0x9 / 413->0x6 を書くが、カート内の本物のCICが
        # 実際に持っている値がこれと違えば、mangle後に必ずズレる。
        # ラウンド0で検証できるのはLSBだけなので、上位3ビットは未確認のまま。
        self.region_bit = region_bit
        self.region_value = region_value
        self.lock = list(LOCK_SEED)
        self.key = list(KEY_SEED)
        # 向きの意味（元コードのTRIS設定と受信ビットのシフトから確定）
        #   0 -> データ1番線で送信、0番線で受信
        #   1 -> データ0番線で送信、1番線で受信
        # 送信時は0番/1番の両方に同じ値を書き、方向レジスタ側で実際に駆動する
        # 線を選んでいる。こちらで再現するときも同じにしておくと安全。
        self.direction = 0
        self.round = 0
        # ラウンドの開始レジスタ。Lock側は FSR = 0x20 + k から 0x30 まで回すので
        # 1ラウンドは 16-k ビット。レジスタ0x20はmangleのスクラッチで、
        # k=0 のときは**その値が先頭ビットになる**。
        # 最初のラウンドは movlw 0x1 で k=1（＝配列の頭から15ビット）。
        self.k = 1
        self.lock_buf = 0x0F
        self.key_buf = 0x0F
        self.k_override = None
        # Keyのスクラッチ(0x30)のLSB。モデルの計算では0x0f(=1)になるが、
        # 実機はFF4・マリオRPGともに0を返す。実測を優先する。
        self.key_scratch_bit = 0
        # 向きとラウンド開始位置の出どころ。元コードは 0x37(=key[6]) と読めるが、
        # 実測のk列は lock[6] の下位ニブルと一致した。両方試せるようにする。
        # 原典では次のXは「3回のmangle直後に選ばれているバンクのレジスタ7」。
        # バンク0=lockストリーム / バンク1=keyストリーム。
        # 向きも同じレジスタから取るが、こちらは別バンクの可能性がある。
        # 前回の実験は両方を同じ出どころから取っていたため切り分けできていない。
        self.k_src = 'lock'
        self.dir_src = 'lock'

    def stream_id(self):
        """握手の冒頭に送る4ビット。常に同じストリームを要求するので固定。

        本来Lockは3番ピンのコンデンサ放電時間で16通りから1つを選ぶが、
        選ぶのはLock自身なので固定でよい。SuperCICも固定にしている。
        """
        return KEY_SEED[0] & 0x0F

    def next_round(self):
        """このラウンドで送るビット列と、Keyが返すべきビット列を返す。

        長さは 16-k で毎ラウンド変わる。k=0 ならスクラッチレジスタが先頭に入る。
        """
        k = self.k if self.k_override is None else self.k_override(self.round)
        tx = ([self.lock_buf & 1] + [v & 1 for v in self.lock])[k:]
        ks = (self.key_buf & 1 if self.key_scratch_bit is None
              else self.key_scratch_bit)
        rx = ([ks] + [v & 1 for v in self.key])[k:]
        direction = self.direction

        # ラウンド0の途中で checkkey が地域申告を keyシード[1] へ書き込む。
        # mangleより前に行われるので、ここで反映しておく。
        if self.round == 0:
            self.key[1] = (self.region_value if self.region_value is not None
                           else (0x9 if self.region_bit else 0x6))

        iters = 0
        for _ in range(MANGLE_PER_ROUND):
            n, self.key_buf, self.lock_buf = mangle_round(self.lock, self.key)
            iters += n
        # Keyがmangleに費やす命令サイクル数。1命令 = クロック4パルス。
        gap = iters * MANGLE_CYCLES_PER_ITER + MANGLE_PER_ROUND * 2 * MANGLE_CALL_OVERHEAD
        self.last_gap_pulses = gap * 4

        # 次ラウンドの向きは、変換後のkey配列7番目のLSBで決まる（元コードの0x37）
        self.direction = (self.key if self.dir_src == 'key' else self.lock)[6] & 1
        # 次ラウンドの開始位置。0なら元コードは loop へ戻り k=1 になるが、
        # 実測はスクラッチ始まりを要求している。実機に選ばせるため素直に入れる。
        self.k = (self.key if self.k_src == 'key' else self.lock)[6] & 0x0F
        self.round += 1
        return tx, rx, direction, self.last_gap_pulses


def _bits(v):
    return "".join(str(b) for b in v)


def main():
    lk = Lock()
    print(f"ストリームID(4bit): 0x{lk.stream_id():x}")
    print()
    print("R   dir  送信(Lock)        期待(Key)")
    print("--  ---  ---------------   ---------------")
    for _ in range(16):
        tx, rx, d, gap = lk.next_round()
        print(f"{lk.round - 1:2d}   {d}   {_bits(tx)}   {_bits(rx)}   待ち {gap}パルス")

    # 破綻していないことの確認。全ゼロ化や固定パターン化は失敗の徴候。
    print()
    lk2 = Lock()
    seen_tx, seen_rx = set(), set()
    for _ in range(256):
        tx, rx, _d, _g = lk2.next_round()
        seen_tx.add(tuple(tx))
        seen_rx.add(tuple(rx))
    print(f"256ラウンドで現れた送信パターン: {len(seen_tx)}種")
    print(f"256ラウンドで現れた期待パターン: {len(seen_rx)}種")
    print(f"最終状態 lock={[hex(v) for v in lk2.lock]}")
    print(f"最終状態 key ={[hex(v) for v in lk2.key]}")


if __name__ == "__main__":
    main()
