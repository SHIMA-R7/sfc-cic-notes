"""
SuperCICのLock(pic14)とKey(pic14)を両方Pythonで走らせ、向かい合わせる。

■ なぜ必要か
実機テストで最初のビット交換から即座に不一致が起きた。データ線の入れ替え
(A4/A5)を試しても不一致の発生命令数が1命令も変わらず、配線問題ではなさそう。
cic_sim.py（D411のROM同士）でやったのと同じ手法で、
**先に純粋ソフトウェア同士の握手が通るかを確かめる**。

通れば: pic14.py のCPUコアは正しい。AVR移植側かハードウェアの問題。
通らなければ: CPUエミュレーションのバグ。実機に持ち込む前に直せる。
"""

import sys

from pic14 import PIC14, load_hex

LOCK_HEX = r"C:\SFC-CIC\reference\supercic-lock.hex"
KEY_HEX = r"C:\SFC-CIC\reference\supercic-key.hex"


class Pair:
    def __init__(self):
        self.lock = PIC14(load_hex(LOCK_HEX))
        self.key = PIC14(load_hex(KEY_HEX))
        self.key_prev = False
        # LOCK/-KEY 選択ピン(GP3): Lockは1、Keyは0。実機のCIC pinoutと同じ扱い。
        # supercic-key.asm は GP2(=0x05 bit2) を外部割り込みトリガに使う設計
        # だったはず。lock/key共通で port_in の初期値は0でよい
        # (ボタン等は押されていない)。

    def wire(self):
        """毎ステップ、互いのデータ線・クロック・リセットを結線しなおす。"""
        lo, ko = self.lock.ram[0x07], self.key.ram[0x07]
        # PORTC bit0/1 がデータ。実機同様、両者の出力をORした値を相手が読む
        # （出力側のTRISが1(入力)なら自分の駆動は0とみなす）
        lt = self.lock.ram[0x87]  # lockのTRISC
        kt = self.key.ram[0x87]
        lo_drv = lo & ~lt & 0x03
        ko_drv = ko & ~kt & 0x03
        bus = (lo_drv | ko_drv) & 0x03
        # Lockの PORTC.2 出力（リセットトリガ）を Key の GP2 入力へ直結する。
        # データ線2本しか結線しておらず、Keyの割り込み契機が常にLowのまま
        # だったため、上のcheck_interrupt()を実装しても一度も発火しなかった。
        reset_line = (lo & 0x04)
        self.lock.port_in[0x07] = bus
        self.key.port_in[0x07] = bus | reset_line
        # Lock の PORTC.2 (0x07 bit2) が Key の外部割り込みトリガ。
        # 実際の割り込み処理(ISRへのジャンプ)は PIC14.check_interrupt() が
        # port_in の履歴を見て行う。ここでは電気的な結線だけを行えばよい
        # （以前はここでKeyオブジェクトを丸ごと再構築するハードリセット処理を
        # 重複して持っていて、check_interrupt()による正規の割り込みより先に
        # 発火し、Keyが毎回ゼロから起動シーケンスをやり直していた）。

    key_prev = False

    def step(self):
        self.wire()
        # サイクルの進みが少ない方を1命令進める
        if self.key.cycles <= self.lock.cycles:
            self.key.step()
        else:
            self.lock.step()


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2_000_000
    p = Pair()
    mismatch_at = None
    for i in range(n):
        p.step()
        if p.lock.ram[0x43] & 0x02 and mismatch_at is None:
            mismatch_at = i
            print(f"最初の不一致: {i}命令目 (lock.pc=0x{p.lock.pc:03x})")
            break
        if p.lock.halted or p.key.halted:
            print(f"{i}命令目でSLEEPに入った")
            break
    else:
        print(f"{n}命令実行して不一致なし")

    print(f"lock seed {''.join(f'{p.lock.ram[0x21+k]:x}' for k in range(15))}")
    print(f"key  seed {''.join(f'{p.lock.ram[0x31+k]:x}' for k in range(15))}")
    print(f"key.pc=0x{p.key.pc:03x}")


if __name__ == "__main__":
    main()
