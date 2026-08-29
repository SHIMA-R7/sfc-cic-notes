"""
本家CICのLock役とKey役を向かい合わせに走らせ、握手の中身を取り出す。

■ 何のために
これまでアセンブラを読んで移植し、外れるたびに実機で総当りしてきた。
2個のROMを実際に会話させれば、**ストリーム・X・ラウンド長・レジスタ0の残留値**が
推測ゼロで手に入る。

■ 配線（segherの pinout.txt より）
    P0.0 DATA_OUT   P0.1 DATA_IN   P0.2 SEED   P0.3 LOCK/-KEY
    P1.0 -HOST_RESET   P1.1 SLAVE_CIC_RESET

    lockのP0.0 -> keyのP0.1        keyのP0.0 -> lockのP0.1
    lockのP1.1 -> keyのハードリセット

■ クロック
両者は同じクロックで動く（ロックステップ）。命令ごとにサイクル数が違うので、
サイクルの進みが少ない方を進める。

■ ストリームIDについて
Lockは起動時、SEEDピンがHighの間カウンタを回して 1〜f のどれかを選ぶ。
実験を実機と揃えたいときは seed_counts でその回数を指定する
（SuperCICは常に 0xf を要求する固定運用）。
"""

import sys

from sm590 import SM590, load_rom

ROM_PATH = r"C:\SFC-CIC\reference\d411-dis.txt"

ADDR_RUN_HOST = 0x147      # ラウンド終了直後に RUN HOST を呼ぶ場所
ADDR_MAIN_LOOP = 0x17e     # X := 1 で仕切り直すところ
ADDR_DIE = 0x100


class Pair:
    def __init__(self, rom, target_id=0xF):
        self.lock = SM590(rom, is_lock=True)
        self.key = SM590(rom, is_lock=False)
        self.key_held = True          # 起動直後はLockがKeyをリセットしている
        self.target_id = target_id
        self.reset_key()

    def reset_key(self):
        k = self.key
        k.pc = 0
        k.acc = k.bl = k.bm = k.c = k.x = 0
        k.skip = False
        k.ram = [0] * 64
        k.ports = [0, 0, 0, 0]
        k.cycles = self.lock.cycles

    def wire(self):
        """毎サイクル、互いの入力を相手の出力から作り直す。

        **データ線は2本の共有ノード。** CICは状況に応じて出力するピンを
        入れ替える（NEXT STREAM BIT の `&5`(ビット0) と `&2`(ビット1) がそれ）。
        片方向だけ張っていたため、入れ替わったときにビットが届いていなかった。

            ノードA = lock P0.0 ⇔ key P0.1
            ノードB = lock P0.1 ⇔ key P0.0

        各ノードの値は両者の駆動のOR。自分の出力は port_r 側でORされる。
        """
        lo, ko = self.lock.ports[0], self.key.ports[0]
        node_a = (lo & 1) | ((ko >> 1) & 1)
        node_b = ((lo >> 1) & 1) | (ko & 1)
        self.lock.inputs[0] = 0x8 | node_a | (node_b << 1) | (self.seed_bit() << 2)
        self.key.inputs[0] = node_b | (node_a << 1)
        # Lockの P1.1 がKeyのリセット
        held = bool(self.lock.ports[1] & 2)
        if held and not self.key_held:
            self.reset_key()
        if self.key_held and not held:
            # **解除された瞬間に**サイクルを揃える。
            # リセットを掛けた時点で揃えると、保持中にLockが進んだぶん
            # Keyが「遅れている」ことになり、解除後に数命令ぶん先行してしまう。
            # その結果、Lockのパルスが Keyの「両線Low」検査の窓に入って
            # Keyが先に死んでいた。
            self.key.cycles = self.lock.cycles
        self.key_held = held

    def seed_bit(self):
        """Lockのシード選択。

        Lockは SEED ピンがHighの間 [1:1] を回し続け、その値がストリームIDになる。
        回数で間接的に決めると wire() の呼ばれ方に左右されるので、
        **[1:1] が目標値に届くまでHighを返す**形にした。
        実機実験はSuperCICがハードコードしている 0xf で行ったので、
        揃えるときは target_id=0xf を指定する。
        """
        return 1 if self.lock.ram[0x11] < self.target_id else 0

    def step(self):
        """サイクルの進みが遅い方を1命令進める。"""
        self.wire()
        if self.key_held or self.key.cycles > self.lock.cycles:
            self.lock.step()
        else:
            self.key.step()


def main():
    rom, n = load_rom(ROM_PATH)
    print(f"ROM {n}番地")
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 14
    p = Pair(rom, seed_counts=seed)

    rounds = []
    last_lock_pc = -1
    for i in range(3_000_000):
        pc = p.lock.pc
        p.step()
        if pc == ADDR_RUN_HOST and last_lock_pc != ADDR_RUN_HOST:
            rounds.append((p.lock.x, list(p.lock.ram[0:16]), list(p.lock.ram[16:32]),
                           list(p.key.ram[0:16]), list(p.key.ram[16:32])))
            if len(rounds) >= 6:
                break
        last_lock_pc = pc
        if p.lock.pc == ADDR_DIE and i > 5000:
            print(f"lockがDIEに落ちた（{i}ステップ目）")
            break

    print(f"ラウンド境界を {len(rounds)} 回通過\n")
    for r, (x, l0, l1, k0, k1) in enumerate(rounds):
        print(f"R{r}: 直後のX={x:x}")
        print(f"   lock bank0 {''.join(f'{v:x}' for v in l0)}   bank1 {''.join(f'{v:x}' for v in l1)}")
        print(f"   key  bank0 {''.join(f'{v:x}' for v in k0)}   bank1 {''.join(f'{v:x}' for v in k1)}")
    if rounds:
        print("\nストリーム(LSBのみ) lock bank0 / bank1")
        for r, (x, l0, l1, _k0, _k1) in enumerate(rounds):
            print(f"  R{r}  {''.join(str(v & 1) for v in l0)}  {''.join(str(v & 1) for v in l1)}")


if __name__ == "__main__":
    main()
