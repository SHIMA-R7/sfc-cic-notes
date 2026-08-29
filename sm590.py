"""
Sharp SM590 のエミュレータと、本家CIC(D411)のROMを走らせるための土台。

■ なぜこれを作るか
2日かけて「アセンブラを目で追って移植する」をやってきたが、
そのたびに上位ビットやラウンド構造を読み違えて実機で総当りに戻っていた。

■ 命令符号について（一度間違えたので記録）
MAMEのSM590は 0x48=RC / 0x49=SC / 0x54=TC / 0x44=COMA。
segherのd411の表記はその逆に見えたので「MAMEが正しい」と直したが、**それが誤り**。

segher自身が 3195a の冒頭にこう書いている:

    ;; NOTE: this chip has the opcodes for sc/rc and coma/nega swapped,
    ;; compared to the other chips!

**チップごとに符号が違う。** d411 は segher の表記どおり
0x48=SC / 0x49=RC / 0x54=COMA / 0x44=TC が正しい。
この3つはすべて mangle の中にあり、間違えると
R0（mangle前）は合うのに R1（mangle後）から壊れる。

■ 出典
    命令の意味   MAME src/devices/cpu/sm510/{sm590op,sm510op,sm590,sm510base}.cpp
    ROM          segher の d411 逆アセンブル（reference/d411-dis.txt）

■ ROMの復元
ビットストリーム(d411.txt)は物理配置なので、そのままでは番地に並ばない。
逆アセンブル一覧が「番地: バイト列」の対応表になっているので、そちらから組む。

■ PCの進み方
SM590のPCは下位7ビットがLFSR（newbit = bit0 == bit1）。
逆アセンブルの番地が 000,040,070,078... と飛ぶのはこのため。
"""

import re


PAGEMASK = 0x7F
# 逆アセンブルの番地は 0x37f まで伸びる。512バイトではなく1024バイト品。
# MAMEの do_branch も (pu<<9)|(pm<<7)|pl で最大 0x3ff を作る。
PRGMASK = 0x3FF
ROM_SIZE = 1024


def next_pc(pc):
    """LFSRで次の番地を出す。MAMEの increment_pc と同じ。

    2バイト命令の第2バイトは **番地+1 ではなく LFSRの次** に置かれる。
    ここを線形に置いてしまい、引数が読めずに最初のTLSで0番地へ飛んでいた。
    """
    msb = (PAGEMASK >> 1) ^ PAGEMASK
    feed = 0 if ((pc >> 1) ^ pc) & 1 else msb
    return (feed | ((pc >> 1) & (PAGEMASK >> 1)) | (pc & ~PAGEMASK)) & PRGMASK


def load_rom(path):
    """逆アセンブル一覧から 番地->バイト のROM像を作る。"""
    rom = [0] * ROM_SIZE
    seen = set()
    # 16進の a-f は英字でもあるので「英字が来たら命令名」では切れない。
    # 実際 "270: 7d fa   tml 37a" の 2バイト目 fa で誤発火し、
    # パラメータを取りこぼして分岐先が壊れていた。
    # 一覧は「バイト列 → 空白2個以上 → 命令名」の桁組みなので、そこで切る。
    pat = re.compile(r"^([0-9a-f]{3}):\s+([0-9a-f]{2}(?:\s[0-9a-f]{2})?)\s{2,}")
    for line in open(path, encoding="utf-8", errors="replace"):
        m = pat.match(line.strip())
        if not m:
            continue
        addr = int(m.group(1), 16)
        for bb in m.group(2).split():
            rom[addr] = int(bb, 16)
            seen.add(addr)
            addr = next_pc(addr)
    return rom, len(seen)


class SM590:
    """MAMEの実装に対応させたSM590コア。周辺は最小限。"""

    def __init__(self, rom, is_lock):
        self.rom = rom
        self.acc = 0
        self.bl = 0
        self.bm = 0
        self.c = 0
        self.x = 0
        self.pc = 0
        self.prev_pc = 0
        self.op = 0
        self.prev_op = 0
        self.param = 0
        self.skip = False
        self.stack = [0, 0]
        self.ram = [0] * 64          # BM(0-3) x BL(0-15)
        # ポート R0-R3。R0 のビット3が LOCK/-KEY 選択（1=lock）。
        self.ports = [0, 0, 0, 0]
        self.inputs = [0x8 if is_lock else 0x0, 0, 0, 0]
        self.halted = False
        self.cycles = 0

    # --- メモリ ---
    def ram_addr(self):
        return ((self.bm << 4) | self.bl) & 0x3F

    def ram_r(self):
        return self.ram[self.ram_addr()] & 0xF

    def ram_w(self, v):
        self.ram[self.ram_addr()] = v & 0xF

    # --- PC ---
    def increment_pc(self):
        self.pc = next_pc(self.pc)

    def do_branch(self, op, param):
        """TL/TLS の飛び先。

        MAMEのSM590は A9=op.b1 / A8=op.b0 / A7=param.b7 だが、
        このROM(D411)ではその並びでは合わない。segherの逆アセンブルの番地を
        突き合わせて確かめた対応は下記（4例で検証済み）:

            78/80 -> 0x100    79/70 -> 0x270
            7d/cb -> 0x34b    7c/b1 -> 0x131

        この違いに気づかず、INIT STREAM(0x270) を飛び越して
        DIEの領域(0x170)へ落ちていた。
        """
        a9 = op & 1
        a8 = (param >> 7) & 1
        a7 = (op >> 1) & 1
        self.pc = ((a9 << 9) | (a8 << 8) | (a7 << 7) | (param & 0x7F)) & PRGMASK

    def push(self):
        self.stack[1] = self.stack[0]
        self.stack[0] = self.pc

    def pop(self):
        self.pc = self.stack[0] & PRGMASK
        self.stack[0] = self.stack[1]

    # --- ポート ---
    def port_w(self, offset, data):
        self.ports[offset & 3] = data & 0xF

    def port_r(self, offset):
        o = offset & 3
        return (self.ports[o] | self.inputs[o]) & 0xF

    # --- 実行 ---
    def step(self):
        self.prev_pc = self.pc
        op = self.rom[self.pc]
        self.increment_pc()
        self.prev_op, self.op = self.op, op
        self.cycles += 1

        if (op & 0xF8) == 0x78:            # TL / TLS は引数を1バイト取る
            self.param = self.rom[self.pc]
            self.increment_pc()
            self.cycles += 1

        if self.skip:                      # 直前の命令がスキップを立てていた
            self.skip = False
            self.op = 0                    # LAX連続の判定を壊さないため
            return

        self.execute(op)

    def execute(self, op):
        hi = op & 0xF0
        if hi == 0x00:                     # ADX x
            self.acc += op & 0xF
            self.skip = bool(self.acc & 0x10)
            self.acc &= 0xF
        elif hi == 0x10:                   # TAX x
            self.skip = (self.acc == (op & 0xF))
        elif hi == 0x20:                   # LBL x
            self.bl = op & 0xF
        elif hi == 0x30:                   # LAX x
            # MAMEの基底実装は「LAXの直後のLAXは無視」だが、このROMでは
            # 初期化に `ldi 6 / ldi b` のような連続があり、そのルールだと
            # 前者が生きて 6 になる。本家のシードテーブル（SuperCICが使い
            # 実機で動いている値）は b なので、素直な代入が正しい。
            self.acc = op & 0xF
        elif hi >= 0x80:                   # TR（ページ内ジャンプ）
            self.pc = (self.pc & ~PAGEMASK) | (op & PAGEMASK)
        elif (op & 0xFC) == 0x60:          # TMI x
            self.skip = bool(self.ram_r() & (1 << (op & 3)))
        elif (op & 0xFC) == 0x64:          # TBA x
            self.skip = bool(self.acc & (1 << (op & 3)))
        elif (op & 0xFC) == 0x68:          # RM x
            self.ram_w(self.ram_r() & ~(1 << (op & 3)))
        elif (op & 0xFC) == 0x6C:          # SM x
            self.ram_w(self.ram_r() | (1 << (op & 3)))
        elif (op & 0xFC) == 0x74:          # LBM x
            self.bm = op & 3
        elif (op & 0xFC) == 0x78:          # TL
            self.do_branch(op, self.param)
        elif (op & 0xFC) == 0x7C:          # TLS
            self.push()
            self.do_branch(op, self.param)
        elif op == 0x40:                   # LDA
            self.acc = self.ram_r()
        elif op == 0x41:                   # EXC
            a = self.acc
            self.acc = self.ram_r()
            self.ram_w(a)
        elif op == 0x42:                   # EXCI
            a = self.acc
            self.acc = self.ram_r()
            self.ram_w(a)
            self.bl = (self.bl + 1) & 0xF
            self.skip = (self.bl == 0)
        elif op == 0x43:                   # EXCD
            a = self.acc
            self.acc = self.ram_r()
            self.ram_w(a)
            self.bl = (self.bl - 1) & 0xF
            self.skip = (self.bl == 0xF)
        elif op == 0x44:                   # TC（D411では 0x54 と入れ替わり）
            self.skip = bool(self.c)
        elif op == 0x45:                   # TAM
            self.skip = (self.acc == self.ram_r())
        elif op == 0x46:                   # ATR
            self.port_w(self.bl, self.acc)
        elif op == 0x47:                   # MTR
            self.port_w(self.bl, self.ram_r())
        elif op == 0x48:                   # SC（D411では 0x49 と入れ替わり）
            self.c = 1
        elif op == 0x49:                   # RC
            self.c = 0
        elif op == 0x4A:                   # STR
            self.ram_w(self.acc)
        elif op == 0x4B:                   # CEND
            self.halted = True
        elif op == 0x4C:                   # RTN
            self.pop()
        elif op == 0x4D:                   # RTNS
            self.pop()
            self.skip = True
        elif op == 0x50:                   # INBM
            self.bm = (self.bm + 1) & 3
        elif op == 0x51:                   # DEBM
            self.bm = (self.bm - 1) & 3
        elif op == 0x52:                   # INBL
            self.bl = (self.bl + 1) & 0xF
            self.skip = (self.bl == 0)
        elif op == 0x53:                   # DEBL
            self.bl = (self.bl - 1) & 0xF
            self.skip = (self.bl == 0xF)
        elif op == 0x54:                   # COMA（D411では 0x44 と入れ替わり）
            self.acc ^= 0xF
        elif op == 0x55:                   # RTA
            self.acc = self.port_r(self.bl)
        elif op == 0x56:                   # BLTA
            self.acc = self.bl
        elif op == 0x57:                   # XBLA
            a = self.acc
            self.acc = self.bl
            self.bl = a
        elif op == 0x5C:                   # ATX
            self.x = self.acc
        elif op == 0x5D:                   # EXAX
            a = self.acc
            self.acc = self.x
            self.x = a
        elif op == 0x70:                   # ADD
            self.acc = (self.acc + self.ram_r()) & 0xF
        elif op == 0x71:                   # ADS
            self.acc += self.ram_r()
            self.skip = bool(self.acc & 0x10)
            self.acc &= 0xF
        elif op == 0x72:                   # ADC
            self.acc += self.ram_r() + self.c
            self.c = (self.acc >> 4) & 1
            self.acc &= 0xF
        elif op == 0x73:                   # ADCS
            self.acc += self.ram_r() + self.c
            self.c = (self.acc >> 4) & 1
            self.skip = (self.c == 1)
            self.acc &= 0xF
        else:
            raise RuntimeError(f"未対応の命令 0x{op:02x} @ 0x{self.prev_pc:03x}")


def main():
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else r"C:\SFC-CIC\reference\d411-dis.txt"
    rom, n = load_rom(path)
    print(f"ROM復元: {n}/{ROM_SIZE} 番地に値が入った")
    holes = [i for i in range(ROM_SIZE) if rom[i] == 0]
    print(f"  値0のまま(未使用またはnop): {len(holes)}箇所")

    cpu = SM590(rom, is_lock=True)
    for _ in range(2000):
        cpu.step()
        if cpu.halted:
            break
    print(f"lock役を2000ステップ実行: PC=0x{cpu.pc:03x} ACC={cpu.acc:x} "
          f"BM={cpu.bm} BL={cpu.bl:x} X={cpu.x:x} halted={cpu.halted}")
    print(f"  RAM bank0 {[f'{v:x}' for v in cpu.ram[0:16]]}")
    print(f"  RAM bank1 {[f'{v:x}' for v in cpu.ram[16:32]]}")


if __name__ == "__main__":
    main()
