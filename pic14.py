"""
PIC16F6xx（14bitコア）のインタプリタ。SuperCICのLock側をそのまま実行するためのもの。

■ なぜこれを作るか
supercic-lock.asm は**実際に動いているLock実装**（sanniのOSCRがSA-1を吸えるのも
これをPICに焼いているから）。アルゴリズムを読み取って自分の設計で組み直すのを
2日やって、そのたびに読み違えて実機で総当りに戻った。
**動いているコードをそのまま実行すれば、読み違えようがない。**

アセンブル済みの supercic-lock.hex があるので、アセンブラも要らない。

■ 命令のサイクル数
1命令1サイクル。goto/call/return と、スキップが成立した命令は2サイクル。
PIC16F630 は内蔵4MHzなら 1命令 = 1us。
"""

import re


# PIC16F630のプログラムメモリは1024ワード(word addr 0x000-0x3FF、
# byte addr 0x0000-0x07FF)。__CONFIG(word 0x2007=byte 0x400E)や
# EEPROM(word 0x2100〜=byte 0x4200〜)は別領域だが、hexファイルには
# 同じファイルに同居している。
PROGRAM_BYTE_LIMIT = 0x0800


def load_hex(path):
    """Intel HEX から プログラムメモリ（14bitワードの配列）を作る。

    以前は全アドレスを &0x7FF で無条件にマスクしていたため、
    EEPROMデータ(byte addr 0x4200 -> &0x7FF = 0x100)がプログラム領域の
    0x100番地を静かに上書きしていた。これが実機・シミュレータ双方で
    起きていた「握手が途中で崩れる」の直接原因だった。
    プログラム領域外(config/eeprom)のレコードは読み捨てる。
    """
    mem = [0x3FFF] * 2048
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line.startswith(":"):
            continue
        n = int(line[1:3], 16)
        addr = int(line[3:7], 16)
        typ = int(line[7:9], 16)
        data = bytes(int(line[9 + 2 * i:11 + 2 * i], 16) for i in range(n))
        if typ == 0:
            if addr >= PROGRAM_BYTE_LIMIT:
                continue        # config word / EEPROM。プログラム領域外
            for i in range(0, n, 2):
                if addr + i >= PROGRAM_BYTE_LIMIT:
                    break
                w = data[i] | (data[i + 1] << 8)
                mem[(addr + i) // 2] = w & 0x3FFF
        elif typ == 1:
            break
    return mem


# STATUS のビット
C, DC, Z = 0, 1, 2


class PIC14:
    def __init__(self, prog):
        self.prog = prog
        self.ram = [0] * 512
        self.w = 0
        self.pc = 0
        self.stack = []
        self.skip = False
        self.cycles = 0
        self.halted = False
        # 周辺: ポートの入力値は外から差し込む
        self.port_in = {0x05: 0, 0x07: 0}      # PORTA, PORTC
        self._prev_in = {0x05: 0, 0x07: 0}     # 割り込みエッジ検出用

    # --- レジスタ ---
    def bank(self):
        return (self.ram[0x03] >> 5) & 1       # STATUS の RP0

    # 両バンクから同じ実体が見えるレジスタ。
    # STATUS を見落としてバンク1で BCF STATUS,5 を実行すると別番地に書いてしまい、
    # バンクが戻らなくなる（PORTAのつもりでTRISAを読んで無限ループした）。
    COMMON = {0x02, 0x03, 0x04, 0x0A, 0x0B}

    def addr(self, f):
        if f == 0:                              # INDF
            f = self.ram[0x04] & 0x7F
            if f == 0:
                return 0
        if f in self.COMMON:
            return f
        if (f & 0x7F) >= 0x70:                  # 0x70-0x7f も共通
            return (f & 0x7F) | 0x80
        return f | (self.bank() << 7)

    def rd(self, f):
        a = self.addr(f)
        if a in (0x05, 0x07):                   # PORTA / PORTC は入力を混ぜる
            tris = self.ram[a | 0x80]
            return ((self.ram[a] & ~tris) | (self.port_in[a] & tris)) & 0xFF
        return self.ram[a] & 0xFF

    def wr(self, f, v):
        self.ram[self.addr(f)] = v & 0xFF

    def setz(self, v):
        self.ram[0x03] = (self.ram[0x03] & ~(1 << Z)) | ((1 if (v & 0xFF) == 0 else 0) << Z)

    def setc(self, c):
        self.ram[0x03] = (self.ram[0x03] & ~1) | (1 if c else 0)

    def getc(self):
        return self.ram[0x03] & 1

    def check_interrupt(self):
        """外部割り込み(GP2/INT)のエッジを見て、条件が揃えばベクタへ飛ぶ。

        supercic-key.asm は idle: goto idle で完全に停止していて、
        Lockからのリセットパルスをこの割り込みでしか受け取れない。
        実機は継続的にピンを監視するが、命令単位のインタプリタでは
        毎ステップ明示的にチェックする必要がある。
        """
        intcon = self.ram[0x0B]
        gie = bool(intcon & 0x80)
        inte = bool(intcon & 0x10)
        cur = self.port_in.get(0x07, 0) & 0x04
        prev = self._prev_in.get(0x07, 0) & 0x04
        self._prev_in[0x07] = self.port_in.get(0x07, 0)
        if not (gie and inte):
            return
        option = self.ram[0x81] if self.addr(0x81) == 0x81 else self.ram[0x01]
        # OPTION_REGのbit6=INTEDG。1なら立ち上がり、0なら立ち下がりで割り込む。
        intedg = bool(self.ram[0x81] & 0x40)
        edge = (not prev and cur) if intedg else (prev and not cur)
        if edge:
            self.stack.append(self.pc)
            self.ram[0x0B] = (intcon & ~0x80) | 0x02   # GIEクリア、INTFセット
            self.pc = 0x0004
            self.cycles += 2

    def step(self):
        """1命令進める。

        ■ サイクル数の数え方（一度間違えたので明記する）
        PIC16では、スキップが成立したとき **スキップした命令自体が2サイクル**
        になり、飛ばされる命令は別途カウントしない
        （データシート:「次の命令は破棄されNOPが実行されるため2TCY命令となる」）。

        以前は「スキップ命令に1 + 飛ばされた命令に2」で**3サイクル**数えていた。
        スキップのたびに1サイクルずつ余計に進むので、握手開始までの
        千数百命令のあいだにLockとKeyが致命的にずれていた。
        """
        self.check_interrupt()
        op = self.prog[self.pc & 0x7FF]
        self.pc = (self.pc + 1) & 0x7FF
        if self.skip:
            self.skip = False
            return              # 費用はスキップ元の命令に計上済み
        self.cycles += 1
        self.exec(op)           # 2サイクル命令とスキップ成立時は exec が +1 する

    def exec(self, op):
        f = op & 0x7F
        d = (op >> 7) & 1
        b = (op >> 7) & 7
        k = op & 0xFF
        top = op >> 8

        def store(v):
            if d:
                self.wr(f, v)
            else:
                self.w = v & 0xFF

        if (op & 0x3F00) == 0x0700:            # ADDWF
            v = self.rd(f) + self.w
            self.setc(v > 0xFF); self.setz(v); store(v)
        elif (op & 0x3F00) == 0x0500:          # ANDWF
            v = self.rd(f) & self.w; self.setz(v); store(v)
        elif (op & 0x3F80) == 0x0180:          # CLRF
            self.wr(f, 0); self.setz(0)
        elif (op & 0x3F80) == 0x0100:          # CLRW
            self.w = 0; self.setz(0)
        elif (op & 0x3F00) == 0x0900:          # COMF
            v = (~self.rd(f)) & 0xFF; self.setz(v); store(v)
        elif (op & 0x3F00) == 0x0300:          # DECF
            v = (self.rd(f) - 1) & 0xFF; self.setz(v); store(v)
        elif (op & 0x3F00) == 0x0B00:          # DECFSZ
            v = (self.rd(f) - 1) & 0xFF; store(v)
            if v == 0: self.skip = True; self.cycles += 1
        elif (op & 0x3F00) == 0x0A00:          # INCF
            v = (self.rd(f) + 1) & 0xFF; self.setz(v); store(v)
        elif (op & 0x3F00) == 0x0F00:          # INCFSZ
            v = (self.rd(f) + 1) & 0xFF; store(v)
            if v == 0: self.skip = True; self.cycles += 1
        elif (op & 0x3F00) == 0x0400:          # IORWF
            v = self.rd(f) | self.w; self.setz(v); store(v)
        elif (op & 0x3F00) == 0x0800:          # MOVF
            v = self.rd(f); self.setz(v); store(v)
        elif (op & 0x3F80) == 0x0080:          # MOVWF
            self.wr(f, self.w)
        elif op == 0x0000 or (op & 0x3F9F) == 0x0000:   # NOP
            pass
        elif (op & 0x3F00) == 0x0D00:          # RLF
            v = self.rd(f); r = ((v << 1) | self.getc()) & 0xFF
            self.setc(v & 0x80); store(r)
        elif (op & 0x3F00) == 0x0C00:          # RRF
            v = self.rd(f); r = ((v >> 1) | (self.getc() << 7)) & 0xFF
            self.setc(v & 1); store(r)
        elif (op & 0x3F00) == 0x0200:          # SUBWF
            v = self.rd(f) - self.w
            self.setc(v >= 0); self.setz(v); store(v & 0xFF)
        elif (op & 0x3F00) == 0x0E00:          # SWAPF
            v = self.rd(f); r = ((v << 4) | (v >> 4)) & 0xFF; store(r)
        elif (op & 0x3F00) == 0x0600:          # XORWF
            v = self.rd(f) ^ self.w; self.setz(v); store(v)
        elif (op & 0x3C00) == 0x1000:          # BCF
            self.wr(f, self.rd(f) & ~(1 << b))
        elif (op & 0x3C00) == 0x1400:          # BSF
            self.wr(f, self.rd(f) | (1 << b))
        elif (op & 0x3C00) == 0x1800:          # BTFSC
            if not (self.rd(f) >> b) & 1:
                self.skip = True; self.cycles += 1
        elif (op & 0x3C00) == 0x1C00:          # BTFSS
            if (self.rd(f) >> b) & 1:
                self.skip = True; self.cycles += 1
        elif (op & 0x3E00) == 0x3E00:          # ADDLW
            v = self.w + k; self.setc(v > 0xFF); self.setz(v); self.w = v & 0xFF
        elif (op & 0x3F00) == 0x3900:          # ANDLW
            self.w &= k; self.setz(self.w)
        elif (op & 0x3800) == 0x2000:          # CALL
            self.stack.append(self.pc)
            self.pc = (op & 0x7FF)
            self.cycles += 1
        elif (op & 0x3800) == 0x2800:          # GOTO
            self.pc = (op & 0x7FF)
            self.cycles += 1
        elif (op & 0x3F00) == 0x3800:          # IORLW
            self.w |= k; self.setz(self.w)
        elif (op & 0x3C00) == 0x3000:          # MOVLW
            self.w = k
        elif op == 0x0009:                     # RETFIE
            self.pc = self.stack.pop() if self.stack else 0; self.cycles += 1
        elif (op & 0x3C00) == 0x3400:          # RETLW
            self.w = k
            self.pc = self.stack.pop() if self.stack else 0
            self.cycles += 1
        elif op == 0x0008:                     # RETURN
            self.pc = self.stack.pop() if self.stack else 0
            self.cycles += 1
        elif op == 0x0063:                     # SLEEP
            self.halted = True
        elif op == 0x0064:                     # CLRWDT
            pass
        elif (op & 0x3E00) == 0x3C00:          # SUBLW
            v = k - self.w; self.setc(v >= 0); self.setz(v); self.w = v & 0xFF
        elif (op & 0x3F00) == 0x3A00:          # XORLW
            self.w ^= k; self.setz(self.w)
        else:
            raise RuntimeError(f"未対応の命令 0x{op:04x} @ 0x{(self.pc-1)&0x7ff:03x}")


def main():
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "H_supercic-lock.hex"
    mem = load_hex(path)
    used = sum(1 for w in mem if w != 0x3FFF)
    print(f"プログラム語数 {used}/2048")
    print("先頭16語:", " ".join(f"{w:04x}" for w in mem[:16]))
    cpu = PIC14(mem)
    for _ in range(200):
        cpu.step()
        if cpu.halted:
            break
    print(f"200命令実行: PC=0x{cpu.pc:03x} W=0x{cpu.w:02x} "
          f"STATUS=0x{cpu.ram[3]:02x} cycles={cpu.cycles}")


if __name__ == "__main__":
    main()
