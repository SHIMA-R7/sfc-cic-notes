"""
lock.asm の mangle ルーチンを命令単位で実行し、cic_model.mangle() と突き合わせる。

■ なぜ必要か
`cic_model.mangle()` はアセンブラを目で追って書いたもので、機械的な検証をしていない。
実機ではラウンド0（初期シードそのまま＝mangleを通っていない）は14/14で一致するのに、
ラウンド1（最初のmangle出力）で完全に外れる。mangleが疑わしいが、
「読み間違えたかどうか」を目で確かめても同じ間違いを繰り返すだけになる。

そこでアセンブラを解釈する側を別に作り、二つの実装を突き合わせる。
どちらが正しいかは分からなくても、**食い違う場所は分かる**。

■ 対象命令
mangleが使うぶんだけ実装する（PIC12/16のミッドレンジ命令）。
"""

import re
import sys

sys.path.insert(0, ".")
import cic_model

ASM = (r"C:\Users\yugo\AppData\Local\Temp\claude"
       r"\C--Users-yugo-Downloads\1e60967b-f798-4f83-ae23-1703d5c45e7d"
       r"\scratchpad\lock.asm")


def parse(path):
    """(ラベル→行番号, 命令リスト) を返す。"""
    labels, prog = {}, []
    for raw in open(path, encoding="utf-8", errors="replace"):
        line = raw.split(";")[0].rstrip()
        if not line.strip():
            continue
        if not line[0].isspace():
            # 行頭がラベル。同じ行に命令が続くこともある
            parts = re.split(r"\s+", line.strip(), maxsplit=1)
            labels[parts[0]] = len(prog)
            if len(parts) == 1:
                continue
            line = "\t" + parts[1]
        body = line.strip()
        m = re.match(r"(\w+)\s*(.*)", body)
        if not m:
            continue
        op = m.group(1).lower()
        args = [a.strip() for a in m.group(2).split(",") if a.strip()]
        prog.append((op, args))
    return labels, prog


# PORTC/STATUS などの名前付きレジスタ。mangle_key には SuperCIC 独自の
# ペアモード検出が埋め込まれていて、これらを触る。mangleの算術には関与しない
# ので、衝突しない番地を割り当てて置き場所だけ用意すれば足りる。
NAMED = {}


def val(a):
    a = a.strip()
    if a.lower().startswith("0x"):
        return int(a, 16)
    if a.isdigit():
        return int(a)
    if a not in NAMED:
        NAMED[a] = 0x1000 + len(NAMED)
    return NAMED[a]


class Pic:
    """mangleの実行に必要な範囲だけのミッドレンジPICコア。"""

    def __init__(self, labels, prog):
        self.labels, self.prog = labels, prog
        self.reg = {}
        self.w = 0
        self.stack = []

    def get(self, a):
        return self.reg.get(val(a), 0) & 0xFF

    def put(self, a, v):
        self.reg[val(a)] = v & 0xFF

    def store(self, args, v):
        """destination指定。'f'ならレジスタへ、'w'または省略ならWへ。"""
        dest = args[1].lower() if len(args) > 1 else "w"
        if dest == "f":
            self.put(args[0], v)
        else:
            self.w = v & 0xFF

    def run(self, start, limit=2000000):
        pc = self.labels[start]
        steps = 0
        while steps < limit:
            steps += 1
            op, args = self.prog[pc]
            pc += 1

            if op == "nop":
                continue
            elif op == "movlw":
                self.w = val(args[0]) & 0xFF
            elif op == "movwf":
                self.put(args[0], self.w)
            elif op == "movf":
                self.store(args, self.get(args[0]))
            elif op == "clrf":
                self.put(args[0], 0)
            elif op == "clrw":
                self.w = 0
            elif op == "addlw":
                self.w = (self.w + val(args[0])) & 0xFF
            elif op == "andlw":
                self.w = self.w & val(args[0])
            elif op == "addwf":
                self.store(args, (self.get(args[0]) + self.w) & 0xFF)
            elif op == "andwf":
                self.store(args, self.get(args[0]) & self.w)
            elif op == "iorwf":
                self.store(args, self.get(args[0]) | self.w)
            elif op == "xorwf":
                self.store(args, self.get(args[0]) ^ self.w)
            elif op == "incf":
                self.store(args, (self.get(args[0]) + 1) & 0xFF)
            elif op == "decf":
                self.store(args, (self.get(args[0]) - 1) & 0xFF)
            elif op == "comf":
                self.store(args, (~self.get(args[0])) & 0xFF)
            elif op == "bsf":
                self.put(args[0], self.get(args[0]) | (1 << val(args[1])))
            elif op == "bcf":
                self.put(args[0], self.get(args[0]) & ~(1 << val(args[1])))
            elif op == "btfsc":
                if not (self.get(args[0]) >> val(args[1])) & 1:
                    pc += 1        # ビットが0なら次を飛ばす
            elif op == "btfss":
                if (self.get(args[0]) >> val(args[1])) & 1:
                    pc += 1        # ビットが1なら次を飛ばす
            elif op == "goto":
                pc = self.labels[args[0]]
            elif op == "call":
                self.stack.append(pc)
                pc = self.labels[args[0]]
            elif op == "return":
                if not self.stack:
                    return steps
                pc = self.stack.pop()
            else:
                raise NotImplementedError(f"未対応の命令: {op} {args}")
        raise RuntimeError("命令数の上限に達した（無限ループ）")


def run_asm(routine, base, seed):
    labels, prog = parse(ASM)
    cpu = Pic(labels, prog)
    for i, v in enumerate(seed):
        cpu.reg[base + 1 + i] = v
    cpu.run(routine)
    return [cpu.reg[base + 1 + i] for i in range(15)]


def main():
    cases = [
        ("mangle_lock", 0x30, cic_model.KEY_SEED, "key seed (0x31-0x3f)"),
        ("mangle_key", 0x20, cic_model.LOCK_SEED, "lock seed (0x21-0x2f)"),
    ]
    ok = True
    for routine, base, seed, name in cases:
        got_asm = run_asm(routine, base, list(seed))
        mine = list(seed)
        cic_model.mangle(mine)
        same = got_asm == mine
        ok = ok and same
        print(f"■ {routine} / {name}: {'一致' if same else '不一致'}")
        print(f"   asm  {[f'{v:02x}' for v in got_asm]}")
        print(f"   mine {[f'{v:02x}' for v in mine]}")
        if not same:
            for i, (a, b) in enumerate(zip(got_asm, mine)):
                if a != b:
                    print(f"     [{i}] asm=0x{a:02x} mine=0x{b:02x}")
        print()

    # 1回だけでは通っても、繰り返すとズレる実装がありうる。
    # 実機では1ラウンドにつき mangle を3回、ラウンドをまたいで積み重ねる。
    print("■ 連続適用（実機と同じく mangle_lock -> mangle_key を繰り返す）")
    labels, prog = parse(ASM)
    cpu = Pic(labels, prog)
    for i, v in enumerate(cic_model.KEY_SEED):
        cpu.reg[0x31 + i] = v
    for i, v in enumerate(cic_model.LOCK_SEED):
        cpu.reg[0x21 + i] = v
    lock, key = list(cic_model.LOCK_SEED), list(cic_model.KEY_SEED)

    for n in range(1, 49):
        cpu.run("mangle")                      # asm側: mangle_lock -> mangle_key
        cic_model.mangle_round(lock, key)      # 自作側
        a_key = [cpu.reg[0x31 + i] for i in range(15)]
        a_lock = [cpu.reg[0x21 + i] for i in range(15)]
        if a_key != key or a_lock != lock:
            print(f"   {n}回目で不一致")
            print(f"     asm  key ={[f'{v:02x}' for v in a_key]}")
            print(f"     mine key ={[f'{v:02x}' for v in key]}")
            print(f"     asm  lock={[f'{v:02x}' for v in a_lock]}")
            print(f"     mine lock={[f'{v:02x}' for v in lock]}")
            return 1
    print("   48回まで完全一致")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
