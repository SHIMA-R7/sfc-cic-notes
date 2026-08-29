"""SA-1カートの応答を領域ごとに切り分けて表示する。

■ なぜ領域ごとに見るのか
SA-1は「全ゼロ」か「読めた」の二値ではない。実測では、MCKを供給した時点で
**自分のI-RAM/BW-RAM窓だけが応答し、ROM窓は0のまま**という中間状態が出た。

    $0000-$2FFF  0x00
    $3000-$5FFF  0x34 が12288バイト
    $6000-$7FFF  0xFF が8192バイト
    $8000-$FFFF  0x00          <- ROMはここ。ここが動くかどうかが全て

先頭16バイトだけを見ていると、この違いがまるごと見えない。実際その見方をしていた
あいだ、MCKの効果は「全ゼロのまま」と誤って記録されかけた。

■ 使い方
    python sa1_probe.py --port COM16 --bank 0 --mck 1
    python sa1_probe.py --port COM16 --bank 0xC0 --mck 1 --label "R0認証後"

--mck はカート1番へ供給するクロックの分周値(OCR2A)。16MHz/(2*(1+n)) が出力周波数で、
1=4MHz / 3=2MHz / 7=1MHz / 0=8MHz。--no-mck でクロックを止めたまま読む。
結果は results/ にタイムスタンプ付きで残す（比較できないと意味がないため）。
"""

import argparse
import collections
import os
import sys
import time

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import TIMING_TIERS, _read_bank_once  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa1_quality import rom_likeness  # noqa: E402

REGIONS = [
    (0x0000, 0x2000, "$0000-$1FFF"),
    (0x2000, 0x3000, "$2000-$2FFF  SA-1レジスタ窓"),
    (0x3000, 0x6000, "$3000-$5FFF  I-RAM窓"),
    (0x6000, 0x8000, "$6000-$7FFF  BW-RAM窓"),
    (0x8000, 0x10000, "$8000-$FFFF  ROM窓  ★ここが本命"),
]


def describe(data):
    lines = []
    for lo, hi, name in REGIONS:
        seg = data[lo:hi]
        c = collections.Counter(seg)
        nz = len(seg) - c.get(0, 0)
        top = " ".join(f"0x{v:02x}x{n}" for v, n in c.most_common(3))
        lines.append(f"  {name:32s} 異なる値{len(c):3d}  非ゼロ{nz:5d}/{len(seg):5d}  {top}")
    return lines


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--bank", default="0")
    p.add_argument("--mck", type=int, default=1,
                   help="カート1番へ出すクロックの分周値。1=4MHz(既定) 3=2MHz 7=1MHz 0=8MHz")
    p.add_argument("--no-mck", action="store_true", help="クロックを供給しない")
    p.add_argument("--tier", type=int, default=0)
    p.add_argument("--label", default="", help="この試行が何なのかを記録に残す")
    p.add_argument("--save", action="store_true", help="生の64KBも保存する")
    a = p.parse_args()
    bank = int(str(a.bank), 0)

    cond = "MCKなし" if a.no_mck else f"MCK OCR={a.mck} ({16 / (2 * (1 + a.mck)):.2f}MHz)"
    head = (f"bank ${bank:02X} / {cond} / タイミング[{TIMING_TIERS[a.tier][3]}]"
            + (f" / {a.label}" if a.label else ""))
    print(head, flush=True)

    data = _read_bank_once(a.port, bank, TIMING_TIERS[a.tier],
                           cart_clock=not a.no_mck, clock_ocr=a.mck,
                           log=lambda m: print(m, flush=True))
    if data is None:
        print("読み出し失敗", flush=True)
        return 1

    out = describe(data)
    for line in out:
        print(line, flush=True)

    rom = data[0x8000:]
    # **any(rom) で判定してはいけない。** 0x02一色でも「非ゼロ」なので通ってしまう。
    # 実際に2026-08-28、全64KBが0x02一色の読みに対して
    # 「★ROM窓が応答しています」と表示した。報告書に書いた
    # 「特定の値だけを異常とみなす」判定の穴が、自分の道具に残っていた。
    # 判定は sa1_quality.rom_likeness に一本化する。
    r = rom_likeness(rom)
    if r["is_rom"]:
        print("\n  ★ ROM窓が応答しています。先頭32バイト:", rom[:32].hex(), flush=True)
        for off in (0x7FC0, 0xFFC0):
            h = data[off:off + 32]
            comp = h[28] | (h[29] << 8)
            csum = h[30] | (h[31] << 8)
            ok = "補数対OK" if ((csum + comp) & 0xFFFF) == 0xFFFF and csum else "補数対NG"
            print(f"  ${off:04X}: 『{h[:21].decode('shift_jis', 'replace')}』 "
                  f"checksum=0x{csum:04x} {ok}", flush=True)
    else:
        print(f"\n  ROM窓はROMとして成立していません: {r['reason']}", flush=True)

    os.makedirs("results", exist_ok=True)
    stamp = time.strftime("%m%d-%H%M%S")
    with open(os.path.join("results", f"{stamp}.txt"), "w", encoding="utf-8") as f:
        f.write(head + "\n" + "\n".join(out) + "\n")
        f.write(f"ROM窓 非ゼロ {sum(1 for b in rom if b)}/{len(rom)}\n")
    if a.save:
        with open(os.path.join("results", f"{stamp}.bin"), "wb") as f:
            f.write(data)
    print(f"\n記録: results/{stamp}.txt", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
