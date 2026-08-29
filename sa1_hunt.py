"""SA-1カートが「開いている瞬間」を捕まえて一気に吸う。

■ なぜこの形なのか
SA-1は施錠/解錠が断続的に切り替わる。長時間かけて多数決を取ると、その間に
SA-1自身が状態を変えてしまい、収束はするのに中身が別物になる（実測済み）。
そこで「多数決で粘る」のをやめ、**開いた瞬間を検出して、その窓のうちに
最速で読み切る**方針に変えた。

■ マッピングの根拠
バンク$00の$FFC0に『SUPER MARIO RPG』のヘッダ（補数対OK）が出た。
LoROMでは $00:$8000-$FFFF が ROM offset 0x0000-0x7FFF なので、
バンク内$FFC0 = ROM 0x7FC0 = **LoROMのヘッダ位置**になる。
つまりこのカートはLoROM配置で読む。

SA-1のSuper MMCレジスタ($2220-$2223)の電源投入時の既定値は 0,1,2,3 で、

    $00-$1F -> ROM block 0 (1MB)
    $20-$3F -> ROM block 1
    $80-$9F -> ROM block 2
    $A0-$BF -> ROM block 3

の順に4MBが並ぶ。各バンクの**上位32KB**だけがROM。128バンク x 32KB = 4MB。

■ 答え合わせ
No-Intro: Super Mario RPG (Japan) / 4,194,304 bytes / CRC32 5527071e
16bitチェックサムはたまたま合うことがあるので、CRC32で判定する。
"""

import argparse
import os
import sys
import time
import zlib

sys.path.insert(0, r"C:\SFC-Dumper\host")
from bankio import TIMING_TIERS, _read_bank_once  # noqa: E402

TARGET_CRC = "5527071e"
TARGET_SIZE = 4194304
HALF = 32768

# LoROM配置での読み出し順。SA-1のSuper MMC既定値 0,1,2,3 に対応する。
LOROM_BANKS = (list(range(0x00, 0x20)) + list(range(0x20, 0x40))
               + list(range(0x80, 0xA0)) + list(range(0xA0, 0xC0)))
# 比較用。sanniがSA-1で読んでいるHiROM側の並び。
HIROM_BANKS = list(range(0xC0, 0x100))


def read_bank(port, bank, tier, log=None):
    return _read_bank_once(port, bank, TIMING_TIERS[tier],
                           log=log or (lambda m: None))


def header_at(data, off):
    """補数対が成立するヘッダなら (タイトル, checksum) を返す。"""
    if len(data) < off + 32:
        return None
    h = data[off:off + 32]
    comp = h[28] | (h[29] << 8)
    csum = h[30] | (h[31] << 8)
    if ((csum + comp) & 0xFFFF) == 0xFFFF and csum:
        return h[:21].decode("shift_jis", "replace").strip(), csum
    return None


def is_open(port, tier):
    """バンク$00を読んで、解錠されているかを判定する。

    「非ゼロがある」では足りない。施錠中もI-RAM窓やBW-RAM窓は応答するので、
    **補数対の成立するヘッダが読めること**を条件にする。ここを緩めると、
    施錠状態のノイズを解錠と誤認して無駄な15分を費やす。
    """
    d = read_bank(port, 0xC0, tier)
    if d is None:
        return None, None
    return (header_at(d, 0x7FC0) or header_at(d, 0xFFC0)), d


def dump(port, banks, tier, use_half, cache, log):
    """指定順にバンクを読む。1バンク1回だけ。窓が閉じる前に読み切るため。"""
    os.makedirs(cache, exist_ok=True)
    out = []
    for i, b in enumerate(banks):
        d = read_bank(port, b, tier)
        if d is None:
            log(f"  bank ${b:02X}: 読み出し失敗。中断")
            return None
        with open(os.path.join(cache, f"bank_{b:03d}.bin"), "wb") as f:
            f.write(d)
        out.append(d[HALF:] if use_half else d)
        if (i + 1) % 16 == 0:
            log(f"  {i + 1}/{len(banks)} バンク")
    return b"".join(out)


def verify(rom, log):
    crc = format(zlib.crc32(rom) & 0xFFFFFFFF, "08x")
    log(f"  {len(rom)} bytes / CRC32 = {crc}")
    for off, name in ((0x7FC0, "LoROM"), (0xFFC0, "HiROM")):
        h = header_at(rom, off)
        if h:
            total = sum(rom) & 0xFFFF
            log(f"  {name}位置にヘッダ 『{h[0]}』 期待=0x{h[1]:04x} 計算=0x{total:04x}"
                f" -> {'一致' if total == h[1] else '不一致'}")
    return crc == TARGET_CRC, crc


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM16")
    p.add_argument("--tier", type=int, default=1)
    p.add_argument("--out", default=r"C:\SFC-ROM\SUPERMARIORPG.sfc")
    p.add_argument("--rounds", type=int, default=99, help="最大何回挑戦するか")
    p.add_argument("--mapping", choices=["lorom", "hirom", "both"], default="lorom")
    a = p.parse_args()

    log = lambda m: print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)
    plans = []
    if a.mapping in ("lorom", "both"):
        plans.append(("LoROM $00-$3F,$80-$BF 上位32KB", LOROM_BANKS, True))
    if a.mapping in ("hirom", "both"):
        plans.append(("HiROM $C0-$FF 全64KB", HIROM_BANKS, False))

    for rnd in range(1, a.rounds + 1):
        log(f"=== 挑戦 {rnd} ===")
        hdr, _ = is_open(a.port, a.tier)
        if hdr is None:
            log("  施錠中（バンク$00に有効なヘッダなし）。20秒待って再試行")
            time.sleep(20)
            continue
        log(f"  解錠を検出: 『{hdr[0]}』 checksum=0x{hdr[1]:04x}")

        for name, banks, use_half in plans:
            log(f"  {name} で読み出し開始（{len(banks)}バンク）")
            start = time.time()
            rom = dump(a.port, banks, a.tier, use_half,
                       a.out + f".banks-{'lo' if use_half else 'hi'}", log)
            if rom is None:
                break
            log(f"  読み出し完了 {time.time() - start:.0f}秒")
            ok, crc = verify(rom, log)
            if ok:
                with open(a.out, "wb") as f:
                    f.write(rom)
                log(f"★★★ No-Intro一致! CRC32={crc} 保存: {a.out}")
                return 0
            with open(a.out + f".{crc}.unverified", "wb") as f:
                f.write(rom)
            log(f"  不一致。{a.out}.{crc}.unverified として保存")
    log("上限回数に達しました")
    return 1


if __name__ == "__main__":
    sys.exit(main())
