"""読み出したデータが「本物のROMらしいか」を判定する。

■ なぜ専用の関数が要るのか
このプロジェクトでは**判定の甘さで3回騙された**。

  1. 0xFF率だけを見て、62バンクが揃って0xFF一色なのを「一致率が高い」と誤認
  2. 0xFF率0.0000を「開いている」と判定し、中身が0x00一色なのに71回連続で誤判定
  3. 「0xFFでないバイトの割合」を実データ率としたが、0x00や0x02で埋まっていても
     1.00になる。8/16/32バンクすべて満点と出たのに、KIRBYの文字列がROM内に
     一つも存在しなかった

**共通の誤りは「特定の値だけを異常とみなした」こと。**
施錠状態は 0xFF / 0x00 / 0x01 / 0x02 / 0x0F など、いろいろな一色になる。
どの値が来ても弾ける基準が要る。

■ 判定の考え方
本物のROMは**バイト値が散らばる**。機械語もグラフィックも圧縮データも、
64KBの中に数十〜200種類以上の異なる値が現れる。
一方、施錠状態は数種類しか現れない。

種類数と最頻値の占有率の2つで見れば、値が何であろうと判別できる。
"""

import collections


def rom_likeness(data):
    """データが本物のROMらしいかを返す。

    戻り値は dict:
      uniq     … 異なるバイト値の種類数
      top      … 最頻値の占有率
      ff       … 0xFF率（参考。判定には使わない）
      is_rom   … ROMらしいか
      reason   … 判定理由
    """
    c = collections.Counter(data)
    uniq = len(c)
    top_val, top_n = c.most_common(1)[0]
    top = top_n / len(data)
    ff = c.get(0xFF, 0) / len(data)

    # 本物のROMなら、64KB中に少なくとも数十種類は現れる。
    # 施錠状態は1〜3種類しかない。境界は余裕をみて50種。
    if uniq < 50:
        return dict(uniq=uniq, top=top, ff=ff, is_rom=False,
                    reason=f"異なる値が{uniq}種しかない(最頻 0x{top_val:02x})")
    # 種類は多くても、1つの値が大半を占めるなら未駆動が混ざっている。
    if top > 0.5:
        return dict(uniq=uniq, top=top, ff=ff, is_rom=False,
                    reason=f"0x{top_val:02x} が{top:.1%}を占める")
    return dict(uniq=uniq, top=top, ff=ff, is_rom=True,
                reason=f"{uniq}種 / 最頻 0x{top_val:02x} {top:.1%}")


def describe(data, label=""):
    r = rom_likeness(data)
    mark = "ROM" if r["is_rom"] else "×  "
    return (f"{label} [{mark}] 異なる値{r['uniq']:3d} 最頻{r['top']:.3f} "
            f"0xFF率{r['ff']:.3f}  {r['reason']}")


def find_marker(data, markers=(b"KIRBY", b"SUPER MARIO", b"NINTENDO")):
    """ROM内に既知の文字列があるか。あれば本物である強い証拠。"""
    for m in markers:
        i = data.find(m)
        if i >= 0:
            return m.decode(), i
    return None, -1
