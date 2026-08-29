# 参照した一次資料

**このディレクトリに第三者のファイルは置きません。** 入手先を記録するだけにしてあります。

解析にあたって実際に読んだのは以下です。手元では同じ場所へ展開して作業しました。
再現する場合は各自で取得してください。

| ファイル | 出典 | ライセンス |
|---|---|---|
| `lock.asm` / `key.asm` | ikari_01 (Maximilian Rehkopf) SuperCIC — sd2snesプロジェクトの一部 | **GPL-2.0-only** |
| `supercic-lock.hex` / `supercic-key.hex` | 同上（ビルド済み） | GPL-2.0-only |
| `d411-dis.txt` | segher による本家CIC(D411)の逆アセンブル。<https://hackmii.com/2010/01/the-weird-and-wonderful-cic/> / Neo-Desktop/SNES-CIC-Disassembly | 明示なし |
| `nescic-dis.txt` / `3195a-dis.txt` | 同上（NES CIC / 3195A） | 明示なし |
| `M_sm590.cpp` / `M_sm590.h` / `M_sm590op.cpp` / `M_sm510op.cpp` | MAME の Sharp SM590 実装 | BSD-3-Clause |

## このリポジトリのライセンスがGPL-2.0-onlyである理由

`cic_model.py` や `nano3_cicbank*` の値（1ビット=372パルス、ラウンド=(16−k)ビット、
シードテーブル等）は、**SuperCICのソースを読んで導いたもの**です。派生物とみなすのが安全で、
SuperCICは "version 2 of the License **only**" と明記しているため、
GPL-3.0 へ移すことはできません。

このため、ダンパー本体の [sfc-nano-reader](https://github.com/SHIMA-R7/sfc-nano-reader)（MIT）
とはリポジトリを分けてあります。**両者のあいだでコードを混ぜないでください。**
