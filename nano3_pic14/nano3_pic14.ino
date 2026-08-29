// Nano-3: SuperCIC の Lock を「アルゴリズムを読んで移植」せず、
// アセンブル済みバイナリ(supercic-lock.hex)をそのまま解釈実行する。
//
// ■ なぜこの形にしたか
// 2日間「アセンブラを読んでアルゴリズムを抽出し、自分の設計で組み直す」を
// やり続け、そのたびに読み違えて実機で総当りに戻った。
// supercic-lock.hex は sanniのOSCRがSA-1を吸うのに実際使っている実装。
// 動いているバイナリをそのまま実行すれば、読み違えようがない。
// PC側の pic14.py で同じインタプリタを検証済み（lock/keyシード表が本家と完全一致）。
//
// ■ タイミングの考え方
// lock.asm は _EC_OSC（外部クロック）設定。PICの1命令 = 外部クロック4発
// （goto/call/スキップ成立時は2命令分＝8発）。実機のKeyは**パルス数**で
// 反応することを確認済み（間引きを変えても同じパルス数で反応した）ので、
// 絶対時間は問わない。だからAVR側は「1命令ぶんのクロックパルスを正確な
// 発数だけ出す」ことだけ気にすればよく、実時間の制約がない。
//
// ■ 入出力のタイミング
// 各PIC命令について: 実行前のポート状態でGPIO読み込みを解決し、
// 実行でGPIOへの書き込みがあれば新しい値を反映してから、
// その命令の長さ分のクロックパルスを送る。
//
// ■ ポート対応（lock.asm より）
//   PIC PORTC.0 -> A4(PC4) -> ソケット2番 -> カート24（データ）
//   PIC PORTC.1 -> A5(PC5) -> ソケット1番 -> カート55（データ）
//   PIC PORTC.2 -> A3(PC3) -> ソケット11番 -> カート25（Keyへのリセット出力）
//   クロック    -> D12(PB4) -> ソケット7番 -> カート56/57
//   PIC PORTA.2 (ホストリセット解除) は物理配線なし。内部状態のみ追跡。
//
// ■ LEDの意味（D13 = PB5）
// アルゴリズム自身が握手の成否を 0x43 のbit1(key invalid)に持っている。
// 外部で判定ロジックを作る必要がない。
//   点灯 = 現在 key valid（照合が合っている）
//   消灯 = key invalid（不一致を検出した）
//
// ■ シリアル
// 開発中の検証用。'S'を送るとシード表を、'?'を送ると現在のラウンド数と
// valid フラグを返す。最終的にPCと繋がない自律運用にするときは、
// setup()のSerial初期化を残したままでも実害はない（read()が何もなければ素通り）。

#include <Arduino.h>
#include "nano3_pic14_rom.h"

#define CLK_BIT _BV(PB4)
#define RST_BIT _BV(PC3)     // 使わない。PIC PORTC.2 が直接カート25を駆動する
// A4/A5とPIC PORTC.0/.1の対応は未確定（旧プロトコルでも swap フラグで
// 両方試していた）。最初の実機テストで最初のビット交換から即座に
// 不一致になったため、まず入れ替えて試す。
#define DA_BIT  _BV(PC5)     // PIC PORTC.0 <-> カート55(A5)
#define DB_BIT  _BV(PC4)     // PIC PORTC.1 <-> カート24(A4)
#define LED_BIT _BV(PB5)

static inline void clkPulse() {
  PORTB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;
}

// ---- PIC16F630 コア ----
// レジスタ番地は Python版(pic14.py)と同じマッピング。
// 0x02=PCL 0x03=STATUS 0x04=FSR 0x05=PORTA 0x07=PORTC
// 0x85=TRISA 0x87=TRISC （バンク1側の実体）
uint8_t picRam[256];
uint8_t picW;
uint16_t picPC;
uint16_t picStack[8];
uint8_t picSp;
bool picSkip;

static inline uint8_t picBank() { return (picRam[0x03] >> 5) & 1; }

// 両バンク共通のレジスタ。ここを忘れるとバンク1での STATUS操作が
// 別番地に書かれてバンクが戻らなくなる（PC版で一度ハマった）。
static inline bool isCommon(uint8_t f) {
  return f == 0x02 || f == 0x03 || f == 0x04 || f == 0x0A || f == 0x0B;
}

static uint8_t picAddr(uint8_t f) {
  if (f == 0) {                       // INDF
    f = picRam[0x04] & 0x7F;
    if (f == 0) return 0;
  }
  if (isCommon(f)) return f;
  if ((f & 0x7F) >= 0x70) return (uint8_t)((f & 0x7F) | 0x80);
  return (uint8_t)(f | (picBank() << 7));
}

// 物理ピンの読み書き。PORTC.0/1 = データ線(1kΩ越し)、PORTC.2 = Keyリセット。
static void picPortWrite(uint8_t addr, uint8_t val) {
  if (addr == 0x07) {                 // PORTC
    uint8_t trisc = picRam[0x87];
    // TRIS=0(出力)のビットだけ実ピンへ反映する
    if (!(trisc & 0x01)) { if (val & 0x01) PORTC |= DA_BIT; else PORTC &= (uint8_t)~DA_BIT; }
    if (!(trisc & 0x02)) { if (val & 0x02) PORTC |= DB_BIT; else PORTC &= (uint8_t)~DB_BIT; }
    if (!(trisc & 0x04)) { if (val & 0x04) PORTC |= RST_BIT; else PORTC &= (uint8_t)~RST_BIT; }
  }
}

static void picTrisWrite(uint8_t addr, uint8_t val) {
  if (addr == 0x87) {                 // TRISC
    DDRC = (uint8_t)((DDRC & ~(DA_BIT | DB_BIT | RST_BIT))
           | ((val & 0x01) ? 0 : DA_BIT)
           | ((val & 0x02) ? 0 : DB_BIT)
           | ((val & 0x04) ? 0 : RST_BIT));
  }
}

static uint8_t picRd(uint8_t f) {
  uint8_t a = picAddr(f);
  if (a == 0x07) {                    // PORTC 読み込みは実ピンの電位を見る
    uint8_t v = picRam[a] & 0xF8;     // 上位ビットは内部ラッチのまま
    if (PINC & DA_BIT) v |= 0x01;
    if (PINC & DB_BIT) v |= 0x02;
    if (PINC & RST_BIT) v |= 0x04;
    return v;
  }
  if (a == 0x05) return picRam[a];    // PORTA は物理配線なし。ラッチをそのまま返す
  return picRam[a];
}

static void picWr(uint8_t f, uint8_t v) {
  uint8_t a = picAddr(f);
  picRam[a] = v;
  if (a == 0x07) picPortWrite(a, v);
  else if (a == 0x87) picTrisWrite(a, v);
}

static void picSetZ(uint8_t v) {
  if (v == 0) picRam[0x03] |= 0x04; else picRam[0x03] &= (uint8_t)~0x04;
}
static void picSetC(bool c) {
  if (c) picRam[0x03] |= 0x01; else picRam[0x03] &= (uint8_t)~0x01;
}
static inline uint8_t picGetC() { return picRam[0x03] & 1; }

// 命令を実行し、消費サイクル数(1か2)を返す。
static uint8_t picExec(uint16_t op) {
  uint8_t f = op & 0x7F;
  uint8_t d = (op >> 7) & 1;
  uint8_t b = (op >> 7) & 7;
  uint8_t k = op & 0xFF;
  uint8_t cyc = 1;

  auto store = [&](uint8_t v) {
    if (d) picWr(f, v); else picW = v;
  };

  if ((op & 0x3F00) == 0x0700) {                    // ADDWF
    uint16_t v = (uint16_t)picRd(f) + picW;
    picSetC(v > 0xFF); picSetZ((uint8_t)v); store((uint8_t)v);
  } else if ((op & 0x3F00) == 0x0500) {              // ANDWF
    uint8_t v = picRd(f) & picW; picSetZ(v); store(v);
  } else if ((op & 0x3F80) == 0x0180) {              // CLRF
    picWr(f, 0); picSetZ(0);
  } else if ((op & 0x3F80) == 0x0100) {              // CLRW
    picW = 0; picSetZ(0);
  } else if ((op & 0x3F00) == 0x0900) {              // COMF
    uint8_t v = (uint8_t)~picRd(f); picSetZ(v); store(v);
  } else if ((op & 0x3F00) == 0x0300) {              // DECF
    uint8_t v = (uint8_t)(picRd(f) - 1); picSetZ(v); store(v);
  } else if ((op & 0x3F00) == 0x0B00) {              // DECFSZ
    uint8_t v = (uint8_t)(picRd(f) - 1); store(v);
    if (v == 0) { picSkip = true; cyc = 2; }
  } else if ((op & 0x3F00) == 0x0A00) {              // INCF
    uint8_t v = (uint8_t)(picRd(f) + 1); picSetZ(v); store(v);
  } else if ((op & 0x3F00) == 0x0F00) {              // INCFSZ
    uint8_t v = (uint8_t)(picRd(f) + 1); store(v);
    if (v == 0) { picSkip = true; cyc = 2; }
  } else if ((op & 0x3F00) == 0x0400) {              // IORWF
    uint8_t v = picRd(f) | picW; picSetZ(v); store(v);
  } else if ((op & 0x3F00) == 0x0800) {              // MOVF
    uint8_t v = picRd(f); picSetZ(v); store(v);
  } else if ((op & 0x3F80) == 0x0080) {              // MOVWF
    picWr(f, picW);
  } else if ((op & 0x3F9F) == 0x0000) {              // NOP
    ;
  } else if ((op & 0x3F00) == 0x0D00) {              // RLF
    uint8_t v = picRd(f); uint8_t r = (uint8_t)((v << 1) | picGetC());
    picSetC(v & 0x80); store(r);
  } else if ((op & 0x3F00) == 0x0C00) {              // RRF
    uint8_t v = picRd(f); uint8_t r = (uint8_t)((v >> 1) | (picGetC() << 7));
    picSetC(v & 1); store(r);
  } else if ((op & 0x3F00) == 0x0200) {              // SUBWF
    int16_t v = (int16_t)picRd(f) - picW;
    picSetC(v >= 0); picSetZ((uint8_t)v); store((uint8_t)v);
  } else if ((op & 0x3F00) == 0x0E00) {              // SWAPF
    uint8_t v = picRd(f); store((uint8_t)((v << 4) | (v >> 4)));
  } else if ((op & 0x3F00) == 0x0600) {              // XORWF
    uint8_t v = picRd(f) ^ picW; picSetZ(v); store(v);
  } else if ((op & 0x3C00) == 0x1000) {              // BCF
    picWr(f, (uint8_t)(picRd(f) & ~(1 << b)));
  } else if ((op & 0x3C00) == 0x1400) {              // BSF
    picWr(f, (uint8_t)(picRd(f) | (1 << b)));
  } else if ((op & 0x3C00) == 0x1800) {              // BTFSC
    if (!((picRd(f) >> b) & 1)) { picSkip = true; cyc = 2; }
  } else if ((op & 0x3C00) == 0x1C00) {              // BTFSS
    if ((picRd(f) >> b) & 1) { picSkip = true; cyc = 2; }
  } else if ((op & 0x3E00) == 0x3E00) {              // ADDLW
    uint16_t v = (uint16_t)picW + k; picSetC(v > 0xFF); picSetZ((uint8_t)v); picW = (uint8_t)v;
  } else if ((op & 0x3F00) == 0x3900) {              // ANDLW
    picW &= k; picSetZ(picW);
  } else if ((op & 0x3800) == 0x2000) {              // CALL
    picStack[picSp++ & 7] = picPC; picPC = op & 0x7FF; cyc = 2;
  } else if ((op & 0x3800) == 0x2800) {              // GOTO
    picPC = op & 0x7FF; cyc = 2;
  } else if ((op & 0x3F00) == 0x3800) {              // IORLW
    picW |= k; picSetZ(picW);
  } else if ((op & 0x3C00) == 0x3000) {              // MOVLW
    picW = k;
  } else if (op == 0x0009) {                         // RETFIE
    picPC = picStack[--picSp & 7]; cyc = 2;
  } else if ((op & 0x3C00) == 0x3400) {              // RETLW
    picW = k; picPC = picStack[--picSp & 7]; cyc = 2;
  } else if (op == 0x0008) {                         // RETURN
    picPC = picStack[--picSp & 7]; cyc = 2;
  } else if (op == 0x0063) {                         // SLEEP
    ;
  } else if (op == 0x0064) {                         // CLRWDT
    ;
  } else if ((op & 0x3E00) == 0x3C00) {              // SUBLW
    int16_t v = (int16_t)k - picW; picSetC(v >= 0); picSetZ((uint8_t)v); picW = (uint8_t)v;
  } else if ((op & 0x3F00) == 0x3A00) {              // XORLW
    picW ^= k; picSetZ(picW);
  }
  return cyc;
}

static void picStep() {
  // ■ サイクル数の数え方（一度間違えた）
  // PIC16では、スキップが成立したとき**スキップした命令自体が2サイクル**になり、
  // 飛ばされる命令は別途カウントしない（データシート:「次の命令は破棄され
  // NOPが実行されるため2TCY命令となる」）。
  // 以前は「スキップ元に2 + 飛ばされた側にも2」で計4サイクル数えており、
  // スキップのたびに2サイクルぶん余計にクロックを出していた。
  // 握手開始までに数百回スキップするので、Keyとの位相が大きくずれる。
  uint16_t op = pgm_read_word(&PIC_ROM[picPC & 0x7FF]);
  picPC = (picPC + 1) & 0x7FF;
  uint8_t cyc;
  if (picSkip) {
    picSkip = false;
    cyc = 0;              // 費用はスキップ元の命令に計上済み
  } else {
    cyc = picExec(op);
  }
  // 1サイクル=クロック4発。命令の長さぶんパルスを出す。
  const uint8_t pulses = cyc * 4;
  for (uint8_t i = 0; i < pulses; i++) clkPulse();
}

static void picReset() {
  memset(picRam, 0, sizeof(picRam));
  picW = 0; picPC = 0; picSp = 0; picSkip = false;
  DDRB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;
  // 起動直後は全ポート入力（ハードリセット直後のPIC相当）
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT | RST_BIT);
}

// ---- 進捗表示 ----
// 0x43 bit1(key invalid)は一度立つとコード側でクリアされない(発見時点のsticky flag)。
// 「起動から何命令目で最初の不一致が起きたか」を記録する。
// 早い段階(数十〜数百命令)なら配線・タイミングのバグ、遅い段階なら別の話。
uint32_t instCount = 0;
uint32_t firstMismatchAt = 0;
uint8_t lastReg43 = 0;

static void checkProgress() {
  instCount++;
  uint8_t r43 = picRam[0x43];
  if ((r43 & 0x02) && !(lastReg43 & 0x02) && firstMismatchAt == 0) {
    firstMismatchAt = instCount;
  }
  lastReg43 = r43;
}

// ■ バンク選択(カートA16-A23)の復活
// このピン(D2-D9)は元々 nano3_bank.ino がバンクカウンタとして駆動していた
// カートの上位アドレス線そのもの(コミット済みの配線、変更なし)。
// nano3_pic14 に載せ替えた際、この出力を引き継ぐのを忘れていて、
// A16-A23 が浮いたままCIC実験をしていた。バンク$00/$C0のつもりで読んでいた
// ものが、電気的には不定なバンクだった可能性がある。
//   D2-D7 = PORTD bit2-7 = A16-A21
//   D8-D9 = PORTB bit0-1 = A22-A23
// 固定バンクを焼き込む。$C0 = 0b11000000 なので A22/A23(D8/D9)だけHigh。
#define TARGET_BANK 0x00

static void writeBank(uint8_t v) {
  DDRD |= 0xFC;                 // D2-D7を出力に
  DDRB |= 0x03;                 // D8-D9を出力に
  PORTD = (PORTD & 0x03) | (uint8_t)(v << 2);
  PORTB = (PORTB & 0xFC) | (uint8_t)(v >> 6);
}

// ■ SYSCK実験用: D6(=A20、バンク値のbit4)を一時的にSYSCK出力へ転用する。
// バンク$00はbit4も0なので、この1本を借りてもバンク$00の正しさには影響しない
// ($C0以降を読むときは元に戻す必要がある)。
// D6=PD6=OC0A。Timer0をCTCモードで使い、ハードウェアでトグルさせる
// (ソフトでは無理。CICのビットバンギングと同じCPUを取り合うことになる)。
// SYSCKは遅延起動しても認証を壊した。おそらくNano-1/2とNano-3が別々の
// 水晶で動いていて位相関係が無いため(実機は全信号が同じ水晶由来)。
// 単純なタイミングの問題ではなく構造的な制約とみて、いったん無効化する。
#define DRIVE_SYSCK 0

static void startSysck() {
  DDRD |= _BV(PD6);
  TCCR0A = _BV(COM0A0) | _BV(WGM01);  // CTCモード、比較一致でOC0Aをトグル
  TCCR0B = _BV(CS00);                 // 分周なし。16MHz/(2*(1+OCR0A))
  OCR0A = 1;                          // 4MHz。MCKと合わせる
}

// 切り分け用: バンク出力を一時的に無効化し、以前の(認証だけが動いていた)
// 状態に戻して認証が復活するか確認する。0にすればD2-D9は触らず放置。
#define DRIVE_BANK 1

void setup() {
  DDRB |= LED_BIT;
#if DRIVE_BANK
  writeBank(TARGET_BANK);
#endif
  // SYSCKは起動直後は出さない。「解錠中にSYSCKを供給するとデータが壊れる」
  // という報告どおり、認証成立前に有効化したら即座に不一致で落ちた(実測)。
  // 認証が安定するまで(実測で約58000命令、1秒未満)待ってから有効化する。
  Serial.begin(115200);
  picReset();
  Serial.println(F("nano3_pic14 ready"));
}

uint32_t lastReport = 0;
bool sysckStarted = false;

void loop() {
  picStep();
  checkProgress();

#if DRIVE_SYSCK
  // 認証が一度でも成立(有効フラグが立つ)してから、余裕を持たせてSYSCKを起動する。
  if (!sysckStarted && instCount > 200000UL) {
    startSysck();
    sysckStarted = true;
  }
#endif

  // LED: 0x43 bit1 が立っていなければ valid（点灯）。
  if (picRam[0x43] & 0x02) PORTB &= (uint8_t)~LED_BIT;
  else PORTB |= LED_BIT;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      Serial.print(F("lock seed "));
      for (uint8_t i = 0; i < 15; i++) Serial.print(picRam[0x21 + i], HEX);
      Serial.print(F("  key seed "));
      for (uint8_t i = 0; i < 15; i++) Serial.print(picRam[0x31 + i], HEX);
      Serial.println();
    } else if (c == '?') {
      Serial.print(F("inst=")); Serial.print(instCount);
      Serial.print(F(" firstMismatchAt=")); Serial.print(firstMismatchAt);
      Serial.print(F(" reg43=0x")); Serial.print(picRam[0x43], HEX);
      Serial.print(F(" PORTA2(hostReset)=")); Serial.println((picRam[0x05] >> 2) & 1);
    } else if (c == 'R') {
      picReset(); instCount = 0; firstMismatchAt = 0; lastReg43 = 0;
      Serial.println(F("reset"));
    }
  }
}
