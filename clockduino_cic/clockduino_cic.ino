// 21.4MHzduino: マスタークロック供給 + CIC(Lock役) 専任ボード
//
// ■ このボードに集約した理由
// CICは「与えられたパルスの数」で状態を進める（実測: 間引きを変えても同じパルス数で
// 反応）。1ビット372パルスを1つも取りこぼさずに送る必要がある。
//
// Nano-3にCIC役とバンク生成役を兼務させたときは、PORTBのリードモディファイライトが
// PCINT割り込みと競合して**バンク上位2ビット(A22,A23)を壊した**（2回発生）。
// sanniがMegaとは別にPIC12F629を置いているのも同じ理由で、MegaはSDカード書き込み・
// 表示・USBを抱えていてµs精度のパルスを並行できない。
//
// **このボードはCIC専任。** 役割としてはsanniのPIC12F629と等価。
//
// ■ ピン割り当て（DIP-28）
//   14 (PB0)  CKOUT 21.477MHz -> カート1番   ※lfuse=0xBFのハードウェア機能
//   15 (PB1)  動作LED
//   16 (PB2)  CICクロック     -> カート56番
//   26 (PC3)  CICリセット     -> カート25番
//   27 (PC4)  CICデータA      -> カート24番 (1kΩ)
//   28 (PC5)  CICデータB      -> カート55番 (1kΩ)
//    2 (PD0)  シリアルRX      <- Uno D3
//    3 (PD1)  シリアルTX      -> Uno D2
//   17,18,19 (PB3,PB4,PB5)    ISP専用(MOSI/MISO/SCK)。**CICには使わない**
//
// **PB3-PB5をISP専用に空けたので、ISP結線とCIC結線を同時に繋いだままにできる。**
// 当初PB4(MISO)にCICクロックを割り当てようとして衝突に気づいた。
//
// ■ クロック周波数
// tick()はPORTBのセット/クリア2命令。21.477MHz/4 = 5.37MHz相当。
// Nano-3(16MHz)では4.00MHzだった。実機のCICクロックは3.072MHz。
// CICはパルス数で動くので通る見込みだが、CIC_NOPSで下げられる:
//   0個=5.37MHz / 1個=3.58MHz / 2個=2.68MHz  ← 実機3.072MHzを挟める
//
// ■ シリアル
// ブートローダが無くUSB-シリアル変換を持たない。デバッグ出力はUnoへ送り、
// UnoがSoftwareSerialで受けてUSBへ中継する。57600bpsまで。

#include <Arduino.h>

#define CLK_BIT _BV(PB2)     // カート56番 (PB4から変更。PB4はISPのMISO)
#define RST_BIT _BV(PC3)     // カート25番
#define DA_BIT  _BV(PC4)     // カート24番
#define DB_BIT  _BV(PC5)     // カート55番
#define LED_BIT _BV(PB1)     // 動作LED (PB5から変更。PB5はISPのSCK)

// CICクロックの周期を伸ばすNOPの数。0=5.37MHz / 1=3.58MHz / 2=2.68MHz
#define CIC_NOPS 1

const uint16_t ID_DELAY   = 2496;   // リセット解除から最初のIDビットまで
const uint16_t ID_BIT     = 60;     // ストリームID 1ビット = 15命令 x 4
const uint16_t POST_ID    = 520;    // ID送信終了から主ループ突入まで
const uint16_t BIT_PULSES = 372;    // 主ループ 1ビット = 93命令 x 4
const uint8_t  DRIVE_PULSES = 24;   // 自分の線を駆動し続ける長さ
const uint8_t  SAMPLE_AT  = 16;     // 相手の線を読む位置
const uint8_t  ROUND0_BITS = 15;

// ラウンド0。送信はlockシードのLSB、期待値はシミュレータ（本家ROM）の出力。
// 実機で15/15一致を確認済み（地域ビットを含む）。
const char TX0[] = "110101111101010";
const char RX0[] = "110101111010100";

// A4/A5 と CIC 0番/1番 の対応。実機でこちら（入れ替えた側）が正しいと確定。
#define PIN0 DB_BIT
#define PIN1 DA_BIT

static uint8_t outMask, inMask;

static inline void tick() {
  PORTB |= CLK_BIT;
#if CIC_NOPS >= 1
  __asm__ __volatile__("nop");
#endif
#if CIC_NOPS >= 2
  __asm__ __volatile__("nop");
#endif
  PORTB &= (uint8_t)~CLK_BIT;
}

static void ticks(uint16_t n) { while (n--) tick(); }

// dir=0 -> 1番線で送信、0番線で受信
static void setDirection(uint8_t dir) {
  outMask = dir ? PIN0 : PIN1;
  inMask  = dir ? PIN1 : PIN0;
  DDRC |= outMask;
  PORTC &= (uint8_t)~outMask;          // アイドルはLow
  DDRC &= (uint8_t)~inMask;
}

static uint8_t exchangeBit(uint8_t myBit) {
  uint8_t got = 0;
  if (myBit) PORTC |= outMask;
  for (uint16_t p = 0; p < BIT_PULSES; p++) {
    if (p == SAMPLE_AT) got = (PINC & inMask) ? 1 : 0;
    if (p == DRIVE_PULSES) PORTC &= (uint8_t)~outMask;
    tick();
  }
  PORTC &= (uint8_t)~outMask;
  return got;
}

// Keyの起動はリセットの立ち下がりで掛かる。アイドルLow・Highパルス・Lowへ戻す。
static void triggerKey() {
  PORTC &= (uint8_t)~RST_BIT;
  DDRC |= RST_BIT;
  ticks(64);
  PORTC |= RST_BIT;
  ticks(12);
  PORTC &= (uint8_t)~RST_BIT;
}

static bool authenticate() {
  DDRB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;

  triggerKey();

  // ストリームIDは 0xf。Key側は btfsc でレベルを読むので、
  // 短いパルスではなく**1ビット周期のあいだ保持**しなければ掴まれない。
  setDirection(1);
  ticks(ID_DELAY);
  PORTC |= outMask;
  ticks(4 * ID_BIT);
  PORTC &= (uint8_t)~outMask;

  ticks(POST_ID);

  setDirection(0);
  uint8_t ok = 0;
  for (uint8_t b = 0; b < ROUND0_BITS; b++) {
    const uint8_t got = exchangeBit(TX0[b] - '0');
    if (got == (uint8_t)(RX0[b] - '0')) ok++;
  }

  // データ線だけ解放する。**クロックは止めない。**
  //
  // Nano-3では認証後にクロックを止めていた（Keyを凍結させてラウンド1の失敗を
  // 防ぐ設計）。だが実測では、その状態で読めるのはROM先頭の128KBだけだった。
  // sanniはCLK2(CICクロック)を立てたきり一度も切らない。
  //
  // 以前これをNano-3で再現しようとしてloop()からソフトウェアでトグルしたら、
  // 全バンクが0x01/0x03一色になって失敗した。原因は周波数の不定さではなく、
  // **バンク生成と兼務していたための割り込み競合**。
  // このボードはCIC専任なので、その問題は起きない。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT);
  PORTC &= (uint8_t)~(DA_BIT | DB_BIT);

  return ok == ROUND0_BITS;
}


static bool authOk;

void setup() {
  Serial.begin(57600);       // UnoのSoftwareSerialが受けられる上限

  DDRB |= LED_BIT;
  PORTB &= (uint8_t)~LED_BIT;
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT | RST_BIT);
  // CKOUT(PB0)は lfuse=0xBF のハードウェア機能。プログラムからは触らない。

  delay(300);                // 電源とカートが落ち着くのを待つ
  Serial.println(F("CIC: start"));
  authOk = authenticate();
  Serial.print(F("CIC: round0 "));
  Serial.println(authOk ? F("MATCH") : F("MISMATCH"));

  if (authOk) PORTB |= LED_BIT;
}

void loop() {
  // 認証は起動時の1回だけ。ここでは結果を示すだけ。
  // 認証成功時はPORTBに触らない（CLK_BITと同じポートなので不用意に触らない方針）。
  if (authOk) return;
  PORTB ^= LED_BIT;          // 点滅 = 不一致
  delay(200);
}
