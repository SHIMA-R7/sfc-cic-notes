// Nano-3: sanni方式。CICの握手をせず、**起動してクロックを流し続けるだけ**。
// あわせてバンク生成役(A16-A23)も兼ねる。
//
// ■ なぜ握手をやめるのか
// sanni/cartreader の setup_Snes() を読んだら、SA-1に対して**CIC認証の握手を
// 一切していなかった**。やっているのはこれだけ:
//
//     DDRG |= (1<<1); PORTG |= (1<<1);   // cicrstPin を High = CICをリセット保持
//     ...クロック3本を設定...
//     PORTG &= ~(1<<1);                  // Low = CICを起動
//     delay(500);                        // リセット解除を待つ
//     getCartInfo_SNES();                // 読む
//
// そして CLK2(CIC/カート56番) は **3.072MHz を流しっぱなし**。
// カートの/RESET(PH0)はHighのまま触らない。
//
// このリグはこれまで「ラウンド0を通してからクロックを止める」方式だった。
// その状態で読めるのはROM先頭128KB($C0-$C1)だけで、残りは0xFF一色だった。
// 先人と同じやり方に戻す。
//
// ■ 配線（CICソケット内で完結。従来と同じ）
//   D12 = PB4  クロック   -> ソケット7番（カート56）
//   A3  = PC3  リセット   -> ソケット11番（カート25）
//   A4  = PC4  データA    -> ソケット2番（カート24）  1kΩ直列
//   A5  = PC5  データB    -> ソケット1番（カート55）  1kΩ直列
//   D13 = PB5  基板上のLED（動作表示）
//
// ■ バンク生成（従来と同じ）
//   A16-A21 -> D2-D7  = PORTD bit2-7
//   A22-A23 -> D8-D9  = PORTB bit0-1
//   RESET   -> D10    = PINB bit2 (PCINT2)
//   STROBE  -> D11    = PINB bit3 (PCINT3)
//
// ■ クロック周波数について
// tick() は PORTB のセット/クリア2命令なので 16MHz/6 ≒ 2.7MHz。
// sanniの3.072MHzに近い。ハードウェアタイマー出力(OC1A=D9)はバンク線A23で
// 埋まっているので使えない。ソフトウェアで回すしかないが、loop()が
// これ以外に何もしなければジッタは小さく保てる。
//
// ■ データ線について
// 握手をしないので、CICのデータ線(A4/A5)は**入力のまま開放**する。
// こちらから駆動すると、Keyの出力とぶつかる。

#include <Arduino.h>

#define CLK_BIT _BV(PB4)
#define RST_BIT _BV(PC3)
#define DA_BIT  _BV(PC4)
#define DB_BIT  _BV(PC5)
#define LED_BIT _BV(PB5)

#define BANK_RESET_BIT  _BV(PB2)
#define BANK_STROBE_BIT _BV(PB3)

volatile uint8_t bankNo = 0;

static inline void writeBank(uint8_t v) {
  // PORTD bit0,1 は RX/TX。PORTB は bit2,3 が入力、bit4,5 がCIC/LED、bit6,7 が水晶。
  PORTD = (uint8_t)((PORTD & 0x03) | (uint8_t)((v & 0x3F) << 2));
  PORTB = (uint8_t)((PORTB & 0xFC) | (uint8_t)((v >> 6) & 0x03));
}

ISR(PCINT0_vect) {
  static uint8_t last = 0;
  const uint8_t now = PINB & (BANK_STROBE_BIT | BANK_RESET_BIT);
  const uint8_t rose = (uint8_t)(now & ~last);
  last = now;
  if (rose & BANK_RESET_BIT) {
    bankNo = 0;
    writeBank(0);
  } else if (rose & BANK_STROBE_BIT) {
    writeBank(++bankNo);
  }
}

static inline void tick() {
  PORTB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;
}

void setup() {
  DDRB |= LED_BIT;
  PORTB |= LED_BIT;              // 通電の目印。点灯しっぱなし

  // バンク線を出力にして0で駆動する。入力のままだとカート側で浮く。
  DDRD |= 0xFC;
  DDRB |= 0x03;
  writeBank(0);

  // CICのデータ線は入力のまま開放。握手をしないので駆動してはいけない。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT);
  PORTC &= (uint8_t)~(DA_BIT | DB_BIT);

  // CICクロックを出力に。
  DDRB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;

  // sanniの手順: まずCICをリセット保持し、クロックを流しながら解除する。
  // このリグのKeyは「アイドルLow・アクティブHigh」と実測で分かっているので、
  // Highに保つ = リセット保持、Lowに落とす = 起動。
  DDRC |= RST_BIT;
  PORTC |= RST_BIT;              // リセット保持
  for (uint16_t i = 0; i < 2000; i++) tick();   // クロックを流しておく

  PORTC &= (uint8_t)~RST_BIT;    // 起動

  // sanniの delay(500) 相当。クロックを流しながら待つ。
  // 2.7MHzで500ms分は多すぎるので、Keyが立ち上がるのに十分な量として
  // 100000tick(約37ms)を流す。足りなければ増やす。
  for (uint32_t i = 0; i < 100000UL; i++) tick();

  // ここからバンク生成役も兼ねる。CICクロックはloop()で流し続ける。
  DDRB &= (uint8_t)~(BANK_RESET_BIT | BANK_STROBE_BIT);
  PCICR |= _BV(PCIE0);
  PCMSK0 = BANK_STROBE_BIT | BANK_RESET_BIT;
  sei();
}

void loop() {
  // CICクロックを流し続ける。sanniのCLK2が常時ONなのと同じ状態を作る。
  //
  // PORTBのリードモディファイライトはISRが書くバンク上位2ビット
  // (PB0,PB1 = A22,A23)と競合する。以前これでバンクを壊したので、
  // 割り込みを止めてから触る。1回あたり数サイクルなので、
  // バンクストローブを取りこぼす心配はない。
  const uint8_t sreg = SREG;
  cli();
  tick();
  SREG = sreg;
}
