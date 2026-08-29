// Nano-3: 電源投入時に自力でCIC認証（ラウンド0）を実行してクロックを止め、
// そのあとバンク生成役(A16-A23)に切り替わる。1枚で兼任するための統合版。
//
// ■ なぜPCと通信しないのか
// 吸い出しにはNano-1(アドレス)とNano-2(バス)が要り、USBはNano-2が使う。
// 3枚同時にUSBを挿すと基板を焼く（実際に2枚失った）ので、Nano-3は自律動作させる。
// したがって手順は焼き込む。
//
// ■ なぜラウンド0だけで止めるのか
// 本家CICのコードは、1ラウンド終わるごとに RUN HOST（ホストのリセット解除）を
// 実行する。16ラウンド完走を待つ必要はない。
//   d411-dis.txt  147: tml 04a   ; RUN HOST
// Key側も同じ構造なら、ラウンド0の直後にカート内チップのリセットが外れる。
//
// そして**クロックを握っているのはこちら**なので、その直後に止めればKeyは凍結する。
// 照合を続けることも、エラーを検出することも、リセットを掛け直すこともできない。
//   「3.072MHzを供給しなければCICは動作できず、P10をLOWに引くこともできない」
//
// 認証 -> クロック停止 -> Nano-2が読む、という段取り。
//
// ■ ラウンド1以降をやらない理由
// 向きが切り替わる瞬間の扱いがまだ実機と合っておらず、R1で必ず崩れる。
// R0は実機で15/15（地域ビット含む）一致することを確認済みなので、そこで止める。
//
// ■ 配線（CICソケット内で完結）
//   D12 = PB4  クロック   -> ソケット7番（カート56）
//   A3  = PC3  リセット   -> ソケット11番（カート25）
//   A4  = PC4  データA    -> ソケット2番（カート24）  1kΩ直列
//   A5  = PC5  データB    -> ソケット1番（カート55）  1kΩ直列
//   D13 = PB5  基板上のLED（結果表示）
//
// ■ LEDの意味
//   点灯しっぱなし = ラウンド0が期待どおり一致した（読み出してよい）
//   点滅           = 一致しなかった（差し直して電源を入れ直す）

#include <Arduino.h>

#define CLK_BIT _BV(PB4)
#define RST_BIT _BV(PC3)
#define DA_BIT  _BV(PC4)     // カート24
#define DB_BIT  _BV(PC5)     // カート55
#define LED_BIT _BV(PB5)

// ── バンク生成役（認証が終わったあとの仕事）─────────────────────────
// CIC役で使うピン(D12=PB4 / A3,A4,A5=PC3,4,5 / D13=PB5)と、バンク役で使うピン
// (D2-D9 / D10 / D11 / A2)は重複しない。だから1枚で兼任できる。
// nano3_bank.ino と同じ「STROBEで+1、RESETで0」方式。ポーリングでは取りこぼすので
// ピン変化割り込みで受ける。
//   A16-A21 -> D2-D7  = PORTD bit2-7
//   A22-A23 -> D8-D9  = PORTB bit0-1
//   RESET   -> D10    = PINB bit2 (PCINT2)
//   STROBE  -> D11    = PINB bit3 (PCINT3)
#define BANK_RESET_BIT  _BV(PB2)
#define BANK_STROBE_BIT _BV(PB3)

volatile uint8_t bankNo = 0;

// 認証後もCICクロック(カート56番)を出し続けるためのフラグ。
// sanniは握手をせずクロックだけ流している。それに合わせる。
volatile bool keepClockRunning = false;

static inline void writeBank(uint8_t v) {
  // PORTD bit0,1 は RX/TX。PORTB は bit2,3 が入力、bit4,5 がCIC/LED、bit6,7 が水晶
  // なので、いずれも壊さないようにマスクして書く。
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

// 実機で確定した値（クロックパルス単位）
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

  // ■ CICクロックを止めない（2026-08-26 変更）
  //
  // 以前はここで D12 を入力に戻し、Keyを凍結させていた。「ラウンド0の直後に
  // 止めればKeyは失敗を検出できない」という設計だった。だが実測では、
  // その状態で読めるのは **ROM先頭の128KB($C0-$C1)だけ**で、
  // 残り62バンクは0xFF一色（＝誰もバスを駆動していない）だった。
  //
  // sanniのcartreaderを読んだら、SA-1に**CIC認証の握手を一切していなかった**。
  // setup_Snes() でクロックを3本設定し、
  //     マスター(1番)     : 常時ON
  //     CIC(56番/3.072MHz): **常時ON**
  //     CPU/PHI2(57番)    : OFF
  // としたまま readHiRomBanks(192,...) を呼ぶだけ。
  // **握手をせずにクロックを流しっぱなしにしている。** 私たちは逆をやっていた。
  //
  // 止める理由だった「56番と57番が短絡している」も既に解消済み
  // (56=Nano-3 D12 / 57=無結線 / マスターは21.4MHzduinoから1番へ)。
  // よってクロックは流し続ける。DDRBは出力のまま、トグルを継続する。
  // ソフトウェアでloop()からトグルする方式は**失敗した**。
  // 周波数が不定(数十kHz程度)かつジッタが大きく、全バンクが0x03一色になって
  // 実データが消えた。CICクロックは3.072MHzの安定した矩形波でなければ
  // 妨害にしかならない。Nano-3にはハードウェアタイマー出力(OC1A=D9)が
  // 使えるが、D9はバンク線A23で埋まっている。**この配線では両立できない。**
  // よって従来どおり止めて開放する。
  PORTB &= (uint8_t)~CLK_BIT;
  DDRB &= (uint8_t)~CLK_BIT;
  // データ線は解放。Nano-2の読み出しの邪魔をしない。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT);
  PORTC &= (uint8_t)~(DA_BIT | DB_BIT);

  return ok == ROUND0_BITS;
}

static bool authOk;


void setup() {
  DDRB |= LED_BIT;
  PORTB &= (uint8_t)~LED_BIT;

  // バンク線(A16-A23)を出力にして0で駆動する。
  // CIC役に専念していた頃はここを入力のまま放置していたが、それは
  // **カート側でA16-A23が浮く**ということだった。認証中も0で固定しておく。
  DDRD |= 0xFC;
  DDRB |= 0x03;
  writeBank(0);

  DDRC &= (uint8_t)~(DA_BIT | DB_BIT | RST_BIT);

  delay(300);          // 電源とカートが落ち着くのを待つ
  authOk = authenticate();

  // 認証が済んだらバンク生成役に切り替わる。ここから先はNano-2のストローブで動く。
  // CICのクロック(D12)は authenticate() の最後で入力に戻してあるので、
  // Keyは凍結したまま。ラウンド1に進んで失敗する余地を与えない。
  if (authOk) PORTB |= LED_BIT;   // 割り込みを有効にする前に点けておく

  DDRB &= (uint8_t)~(BANK_RESET_BIT | BANK_STROBE_BIT);   // D10/D11 を入力に
  PCICR |= _BV(PCIE0);
  PCMSK0 = BANK_STROBE_BIT | BANK_RESET_BIT;              // A2やD12は含めない
  sei();
}

void loop() {
  // バンクの更新は割り込みが行う。ここは認証結果を示すだけ。
  //
  // **PORTBをリードモディファイライトしてはいけない。**
  // `PORTB |= LED_BIT` は「読む→変える→書く」の3段階で、その隙間にISRが入ると、
  // ISRが書いたバンク上位2ビット(PB0,PB1 = A22,A23)を古い値で上書きしてしまう。
  // バンク$C0以降はこの2ビットが両方1なので、潰れると$00-$3Fの全く別のバンクを
  // 読むことになる。実測では「前回と98%相違」「隣接バンクが同一内容」として現れた。
  // 認証成功時はsetupで一度点ければ済むので、ここでは何もしない。
  if (authOk) return;

  // 失敗時の点滅も同じ理由で割り込みを止めてから触る。
  const uint8_t sreg = SREG;
  cli();
  PORTB ^= LED_BIT;                // 点滅 = 不一致。カートを挿してから通電し直す
  SREG = sreg;
  delay(200);
}
