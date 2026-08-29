// Nano-3: 電源投入時に自力でCIC認証（ラウンド0）を実行し、終わったらクロックを止める。
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

  // クロックを止める。ただし**Lowで駆動したままにしてはいけない。**
  // このリグはカート56番と57番を短絡させてあるので、
  // D12をLowに保つと PHI2 まで Low に縛ってしまう。
  // これまで成功した吸い出しでは PHI2 は常に浮かせていた。
  // 入力に戻して開放する。誰もクロックを供給しないのでKeyは凍結したまま。
  PORTB &= (uint8_t)~CLK_BIT;
  DDRB &= (uint8_t)~CLK_BIT;
  // データ線は解放。Nano-2の読み出しの邪魔をしない。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT);
  PORTC &= (uint8_t)~(DA_BIT | DB_BIT);

  return ok == ROUND0_BITS;
}

static bool authOk;

// 読み出す対象バンク。Nano-3はCIC専任でストローブを受けないので、
// 1回の焼き込みで読めるのはここで指定した1バンクだけになる。
// $00 と $C0 の両方を見たいときは、値を変えて焼き直す。
const uint8_t TARGET_BANK = 0x00;

void setup() {
  DDRB |= LED_BIT;
  PORTB &= (uint8_t)~LED_BIT;

  // バンク線(A16-A23)を TARGET_BANK で駆動する。
  //
  // ここは以前「バンク出力には触らない。Nano-3は今回CIC専任」として入力のままに
  // していたが、それは**カート側でA16-A23が浮く**ということだった。浮いた
  // アドレス線で読んだ結果は「バンク$00を読んだ」ことにならない。
  // プランA(ラウンド0だけで吸い出す)を否定した実測は、この状態で取っている。
  //   A16-A21 -> D2-D7 = PORTD bit2-7
  //   A22-A23 -> D8-D9 = PORTB bit0-1
  DDRD |= 0xFC;                                    // D2-D7 を出力
  PORTD = (uint8_t)((PORTD & 0x03) | (uint8_t)((TARGET_BANK & 0x3F) << 2));
  DDRB |= 0x03;                                    // D8-D9 を出力
  PORTB = (uint8_t)((PORTB & 0xFC) | (uint8_t)((TARGET_BANK >> 6) & 0x03));

  DDRC &= (uint8_t)~(DA_BIT | DB_BIT | RST_BIT);

  delay(300);          // 電源とカートが落ち着くのを待つ
  authOk = authenticate();
}

void loop() {
  if (authOk) {
    PORTB |= LED_BIT;              // 点灯しっぱなし = 認証一致。読み出してよい
  } else {
    PORTB ^= LED_BIT;              // 点滅 = 不一致。電源を入れ直す
    delay(200);
  }
}
