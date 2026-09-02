// Nano-3: CIC認証（ラウンド0）→ バンク生成 → OLED進捗表示 を1枚で兼任する統合版。
//
// **このファイルはGPLです。** CIC認証の実装は SuperCIC（GPL）を読んで導いた値に
// 基づいており、MITの sfc-nano-reader 本体には置けない。OLED表示部分は
// 同リポジトリの nano3_bank.ino（MIT）から持ってきたもので、GPLへの取り込みは問題ない。
//
// ■ なぜ1枚で足りるのか — ピンが重ならない
//   CIC   : D12(PB4) クロック / A3(PC3) リセット / A4(PC4)・A5(PC5) データ / D13(PB5) LED
//   バンク: D2-D9(PORTD/PB0,PB1) / D10(PB2) リセット / D11(PB3) ストローブ
//   OLED  : A0(PC0) SCL / A1(PC1) SDA        ← ソフトI2C
//   計数  : A2(PC2) バイトストローブ
// 重複ゼロ。しかもCIC線を使うのは認証中（電源投入後300ms〜）だけで、その後は解放する。
//
// ■ 踏んではいけない地雷が3つある
//
// 1. **PORTBをリードモディファイライトしてはいけない。**
//    `PORTB |= LED_BIT` は読む→変える→書くの3段階で、その隙間にPCINT0のISRが入ると
//    ISRが書いたバンク上位2ビット(PB0,PB1 = A22,A23)を古い値で上書きする。
//    バンク$C0以降はこの2ビットが両方1なので、潰れると$00-$3Fの別バンクを読む。
//    実測では「前回と98%相違」「隣接バンクが同一内容」として現れた。
//    LEDはsetupで一度だけ書き、loop()では触らない。
//
// 2. **PCMSK1にA0/A1を含めてはいけない。** そこはI2Cの出力線で、
//    自分が出した変化で割り込みが掛かり続ける。A2(PCINT10)だけを有効にする。
//    CICのA3/A4/A5も同じ理由で含めない。
//
// 3. **OLEDの初期化は認証のあと。** U8x8は内部でdelay()を使う。
//    認証はクロックのパルス数で時間を測っているので、先に走らせると壊れる。
//
// ■ LEDとOLEDの役割
// 認証結果はOLEDに文字で出す。LED(D13)は補助で、点灯=成立 / 点滅=不一致。
// 以前はLEDしか無く、「OLEDが点かない」ことでファームの取り違えに気づけなかった。

#include <Arduino.h>
#include <U8x8lib.h>


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
// OLEDはソフトI2C。D13はLEDが負荷になるので使わず、A0/A1に逃がしてある。
U8X8_SH1106_128X64_NONAME_SW_I2C oled(/*clock=*/ A0, /*data=*/ A1,
                                      /*reset=*/ U8X8_PIN_NONE);
#define BYTE_BIT _BV(PC2)          // A2 = Nano-2 D4から分岐した1バイトごとのパルス
volatile uint16_t byteCount = 0;

// ── CIC認証のやり直し ────────────────────────────────────────────
// **電源投入時に1回だけ認証する作りだった。** やり直すには電源を切るしかなく、
// 実験のたびに電源サイクルが要る原因になっていた。
//
// バンクリセット線(D10/PB2)のパルス幅で指示を分ける。
//   短い → 従来どおりバンクを0に戻す
//   長い → それに加えて認証をやり直す
// Nano-1で /WR を足したときと同じ手。配線を増やさずに合図を送れる。
//
// **ISRの中では認証しない。** 認証は数十ms掛かるうえ ticks() でクロックを刻むので、
// ISR内で回すとバンクストローブを取りこぼす。フラグだけ立てて loop() で実行する。
const uint16_t REAUTH_COUNT_MIN = 400;   // これ以上ならやり直し指示
const uint16_t REAUTH_COUNT_MAX = 1600;  // 上限。異常に長いとき抜けるため
volatile bool reauthRequest = false;

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
    // **従来の動作を先に済ませる。ここを遅らせてはいけない。**
    // Nano-1で同じことをやって読み出しを壊した（0/6）。
    bankNo = 0;
    byteCount = 0;          // 1バンクの読み出し開始。バイト位置も0から
    writeBank(0);
    // そのあと、まだHIGHのままかを数える。長ければ「CIC認証をやり直せ」の合図。
    // Nano-2側は resetNano3Bank() を長く出すだけでよく、**配線を増やさずに済む。**
    uint16_t n = 0;
    while ((PINB & BANK_RESET_BIT) && n < REAUTH_COUNT_MAX) n++;
    if (n >= REAUTH_COUNT_MIN) reauthRequest = true;   // ISR内では走らせない
  } else if (rose & BANK_STROBE_BIT) {
    writeBank(++bankNo);
  }
}

// PCINT1 は PORTC の変化。A2だけを有効にしてある。
// 1バイト約47us＝約21kHz。表示用の数え上げなので、OLED更新中に取りこぼしても
// 表示がわずかにずれるだけで実害はない。
ISR(PCINT1_vect) {
  static uint8_t last = 0;
  const uint8_t now = PINC & BYTE_BIT;
  if (now && !last) byteCount++;
  last = now;
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

static bool authOk;   // loop() の再認証からも書くのでファイル全体で持つ


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

  // ここから先は時間に厳しい仕事が無いので、OLEDを起こしてよい。
  pinMode(A2, INPUT);
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.clear();
  oled.drawString(0, 0, "SFC DUMPER");
  oled.drawString(0, 2, authOk ? "CIC round0 OK " : "CIC FAILED    ");
  oled.drawString(0, 4, authOk ? "waiting..."     : "replug & power");

  DDRB &= (uint8_t)~(BANK_RESET_BIT | BANK_STROBE_BIT);   // D10/D11 を入力に
  PCICR |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 = BANK_STROBE_BIT | BANK_RESET_BIT;              // D12は含めない
  PCMSK1 = _BV(PCINT10);   // A2のみ。A0/A1(I2C)とA3-A5(CIC)は絶対に含めない
  sei();
}

void loop() {
  // ── CIC認証のやり直し（ISRが立てたフラグを見て実行する）──────────
  // **ISRの中では絶対に走らせない。** 認証は数十ms掛かり ticks() でクロックを刻むので、
  // ISR内で回すとバンクストローブを取りこぼす。
  if (reauthRequest) {
    noInterrupts(); reauthRequest = false; interrupts();

    // **認証中はすべての割り込みを止める。**
    // authenticate() は ticks() でクロックのパルス数を数えて時間を測る。
    // 割り込みが1回でも入ると波形が伸びて握手が壊れる。
    //
    // setup() から呼ぶときは sei() より前なので元から割り込みが無い。
    // **loop() から呼ぶときは全部有効なので、ここで明示的に止める必要がある。**
    // PCIE0(バンク) だけ落としても、PCIE1(A2のバイトストローブ) と
    // Timer0(millis) が残る。それでは足りない。
    noInterrupts();
    authOk = authenticate();     // 入口で線を確保し、出口で解放する自己完結型
    interrupts();

    // バンク側の状態を作り直す。認証中にPORTB/PORTCを触っているため。
    bankNo = 0; byteCount = 0;
    writeBank(0);

    oled.drawString(0, 2, authOk ? "CIC round0 OK " : "CIC FAILED    ");
    oled.drawString(0, 4, authOk ? "re-auth done  " : "re-auth FAILED");
    return;                      // 描画は次の周回に回す
  }

  // 割り込みで進む値を、人が読める速さで描くだけ。
  // I2Cは数ms掛かるので、毎回描くと割り込みを塞ぐ時間が増える。
  //
  // **PORTBには触らない。** ISRが書いたバンク上位2ビットを壊すため（冒頭の地雷1）。
  // 認証失敗の表示もOLEDで済ませ、LEDの点滅はやめた。
  static uint32_t lastDraw = 0;
  static uint8_t shownBank = 255;
  if (millis() - lastDraw < 120) return;
  lastDraw = millis();

  uint8_t b;
  uint16_t n;
  noInterrupts();          // 16bitの読み出しは分割されると壊れるので止めて読む
  b = bankNo;
  n = byteCount;
  interrupts();

  char line[17];
  if (b != shownBank) {
    shownBank = b;
    snprintf(line, sizeof(line), "Bank $%02X       ", (unsigned)b);
    oled.drawString(0, 4, line);
  }
  snprintf(line, sizeof(line), "%5u/65536    ", (unsigned)n);
  oled.drawString(0, 6, line);
}
