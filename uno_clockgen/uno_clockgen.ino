// Uno: カート1番（マスタークロック）へ、PCから周波数を変えられるクロックを出す。
//
// ■ なぜ必要か
// clockduino は 21.477MHz を CKOUT(lfuse=0xBF) で出しており、**ソフトから止められない。**
// Unoなら 16MHz を分周して出せるうえ、シリアルで実行中に切り替えられる。
// sanniは SA-1 の解錠に **4MHz** を使っている（21.477MHzではない）。
//   set_freq(400000000ULL, SI5351_CLK0);  // EXT 4 MHz
//   // Set clocks to 4Mhz/1Mhz for better SA-1 unlocking
// この「遅いクロックなら暴れないのか」を実機で確かめるための道具。
//
// ■ 配線（2026-09-02 変更）
//   A5  (PC5)  -> **カート54番（/WR）を観測する**。入力。駆動しない
//   D10 (OC1B) -> **カート57番（PHI2 / CPUクロック）**
//   D8         -> clockduino の VCC（カート1番へ 21.477MHz を出す board の電源）
//   GNDは治具と共通にすること。
//
//   **clockduino の D9 は 57番から外すこと。** 給電中は常に出ているので
//   ソフトで止められず、D10と同時に繋ぐと出力どうしがぶつかる。
//   1番は clockduino、57番は Uno、と担当を分ける。
//   このUnoはPCのUSBから給電される（治具の電源電圧を変えても影響を受けない）。
//
// ■ 出せる周波数  16MHz / (2 * (OCR1A+1))
//   OCR1A=0 : 8.000 MHz      =4 : 1.600 MHz
//   OCR1A=1 : 4.000 MHz  ★  =5 : 1.333 MHz
//   OCR1A=2 : 2.667 MHz      =6 : 1.143 MHz
//   OCR1A=3 : 2.000 MHz      =7 : 1.000 MHz
//
// **sanniのPHI2は 3.579545 MHz だが、16MHzの整数分周では作れない。**
//   16/3.579545 = 4.47 で割り切れない。挟めるのは 4.000MHz(+11.7%) と
//   2.667MHz(-25.5%)。**4.000MHz が最も近い。**
//   高速PWM(f = 16/(TOP+1))を使えば 3.200MHz(-10.6%) も出せるので、
//   コマンド 'p' で切り替えられるようにした。デューティは60%になる。
//
// ■ コマンド（115200bps）
//   x    クロックを止める。**D10はLOWで固定**（浮かせない）
//   z    D10を入力(ハイインピーダンス)にする。**配線の影響を切り分けるため**
//   s    /WR(A5)の監視を開始する。カウンタを0にして立ち下がりを数え始める
//   e    監視を終了し「立ち下がり回数 現在のレベル」を返す
//   v    clockduino(21.477MHz)へ給電する。D8=HIGH
//   w    clockduino への給電を止める。D8=LOW
//
// ■ D8 で clockduino の VCC を直接駆動している
// AVRのGPIOは1本あたり定格20mA・絶対最大40mA。21.477MHzで動くATmega328は
// 12〜20mA食うので**定格の境目**である。裸のDIPなら通るが、LEDやレギュレータの
// 載った基板だと超える。発熱するようなら 2N7000 でGND側を切る方式へ変えること。
//
// ■ 排他制御は不要になった
// D10がカート57番に移ったので、1番(clockduino)と57番(D10)は別の線になった。
// 両方を同時に出せる。sanniの構成がまさにそれである。
//   0-9  OCR1A をその値にしてクロックを出す
//   ?    現在の状態を返す
//
// ■ 既定は「ハイインピーダンス」。**LOW固定にしてはいけない。**
// 最初は「浮かせないほうが安全」と考えてLOW固定を既定にしたが、**これが誤りだった。**
// カービィSDXで実測したところ、はっきり分かれた。
//
//     D10 = LOW固定        $C0 は 1種のみ → 読めない  0/3   （電流 92mA）
//     D10 = ハイインピーダンス $C0 は 256種   → 読める    3/3   （電流 72mA）
//
// SA-1はクロック入力の遷移だけでなくレベルも見ているらしい。浮いていれば
// 「未接続」、LOWだと「クロックが来ていて停止中」と解釈するのかもしれない（推測）。
// 電流まで20mA違うので、LOW固定では何かが余分に動いている。
//
// 「クロックを与えない」とは「駆動しない」であって「LOWにする」ではない。
// 従来は配線していなかったので結果的にハイインピーダンスで、区別が付いていなかった。

const uint8_t CLK_PIN = 10;      // OC1B
const uint8_t CD_PWR  = 8;       // clockduino(21.477MHz)のVCC
const uint8_t WR_WATCH = A5;     // カート54番(/WR)の観測。**入力のまま。絶対に駆動しない**
volatile uint16_t wrFalls = 0;   // /WR の立ち下がり回数
volatile bool     watching = false;

// PCINT1 は PORTC の変化でまとめて呼ばれる。A5(PC5)の立ち下がりだけ数える。
ISR(PCINT1_vect) {
  static uint8_t last = _BV(PC5);
  const uint8_t now = PINC & _BV(PC5);
  if (watching && last && !now) wrFalls++;   // HIGH -> LOW
  last = now;
}
int8_t current = -1;             // -1 = 停止 / -2 = ハイインピーダンス
bool cdOn = false;               // clockduinoに給電しているか

void cdPower(bool on) {
  pinMode(CD_PWR, OUTPUT);
  digitalWrite(CD_PWR, on ? HIGH : LOW);
  cdOn = on;
}

void stopClock() {
  TCCR1A = 0; TCCR1B = 0;        // タイマー切り離し
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CLK_PIN, LOW);    // 浮かせずLOWに落とす
  current = -1;
}

// 高速PWM。f = 16MHz/(TOP+1)。CTCでは作れない周波数を埋めるため。
// デューティは (OCR1B+1)/(TOP+1) なので厳密な50%にはならない。
void startFastPwm(uint8_t top) {
  pinMode(CLK_PIN, OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);   // 高速PWM, TOP=OCR1A
  OCR1A = top;
  OCR1B = top / 2;
  current = 100 + top;
}

void startClock(uint8_t ocr) {
  pinMode(CLK_PIN, OUTPUT);
  TCCR1A = _BV(COM1B0);              // 比較一致で OC1B をトグル
  TCCR1B = _BV(WGM12) | _BV(CS10);   // CTC(TOP=OCR1A), 分周なし
  OCR1A = ocr;
  OCR1B = 0;                         // TOPより手前で1回トグル
  current = ocr;
}

void hiZ() {
  // ここでは clockduino の状態に触らない（給電したままD10だけ手を離す用途がある）
  // D10を入力にして、カート1番から手を離す。
  // 「Unoを繋いだことが原因か」を確かめるときに使う。
  TCCR1A = 0; TCCR1B = 0;
  pinMode(CLK_PIN, INPUT);
  current = -2;
}

void startWatch() {
  pinMode(WR_WATCH, INPUT);      // **駆動しない。** プルアップも掛けない
  wrFalls = 0;
  watching = true;
  PCICR  |= _BV(PCIE1);
  PCMSK1 |= _BV(PCINT13);        // A5 だけ
}

void endWatch() {
  watching = false;
  Serial.print(F("wr_falls="));
  Serial.print(wrFalls);
  Serial.print(F(" level="));
  Serial.println((PINC & _BV(PC5)) ? F("HIGH") : F("LOW"));
}

void report() {
  Serial.print(F("cart1="));
  Serial.print(cdOn ? F("21.477MHz") : F("なし"));
  Serial.print(F(" / cart57="));
  if (current == -2) { Serial.println(F("hi-Z")); return; }
  if (current == -1) { Serial.println(F("LOW固定")); return; }
  if (current >= 100) {                       // 高速PWM
    Serial.print(16000.0 / (current - 100 + 1) / 1000.0, 3);
    Serial.println(F(" MHz (高速PWM)")); return;
  }
  if (current < 0) { Serial.println(F("clk=off")); return; }
  // 16000 kHz / (2*(n+1))
  Serial.print(16000.0 / (2.0 * (current + 1)) / 1000.0, 3);
  Serial.println(F(" MHz"));
}

void setup() {
  Serial.begin(115200);
  pinMode(WR_WATCH, INPUT);      // /WR は観測のみ。**出力にしない**
  cdPower(false);                // **既定は clockduino に給電しない**
  hiZ();                         // **既定はハイインピーダンス**（LOW固定は駄目）
  Serial.println(F("ready: cart57= z:hi-Z x:LOW 0-9:CTC p:3.2MHz / cart1= v:on w:off / ?:status"));
}

void loop() {
  if (!Serial.available()) return;
  int c = Serial.read();
  if (c == 'x' || c == 'X') { stopClock(); report(); }
  else if (c >= '0' && c <= '9') { startClock(c - '0'); report(); }
  else if (c == 'z' || c == 'Z') { hiZ(); report(); }
  else if (c == 'v' || c == 'V') { cdPower(true);  report(); }
  else if (c == 'p' || c == 'P') { startFastPwm(4); report(); }   // 3.200MHz
  else if (c == 'w' || c == 'W') { cdPower(false); report(); }
  else if (c == 's' || c == 'S') { startWatch(); Serial.println(F("watch=on")); }
  else if (c == 'e' || c == 'E') { endWatch(); }
  else if (c == '?') report();
}
