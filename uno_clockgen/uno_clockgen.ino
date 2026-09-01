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
// ■ 配線
//   D10 (OC1B) -> カート1番。GNDは治具と共通にすること。
//   このUnoはPCのUSBから給電される（治具の電源電圧を変えても影響を受けない）。
//
// ■ 出せる周波数  16MHz / (2 * (OCR1A+1))
//   OCR1A=0 : 8.000 MHz      =4 : 1.600 MHz
//   OCR1A=1 : 4.000 MHz  ★  =5 : 1.333 MHz
//   OCR1A=2 : 2.667 MHz      =6 : 1.143 MHz
//   OCR1A=3 : 2.000 MHz      =7 : 1.000 MHz  ★ sanniのCPUクロック相当
//
// ■ コマンド（115200bps）
//   x    クロックを止める。**D10はLOWで固定**（浮かせない）
//   z    D10を入力(ハイインピーダンス)にする。**配線の影響を切り分けるため**
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
int8_t current = -1;             // -1 = 停止

void stopClock() {
  TCCR1A = 0; TCCR1B = 0;        // タイマー切り離し
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CLK_PIN, LOW);    // 浮かせずLOWに落とす
  current = -1;
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
  // D10を入力にして、カート1番から手を離す。
  // 「Unoを繋いだことが原因か」を確かめるときに使う。
  TCCR1A = 0; TCCR1B = 0;
  pinMode(CLK_PIN, INPUT);
  current = -2;
}

void report() {
  if (current == -2) { Serial.println(F("clk=hi-Z")); return; }
  if (current < 0) { Serial.println(F("clk=off")); return; }
  // 16000 kHz / (2*(n+1))
  uint32_t hz10 = 160000000UL / (2UL * (current + 1));   // 0.1Hz単位を避け kHz*10 で
  Serial.print(F("clk=on OCR1A=")); Serial.print(current);
  Serial.print(F(" ")); Serial.print(hz10 / 10000.0, 3); Serial.println(F(" MHz"));
}

void setup() {
  Serial.begin(115200);
  hiZ();                         // **既定はハイインピーダンス**（LOW固定は駄目）
  Serial.println(F("uno_clockgen ready (z=hi-Z[既定], x=LOW固定, 0-9=on, ?=status)"));
}

void loop() {
  if (!Serial.available()) return;
  int c = Serial.read();
  if (c == 'x' || c == 'X') { stopClock(); report(); }
  else if (c >= '0' && c <= '9') { startClock(c - '0'); report(); }
  else if (c == 'z' || c == 'Z') { hiZ(); report(); }
  else if (c == '?') report();
}
