// Nano-3: D11とD12が短絡しているかを、外部の測定器なしで判定する。
//
// ■ 何を確かめるのか
// カート56番が「Nano-2のD6(バンクストローブ)にLowで引かれている」ことは実測で確定した。
// そこへ至る経路が2通り考えられる:
//
//   (a) Nano-3のD11とD12が半田ブリッジで短絡している
//       → カート56 - D12 - [ブリッジ] - D11 - Nano-2 D6
//   (b) カート56番に、ストローブ網へ行く2本目の線が付いている
//
// **どちらなのかは、Nano-3の内側から見れば分かる。**
// D11に内部プルアップを掛け、D12を出力Lowにする。
// 短絡していればD11はLowに引き倒される。していなければHighのまま。
//
// ■ 交絡に注意
// Nano-2のD6は setup() で出力Lowに固定される。繋がっている限りD11は常にLowになり、
// ブリッジの有無を判別できない。**測る前にNano-2を全ピン入力にしておくこと**
// (nano2_probe を焼いて 'z' を送る)。
//
//   D11 = PB3  バンクストローブ入力（Nano-2 D6から）
//   D12 = PB4  CICクロック出力（カート56番へ）

#include <Arduino.h>

const uint8_t STROBE_PIN = 11;   // PB3
const uint8_t CLK_PIN = 12;      // PB4

static void report(const char *label) {
  delay(30);
  Serial.print(label);
  Serial.print(F(" -> D11 = "));
  Serial.println(digitalRead(STROBE_PIN) ? "HIGH" : "LOW");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("=== D11/D12 短絡テスト ==="));
  Serial.println(F("(事前にNano-2を全ピン入力にしておくこと)"));

  // D11は内部プルアップで持ち上げておく。誰も引かなければHighのはず。
  pinMode(STROBE_PIN, INPUT_PULLUP);

  pinMode(CLK_PIN, INPUT);
  report("D12=開放      ");

  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  report("D12=LOW出力   ");     // 短絡していればここでLOWになる

  digitalWrite(CLK_PIN, HIGH);
  report("D12=HIGH出力  ");

  pinMode(CLK_PIN, INPUT);
  report("D12=開放に戻す");

  pinMode(STROBE_PIN, INPUT);
  Serial.println(F("=== 判定 ==="));
  Serial.println(F("D12=LOW出力 のときだけ D11 が LOW になったら -> 短絡している"));
  Serial.println(F("常に HIGH なら -> 短絡していない"));
}

void loop() {}
