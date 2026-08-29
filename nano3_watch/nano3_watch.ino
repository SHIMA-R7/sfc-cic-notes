// Nano-3: D12(カート56番)の網が、外から引かれているかを自分で見張る。
//
// ■ なぜUnoを使わないのか
// これまで「カート56番」の電圧はUnoのA4で測っていた。しかしその監視線が本当に
// カート56番へ着地しているかは**一度も検証していない**。
// 監視線がストローブ線側に載っていれば、測定はすべて別の場所を見ていたことになる。
//
// ここではNano-3の内部だけで完結させる。D12に内部プルアップを掛けて digitalRead する。
//   誰も引いていなければ HIGH
//   Nano-2のD6を駆動したときだけ LOW になるなら、その2つの網は本当に繋がっている
//   D6を駆動しても HIGH のままなら、**Uno経由の観測が間違っていた**
//
// D11(バンクストローブ入力)も同時に見る。こちらはNano-2 D6と正規に繋がっているので、
// D6を駆動すればLOWになるはず。**これが対照になる**（測定手法が働いている証拠）。

#include <Arduino.h>

const uint8_t CLK_PIN = 12;      // PB4 -> カート56番
const uint8_t STROBE_PIN = 11;   // PB3 <- Nano-2 D6（正規のバンクストローブ）

void setup() {
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(STROBE_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(200);
  Serial.println(F("WATCH: D12(cart56) と D11(strobe) を内部プルアップで監視"));
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 400) return;
  last = millis();
  Serial.print(F("D12(cart56)="));
  Serial.print(digitalRead(CLK_PIN) ? "HIGH" : "LOW ");
  Serial.print(F("   D11(strobe)="));
  Serial.println(digitalRead(STROBE_PIN) ? "HIGH" : "LOW");
}
