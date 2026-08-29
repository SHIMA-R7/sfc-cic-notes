// Nano-3: CIC配線の静的チェック用。テスターで直接測るためのもの。
//
// 握手を試す前に「そもそもNano-3から線を叩けているか」「相手の電位はどうなっているか」
// を確かめる。動いている信号を追いかける前に、止まった状態を確認する。
//
//   D12 = PB4  クロック  -> ソケット7番 (カート56)
//   A3  = PC3  リセット  -> ソケット11番 (カート25)
//   A4  = PC4  データA   -> ソケット2番 (カート24)   1kΩ直列
//   A5  = PC5  データB   -> ソケット1番 (カート55)   1kΩ直列
//
// シリアル(115200)で1文字送る:
//   1/2/3/4 = D12 / A3 / A4 / A5 を High固定で出力
//   q/w/e/r = 同じ順に Low固定で出力
//   z       = 全部入力(開放)に戻す。相手が駆動している電位を見たいとき
//   ?       = 4本を入力にして現在の読み値を報告
//   v       = A6/A7で「抵抗の向こう側」の電圧を読む（出力状態は変えない）
//
// ■ A6/A7を電圧計にした理由
// A4/A5は1kΩの手前にあるので、そこを読んでも自分が出した値しか見えない。
// 知りたいのは抵抗の向こう、つまり相手と実際にぶつかっている点の電位。
// A6/A7はアナログ入力専用で駆動能力がないため、そこへ直結しても信号を乱さない。
//   A6 -> ソケット2番 (カート24)   A4の抵抗の向こう側
//   A7 -> ソケット1番 (カート55)   A5の抵抗の向こう側

#include <Arduino.h>

const uint8_t PINS[4] = {12, A3, A4, A5};
const char *NAMES[4] = {"D12 clk (cart56)", "A3 rst (cart25)",
                        "A4 dataA (cart24)", "A5 dataB (cart55)"};

static void allInput() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PINS[i], INPUT);      // プルアップ無し。相手の駆動をそのまま見る
  }
}

static void pullupRead() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
  }
  delay(20);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(NAMES[i]);
    Serial.print(F(" pullup= "));
    Serial.println(digitalRead(PINS[i]) ? "HIGH (入力は健全)" : "LOW  (持ち上がらない)");
  }
  for (uint8_t i = 0; i < 4; i++) pinMode(PINS[i], INPUT);
}

static void report() {
  allInput();
  delay(5);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(NAMES[i]);
    Serial.print(" = ");
    Serial.println(digitalRead(PINS[i]) ? "HIGH" : "LOW");
  }
}

void setup() {
  allInput();
  Serial.begin(115200);
  Serial.println(F("pin test ready. 1234=High qwer=Low z=open ?=read"));
}

void loop() {
  if (!Serial.available()) return;
  const char c = (char)Serial.read();
  const char *hi = "1234", *lo = "qwer";

  for (uint8_t i = 0; i < 4; i++) {
    if (c == hi[i]) {
      pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH);
      Serial.print(NAMES[i]); Serial.println(F(" -> HIGH 出力"));
      return;
    }
    if (c == lo[i]) {
      pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], LOW);
      Serial.print(NAMES[i]); Serial.println(F(" -> LOW 出力"));
      return;
    }
  }
  if (c == 'z') { allInput(); Serial.println(F("全部 入力(開放)")); }
  else if (c == '?') report();
  else if (c == 'g') pullupRead();
  else if (c == 'v') {
    // 出力の状態は変えない。今まさに駆動している最中の向こう側を見たいので。
    for (uint8_t i = 0; i < 2; i++) {
      const uint8_t ch = i ? A7 : A6;
      analogRead(ch);                       // 入力切替直後の1回目は捨てる
      const uint16_t v = analogRead(ch);
      Serial.print(i ? F("A7 = ソケット1番(カート55): ")
                     : F("A6 = ソケット2番(カート24): "));
      Serial.print(v * 5.0f / 1023.0f, 2);
      Serial.print(F(" V  (raw "));
      Serial.print(v);
      Serial.println(F(")"));
    }
  }
}
