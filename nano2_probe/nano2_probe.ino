// Nano-2: どのピンがカート56番の線に繋がっているかを総当たりで特定する診断用。
//
// ■ なぜ要るのか
// カート56番が「Nano-2のスケッチが走ると2.58Vに落ちる」ことは実測で確定した
// （ブートローダ中＝全ピン入力のときだけ5.00Vになる）。
// しかし nano2_master のソースには D11(PB3) を駆動する箇所が無い。
// **配線がD11だという申告と、コードの実態が食い違っている。**
//
// setup() が Low で駆動しているピンは D4/D5/D6/D10/D13 など複数ある。
// カート56番の線がそのどれかに載っていれば、同じ症状になる。
// 推測で当てるより、1本ずつ駆動して外から見る方が速い。
//
// ■ 使い方
// 起動時は**全ピン入力**。1文字送ると、そのピンだけを Low 出力にする。
//   '2'〜'9' = D2〜D9 / 'a'=D10 'b'=D11 'c'=D12 'd'=D13
//   'A'〜'F' = A0〜A5
//   'z' = 全部入力に戻す
// UnoのA4でカート56番を見ながら回せば、電圧が落ちたピンが当たり。

#include <Arduino.h>

const uint8_t PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                        A0, A1, A2, A3, A4, A5};
const char KEYS[] = "23456789abcdABCDEF";
const uint8_t N = sizeof(PINS);

static void allInput() {
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);   // プルアップ無し
}

void setup() {
  allInput();
  Serial.begin(115200);
  Serial.println(F("NANO2 PROBE: all inputs. send pin key to drive LOW, z=release"));
}

void loop() {
  if (!Serial.available()) return;
  const char c = (char)Serial.read();
  if (c == 'z') {
    allInput();
    Serial.println(F("ALL INPUT"));
    return;
  }
  for (uint8_t i = 0; i < N; i++) {
    if (c == KEYS[i]) {
      allInput();
      pinMode(PINS[i], OUTPUT);
      digitalWrite(PINS[i], LOW);
      Serial.print(F("DRIVE LOW: pin "));
      Serial.println(PINS[i]);
      return;
    }
  }
}
