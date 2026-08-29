// Uno: 電源リレー + 電圧監視 + 21.4MHzduinoのデバッグ中継
//
// ■ 3役を1枚に載せた理由
// このボードは**カートのバスに一切触らない**。読み出しのタイミングに影響しないので、
// 「精密なタイミングを要する仕事は専任させる」という原則の対象外。
// Nano-1/2/3 と 21.4MHzduino がバス側を担い、こちらは外から支える側に回る。
//
// ■ ピン割り当て
//   D0/D1   PCとのUSBシリアル（ハードウェアUART）
//   D2      SoftwareSerial RX <- 21.4MHzduino の TX(PD1)
//   D3      SoftwareSerial TX -> 21.4MHzduino の RX(PD0)
//   D9      リレー制御
//   D10-D13 ISP用に予約（ArduinoISPを焼いたときだけ使う。ここでは触らない）
//   A0      カート56番(CICクロック)の電圧監視
//   A1      カート25番(CICリセット)の電圧監視
//   A2      +5Vレールの電圧監視
//
// ■ リレーの極性（実測で確認済み。憶測で決めていない）
//
//     HIGH : 電源ON   /   LOW : 電源OFF   /   **浮き(入力) : 電源ON**
//
// リレーモジュールの入力は電流駆動なので、駆動されなければ非励磁になる。
// 負荷が常閉(NC)側なので非励磁＝通電。
// **これは好都合。** ISP書き込み中はこのピンを誰も駆動しないが、そのとき
// リグの電源は入ったままになる。切りたいときだけLOWにすればよい。
// 常開(NO)へ繋ぎ替えると逆に危険（浮き＝電源断で、書き込み中に落ちる）。
//
// **D12からD9へ移した。** ArduinoISPがD12をMISOに使うため。
//
// ■ 電圧監視でできること・できないこと
// ADCはフリーランニングでも約6.5us間隔＝153kHz。ナイキストで76kHzまで。
//   ×  CICクロック(1〜3MHz)の波形を見る、パルスを数える
//   ○  クロックが出ているか止まっているかの判定（矩形波なら平均が中間値になる）
//   ○  電圧レベルの異常検出（「1.75Vしか出ていない」「9Vが掛かっている」）
//   ○  カート25番のリセットパルス（数十us〜ms単位なので波形として追える）
//
// 今までテスターで人手でやっていた判定を、自動で連続して行えるようにする。

#include <Arduino.h>
#include <SoftwareSerial.h>

const uint8_t RELAY_PIN = 9;          // PB1
const uint8_t DBG_RX_PIN = 2;
const uint8_t DBG_TX_PIN = 3;

const uint8_t MON_CIC_CLK = A0;       // カート56番
const uint8_t MON_CIC_RST = A1;       // カート25番
const uint8_t MON_VCC     = A2;       // +5Vレール

// 電源を落としてから入れ直すまで。コンデンサが抜けきらないとSA-1が初期状態に
// 戻らない。実測で10秒待つと復帰した事例があるので、余裕をみて8秒。
const uint16_t OFF_MS = 8000;
const uint16_t ON_SETTLE_MS = 3000;

SoftwareSerial dbg(DBG_RX_PIN, DBG_TX_PIN);

static void powerOn()  { PORTB |= _BV(PB1); }
static void powerOff() { PORTB &= (uint8_t)~_BV(PB1); }

// 平均電圧を返す。矩形波なら中間値、静止していれば0Vか5Vに寄る。
static float measure(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) sum += analogRead(pin);
  return (sum / (float)samples) * 5.0f / 1023.0f;
}

static void report() {
  Serial.print(F("V cic_clk="));
  Serial.print(measure(MON_CIC_CLK, 32), 2);
  Serial.print(F(" cic_rst="));
  Serial.print(measure(MON_CIC_RST, 32), 2);
  Serial.print(F(" vcc="));
  Serial.print(measure(MON_VCC, 32), 2);
  Serial.print(F(" relay="));
  Serial.println((PORTB & _BV(PB1)) ? F("ON") : F("OFF"));
}

void setup() {
  // 先にポートビットをHighにしてから出力へ切り替える。
  // 逆順だと数サイクルLOWが出て、一瞬リグの電源が切れる。
  PORTB |= _BV(PB1);
  DDRB |= _BV(PB1);

  Serial.begin(115200);
  dbg.begin(57600);          // 21.4MHzduinoと同じ速度
  Serial.println(F("UNO CONSOLE (0=on 1=off c=cycle ?=state v=volts f=float)"));
}

void loop() {
  // 21.4MHzduinoからのデバッグ出力をPCへ中継する。
  // SoftwareSerialは受信中に割り込みを止めるが、このボードは
  // タイミングに厳しい仕事をしていないので実害がない。
  while (dbg.available()) Serial.write(dbg.read());

  if (!Serial.available()) return;
  const char c = (char)Serial.read();

  switch (c) {
    case '0':
      powerOn();
      Serial.println(F("POWER ON"));
      break;
    case '1':
      powerOff();
      Serial.println(F("POWER OFF"));
      break;
    case 'c':
      powerOff();
      Serial.println(F("CYCLING: off"));
      delay(OFF_MS);
      powerOn();
      delay(ON_SETTLE_MS);
      Serial.println(F("CYCLING: on"));
      break;
    case 'f':
      // ピンを浮かせる。ISPモード時と同じ状態を再現して確認するため。
      // 実測では浮き＝通電なので、これで電源は切れない。
      DDRB &= (uint8_t)~_BV(PB1);
      PORTB &= (uint8_t)~_BV(PB1);
      Serial.println(F("PIN FLOATING"));
      break;
    case '?':
      Serial.println((PORTB & _BV(PB1)) ? F("STATE ON") : F("STATE OFF"));
      break;
    case 'v':
      report();
      break;
    default:
      break;                 // 改行などは無視
  }
}
