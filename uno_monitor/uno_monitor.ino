// Uno: 電源リレー + CICクロック/リセットの監視
//
// ■ なぜUnoに載せるのか
// このボードは**カートのバスに一切触らない**。読み出しのタイミングに影響しないので、
// 「精密なタイミングを要する仕事は専任させる」という原則の対象外。
// Nano-1/2/3 がバス側を担い、こちらは外から支える側に回る。
//
// ■ 監視を足した理由（2026-08-28）
// CICデータ線(カート24/55)にはNano-3のA6/A7という観測線があり、そのおかげで
// 「データ線は健全」を30分で断定できた。
// 一方 **クロック(56)とリセット(25)には観測線が無く、推測しかできなかった。**
// Keyが起動しないのがクロック起因かリセット起因か切り分けられずに詰まったので、
// 同じ観測手段をこの2本にも用意する。
//
// ■ ピン割り当て
//   D0/D1   PCとのUSBシリアル
//   D7      リレー制御（2026-08-29にD9から移設。プロトコルは従来と互換）
//   A4      カート56番（CICクロック）の監視
//   A5      カート25番（CICリセット）の監視
//   D10-D13 ISP用に予約。触らない
//
// A4/A5はハードウェアI2CのSDA/SCLでもあるが、このスケッチはI2Cを使わないので
// アナログ入力として自由に使える。アナログ入力は駆動能力を持たないため、
// 直結しても相手の信号を乱さない。
//
// ■ ADCの速度と、何が見えて何が見えないか
// analogRead は1回約104us。カート56番のクロックは数MHzなので**波形は見えない**。
// しかしサンプリングがクロックと非同期なので、多数回読めば位相はばらける。
//
//   クロックが出ている      -> min≒0 かつ max≒1023（両方のレールを踏む）
//   Lowで止まっている        -> min≒max≒0
//   Highで止まっている       -> min≒max≒1023
//
// **min/max の開きを見るのが本質で、平均値だけを見てはいけない。**
// 平均だけだと「2.5Vの直流」と「0Vと5Vを往復する矩形波」が区別できない。
//
// カート25番のリセットは数十us〜ms単位のパルスなので、こちらは素直に波形として追える。
//
// ■ 極性（実測で確認済み。憶測で決めていない）
//     HIGH: 電源ON / LOW: 電源OFF / 浮き(入力): 電源ON
// リレーモジュールの入力は電流駆動で、駆動されなければ非励磁。負荷が常閉(NC)側なので
// 非励磁＝通電。ISP書き込み中に電源が落ちないので好都合。

#include <Arduino.h>

const uint8_t RELAY_PIN = 7;      // PD7（2026-08-29にD9から移設）
const uint8_t MON_CLK = A4;       // カート56番
const uint8_t MON_RST = A5;       // カート25番

const uint16_t OFF_MS = 8000;
const uint16_t ON_SETTLE_MS = 300;

static void powerOn() { PORTD |= _BV(PD7); }
static void powerOff() { PORTD &= (uint8_t)~_BV(PD7); }

// Arduinoのプリプロセッサは関数プロトタイプをファイル先頭に差し込むため、
// 構造体を引数に取る関数は「'Stat' has not been declared」で落ちる。
// 素直にモジュール変数で受け渡す。
uint16_t st_lo, st_hi, st_cross, st_n;
uint32_t st_sum;

static void survey(uint8_t pin, uint16_t samples) {
  st_lo = 1023;
  st_hi = 0;
  st_sum = 0;
  st_n = samples;
  st_cross = 0;
  bool prevHigh = false;
  bool first = true;
  for (uint16_t i = 0; i < samples; i++) {
    const uint16_t v = analogRead(pin);
    if (v < st_lo) st_lo = v;
    if (v > st_hi) st_hi = v;
    st_sum += v;
    const bool high = (v > 512);
    if (!first && high != prevHigh) st_cross++;
    prevHigh = high;
    first = false;
  }
}

static void printStat(const __FlashStringHelper *name) {
  const float k = 5.0f / 1023.0f;
  Serial.print(name);
  Serial.print(F(" min="));
  Serial.print(st_lo * k, 2);
  Serial.print(F("V max="));
  Serial.print(st_hi * k, 2);
  Serial.print(F("V avg="));
  Serial.print((st_sum / (float)st_n) * k, 2);
  Serial.print(F("V cross="));
  Serial.print(st_cross);
  Serial.print(F("/"));
  Serial.print(st_n);
  // 判定はホスト側でもできるが、目視で分かる方が速い
  if (st_hi - st_lo < 100) {
    Serial.println(st_hi > 512 ? F("  -> Highで静止") : F("  -> Lowで静止"));
  } else {
    Serial.println(F("  -> 振れている(信号あり)"));
  }
}

static void report(uint16_t samples) {
  survey(MON_CLK, samples);
  printStat(F("A4 cart56 CLK:"));
  survey(MON_RST, samples);
  printStat(F("A5 cart25 RST:"));
}

// 浮いているピンと、Lowに駆動されているピンを区別する。
// プルアップを掛けてHighへ動けば「誰も駆動していない＝配線が来ていない」。
static void pullupTest() {
  for (uint8_t i = 0; i < 2; i++) {
    const uint8_t pin = i ? MON_RST : MON_CLK;
    pinMode(pin, INPUT);
    delay(2);
    const uint16_t off = analogRead(pin);
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    const uint16_t on = analogRead(pin);
    pinMode(pin, INPUT);
    const float k = 5.0f / 1023.0f;
    Serial.print(i ? F("A5 cart25: ") : F("A4 cart56: "));
    Serial.print(F("開放時="));
    Serial.print(off * k, 2);
    Serial.print(F("V プルアップ時="));
    Serial.print(on * k, 2);
    Serial.println(on > 900 && off < 200
                       ? F("V  -> 浮いている（配線が来ていない疑い）")
                       : F("V  -> 何かが駆動している"));
  }
}


// 電源投入から3秒間、**隙間なく**記録する。
// 'v' を繰り返す方式では取りこぼす。認証は300ms程度しか続かず、
// analogRead 1回104us・報告に62msかかるので、観測の穴の方が事象より長い。
// 100msごとのバケツに min/max を畳んで、あとでまとめて吐く。
#define BUCKETS 30
uint8_t bkLo[BUCKETS], bkHi[BUCKETS];

static void timeline(uint8_t pin, const __FlashStringHelper *name,
                     bool cycle) {
  if (cycle) {
    powerOff();
    delay(OFF_MS);
    powerOn();
  }
  const uint32_t t0 = millis();
  for (uint8_t b = 0; b < BUCKETS; b++) {
    uint16_t lo = 1023, hi = 0;
    const uint32_t end = t0 + (uint32_t)(b + 1) * 100;
    while ((int32_t)(millis() - end) < 0) {
      const uint16_t v = analogRead(pin);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    bkLo[b] = lo >> 2;
    bkHi[b] = hi >> 2;
  }
  Serial.print(F("TIMELINE "));
  Serial.println(name);
  for (uint8_t b = 0; b < BUCKETS; b++) {
    Serial.print((b + 1) * 100);
    Serial.print(F("ms lo="));
    Serial.print(bkLo[b] * (5.0f / 255.0f), 2);
    Serial.print(F(" hi="));
    Serial.print(bkHi[b] * (5.0f / 255.0f), 2);
    Serial.println(bkHi[b] - bkLo[b] > 25 ? F("  *** 振れた") : F(""));
  }
  Serial.println(F("TIMELINE END"));
}

void setup() {
  // 先にポートビットをHighにしてから出力へ切り替える。
  // 逆順だと数サイクルLowが出て、一瞬リグの電源が切れる。
  PORTD |= _BV(PD7);
  DDRD |= _BV(PD7);
  pinMode(MON_CLK, INPUT);
  pinMode(MON_RST, INPUT);
  Serial.begin(115200);
  Serial.println(F("RELAY+MON READY (0=on 1=off c=cycle ?=state v=volts "
                   "p=pullup b=boot B/C=timeline)"));
}

void loop() {
  if (!Serial.available()) return;
  const char ch = (char)Serial.read();
  switch (ch) {
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
    case '?':
      Serial.println((PORTD & _BV(PD7)) ? F("STATE ON") : F("STATE OFF"));
      break;
    case 'v':
      report(300);
      break;
    case 'p':
      pullupTest();
      break;
    case 'T':
      // 電源はそのまま。ホストがNano-3をDTRでリセットする瞬間に合わせて使う。
      timeline(MON_CLK, F("A4 cart56 CLK (no cycle)"), false);
      break;
    case 'U':
      timeline(MON_RST, F("A5 cart25 RST (no cycle)"), false);
      break;
    case 'B':
      timeline(MON_CLK, F("A4 cart56 CLK"), true);
      break;
    case 'C':
      timeline(MON_RST, F("A5 cart25 RST"), true);
      break;
    case 'b':
      // 電源投入の瞬間から監視する。Nano-3の認証は投入後およそ300msで走るので、
      // クロックとリセットパルスが実際に出ているかはここでしか捕まえられない。
      powerOff();
      delay(OFF_MS);
      Serial.println(F("BOOT CAPTURE: power on"));
      powerOn();
      report(1500);            // 約0.3秒ぶん x2ch
      Serial.println(F("-- 200ms後 --"));
      delay(200);
      report(1500);
      Serial.println(F("-- 1s後 --"));
      delay(1000);
      report(1500);
      break;
    default:
      break;
  }
}
