// 電源リレー制御。リーダー(Nano-1/2/3とカート)の電源をPCから入切する。
//
// ■ なぜ必要か
// SA-1カートは「電源投入直後の短い時間だけよく開く」ことが実測で分かっている。
// 再投入直後に残り12バンクを測ったら、60回かけても取れなかったものが
// **1回読みで100%**埋まった。しかしその窓は数分で閉じる。
//
// 従来は電源の入切が人手だったので、その最良の条件を無人で作れなかった。
// これがあれば「電源OFF→ON→即座に全バンク読む→CRC判定→繰り返す」を
// 一晩中回せる。
//
// ■ 配線
//   D12 -> リレーの制御入力
//   **HIGH = リーダーに通電**
//   **LOW  = リーダーの電源が切れる**
//
// 当初「HIGHで電源が切れる」と聞いていたが、**実測は逆だった**。
// HIGHにするとNano-2のCOMポートが現れ、LOWにすると消えた。
// アクティブLOW型のリレーモジュールでよくある食い違い。
// 憶測で合わせず、ポートの出現/消失で確認した結果をそのまま採用する。
//
// ■ 浮かせた場合も実測した（重要）
//
//     HIGH : 電源ON   /   LOW : 電源OFF   /   **浮き(入力) : 電源ON**
//
// リレーモジュールの入力は電流駆動なので、駆動されなければ非励磁になる。
// 負荷が常閉(NC)側に繋がっているため、非励磁＝通電。
//
// **これは好都合。** ISP書き込み中やシリアル素通し中はこのピンを誰も駆動しないが、
// そのときリグの電源は入ったままになる。切りたいときだけLOWにすればよい。
// 常開(NO)側へ繋ぎ替えると逆に危険（浮き＝電源断になり、書き込み中に落ちる）。
//
// ■ 安全な初期状態
// 起動時は必ず LOW(通電) にする。リセットや書き込み直後に電源が切れると、
// 読み出し中のNanoが不意に落ちる。pinModeより先にPORTBを立てておくことで、
// 出力に切り替わる瞬間に一瞬もHIGHを出さない
// (nano2_masterの CART_RESET_PIN で同じ配慮をしているのと同じ理由)。
//
// ■ シリアルコマンド
//   '0' -> 通電 (LOW)
//   '1' -> 電源断 (HIGH)
//   'c' -> 電源を切って一定時間待ち、入れ直す (サイクル)
//   '?' -> 現在の状態を返す
// 応答は必ず1行返す。PC側が待てるようにするため。

#include <Arduino.h>

// **D12からD9へ移した（2026-08-28）。**
// 同じUnoでISP書き込みも行うため。ArduinoISPは D10(RESET) / D11(MOSI) /
// D12(MISO) / D13(SCK) を使うので、D12のままだとMISOと衝突する。
// D9 = PB1 は ArduinoISP も他の役も使っていない。
const uint8_t RELAY_PIN = 9;

// 電源を落としてから入れ直すまでの待ち時間。
// コンデンサが抜けきらないとSA-1が初期状態に戻らない。実測で「10秒待つ」と
// 復帰した事例があるので、既定は余裕をみて8秒にしてある。
const uint16_t OFF_MS = 8000;
// 投入後、Nanoのブートローダが起動しシリアルが列挙されるまでの待ち。
const uint16_t ON_SETTLE_MS = 3000;

static void powerOn() {
  PORTB |= _BV(PB1);    // D9 = HIGH = 通電（実測で確認）
}

static void powerOff() {
  PORTB &= ~_BV(PB1);   // D9 = LOW = 電源断
}

void setup() {
  // 先にポートビットをHighにしてから出力へ切り替える。
  // 逆順だと数サイクルLOWが出て、一瞬だけリーダーの電源が切れる。
  PORTB |= _BV(PB1);
  DDRB |= _BV(PB1);

  Serial.begin(115200);
  while (!Serial) { /* Unoでは即座に返る */ }
  Serial.println(F("RELAY READY (0=on 1=off c=cycle ?=state)"));
}

void loop() {
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
      // ピンを入力に戻して**浮かせる**。
      // ISPモードや素通しモードでは、リレー制御ピンは誰も駆動しない。
      // そのとき電源がどちらに倒れるかを実測で確かめるためのコマンド。
      // 多くのリレーモジュールは入力が電流駆動で、浮けば非励磁になる。
      // 非励磁でどちらの接点が閉じるかは、NC/NOどちらに繋いだかで決まる。
      DDRB &= (uint8_t)~_BV(PB1);
      PORTB &= (uint8_t)~_BV(PB1);   // プルアップも切る
      Serial.println(F("PIN FLOATING"));
      break;
    case '?':
      Serial.println((PORTB & _BV(PB1)) ? F("STATE ON") : F("STATE OFF"));
      break;
    default:
      break;   // 改行などは無視
  }
}
