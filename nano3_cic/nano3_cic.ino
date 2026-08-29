// Nano-3: CIC Lock側の再生機（開発用・単独動作）
//
// ■ この firmware は何をしないか
// 判断を一切しない。認証が成立したかどうかも、ビットが合っているかどうかも見ない。
// PCから「各ラウンドで送るビット列」と「線の向き」を受け取り、そのとおりに吐いて、
// 返ってきたビットをそのまま持ち帰るだけ。
//
// そうする理由は二つある。
//   1. アルゴリズムやタイミングを直すたびに書き込み直すのは遅い。PC側で完結させたい
//   2. 「成立したか」を装置側で判定させたのが、これまでの誤検出5連発の原因だった
//
// ■ クロックはこちらが握る
// カート56番のCICクロックは本来コンソールの発振器が配るもので、Lock側とKey側の
// 両方が同じクロックで動く（ロックステップ）。そのクロックを出すのがこちらなので、
// 実時間に追従する必要がない。**1パルスずつ止めながら進められる。**
// タイミングはすべて「クロックを何発叩いたか」で数える。
//
// Key側はPIC同様 クロック/4 が命令実行速度になる。元コードの「15サイクル/ビット」は
// 命令サイクルなので、クロックパルスでは 15*4 = 60発が1ビット周期になる。
//
// ■ ビットの表現
// 元コードは「線に値を書く→数命令後に相手の線を読む→線を0に戻す」という動きをする。
// つまり 1 は周期の頭に出る短いHighパルス、0 は何も出さない。両者が同じ窓で
// 撃ち合い、同じ窓で相手を覗く。だからサンプル位置は周期の頭に寄っている。
//
// 周期の残りはアイドルで、そこは**両線ともLowでなければならない**。
// Key側は毎ビットそれを検査していて、Highが残っていると die する:
//     btfsc GPIO, 0 / goto die / btfsc GPIO, 1 / goto die
// 「起動して1回だけ遷移し、あとは永久に沈黙」はこの die の姿。
//
// ■ 配線
//   D12 = PB4  CICクロック   -> CICソケット7番（カート56番のネット）
//   A3  = PC3  Keyへのリセット -> ソケット11番（カート25番）
//   A4  = PC4  データ線A      -> ソケット2番（カート24番）  1kΩ直列
//   A5  = PC5  データ線B      -> ソケット1番（カート55番）  1kΩ直列
//
// A4/A5 のどちらが CIC の 0番/1番 かは未確定。可能性は2通りしかないので
// SWAP フラグで入れ替えて両方試す。配線をやり直す必要はない。

#include <Arduino.h>

#define CLK_BIT  _BV(PB4)   // D12
#define RST_BIT  _BV(PC3)   // A3
#define DA_BIT   _BV(PC4)   // A4
#define DB_BIT   _BV(PC5)   // A5

const uint8_t MAX_ROUNDS = 16;
// ラウンドの長さは固定ではない。Lock側は FSR = 0x20 + (0x37 & 0xf) から
// 0x30 まで回すので、1ラウンドは 16-k ビット（k は毎ラウンド変わる）。
// 全ラウンド15ビット固定にしていたのが構造的な誤りだった。
const uint8_t MAX_ROUND_BITS = 16;
const uint16_t ID_BIT_PULSES = 60;   // 15命令 x 4

// PCから渡される設定
struct Params {
  uint8_t  rounds;
  uint16_t idDelay;     // リセット解除から最初のIDビットまでのクロックパルス数
  // 1ビット周期のクロックパルス数。
  // 「15サイクル/ビット」はストリームID送信のときの値であって、主ループは違う。
  // Key側の主ループは wait(0x13)=62命令 を含み、1ビットおよそ90命令 ≒ 360パルス。
  // 255では収まらないので16bit。
  uint16_t bitPulses;
  uint8_t  drivePulses; // 自分の線を駆動し続けるパルス数（既定16）
  uint8_t  sampleAt;    // 相手の線を読むパルス位置（既定8）
  uint8_t  halfDelay;   // クロック半周期の水増し（0でおよそ2MHz）
  bool     rstActiveHigh;
  bool     swapPins;
  bool     listenOnly;   // 聴くだけモード。こちらは線を一切駆動しない
  uint8_t  decim;        // 何パルスに1回記録するか。窓を伸ばしたいとき
  // データ線の極性。相手のアイドルがHighなので、こちらもアイドルHigh・
  // ビットはLowパルス、という可能性がある。読みも合わせて反転する。
  bool     invertData;
  // 電位測定はクロックを約400us止める。相手が静的論理でないと状態が壊れる恐れが
  // あるので、握手を試すときは切れるようにしておく。
  bool     probe;
  // ID送信終了から主ループ突入までのクロックパルス数。
  // Lock側の実際の流れ(bcf/wait(1)/TRIS切替/wait(0x22)/nop×2)を数えると
  // 約130命令 = 520パルス。ここがずれると全ビットが崩れるので可変にする。
  uint16_t postId;
  // ストリームIDの出し方。Key側は btfsc GPIO,0 で「レベルを読む」だけなので、
  // 短いパルスでは掴めていない可能性がある。
  //   0 = 24パルス幅のパルス（主ループと同じ出し方）
  //   1 = 1ビット周期のあいだレベルを保持する
  //   2 = IDを送らない（Keyは0000として進むはず）
  uint8_t  idMode;
  // ラウンド境界の待ちのあいだ、両線ともLowに駆動するか。
  // 片方を入力のまま浮かせると線がHighになり、ラウンド先頭の
  // 「アイドル中は両線Low」検査に引っかかって die する疑いがある。
  // Keyもmangle中は出力をLowにしているはずなので、衝突はしない。
  bool     gapBothLow;
  // 向きの切り替えを待ちの「前」に置くか。
  // LockもKeyもmangleのあとにTRISを切り替えるが、切り替え時刻がずれると
  // どちらも駆動していない線が生じて浮き、ラウンド先頭の検査で die する。
  bool     swapBeforeGap;
  // ラウンド0を正規の手順で通した直後から、入力線を1パルスごとに記録する。
  // 「ラウンド1でKeyが本当に喋っているのか、死んで沈黙しているのか」を
  // 理屈ではなく波形で見るためのもの。
  bool     traceRound1;
  // トレース対象のラウンド番号。ラウンド1を解いた「Keyに聞く」方法を
  // 以降のラウンドにも使うため可変にした。
  uint8_t  traceRound;
};

// 聴くだけモードの記録先。1サンプル2bit(A4,A5)を4個ずつ詰める。
const uint16_t LISTEN_BYTES = 400;      // 1600サンプル
uint8_t listenBuf[LISTEN_BYTES];

Params prm;
uint8_t txBits[MAX_ROUNDS][2];   // 送るビット（最大16bit詰め）
uint8_t nBits[MAX_ROUNDS];       // そのラウンドのビット数
uint8_t dirBits[MAX_ROUNDS];     // 各ラウンドの向き
// ラウンド境界でKeyがmangleに費やす時間（クロックパルス数）。
// mangleは桁上がりが消えるまで回るので所要時間がラウンドごとに違う。
// その回数はPC側のモデルが正確に知っているので、計算して渡してもらう。
uint16_t gapPulses[MAX_ROUNDS];
// 待ちのどこで次ラウンドの向きへ切り替えるか（パルス数）。
// Keyはmangleを終えた瞬間に切り替える。こちらがラウンド先頭で切り替えると
// 時刻がずれ、同じ線を二人で駆動するか誰も駆動しない窓ができる。
// 0なら待ちの直前、gapPulses[r]なら待ちの直後（従来と同じ）。
uint16_t switchAt[MAX_ROUNDS];
uint8_t rxBits[MAX_ROUNDS][2];   // 受け取ったビット

// 出力側/入力側のポートマスク。向きによって毎ラウンド入れ替わる。
uint8_t outMask, inMask;

static inline void tick() {
  PORTB |= CLK_BIT;
  for (uint8_t i = 0; i < prm.halfDelay; i++) __asm__ __volatile__("nop");
  PORTB &= (uint8_t)~CLK_BIT;
  for (uint8_t i = 0; i < prm.halfDelay; i++) __asm__ __volatile__("nop");
}

static inline void ticks(uint16_t n) {
  while (n--) tick();
}

// 向きを設定する。
//   dir=0 -> CIC 1番線で送信、0番線で受信
//   dir=1 -> CIC 0番線で送信、1番線で受信
// pin0/pin1 と A4/A5 の対応は未確定なので swapPins で入れ替える。
static void setDirection(uint8_t dir) {
  const uint8_t pin0 = prm.swapPins ? DB_BIT : DA_BIT;
  const uint8_t pin1 = prm.swapPins ? DA_BIT : DB_BIT;
  outMask = dir ? pin0 : pin1;
  inMask  = dir ? pin1 : pin0;
  DDRC |= outMask;
  driveIdle();                            // 出力側はアイドル電位から始める
  DDRC &= (uint8_t)~inMask;               // 入力側は開放（1kΩ経由で相手が駆動する）
}

// 1ビット周期を実行し、相手の線から読んだ値を返す。
static inline void driveIdle() {
  if (prm.invertData) PORTC |= outMask; else PORTC &= (uint8_t)~outMask;
}

static inline void driveActive() {
  if (prm.invertData) PORTC &= (uint8_t)~outMask; else PORTC |= outMask;
}

static uint8_t exchangeBit(uint8_t myBit, uint16_t period) {
  uint8_t got = 0;
  if (myBit) driveActive(); else driveIdle();
  for (uint16_t p = 0; p < period; p++) {
    if (p == prm.sampleAt) {
      const uint8_t lvl = (PINC & inMask) ? 1 : 0;
      got = prm.invertData ? (uint8_t)(lvl ^ 1) : lvl;
    }
    if (p == prm.drivePulses) driveIdle();
    tick();
  }
  driveIdle();
  return got;
}

// リセットパルスでKeyを起動する。元コードは3命令ぶんのパルスを出す。
static void triggerKey() {
  const uint8_t active = prm.rstActiveHigh ? RST_BIT : 0;
  PORTC = (uint8_t)((PORTC & ~RST_BIT) | (prm.rstActiveHigh ? 0 : RST_BIT));
  DDRC |= RST_BIT;
  ticks(64);                                    // 起動前に少し流す
  PORTC = (uint8_t)((PORTC & ~RST_BIT) | active);
  ticks(12);                                    // 3命令ぶん
  PORTC = (uint8_t)((PORTC & ~RST_BIT) | (prm.rstActiveHigh ? 0 : RST_BIT));
}

// 「誰か居るか」だけを見る。データ線は入力のまま、クロックだけ流して観測する。
// こちらが何も駆動しないので、相手が何を出していてもぶつからない。
static void runListen() {
  // 両線を開放にすると線が浮いてHighになり、Keyは「アイドル中にHighがある」と
  // 判定して die する。実機のLockと同じく、自分の担当線はLowに保ったまま聴く。
  triggerKey();
  setDirection(prm.swapPins ? 1 : 0);   // 担当線をLow固定、もう一方を観測
  ticks(prm.idDelay);

  for (uint16_t i = 0; i < LISTEN_BYTES; i++) {
    uint8_t packed = 0;
    for (uint8_t s = 0; s < 4; s++) {
      for (uint8_t d = 0; d < prm.decim; d++) tick();
      const uint8_t c = PINC;
      packed |= (uint8_t)((((c & DA_BIT) ? 1 : 0) | ((c & DB_BIT) ? 2 : 0)) << (s * 2));
    }
    listenBuf[i] = packed;
  }
}

// 抵抗の向こう側の電位。A6/A7はアナログ入力専用なので線を乱さない。
//   A6 -> ソケット2番(カート24)   A7 -> ソケット1番(カート55)
uint8_t vProbe[4];   // [0,1]=Low駆動時のA6,A7  [2,3]=High駆動時

static uint8_t rd8(uint8_t ch) {
  analogRead(ch);                    // チャネル切替直後の1回目は捨てる
  return (uint8_t)(analogRead(ch) >> 2);
}

// Keyを走らせた状態で「こちらが線を取れるか」を測る。
// クロックを止めている間Keyは進まないので、ADCに時間をかけても握手は壊れない。
static void probeDrive() {
  PORTC &= (uint8_t)~outMask;        // Low駆動
  vProbe[0] = rd8(A6);
  vProbe[1] = rd8(A7);
  PORTC |= outMask;                  // High駆動
  vProbe[2] = rd8(A6);
  vProbe[3] = rd8(A7);
  PORTC &= (uint8_t)~outMask;
}

static void runSession() {
  DDRB |= CLK_BIT;
  PORTB &= (uint8_t)~CLK_BIT;

  if (prm.listenOnly) { runListen(); return; }

  triggerKey();

  // ストリームIDは0番線で送る（元コードの TRIS 設定に合わせる）
  setDirection(1);
  ticks(prm.idDelay);
  if (prm.probe) probeDrive();   // クロックを止めるので、既定では行わない
  // ストリームIDだけは15命令/ビット。Key側の受信部が wait(0x2)=11サイクルしか
  // 挟んでいないため、主ループの周期とは別物になる。
  if (prm.idMode == 2) {
    ticks(4 * ID_BIT_PULSES);                       // 何も出さずに時間だけ進める
  } else if (prm.idMode == 1) {
    driveActive();                                  // ID=0xf なので4ビットとも1
    ticks(4 * ID_BIT_PULSES);
    driveIdle();
  } else {
    for (uint8_t i = 0; i < 4; i++) exchangeBit(1, ID_BIT_PULSES);
  }

  // ID送信後、線の向きが入れ替わって本編に入る
  ticks(prm.postId);

  if (prm.swapBeforeGap) setDirection(dirBits[0]);

  setDirection(dirBits[0]);
  for (uint8_t r = 0; r < prm.rounds; r++) {

    // ラウンド1を正規どおり送信しながら、入力線を1パルスごとに記録する。
    // 最初は記録に専念して送信を止めていたが、それではこちらが黙ったせいで
    // Keyが不一致を検出して die するだけで、本当の姿が見えなかった。
    if (prm.traceRound1 && r == prm.traceRound) {
      const uint16_t txr = (uint16_t)txBits[r][0] | ((uint16_t)txBits[r][1] << 8);
      const uint8_t nb = nBits[r];
      uint16_t idx = 0;
      uint8_t k = 0, packed = 0;
      for (uint8_t b = 0; b < nb && idx < LISTEN_BYTES; b++) {
        if ((txr >> b) & 1) driveActive(); else driveIdle();
        for (uint16_t p = 0; p < prm.bitPulses && idx < LISTEN_BYTES; p++) {
          if (p == prm.drivePulses) driveIdle();
          tick();
          // 間引き。1600サンプルで15ビット(5580パルス)を覆うには4以上が要る。
          // Keyのパルスは13パルス幅なので、4なら3サンプル残り取りこぼさない。
          if (prm.decim > 1 && (p % prm.decim)) continue;
          const uint8_t c = PINC;
          packed |= (uint8_t)((((c & DA_BIT) ? 1 : 0)
                               | ((c & DB_BIT) ? 2 : 0)) << (k * 2));
          if (++k == 4) { listenBuf[idx++] = packed; packed = 0; k = 0; }
        }
      }
      driveIdle();
      // ラウンドのビットを出し終えても、バッファが埋まるまで記録を続ける。
      // ここで止めると残りは前回の記録が残ったままになり、
      // 「待ち時間と次のラウンド」を見ているつもりで古いデータを見てしまう。
      while (idx < LISTEN_BYTES) {
        for (uint8_t d = 0; d < prm.decim; d++) tick();
        const uint8_t c = PINC;
        packed |= (uint8_t)((((c & DA_BIT) ? 1 : 0)
                             | ((c & DB_BIT) ? 2 : 0)) << (k * 2));
        if (++k == 4) { listenBuf[idx++] = packed; packed = 0; k = 0; }
      }
      return;
    }
    uint16_t tx = (uint16_t)txBits[r][0] | ((uint16_t)txBits[r][1] << 8);
    uint16_t rx = 0;
    for (uint8_t b = 0; b < nBits[r]; b++) {
      if (exchangeBit((tx >> b) & 1, prm.bitPulses)) rx |= (uint16_t)1 << b;
    }
    rxBits[r][0] = (uint8_t)rx;
    rxBits[r][1] = (uint8_t)(rx >> 8);
    // Keyがmangleしている間こちらも待つ。待ちの途中で向きを切り替える。
    {
      const uint8_t nextDir = ((uint8_t)(r + 1) < prm.rounds)
                              ? dirBits[r + 1] : dirBits[r];
      uint16_t before = switchAt[r];
      if (before > gapPulses[r]) before = gapPulses[r];
      ticks(before);
      if ((uint8_t)(r + 1) < prm.rounds) setDirection(nextDir);
      ticks((uint16_t)(gapPulses[r] - before));
    }
  }

  // 終わったら線を解放しておく。挿しっぱなしで放置しても電流が流れない。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT);
  PORTC &= (uint8_t)~(DA_BIT | DB_BIT);
}

static bool readExact(uint8_t *dst, uint8_t n) {
  uint32_t deadline = millis() + 3000;
  uint8_t got = 0;
  while (got < n) {
    if (Serial.available()) dst[got++] = (uint8_t)Serial.read();
    else if (millis() > deadline) return false;
  }
  return true;
}

void setup() {
  // バンク出力は使わない。開発中はカートのバンク線に触らせない方が安全なので入力のまま。
  DDRC &= (uint8_t)~(DA_BIT | DB_BIT | RST_BIT);
  Serial.begin(115200);
}

// 基板上のLED(D13=PB5)。クロックはD12なので競合しない。
// 判定はPC側が持っているので、点灯はPCから指示する。
//   消灯 = Keyが応答していない -> 差し直す
//   点灯 = ラウンド0が14/14通った -> 触らない
#define LED_BIT _BV(PB5)

void loop() {
  Serial.write('K');            // 準備完了

  uint8_t first;
  if (!readExact(&first, 1)) return;
  if (first == 0xFE) {          // LED制御。握手はしない
    uint8_t on;
    if (!readExact(&on, 1)) return;
    DDRB |= LED_BIT;
    if (on) PORTB |= LED_BIT; else PORTB &= (uint8_t)~LED_BIT;
    return;
  }

  uint8_t hdr[13];
  hdr[0] = first;
  if (!readExact(hdr + 1, sizeof(hdr) - 1)) return;

  prm.rounds        = hdr[0] > MAX_ROUNDS ? MAX_ROUNDS : hdr[0];
  prm.idDelay       = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
  prm.bitPulses     = (uint16_t)hdr[3] | ((uint16_t)hdr[9] << 8);
  prm.drivePulses   = hdr[4];
  prm.sampleAt      = hdr[5];
  prm.halfDelay     = hdr[6];
  prm.rstActiveHigh = (hdr[7] & 0x01) != 0;
  prm.swapPins      = (hdr[7] & 0x02) != 0;
  prm.listenOnly    = (hdr[7] & 0x04) != 0;
  prm.decim         = hdr[7] >> 3;
  if (prm.decim == 0) prm.decim = 1;
  prm.invertData    = (hdr[8] & 0x01) != 0;
  prm.probe         = (hdr[8] & 0x02) != 0;
  prm.postId        = (uint16_t)hdr[10] | ((uint16_t)hdr[11] << 8);
  prm.idMode        = (hdr[8] >> 2) & 0x03;
  prm.gapBothLow    = (hdr[8] & 0x10) != 0;
  prm.swapBeforeGap = (hdr[8] & 0x20) != 0;
  prm.traceRound1   = (hdr[8] & 0x40) != 0;
  prm.traceRound    = hdr[12];

  for (uint8_t r = 0; r < prm.rounds; r++) {
    uint8_t b[8];
    if (!readExact(b, 8)) return;
    dirBits[r]  = b[0];
    txBits[r][0] = b[1];
    txBits[r][1] = b[2];
    gapPulses[r] = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
    nBits[r] = b[5] > MAX_ROUND_BITS ? MAX_ROUND_BITS : b[5];
    switchAt[r] = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
  }

  // 握手の間は割り込みを止める。millis()のタイマー割り込みが1msごとに数usぶん
  // クロックを止めてしまう。カート内のCICはPICではなくSharpの4bit MCUで、
  // 静的論理とは限らない。クロックが途切れると状態が壊れる恐れがある。
  // Nano-1/Nano-3の本番ファームで TIMSK0=0 していたのと同じ理由。
  cli();
  runSession();
  sei();

  if (prm.listenOnly || prm.traceRound1) {
    for (uint16_t i = 0; i < LISTEN_BYTES; i++) Serial.write(listenBuf[i]);
  } else {
    for (uint8_t r = 0; r < prm.rounds; r++) Serial.write(rxBits[r], 2);
    Serial.write(vProbe, 4);
  }
  Serial.flush();
}
