// 21.477270MHzでの発振確認用。avr-libcの_delay_msはF_CPUを見てループ回数を
// 計算するので、実際の周波数が違えば体感の点滅速度が目に見えてずれる。
#define F_CPU 21477270UL
#include <util/delay.h>
#include <avr/io.h>

int main(void) {
  DDRB |= (1 << 5);   // D13 = PB5 = L LED
  while (1) {
    PORTB |= (1 << 5);
    _delay_ms(1000);
    PORTB &= ~(1 << 5);
    _delay_ms(1000);
  }
}
