/*
 *
 * bscan:
 * 1 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0
 * 0 * * * * * * * * 0 1 1 1 0 0 1
 * 1 0 1 0 0 0 1 1 1 1 1 0 0 0 0 0
 * 0 1 1 1 1 1 1 1 1 1 1 0 1 0 1 0
 * 1 * * * * * * * * * * * * * * *
 * * * * * * 0 1 1 1 1 1 1 1 1 0 1
 * x 1 * * x * * * * * * * * * * *
 * * 1
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MS_DELAY 3000
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

#define TR1 (PINB & _BV(DDB3)) /* p11 */
#define TR2 (PINB & _BV(DDB5)) /* p13 */
#define TR3 (PINC & _BV(DDC1)) /* pa1 */
#define TR4 (PINC & _BV(DDC3)) /* pa3 */
#define S1_HI (PORTB |= _BV(DDB2)) /* p10 */
#define S2_HI (PORTB |= _BV(DDB4)) /* p12 */
#define S3_HI (PORTC |= _BV(DDC0)) /* pa0 */
#define S4_HI (PORTC |= _BV(DDC2)) /* pa2 */

#define POWEROFF_THRESHOLD 50 /* ms */

#define TMS_PIN DDD6
#define TDI_PIN DDD3
#define TDO_PIN DDD2
#define TCK_PIN DDD5
#define NTRST_PIN DDD4

#define TDO (PIND & _BV(TDO_PIN))
#define TCK_HI (PORTD |= _BV(TCK_PIN))
#define TCK_LO (PORTD &= ~_BV(TCK_PIN))
#define TMS_HI (PORTD |= _BV(TMS_PIN))
#define TMS_LO (PORTD &= ~_BV(TMS_PIN))
#define TDI_HI (PORTD |= _BV(TDI_PIN))
#define TDI_LO (PORTD &= ~_BV(TDI_PIN))
#define NTRST_HI (PORTD |= _BV(NTRST_PIN))
#define NTRST_LO (PORTD &= ~_BV(NTRST_PIN))

/* control waveform on D7-4052p10, B0-4052p9 */
#define C1 ((PIND & _BV(DDD7)) >> 6)
#define C2 ((PINB & _BV(DDB0)) << 2)

#define BTN(S_PORT, S_DDR, S_PIN, TR_PORT, TR_PIN) ({ \
	cnt = 0; \
	if (TR_PORT & _BV(TR_PIN)) { S_PORT |= _BV(S_PIN); prev = 1; } \
	else { S_PORT &= ~_BV(S_PIN); prev = 0; } \
	S_DDR |= _BV(S_PIN); \
	while(cnt < 20) { \
		if (TR_PORT & _BV(TR_PIN)) { \
			if (prev == 1) continue; \
			S_PORT |= _BV(S_PIN); \
			prev = 1; \
			cnt++; \
		} else { \
			if (prev == 0) continue; \
			S_PORT &= ~_BV(S_PIN); \
			prev = 0; \
			cnt++; \
		} \
	} \
	S_DDR &= ~_BV(S_PIN); \
})


volatile uint8_t rx, flag = 0;
volatile uint8_t p0;
volatile uint8_t _p0;
volatile uint8_t edge;
volatile uint8_t lcd0[4];
volatile uint8_t lcd0_flag;

volatile uint8_t *bs_buf;
uint8_t bs_bufs[4][114];
uint8_t display[2][44]; // 42 actually, but...

ISR(USART_RX_vect)
{
	rx = UDR0;
	flag = 1;
}

void uart_init (void)
{
	UBRR0H = (BAUDRATE>>8);
	UBRR0L = BAUDRATE;
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
}

void uart_tx (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

uint8_t hex_nibble(uint8_t val) {
	return (val<10)?(val+0x30):(val+0x37);
}

void uart_hex(uint8_t data) {
	uart_tx(hex_nibble((data >> 4) & 0x0F));
	uart_tx(hex_nibble((data >> 0) & 0x0F));
}

void uart_str(char * s)
{
	while(*s)
	uart_tx(*s++);
}

void _tck(){
    TCK_HI;
    _delay_us(5);
    TCK_LO;
    _delay_us(3);
}

void _tms(uint8_t val){
    if(val)
        TMS_HI;
    else
        TMS_LO;
    _delay_us(2);
    _tck();
}

uint8_t _shift(uint8_t val) {
    if(val)
        TDI_HI;
    else
        TDI_LO;
    _delay_us(2);
    _tck();
    return TDO?1:0;
}

void bscan() {
	uint8_t cnt;

	NTRST_HI;
	_delay_us(5);
	//reset tap
	TMS_HI;
	for (cnt = 0; cnt < 10; cnt++){
		_tck();
	}
	// shift-ir
	_tms(0); _tms(1); _tms(1); _tms(0); _tms(0);
	// SAMPLE/PRELOAD
	_shift(0);_shift(1);_shift(0);_shift(0);_shift(0);_shift(0);_shift(0);TDI_LO;
	// to shift-dr 11100
	_tms(1);_tms(1);_tms(1);_tms(0);_tms(0);

	*bs_buf = TDO?1:0;
	for (cnt = 1; cnt < 114; cnt++){
		*(bs_buf + cnt) = _shift(0);
	}
	NTRST_LO;
}

const uint8_t transcoder[42] = { // starts from lcd pin 2
/* 96, 100 - unknowns! */
	 68, 67, 66, 65, 74, 73, 72, 71, 70, 69, 20, 19, 75, 76, 77, 78,
	 79, 80, 81, 21, 82, 83, 84, 22, 100/**/,101, 23,102,103,104,105,106,
	107,108, 24,109,110,111,112, 17, 18, 96/**/
};

void fill_display(uint8_t *dst_buf, uint8_t *src_buf) {
	uint8_t cnt;

	for(cnt = 0; cnt< 42; cnt++)
		*(dst_buf+cnt) = *(src_buf+transcoder[cnt]);
}

int main (void) {
    char _ch;
    char cnt, prev;
    uint8_t tmp;

	uint8_t state;
	uint8_t hexd[11];

    uart_init();
    sei();

    /*Set to one the fifth bit of DDRB to one
    **Set digital pin 13 to output mode */
    //DDRB |= _BV(DDB5);

    // power down pin 9
    DDRB |= _BV(DDB1);

    /* Set pin 10 output low */
//    PORTB &= ~_BV(DDB2);
    TCK_LO;
    TMS_HI;
    TDI_HI;
    DDRD |= _BV(TCK_PIN);
    DDRD |= _BV(TMS_PIN);
    DDRD |= _BV(TDI_PIN);

    NTRST_LO;
    DDRD |= _BV(NTRST_PIN);

	while(1) {
	/* flag is set in uart rx interrupt
	 * and signifies command from user.
	 * so this is an idle loop */
	_p0 = 0xFF;
	edge = 0;
	state = 0;
	while(!flag) {
		/* check if device is powered on:
		 *   read lcd control lines every 200us
		 *   p0 <= 00000xx0: 0, 2, 4, 6
		 *   edge must be within 4ms
		 */
		p0 = C1 | C2;
		if (p0 != _p0) {
			edge = 0;
			_p0 = p0;
		} else {
			if (edge < 200)
				edge++;
			else { // reset state if powered down
				state = 0;
			}
		}

		switch(p0) {
			case 0x00:
				state |= 0x01;
			break;
			case 0x02:
				state |= 0x02;
				if(edge == 1 && state == 0x0F) {
					bs_buf = bs_bufs[1];
					bscan();
					fill_display(display[0], bs_bufs[1]);
				}
			break;
			case 0x04:
				state |= 0x04;
			break;
			case 0x06:
				state |= 0x08;
				if(edge == 1 && state == 0x0F) {
					bs_buf = bs_bufs[3];
					bscan();
					fill_display(display[1], bs_bufs[3]);
				}
			break;
		}

		_delay_us(200);
	}
	flag = 0;

	if (rx == 'q') {
		uart_str("P\r\n");
		PORTB |= _BV(DDB1);
		_delay_ms(50);
		PORTB &= ~_BV(DDB1);
	}
	else if (rx == '1') { // TR4, S2
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB4, PINC, DDC3); // S(PORT, DDR, P#), TR(PORT, P#)
		uart_str("B:1\r\n");
	}
	else if (rx == '2') { // TR3, S4
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTC, DDRC, DDC2, PINC, DDC1);
		uart_str("B:2\r\n");
	}
	else if (rx == '3') { // TR3, S1
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB2, PINC, DDC1);
		uart_str("B:3\r\n");
	}
	else if (rx == '4') { // TR4, S4
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTC, DDRC, DDC2, PINC, DDC3);
		uart_str("B:4\r\n");
	}
	else if (rx == '5') { // TR3, S2
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB4, PINC, DDC1);
		uart_str("B:5\r\n");
	}
	else if (rx == '6') { // TR1, S1
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB2, PINB, DDB3);
		uart_str("B:6\r\n");
	}
	else if (rx == '7') { // TR3, S3
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTC, DDRC, DDC0, PINC, DDC1);
		uart_str("B:7\r\n");
	}
	else if (rx == '8') { // TR2, S2
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB4, PINB, DDB5);
		uart_str("B:8\r\n");
	}
	else if (rx == '9') { // TR2, S3
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTC, DDRC, DDC0, PINB, DDB5);
		uart_str("B:9\r\n");
	}
	else if (rx == '0') { // TR2, S1
		if(state != 0x0F) { uart_str("B:PWR_OFF\r\n"); continue; }
		BTN(PORTB, DDRB, DDB2, PINB, DDB5);
		uart_str("B:0\r\n");
	}
	else if (rx == 'l') {
		if(state != 0x0F) {
			uart_str("L:ERR_PWR\r\n");
			continue;
		}
		uart_tx('L');
		for(tmp = 0; tmp < 2; tmp++){
			for(cnt = 0; cnt < 11; cnt++){
				uart_tx(hex_nibble(
					display[tmp][cnt*4+0] << 3 |
					display[tmp][cnt*4+1] << 2 |
					display[tmp][cnt*4+2] << 1 |
					display[tmp][cnt*4+3] << 0));
			}
		}
		uart_str("\r\n");
	}
	else {
		uart_str("CMD_UNK\r\n");
	}
	}
}

