/*
 * I2C_Slave_Sender_ATMega88V_Byte_Sequence_Interrupt.c
 *
 * Created: 2015/09/01 13:49:33
 *
 * I2C Slaveからバイト列を送信するテスト(Interrupt処理)
 *
 *  Author: gizmo
 *
 * クロック：内蔵8MHz
 * デバイス：ATmega88V
 *
 * PC4:SDA
 * PC5:SCK
 *
 * PortD: SWx8
 * PortB: LEDx8
 * PortC PC0: LED
 *
 */

#define 	F_CPU 8000000UL  // 8 MHz

#include 	<avr/io.h>
#include	<avr/interrupt.h>
#include	<util/delay.h>

// TWI Slave Address
#define TWI_SLAVE_ADDRESS 0xFE

// 状態値
#define SR_SLA_ACK   0xA8		// SLA_R 受信チェック
#define	SR_DATA_ACK	 0xB8		// 送信パケットチェック バイト列の送信
#define SR_DATA_NACK 0xC0		// 送信パケットチェック 最後のバイトを送信

volatile uint8_t sdata_n;
volatile uint8_t sdata[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

void twi_error(uint8_t code)
{
	uint8_t twi_status = TWSR & 0xF8;
	
	PORTC = (1 << PC0);

	while(1) {
		PORTB = code;
		_delay_ms(1000);
		PORTB = twi_status;
		_delay_ms(1000);
	}
}

void twi_init()
{
	// 8MHz clk, bit rate 100kHz
	TWBR = 2;
	TWSR = 0x02;
	
	// slave address
	TWAR = TWI_SLAVE_ADDRESS;
	
	// TWI割り込みの有効化
	// Enable TWI port, ACK, IRQ and clear interrupt flag
	TWCR = ((1<<TWEN) | (1<<TWEA) | (1<<TWIE) | (1<<TWINT));
}

/*
//データパケットを送る。
void twi_send(uint8_t *sdata, int data_len)
{
	// SLA+Rを受信
	//
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);

	//　Wait for TWINT Flag set.
	while(!(TWCR & (1<<TWINT)))
		;

	if((TWSR & 0xF8) != SR_SLA_ACK)
		twi_error(0xAA);

	// データ(バイト列)を送信
	//
	for (int i = 0; i < data_len - 1; i++) {
		TWDR = sdata[i];
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	
		//　Wait for TWINT Flag set.
		while(!(TWCR & (1<<TWINT)))
			;
			
		if((TWSR & 0xF8) != SR_DATA_ACK)
			twi_error(i);
	}
	
	// 最後のバイトを送信
	//
	TWDR = sdata[data_len - 1];
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		
	//　Wait for TWINT Flag set.
	while(!(TWCR & (1<<TWINT)))
		;
		
	if((TWSR & 0xF8) != SR_DATA_NACK)
		twi_error(0x55);	
}
*/

ISR (TWI_vect)
{
	// 割り込みごとにLEDを点滅
	PORTC ^= (1 << PC0);
	
	// 配列インデックス・チェック
	if (sdata_n > 8)
		twi_error(0xAA);
	
	switch (TWSR & 0xF8) {
	case SR_SLA_ACK:
		sdata_n = 0;
	case SR_DATA_ACK:
		TWDR = sdata[sdata_n++];
		break;
	case SR_DATA_NACK:
		break;
	default:
		twi_error(0x55);
	}
	
	TWCR |= (1<<TWINT);	// Clear TWI interrupt flag
}

int main(){
	

	// sw input / pull up
	DDRD = 0x00;
	PORTD = 0xFF;

	// LED output
	DDRB = 0xFF;
	DDRC = 0x01;

	// LED Check
	PORTC |= (1 << PC0);
	for (int i = 0; i <= 8; i++) {
		PORTB = (0xFF >> i);
		_delay_ms(100);
	}
	PORTC &= ~(1 << PC0);
	
	//通信モジュール初期化
	twi_init();
	
	sei();

	for(;;) {
		// swの押し下げ状態をパケットの先頭と末尾に代入
		sdata[0] = ~PIND;
		sdata[7] = sdata[0];

		//データを送信する。
		//twi_send(sdata, 8);
	}
}
