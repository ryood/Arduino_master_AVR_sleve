/*
 * I2C_Slave_Sender_Interrupt.c
 *
 * Created: 2015/08/28 12:00:35
 *  Author: gizmo
 *
 * クロック：内蔵8MHz
 * デバイス：ATmega88V
 *
 * PC4:SDA
 * PC5:SCK
 *
 * PortD: SWx8
 * PB1:   LED
 */ 

#define 	F_CPU 8000000UL  // 8MHz

#include 	<avr/io.h>
#include	<avr/interrupt.h>
#include	<util/delay.h>

//TWI状態値
#define SR_SLA_ACK  0xA8		//SLA_R 受信チェック
#define	SR_DATA_ACK	0xC0		//送信パケットチェック
#define	SR_ENDP_ACK	0xA0		//終了orリピートチェック

volatile uint8_t sdata;

//------------------------------------------------

void twi_error(){
	PORTB = 0x02;
	while(1);
}

void twi_init(){
	// 8MHz clk, bit rate 100kHz
	TWBR = 2;
	TWSR = 0x02;
	
	//slave address
	TWAR=0xfe;
	
	// 割り込みを許可
	//TWCR = (1 << TWIE) | (1<< TWEN);
	TWCR = ((1<<TWEN) | (1<<TWEA) | (1<<TWIE) | (1<<TWINT));	// Enable TWI port, ACK, IRQ and clear interrupt flag
}

/*
//データパケットを１バイト送る。
void twi_send(uint8_t sdata){
	
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	
	//　Wait for TWINT Flag set.
	while( !( TWCR & (1<<TWINT) ) );
	
	if( (TWSR & 0xF8 ) != SR_SLA_ACK )
		twi_error();
		
	// データを送信
	TWDR = sdata;
	
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN );

	//　Wait for TWINT Flag set.
	while(  ! ( TWCR & ( 1<<TWINT ) ) );
	
	if((TWSR & 0xF8 ) != SR_DATA_ACK)
		twi_error();
}
*/

ISR (TWI_vect)
{
	// 割り込みごとにLEDを点滅
	PORTB ^= 0x02;
	
	switch (TWSR & 0xF8) {
	case SR_SLA_ACK:
		TWDR = sdata;
		break;
	case SR_DATA_ACK:
		break;
	default:
		twi_error();
	}
	
	TWCR |= (1<<TWINT);	// Clear TWI interrupt flag
}

//------------------------------------------------

int main(){
	//sw input / pull up
	DDRD = 0x00;
	PORTD = 0xFF;
	
	//Error LED
	DDRB = 0x02;	//0b00000010

	//PORTB = 0x02;

	//通信モジュール初期化
	twi_init();
	
	sei();
	
	for(;;) {
		// swの押し下げ状態を読み取る
		sdata = ~PIND;
		
		//データを送信する。
		//twi_send(sdata);
		
		
		_delay_ms(100);		
	}
}

