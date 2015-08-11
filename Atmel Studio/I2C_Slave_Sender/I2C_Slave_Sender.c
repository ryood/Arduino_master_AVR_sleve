/*
 * I2C_Slave_Sender.c
 *
 * Created: 2015/08/11 7:58:13
 *  Author: gizmo
 *
 * クロック：内蔵８ＭＨｚ
 * デバイス：ATmega328P
 *
 * PC4:SDA
 * PC5:SCK
 *
 * PortD: SWx8
 * PB1:   LED
 */ 

#define 	F_CPU 8000000UL  // 8 MHz

#include 	<avr/io.h>
#include	<util/delay.h>

//状態値
//#define	SR_SLA_ACK	0x60	//SLA_W 受信チェック
#define SR_SLA_ACK  0xA8		//SLA_R 受信チェック
//#define	SR_DATA_ACK	0x80	//受信パケットチェック
#define	SR_DATA_ACK	0xC0		//送信パケットチェック
#define	SR_ENDP_ACK	0xA0		//終了orリピートチェック

//proto
void twi_init(void);
void twi_send(uint8_t sdata);

//------------------------------------------------

void twi_error(){
	PORTB = 0x02;
	while(1);
}

void twi_init(){
	//TWBR = 0xFF;	//分周	2KHz
	// 8MHz clk, bit rate 100kHz
	TWBR = 2;
	TWSR = 0x02;
	TWCR = 1<< TWEN;
	
	//slave address
	TWAR=0xfe;
}

int main(){
	uint8_t sdata;


	//sw input / pull up
	DDRD = 0x00;
	PORTD = 0xFF;
	
	//Error LED
	DDRB = 0x02;	//0b00000010

	//PORTB = 0x02;

	//通信モジュール初期化
	twi_init();
	
	for(;;) {
		// swの押し下げ状態を読み取る
		sdata = ~PIND;
		
		//データを送信する。
		twi_send(sdata);		
	}
}

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
