/*

I2C Slave Receiver  2013.6.30

仕様書の記述に合わせてコーディング。

クロック：内蔵８ＭＨｚ
デバイス：ATmega328P

	PC4:SDA
	PC5:SCK

	Display
	PD6: LED1
	PD7: LED2
*/

#define 	F_CPU 8000000UL  // 8 MHz

#include 	<avr/io.h>
#include	<util/delay.h>

//状態値
#define	SR_SLA_ACK	0x60	//SLA_W 受信チェック
#define	SR_DATA_ACK	0x80	//受信パケットチェック
#define	SR_ENDP_ACK	0xA0	//終了orリピートチェック

//proto
void twi_init(void);
uint8_t twi_receive(void);

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
	uint8_t rdata;


	//Display LED output
	DDRD = 0xFF;	//0b11111111
	
	//Error LED
	DDRB = 0x02;	//0b00000010

//	PORTD = 0xFF;

	//通信モジュール初期化
	twi_init();
	
	for(;;) {
		//データを受信する。
		rdata = twi_receive();
		
		//DISPLAY// Received data.
		PORTD = rdata;
	}
}


//データパケットを１バイト送る。
uint8_t twi_receive(void){
	uint8_t rdata;

	//1.To initiate the Slave Receiver mode, TWAR and TWCR must be initialized as follows:
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);

	//2.Wait for TWINT Flag set. →Own SLA+W has been received; ACK has been returned
	while( !( TWCR & (1<<TWINT) ) );

	//3.Check value of TWI Status Register. Mask prescaler bits. If status different from SR_SLA_ACK go to ERROR
	if( (TWSR & 0xF8 ) != SR_SLA_ACK )  //
		twi_error();	//if error
		
	//3. Clear TWINT bit in TWCR to start transmission of address
	TWCR = ( 1<<TWINT ) | ( 1<<TWEA) | ( 1<<TWEN );

	//4.Wait for TWINT Flag set.
	while(  ! ( TWCR & ( 1<<TWINT ) ) );
	
	//5.Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
	if((TWSR & 0xF8 ) != SR_DATA_ACK)
		twi_error();	//if error

	// DATA RECEIVED.
	rdata = TWDR;
	
	//
	TWCR = ( 1<<TWINT ) | ( 1<<TWEA) | ( 1<<TWEN );
	while( !( TWCR & (1<<TWINT) ) );//Wait for TWINT Flag set.

	if( (TWSR & 0xF8 ) != SR_ENDP_ACK )  //
		twi_error();	//if error

	return rdata;

}
