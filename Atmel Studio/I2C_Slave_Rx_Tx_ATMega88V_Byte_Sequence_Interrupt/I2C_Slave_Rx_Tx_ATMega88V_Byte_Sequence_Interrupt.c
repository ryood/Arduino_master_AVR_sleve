/*
 * I2C_Slave_Rx_Tx_ATMega88V_Byte_Sequence_Interrupt.c
 *
 * Created: 2015/09/24 5:57:31
 *  Author: gizmo
 *
 * クロック：内蔵8MHz
 * デバイス：ATmega88V
 * Fuse Bit: L:E2h H:DFh E:01h
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

 // TWI ステータスコード
 // Rx
 #define	TWI_SLA_W_ACK		0x60
 #define	TWI_RX_DATA_ACK		0x80
 #define	TWI_RX_STOP			0xA0
 // Tx
 #define	TWI_SLA_R_ACK		0xA8
 #define	TWI_TX_DATA_ACK		0xB8
 #define	TWI_TX_DATA_NACK	0xC0

 volatile uint8_t TXdata_n;
 volatile uint8_t TXdata[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

 volatile uint8_t RXdata;

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

ISR (TWI_vect)
{
	// 割り込みごとにLEDを点滅
	PORTC ^= (1 << PC0);
 
	// 配列インデックス・チェック
	if (TXdata_n > 8)
		twi_error(0xAA);

	switch (TWSR & 0xF8) {
	// Slave TX
	case TWI_SLA_R_ACK:
		TXdata_n = 0;
	case TWI_TX_DATA_ACK:
		TWDR = TXdata[TXdata_n++];
		break;
	case TWI_TX_DATA_NACK:
		break;
	// Slave RX
	case TWI_SLA_W_ACK:
		break;
	case TWI_RX_DATA_ACK:
		RXdata = TWDR;
		break;
	case TWI_RX_STOP:
		break;
	default:
		twi_error(0x55);
	}

	TWCR |= (1<<TWINT);	// Clear TWI interrupt flag
}

int main()
{
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
		TXdata[0] = ~PIND;
		TXdata[7] = TXdata[0];

		// 受信データとボタンの押し下げ状態をLEDに表示
		PORTB = (1 << RXdata) | TXdata[0];
	}
}
