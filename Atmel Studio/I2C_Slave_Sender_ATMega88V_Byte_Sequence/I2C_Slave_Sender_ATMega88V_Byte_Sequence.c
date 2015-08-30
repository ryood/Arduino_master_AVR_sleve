/*
 * I2C_Slave_Sender_ATMega88V_Byte_Sequence.c
 *
 * I2C Slave����o�C�g��𑗐M����e�X�g
 *
 * Created: 2015/08/30 18:11:47
 *  Author: gizmo
 *
 * �N���b�N�F����8MHz
 * �f�o�C�X�FATmega88V
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
#include	<util/delay.h>

//��Ԓl
#define SR_SLA_ACK   0xA8		// SLA_R ��M�`�F�b�N
#define	SR_DATA_ACK	 0xB8		// ���M�p�P�b�g�`�F�b�N �o�C�g��̑��M
#define SR_DATA_NACK 0xC0		// ���M�p�P�b�g�`�F�b�N �Ō�̃o�C�g�𑗐M

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
	TWAR=0xfe;
	
	// TWI�̗L����
	//TWCR = (1<<TWEN);
}

//�f�[�^�p�P�b�g�𑗂�B
void twi_send(uint8_t *sdata, int data_len)
{
	// SLA+R����M
	//
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);

	//�@Wait for TWINT Flag set.
	while(!(TWCR & (1<<TWINT)))
		;

	if((TWSR & 0xF8) != SR_SLA_ACK)
		twi_error(0xAA);

	// �f�[�^(�o�C�g��)�𑗐M
	//
	for (int i = 0; i < data_len - 1; i++) {
		TWDR = sdata[i];
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	
		//�@Wait for TWINT Flag set.
		while(!(TWCR & (1<<TWINT)))
			;
			
		if((TWSR & 0xF8) != SR_DATA_ACK)
			twi_error(i);
	}
	
	// �Ō�̃o�C�g�𑗐M
	//
	TWDR = sdata[data_len - 1];
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		
	//�@Wait for TWINT Flag set.
	while(!(TWCR & (1<<TWINT)))
		;
		
	if((TWSR & 0xF8) != SR_DATA_NACK)
		twi_error(0x55);	
}

int main(){
	
	uint8_t sdata[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

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
	
	//�ʐM���W���[��������
	twi_init();

	for(;;) {
		// sw�̉���������Ԃ��p�P�b�g�̐擪�Ɩ����ɑ��
		sdata[0] = ~PIND;
		sdata[7] = sdata[0];

		//�f�[�^�𑗐M����B
		twi_send(sdata, 8);
	}
}
