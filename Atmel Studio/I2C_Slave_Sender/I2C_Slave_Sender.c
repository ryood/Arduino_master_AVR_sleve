/*
 * I2C_Slave_Sender.c
 *
 * Created: 2015/08/11 7:58:13
 *  Author: gizmo
 *
 * �N���b�N�F�����W�l�g��
 * �f�o�C�X�FATmega328P
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

//��Ԓl
//#define	SR_SLA_ACK	0x60	//SLA_W ��M�`�F�b�N
#define SR_SLA_ACK  0xA8		//SLA_R ��M�`�F�b�N
//#define	SR_DATA_ACK	0x80	//��M�p�P�b�g�`�F�b�N
#define	SR_DATA_ACK	0xC0		//���M�p�P�b�g�`�F�b�N
#define	SR_ENDP_ACK	0xA0		//�I��or���s�[�g�`�F�b�N

//proto
void twi_init(void);
void twi_send(uint8_t sdata);

//------------------------------------------------

void twi_error(){
	PORTB = 0x02;
	while(1);
}

void twi_init(){
	//TWBR = 0xFF;	//����	2KHz
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

	//�ʐM���W���[��������
	twi_init();
	
	for(;;) {
		// sw�̉���������Ԃ�ǂݎ��
		sdata = ~PIND;
		
		//�f�[�^�𑗐M����B
		twi_send(sdata);		
	}
}

//�f�[�^�p�P�b�g���P�o�C�g����B
void twi_send(uint8_t sdata){
	
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	
	//�@Wait for TWINT Flag set.
	while( !( TWCR & (1<<TWINT) ) );
	
	if( (TWSR & 0xF8 ) != SR_SLA_ACK )
		twi_error();
		
	// �f�[�^�𑗐M
	TWDR = sdata;
	
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN );

	//�@Wait for TWINT Flag set.
	while(  ! ( TWCR & ( 1<<TWINT ) ) );
	
	if((TWSR & 0xF8 ) != SR_DATA_ACK)
		twi_error();
}
