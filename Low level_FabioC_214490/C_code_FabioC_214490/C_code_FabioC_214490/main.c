/*
 * C_code_FabioC_214490.c
 *
 * Created: 26/06/2020 14:55:05
 * Author : fafic
 */ 
#define F_CPU 16000000

#define ADC0 0x00
#define ADC1 0x01
#define ADC2 0x02
#define ADC3 0x03
#define ADC4 0x04

#define DOT 0x01  //used for 7 seg led, the value coming from SPI interface 74HC595 device
#define M0 0x7E   // first gear
#define M1 0x12
#define M2 0xBC
#define M3 0xB6
#define M4 0xD2
#define M5 0xE6
#define M6 0xEE
#define M7 0x32   //last gear

#define SS 0x02 //PB2  //pins for SPI interface
#define SCK 0x05 //PB5
#define MOSI 0x03 //PB3
#define MISO 0x04 //PB4

#define SLAVEBASEADDRESSi2c 0x20  //first available slave address on PCF8574					  
#define R 0x01					  //I2C read mode
#define W 0x00					  //I2C write mode

#define  URSEL 0x07	//last bit of UCSR0C register

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

volatile const uint8_t marce[] = {M0, M1, M2, M3, M4, M5, M6, M7}; //array used to handle the gear changes
	
volatile uint8_t ADCval, sel, ADCx, I2Cval, m; //ADCval is the analog value sent out, 
											   //sel is a selection bit for usart transmission
											   //ADCx is the analog source of 5 potentiometer actually used
											   //I2Cval is the value read from PCF8574
											   //m is the actual gear, the index of the marce[] array

volatile unsigned char charToSend, valToSend;  //firstly is sent to PC a character that identifies the button or potentiometer used
											   //then the value is sent out
											   /* x = leftX, 
											      y = leftY, 
												  s = steering wheel, 
												  a = accelerate, 
												  f = break/reverse, 
												  b = button */

//ports for fastPWM and for external interrupt
void initPort(void){
	DDRD |= (1<<PORTD6)|(1<<PORTD5); //Port for FastPWM as output port
	DDRD &= ~(1<<PORTD2); //Port for external interrupt coming from PC8574 int pin as input port
	PORTD |= (1<<PORTD2); //activates the pull-up resistor
}  

void initPortSPI(void){
	DDRB=(1<<SS)|(1<<MOSI)|(1<<SCK); //SS, MOSI and SCK pin as output pin
	PORTB=(0<<SS)|(0<<MOSI); //SS to 0, and MOSI to 0
}

//ports for analog mux 4051
void initAMux(void){ //output selector of AMUX
	DDRC |=(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3); //s0, s1, s2 of 4051 mux as output pins
	DDRC &= ~(1<<PORTC0); //z pin of PC0 as input, where we put the analog value to convert
	DDRC |= (1<<PORTC0);//pull-up activated
	PORTC &= (~(1<<PORTC1)) & (~(1<<PORTC2)) & (~(1<<PORTC3)); //write 000 to s0,s1,s3
}

//initializing ADC converter
void initADC(void){
	ADMUX = (1<<REFS0); //using AVCC with external capacitor
	ADCSRA = (1<<ADPS2)|(1<<ADIE); // prescaler 16 for greater precision, enable interrupt
}

//init timer1 with OVF interrupt for counting 0,032 sec
void initTimer1(){
	TCNT1 = 0; //initialize to 0 the timer value
	TIMSK1 = (1<<TOIE1); //enable timer overflow interrupt
	TCCR1B = (1<<CS11); //set a prescaler of 8
}

void disableTimer1(void){
	TCCR1B = 0x00;
}

//init timer 0 for fastPWM
void initTimer0(){
	TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00)|(1<<WGM01);//Fast PWM mode //clear OC0A-B on compare match, set OC0A-B at BOTTOM
	TIMSK0 = 0x00;
	TCCR0B = (1<<CS00) | (1<<CS02);// prescaler of 1024
}

void disableTimer0(void){
	TCCR0B = 0x00;
}

//init INT0 external interrupt
void  initExternalInterrupt0(void){
	EIMSK |= (1<<INT0);  //enable INT0
	EICRA |= (1<<ISC01); //falling edge
}

void disableINT0(void){
	EIMSK &= ~(1<<INT0);
}

//init serial with 38400 of baud
void initUSART(void){
	UBRR0H = 0x19>>8; //baud 38400
	UBRR0L = 0x19;
	UCSR0C |= (1<<URSEL)|(1<<UCSZ00)|(1<<UCSZ01); //no parity, 8 bit char, 1 stop bit
}

//enable serial and TX enabling and UDR0 interrupt
void enableUSART(void){
	UCSR0B |= (1<<TXEN0)|(1<<UDRIE0); //enable transmission, enable UDRE interrupt
}

//disable USART serial
void disableUSART(void){
	UCSR0B &= ((~(1<<TXEN0)) & (~(1<<UDRIE0)));
}

//transmit a character to identify which components has to send out value
void usart_TransmitChar(unsigned char c){
	charToSend = c;
	enableUSART(); //enable usart here
}

//transmit a value of the components
void usart_TransmitVal(unsigned char v){
	valToSend = v;
}

//init ATmega328p SPI as master
void init_MasterSPI(void){
	SPCR |= (1<<MSTR)|(1<<SPR1); //in master mode, prescaler of 64 at 250khz
}

void enableSPI(void){
	SPCR |= (1<<SPIE)|(1<<SPE); //enable interrupt, enable spi
}

void disableSPI(void){
	SPCR &= (~(1<<SPIE)) & (~(1<<SPE)); //disable interrupt, disable spi
}

//send value to 74HC595 device
void send_SPI(unsigned char data){
	enableSPI();
	PORTB &= ~(1<<SS); //put to 0 SS pin
	SPDR = data;
}

//init I2C serial interface
void initI2C(void){
	//frequency of i2c SCL of 333kHz
	TWSR = (0<<TWPS0)|(0<<TWPS0);//puts 0 to TWSR
	TWBR = (1<<TWBR4); //16
}

//start bit of I2C
void startI2C(void){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //|(1<<TWIE); 
	while ((TWCR & (1<<TWINT))==0);
}

//addressing the slave device PCF8574
void addressI2C(uint8_t slaveAddress, uint8_t RW){
	TWDR = (slaveAddress<<1) | RW; //put 0x20 slave address and read bit
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT))==0);
}

//read the value coming from the slave
unsigned char readI2C(void){
	TWCR = (1<<TWINT)|(1<<TWEN); //without ACK 
	while ((TWCR & (1<<TWINT))==0);
	return TWDR;
}

void disableI2C (void){
	TWCR = 0x00;
}


//stop bit of I2C communication
void stopI2C (void) {
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);//|(1<<TWIE);
	disableI2C();
}


//go to sleep CPU and Flash
void goToSleep()
{
	set_sleep_mode(SLEEP_MODE_IDLE); //sleep mode for CPU and Flash
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
}


void startADC(){
	ADMUX = (ADMUX & 0xF0) | (0x00 & 0x0F); //Z pin of 4051 connected to A0 PC0 of ATmega328p
	ADCSRA |= (1 << ADSC); //start conversion
}

//enable ADC peripheral
void enableADC(void){
	ADCSRA |= (1 << ADEN);
}

//disable ADC peripheral
void disableADC(void){
	ADCSRA &=  ~(1 << ADEN); 
}

//setup ALL
void enableALL(void){
	ADCx = ADC0;
	sel = 0x00;
	m = 0x00;
	
	initPortSPI();
	init_MasterSPI();
	
	initPort();
	initAMux();
	initADC();
	
	initTimer1(); //start timer OVF interrupt for counting 0.032 sec
	initTimer0();
	
	initUSART();  //start Serial 38400
	initI2C();
	initExternalInterrupt0();
	send_SPI(marce[m]);
}
//used to fastPWM because I put the pedals potentiometer fixed to the greatest value
 uint8_t rangeFastPWM_FRENO(uint8_t val){
	if(val <= 150)
		return 9*28;
	if ((val <= 155) & (val > 150))
		return 8*28;
	if ((val <= 160) & (val > 155))
		return 7*28;
	if ((val <= 165) & (val > 160))
		return 6*28;
	if ((val <= 170) & (val > 165))
		return 5*28;
	if ((val <= 175) & (val > 170))
		return 4*28;
	if ((val <= 180) & (val > 175))
		return 3*28;
	if ((val <= 185) & (val > 180))
		return 2*28;
	if ((val <= 195) & (val > 185))
		return 1*28;
	if (val > 195)
		return 0*28;
	return 0;
}
 uint8_t rangeFastPWM_ACCELLERATORE(uint8_t val){
	if(val <= 130)
		return 9*28;
	if ((val <= 135) & (val > 130))
		return 8*28;
	if ((val <= 140) & (val > 135))
		return 7*28;
	if ((val <= 145) & (val > 140))
		return 6*28;
	if ((val <= 150) & (val > 145))
		return 5*28;
	if ((val <= 155) & (val > 150))
		return 4*28;
	if ((val <= 160) & (val > 155))
		return 3*28;
	if ((val <= 165) & (val > 160))
		return 2*28;
	if ((val <= 170) & (val > 165))
		return 1*28;
	if (val > 170)
		return 0*28;
	return 0;
}

//MAIN
int main(void)
{
	enableALL();
	sei();
	
	while (1)
	{
		goToSleep();
	}
}

//i2c starts and reads from PFC8574 when the external int occurs 
ISR(INT0_vect){
	startI2C();	
	addressI2C(SLAVEBASEADDRESSi2c, R);
	unsigned char I2Cval = readI2C();
	stopI2C();
	
	if((UCSR0A | ~(1<<UDRE0))){
		switch (I2Cval){
			case 0xFE: //Y
				usart_TransmitChar('b'); //send b character first to recognize that a button is pushed
				usart_TransmitVal(1+48); //send value
				break;
			case 0xFD://B
				usart_TransmitChar('b');
				usart_TransmitVal(2+48);
				break;
			case 0xFB://X
				usart_TransmitChar('b');
				usart_TransmitVal(3+48);
				break;
			case 0xF7://A
				usart_TransmitChar('b');
				usart_TransmitVal(4+48);
				break;
			case 0xEF://start
				usart_TransmitChar('b');
				usart_TransmitVal(5+48);
				break;
			case 0xDF://select
				usart_TransmitChar('b');
				usart_TransmitVal(6+48);
				break;
			case 0xBF://L1 gear dowm
				usart_TransmitChar('b');
				usart_TransmitVal(7+48);
				if (m!=0) m--;
				send_SPI(marce[m]);
				break;
			case 0x7F://R1 gear UP
				usart_TransmitChar('b');
				usart_TransmitVal(8+48);
				if (m!=7) m++;
				send_SPI(marce[m]);
				break;
			default://release buttons
				usart_TransmitChar('b');
				usart_TransmitVal(0+48);
				break;
		}
	}
	
	
	
}

//each 0,032 sec the ADC peripheral is enabled and reads value from potentiometer to send out
ISR(TIMER1_OVF_vect){ 
	enableADC();
	startADC();
	TCNT1 = 0x00;
}

//used to check when UDR0 register is empty and it is ready to receive a data to send out throught serial
ISR(USART_UDRE_vect){
	switch (sel)
	{
		case 0: //case 0 send the character
			UDR0 = charToSend;
			sel=1;
			break;
		case 1: //case 1 send the related value and disable USART, an entire packer char+value is sent
			UDR0 = valToSend;
			disableUSART();
			sel=0;
		break;
	}
}

//used when a convention is terminated
ISR (ADC_vect){
	
	if((UCSR0A | ~(1<<UDRE0))){
		ADCval=ADC>>2; //read the 10 bit value from ADC and shift it by 2 to right ADLAR = 0
		switch (ADCx)
		{
		case ADC0: //reading from X axis of Joystick shield
			usart_TransmitChar('x'); //send c character
			ADCx=ADC1;
			PORTC = (PORTC & (~(1<<PORTC3) & ~(1<<PORTC2))) | (1<<PORTC1); //sel 0x01 ADC1
			break;
		case ADC1: //reading from Y axis of Joystick shield
			usart_TransmitChar('y'); 
			ADCx=ADC2;
			PORTC = (PORTC & (~(1<<PORTC3) & ~(1<<PORTC1))) |(1<<PORTC2); //sel 0x02 ADC2
			break;
		case ADC2: //reading from f potentiometer for break/reverse
			usart_TransmitChar('f');
			ADCx=ADC3;
			PORTC = (PORTC & ~(1<<PORTC3))|((1<<PORTC2)|(1<<PORTC1)); //sel 0x03 ADC3
			OCR0B=rangeFastPWM_FRENO(ADCval); //set OCR0B for fast PWM
			break;
		case ADC3:
			usart_TransmitChar('a');//reading from a potentiometer for accelerate
			ADCx=ADC4;
			PORTC = (PORTC & (~(1<<PORTC1) & ~(1<<PORTC2))) | (1<<PORTC3); //sel 0x04 ADC4
			OCR0A=rangeFastPWM_ACCELLERATORE(ADCval);//set OCR0A for fast PWM
			break;
		case ADC4:
			usart_TransmitChar('s');//reading from s potentiometer for steering wheel	
			ADCx=ADC0;
			PORTC = (PORTC & (~(1<<PORTC2) & ~(1<<PORTC1) & ~(1<<PORTC3))); //sel 0x04 ADC0
			break;
			}
		usart_TransmitVal(ADCval); //send out the value after the character
	}
	disableADC();
}

//used to check when a SPI packet is sent out
ISR (SPI_STC_vect){
	PORTB |= (1<<SS); //put to 1 SS pin
	disableSPI(); //disable SPI
}


