/*
 * CarrierFW.c
 *
 * Created: 11.02.2016 14:10:26
 * Author : Bjorn
 */
// Firmware for DNS carrier. 
// Initially written for ADXRS290 gyros. Can be adapted to different sensors.

// Dette er bare noen dumme kommentarer for å teste git. 

//ATmega168A
// Note: For ATmega328, the BOD and BOOT fuses have exchanged places. Take care of this if bootloader is implemented.
//********************************

// FUSE BITS

// Fuse extended = 0xF9
//  7      1  -
//  6      1  -
//  5      1  -
//  4      1  -
//  3      1  -
//  2      1  BOD disabled
//  1      1  BOD disabled
//  0      1  BOD disabled

// Fuse high = 0xDF
//  7      1  PC6 is RESET
//  6      1  debugWIRE not enabled
//  5      0  SPI programming enabled
//  4      1  Watchdog timer not always on
//  3      1  EEPROM not preserved during erase
//  2      1  Minimum boot section
//  1      1  -''-
//  0      1  Reset vector at 0

// Fuse low = 0xE2
//  7      1  CKDIV8 not enabled
//  6      1  CKOUT disabled
//  5      1  Max start-up time
//  4      0  -''-
//  3      0  8 MHz internal RC
//  2      0  -''-
//  1      1  -''-
//  0      0  -''-

// =============================================================================
// PORTS

//	PB0		CS1
//	PB1		CS7
//	PB2		CS3
//	PB3		MOSI
//	PB4		MISO
//	PB5		SCLK
//	PB6		CS8
//	PB7		CS4

//	PC0		TWI addr bit 0
//	PC1		TWI addr bit 1
//	PC2		TWI addr bit 2
//	PC3		TWI addr bit 3
//	PC4		SDA
//	PC5		SCL
//	PC6		Reset/
//	PC7		TWI addr bit 1

//	PD0		RXD
//	PD1		TXD
//	PD2		AUX
//	PD3		nc
//	PD4		nc
//	PD5		CS6
//	PD6		CS5
//	PD7		CS2

#include <avr/io.h>
#include <stdlib.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Symbol definitions
//#define F_CPU	8000000UL					// CPU clock 8 MHz Moved to makefile
#define TWIaddress				0x20
#define BUFSIZE					650
#define NUMBER_OF_GYROS			8
#define SPI_TX_BUFFER_SIZE		20
#define SPI_RX_BUFFER_SIZE		20
#define TWI_RX_BUFFER_SIZE		20	// TODO: adjust buffer sizes.
#define TWI_TX_BUFFER_SIZE		20

// Function prototypes
void spiRW(char *pointer, uint8_t length, uint8_t device);
void assertSS(uint8_t device);
void releaseSS(uint8_t device);
void ConsoleTX(char *pointer);

// Gyro SPI commands
const char AD_Activate[]	= {0x10,0x02};
const char AD_Filter_20Hz[]	= {0x11,0x07};
const char AD_ReadData[]	= {0x88,0x00,0x00,0x00,0x00};
	
// Test array
const char testdata[] = "TWItest\n" ;

//----------------------------------------------------------------------------------------------
// Variable definitions

// The measurement results are processed and stored for readout either in buffer[] or in 
// the accumulators.
volatile uint8_t	bufType;					// Determines which data to store in buffer.
volatile uint8_t	bufFlag;					// Status flag for buffer
volatile int16_t	buffer[BUFSIZE];			// Ring buffer for sensor data
volatile uint16_t	bufRDpos;					// Buffer read counter
volatile uint16_t	bufWRpos;					// Buffer write counter
//volatile int32_t	xsum;						// Accumulator of x samples.
//volatile int32_t	ysum;						// Accumulator of y samples.
volatile int32_t	xaccumulator[2];			// Accumulator of x sample set means, from first and second half of indexing interval.
volatile int32_t	yaccumulator[2];			// Accumulator of y sample set means, from first and second half of indexing interval.
volatile uint16_t	accCounter;					// Counts number of means accumulated for one axis.
volatile uint8_t	accHalf;					// Indicates if accumulation is for first or second half of indexing interval.
volatile uint16_t	Nsamples;					// Number of means to be accumulated for one axis for half the indexing interval..
volatile uint16_t	chksum;						// XOR of words stored to buffer.

volatile uint8_t	TWI_RxBuf[TWI_RX_BUFFER_SIZE];
volatile uint8_t	TWI_RxPos;
volatile uint8_t	TWI_TxBuf[TWI_TX_BUFFER_SIZE];
volatile uint8_t	*TWI_TxPtr;
volatile uint16_t	TWI_TxPos;
volatile uint16_t	TWI_TxLength;
volatile uint8_t	TWI_Error;					// Stores error codes from the TWI interrupt handler.
volatile uint8_t	TWI_rxflag;					// Set when a command has been received.
volatile uint8_t	TWI_txflag;					// Set when a data buffer has been transmitted..

volatile char		*Console_TxPtr;
volatile uint8_t	console_txflg;

volatile char		SPI_TxBuf[SPI_TX_BUFFER_SIZE];
volatile char		*SPI_TxPtr;
volatile char		SPI_RxBuf[SPI_RX_BUFFER_SIZE];
volatile char		*SPI_RxPtr;
volatile uint8_t	spi_txflg;
volatile uint8_t	spi_rxflg;
volatile uint8_t	spi_length;

char plotstring[160];

//	INTERRUPT ROUTINES
//==============================================================================
// Timer 2
// The timer2 interrupt occurs at 40 Hz and is used for
// sample timing.
//------------------------------------------------------------------------------
ISR(TIMER2_COMPA_vect ){
	uint8_t i;
	uint32_t xsum,ysum;
	
	switch(bufType){
		case 'M':
		// Calculate and store only the mean of the sample set.
			xsum = 0;
			ysum = 0;
			chksum = 0;
			for(i=0;i<NUMBER_OF_GYROS;i++){
				spiRW(AD_ReadData,5,i);
				xsum += (SPI_RxBuf[1]+(SPI_RxBuf[2]<<8));
				ysum += (SPI_RxBuf[3]+(SPI_RxBuf[4]<<8));
			}
			*(buffer + bufWRpos++) = (int16_t)xsum/NUMBER_OF_GYROS;			// Mean of x sensors.
			chksum = chksum ^ (int16_t)xsum/NUMBER_OF_GYROS;
			if(bufWRpos == BUFSIZE) bufWRpos = 0;
			if(bufWRpos == bufRDpos) bufFlag = 'V';							// Check for buffer overflow.
			*(buffer + bufWRpos++) = (int16_t)ysum/NUMBER_OF_GYROS;			// Mean of y sensors.
			chksum = chksum ^ (int16_t)xsum/NUMBER_OF_GYROS;
			if(bufWRpos == BUFSIZE) bufWRpos = 0;
			if(bufWRpos == bufRDpos) bufFlag = 'V';							// Check for buffer overflow.
			*(buffer + bufWRpos++) = chksum;								// Sync / checksum.
			if(bufWRpos == BUFSIZE) bufWRpos = 0;
			if(bufWRpos == bufRDpos) bufFlag = 'V';							// Check for buffer overflow.
			break;
		case 'S':
		// Store the full sample set.
			chksum = 0;
			for(i=0;i<NUMBER_OF_GYROS;i++){
				spiRW(AD_ReadData,5,i);
				*(buffer + bufWRpos++) = (SPI_RxBuf[1]+(SPI_RxBuf[2]<<8));
				chksum = chksum ^ (SPI_RxBuf[1]+(SPI_RxBuf[2]<<8));
				if(bufWRpos == BUFSIZE) bufWRpos = 0;
				if(bufWRpos == bufRDpos) bufFlag = 'V';						// Check for buffer overflow.
				*(buffer + bufWRpos++) = (SPI_RxBuf[3]+(SPI_RxBuf[4]<<8));
				chksum = chksum ^ (SPI_RxBuf[3]+(SPI_RxBuf[4]<<8));
				if(bufWRpos == BUFSIZE) bufWRpos = 0;
				if(bufWRpos == bufRDpos) bufFlag = 'V';						// Check for buffer overflow.
			}
			*(buffer + bufWRpos++) = chksum;
			if(bufWRpos == BUFSIZE) bufWRpos = 0;
			if(bufWRpos == bufRDpos) bufFlag = 'V';							// Check for buffer overflow.
			break;
		case 'A':
		// Calculate the running accumulated mean of sample set means.
			xsum = 0;
			ysum = 0;
			for(i=0;i<NUMBER_OF_GYROS;i++){
				spiRW(AD_ReadData,5,i);
				xsum += (SPI_RxBuf[1]+(SPI_RxBuf[2]<<8));
				ysum += (SPI_RxBuf[3]+(SPI_RxBuf[4]<<8));
			}
			xaccumulator[accHalf] += xsum/NUMBER_OF_GYROS;
			yaccumulator[accHalf] += ysum/NUMBER_OF_GYROS;
			if(accCounter++ == 0) bufFlag = 'V';							// More than 2^16 means accumulated.
			break;
		case 'C':
		// Accumulate exactly two times N sample set means. Flag the completion of this command in the buffer status.
		// The number N is included in the 'C' command from the master.
		// TODO: The command parser must initialize accCounter=0, accHalf=0, Nsamples=N and bufFlag='A'.
		//
			if(accCounter++ < Nsamples){
				xsum = 0;
				ysum = 0;
				for(i=0;i<NUMBER_OF_GYROS;i++){
					spiRW(AD_ReadData,5,i);
					xsum += (SPI_RxBuf[1]+(SPI_RxBuf[2]<<8));
					ysum += (SPI_RxBuf[3]+(SPI_RxBuf[4]<<8));
				}
				xaccumulator[accHalf] += xsum;
				yaccumulator[accHalf] += ysum;
			}
			else if(accHalf == 0){											// Start accumulating second half
					accHalf = 1;
					accCounter = 0;
					bufFlag = 'H';
				}
				else bufFlag = 'F';											// Accumulation finished.
			break;
		default:
			break;
	}
			
}

/*
// SPI is handled by polling inside the timer interrupt.
// Remember that gyros are also accessed during initialization.
//------------------------------------------------------------------------------
// SPI interrupt handler
ISR(SPI_STC_vect){
	*SPI_RxPtr++ = SPDR;					// Write received byte to buffer
	++SPI_TxPtr;
	if(--spi_length>0){
		SPDR = *SPI_TxPtr;					// Transmit next byte.
	}
	else{
		SPCR &= (~(1<<SPIE));				// Disable SPI interrupt.
		spi_txflg = 0;
	}
}
*/

//------------------------------------------------------------------------------
// TWI interrupt handler
// The TWEA bit is set during initialization and remains always set, to enable the slave to respond to its own address. 
// 
ISR(TWI_vect){
	uint8_t status,data;
	data=TWDR;														// Read data immediately.
	status=TWSR & 0xF8;
	
	switch(status){
		case 0x60:													// Address and write command received, ACK returned.
			TWI_RxPos = 0;											// Initialize write pointer.
			break;
		case 0x80:													// Data received, ACK returned.
			if(TWI_RxPos < TWI_RX_BUFFER_SIZE){
				*(TWI_RxBuf + TWI_RxPos++) = data; 					// Store received byte if receive buffer is not full.
			}
			else TWI_Error = 1;										// Set error code if receive buffer overflow.								
			break;
		case 0x88:													// Data received, NAK returned. Shall not occur.
			break;
		case 0xA0:													
			// STOP or repeated START received. Enters not addressed slave mode.														
			// Must hold clock while parsing the received command to see if something needs to be set up for 														
			// a subsequent read command.														
			TWI_RxPos = 0;											// Reset receive pointer to read command.
			data = *(TWI_RxBuf + TWI_RxPos++);
			switch(data){
				case 'Q':											// Test command &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& TODO: Should check all of message, not only first char.
					TWI_TxPtr = testdata;							// Set up pointers for reading test data.
					TWI_TxLength = 8;
					TWI_TxPos = 0;
					break;
				default:											// Command does not require immediate action, leave for background parser.
					TWI_RxPos = 0;									// Reset receive pointer for the background parser that handles commands that do not imply reading..													
					TWI_rxflag = 1;									// Set flag to indicate that command has been received.	
			}			
			break;
		case 0xA8:													// Address and read command received.
			TWDR = *(TWI_TxPtr + TWI_TxPos++);						// Send first data. Pointer, pos and length was set when read command was received.
			break;
		case 0xB8:													// Data transmitted, ACK received.
			if(TWI_TxPos < TWI_TxLength){ 							// Continue transmission if more data.
				TWDR = *(TWI_TxPtr + TWI_TxPos++);
			}
			else{
				TWDR = 0xFF;										// Transmit dummy data if real data is finished and master wants more.
				TWI_txflag = 1;										// Flag that data buffer has been transmitted.
				TWI_Error = 2;
			}
			break;
		case 0xC0:													// Data transmitted, NAK received.Enters not addressed slave mode.
			TWI_txflag = 1;
			if(TWI_TxPos < TWI_TxLength) TWI_Error = 3;				// Not all requested data was read.
			break;
		default:
			break;
		
	}
	TWCR = TWCR | (1<<TWINT) | (1<<TWEA);							// Clear interrupt to start operation of TWI hardware again.
}

//------------------------------------------------------------------------------
// UART Data Register Empty interrupt handler
ISR(USART_UDRE_vect){
	//PORTD ^= (1<<2);  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  
	Console_TxPtr++;
	if(*Console_TxPtr){
		UDR0 = *Console_TxPtr;					// Transmit next byte.
	}
	else{
		UCSR0B = 0x08;							// Disable UDRE interrupt
		_delay_us(600);
		console_txflg = 0;
	}
	//PORTD ^= (1<<2);  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
}
/*
//------------------------------------------------------------------------------
// UART Transmission complete interrupt handler
ISR(USART_TX_vect){
	//UCSR0B = 0x08;								// Disable TXC interrupt.
	console_txflg = 0;
}
*/
//	MAIN
//==============================================================================
int main(void){
	uint8_t i;
	uint16_t jj;
	uint8_t CMD_Error;									// Parse error in received command.
														//  1: Non-numeric character i number field.
														//  2: Command not terminated by '\n'.
	//--------------------------------------------------------------------------
	//		Initialize ports
	DDRB  = (1<<0) | (1<<1) | (1<<2) | (1<<6) | (1<<7);	// B0, B1, B2, B6, B7 are Chip Select outputs.
	DDRB |= (1<<3) | (1<<5);							// PB3 is MOSI, PB5 is SCLK.
	PORTB = 0xEF;										// All outputs high.
	
	DDRC  = 0;											// All port pins of port C are TWI address inputs
	PORTC = 0x8F;										// Pull-ups on all TWI address pins.
	
	DDRD  = (1<<2);										// Pin D2 is test output.
	DDRD |= (1<<5) | (1<<6) | (1<<7);					// D5, D6, D7 are Chip Select outputs.
	PORTD = 0xE0;										// Test output low.
	
	//--------------------------------------------------------------------------
	//		Initialize Timer 2
	// This timer generates a 40 Hz interrupt that is used for sample timing.
	// The timer interrupt is not enabled until system initialization is complete.
	//
	TCNT2  = 0;									// Clear counter value.
	TCCR2A = 0x02;								// CTC mode.
	TCCR2B = 0x07;								// Prescaler 1024.
	OCR2A  = 195;								// Interrupt freq 8e6/1024/195 = 40 Hz
	
	//--------------------------------------------------------------------------
	//		Initialize TWI
	TWAR = (TWIaddress)<<1;		// Slave address is 0x20	//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	// TWAR = (PINC & 0x0F) | (PINC & 0x80)>>3;	//TODO: Replace fixed address with address read from pins.
	TWCR = (1<<TWINT)
			|(1<<TWEA)
			|(1<<TWEN)
			|(1<<TWIE);					// Prepare I2C slave to receive command.
	TWI_Error = 0;						// Clear TWI error byte.
	
	//--------------------------------------------------------------------------
	//		Initialize SPI
	SPCR = 0x5C;		// SPI enable, MSB first, master, clock 2 MHz, CPOL=1, CPHA=1.
	
	//--------------------------------------------------------------------------
	//		Initialize UART
	//	The UART is used for console output during debugging.
	
	UCSR0A = 0x00;
	UCSR0B = 0x08;								// Enable transmitter.
	UCSR0C = 0x06;								// 8 bits, 1 stop bit, no parity.
	UBRR0  = 12;								// Baud rate 38.4 kbit/s
	console_txflg = 0;							// No transmission ongoing.
	
	//--------------------------------------------------------------------------
	//		Initialize variables
	CMD_Error = 0;
	TWI_Error = 0;
	bufType = '0';
	TWI_rxflag = 0;
	
	//--------------------------------------------------------------------------
	//		Enable interrupts
	_delay_ms(50);
	sei();

	//--------------------------------------------------------------------------
	// Initialize gyros
	for(i=0;i<NUMBER_OF_GYROS;i++){
		spiRW(AD_Activate,2,i);
		spiRW(AD_Filter_20Hz,2,i);
	}
	
	//--------------------------------------------------------------------------
	// Enable timer interrupts
	TIMSK2 = 0x02;						// Enable Timer2 output compare match A interrupt.
	
	//--------------------------------------------------------------------------
	//		Main loop	
	while(1){
		// Parse incoming message.
		if(TWI_rxflag){
			TWI_rxflag = 0;
			switch(TWI_RxBuf[0]){
				case 'C':
					// Command syntax 'Cxxxx\n' where xxxx is ascii-coded number of samples to accumulate.
					// No response to this command.
					bufType = 'C';
					bufFlag = 'A';
					accCounter = 0;
					accHalf = 0;
					jj = 0;
					for(i=0;i<4;i++){
						if((TWI_RxBuf[i+1]>=0x30) && (TWI_RxBuf[i+1]<=0x39)){
							jj = jj*10 + (TWI_RxBuf[i+1]-0x30);
						}
						else{
							CMD_Error = 1;
							break;
						}
					}
					Nsamples = jj;
					if(TWI_RxBuf[i+1] != '\n') CMD_Error = 2;
					break;
					
				case 'B':
					// Query buffer status.
					// Command syntax 'B\n'
					// Response syntax: 'bfxxxx\n' where 'b' is buffer type, 'f' is buffer flag (A,H,F)
					// and 'xxxx' is number of samples accumulated.
					if(TWI_RxBuf[1] != '\n'){CMD_Error = 2; break;}				// Syntax error.
					i = 0;
					// If any error codes are set, these will be returned instead of the buffer status.
					// Reported error codes are cleared. The buffer status can then be obtained by issuing the 'B' 
					// command again.
					if(TWI_Error || CMD_Error){
						TWI_TxBuf[i++] = 'E';
						if(TWI_Error){
							TWI_TxBuf[i++] = 'T';	
							TWI_TxBuf[i++] = TWI_Error + 0x30;
							TWI_Error = 0;
						}
						if(CMD_Error){
							TWI_TxBuf[i++] = 'C';
							TWI_TxBuf[i++] = CMD_Error + 0x30;
							CMD_Error = 0;
						}
						TWI_TxBuf[i] = '\n';
					}
					else{
						TWI_TxBuf[0] = bufType;
						TWI_TxBuf[1] = bufFlag;
						for(i=5;i>1;i--){
							TWI_TxBuf[i] = accCounter % 10;
							accCounter = accCounter / 10;
						}
						TWI_TxBuf[6] = '\n';
					}
				default:
					break;
			}	
		}
	}
}

//==============================================================================
// Read or write SPI data
//		Input parameters:
//			*pointer	Pointer to the string to be transmitted. 
//			length		Number of bytes to be transmitted.
//			device		Number (1-8) of gyro to be selected.
//		Received bytes are stored in SPI_RxBuf.
// TODO: Check if SPSR & 0x80 becomes 1 at start of transmission, this may cause loss of reception.
void spiRW(char *pointer, uint8_t length, uint8_t device){
	assertSS(device);
	//_delay_us(10);							// Wait approx. 10 µs.
	SPI_RxPtr = SPI_RxBuf;
	SPI_TxPtr = pointer;
	spi_length = length;
	spi_txflg = 1;
	SPDR = *pointer;
	while(spi_txflg){
		while((SPSR & 0x80)	== 0);				// Wait for byte to be transmitted.
		*SPI_RxPtr++ = SPDR;					// Write received byte to buffer
		++SPI_TxPtr;
		if(--spi_length>0){
			SPDR = *SPI_TxPtr;					// Transmit next byte.
		}
		else{
			spi_txflg = 0;
		}
	}
	releaseSS(device);
}

//==============================================================================
// Assert SS signal
//		Input parameter:
//			device		Number (1-8) of gyro to be selected.

void assertSS(uint8_t device){
	switch(device){
		case 1:
			PORTB &= ~(1<<0);
			break;
		case 2:
			PORTD &= ~(1<<7);
			break;
		case 3:
			PORTB &= ~(1<<2);
			break;
		case 4:
			PORTB &= ~(1<<7);
			break;
		case 5:
			PORTD &= ~(1<<6);
			break;
		case 6:
			PORTD &= ~(1<<5);
			break;
		case 7:
			PORTB &= ~(1<<1);
			break;
		case 8:
			PORTB &= ~(1<<6);
			break;
		default:
			;
	}	
}

//==============================================================================
// Release SS signal
//		Input parameter:
//			device		Number (1-8) of gyro to be selected.

void releaseSS(uint8_t device){
	switch(device){
		case 1:
		PORTB |= (1<<0);
		break;
		case 2:
		PORTD |= (1<<7);
		break;
		case 3:
		PORTB |= (1<<2);
		break;
		case 4:
		PORTB |= (1<<7);
		break;
		case 5:
		PORTD |= (1<<6);
		break;
		case 6:
		PORTD |= (1<<5);
		break;
		case 7:
		PORTB |= (1<<1);
		break;
		case 8:
		PORTB |= (1<<6);
		break;
		default:
		;
	}
}

//==============================================================================
// Send message to console
//		The global variable console_txflg is set when this routine is called, and is
//		cleared by the interrupt routine when the transmission is completed.

void ConsoleTX(char *pointer){
	cli();
		PORTD ^= (1<<2);  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	while(console_txflg);						// Wait for previous transmission to complete.
	Console_TxPtr = pointer;					// Initialize pointer to string.
	console_txflg = 1;							// Indicate that transmission is ongoing.
	UDR0 = *Console_TxPtr;						// Send first character of string.
	UCSR0B = 0x28;								// Enable UDRE interrupt.
		PORTD ^= (1<<2);  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		sei();
}

