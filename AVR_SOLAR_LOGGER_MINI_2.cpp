/*
 * AVR_SOLAR_LOGGER_MINI_2.cpp
 *
 * Created: 3.3.2016 15:22:20
 *  Author: Tomy2
 */ 

// ARDUINO MINI PRO + W25Q64BV + LTC3105 + SOLAR + DS18B20 + ADC 1.1ref and averaging + EXTRA SLEEP + BOOT communication + ASCII output + faster ADC + more power saving

/*
ACTIVE
	ATMEGA328P - 3.9 active (3.3V at 8MHz); 0.75mA idle
			   - 1.1mA for 45us (half write time) SCK LED (1k ohm)
	W25Q64BV - 25uA Standby
			 - 20mA Write
			 - 90us Write 8 bytes to 4 pages (2 bytes each)
			 - 3us enter Power Down
			 - 3us release from Power Down
			 - 1-10ms Power Up
	ADC - 3.72ms for 3 channels, 10 samples at 125kHz ADC clock (first conversion after ref change 25 ADC clock cycles; single conversion 13 ADC clock cycles)
	DS18B20 - 6.96ms to get data to the MCU
			- 1mA Active
			- 750ms 12-bit conversion
			- 375ms 11-bit conversion
			- 187.5ms 10-bit conversion
			- 93.75ms 9-bit conversion
SLEEP
	ATMEGA328P - 4.2uA (theoretically; with WDT enabled) (with regulator 54uA)
			   - 2ms Start-up time from Power-down mode (16k CK)
			   - 60us wake-up time with BOD disabled
	W25Q64BV - 25uA Standby (if not Powered Down)
			 - 1uA Power Down (worth it for >4s sleep)
	ADC - 
	DS18B20 - 750nA Standby

Arduino Mini Pro
AREF pin 100nF capacitor. Time to charge from 0V to 1.1V is 1.29ms, from 0V to 3.29V is 18.56ms. (AREF input resistance 32k ohm)

DS18B20
Supplying current to the DS18B20 is through the use of an external power supply tied to the VDD in. The advantage to this is that the strong
pullup is not required on the DQ line, and the bus master need not be tied up holding that line high during temperature conversions. This allows
other data traffic on the 1-Wire bus during the conversion time.

10.10.1 Analog to Digital Converter
If enabled, the ADC will be enabled in all sleep modes. To save power, the ADC should be disabled before entering any sleep mode. When the ADC is turned off and
on again, the next conversion will bean extended conversion.

10.9
Module shutdown can be used in Idle mode and Active mode to significantly reduce the overall power consumption. In all other sleep modes, the clock is already stopped.

10.2
When the BOD has been disabled, the wake-up time from sleep mode will be approximately 60 ?s to ensure that the BOD is working correctly before the MCU continues executing code.
Upon wake-up from sleep, BOD is automatically enabled again.

9.2.2 Clock Startup Sequence
When starting up from Power-save or Power-down mode, VCC is assumed to be at a sufficient level and only the start-up time is included.

14.2.6 Unconnected Pins
If some pins are unused, it is recommended to ensure that these pins have a defined level. Even though most of the digital inputs are disabled in the deep sleep modes
as described above, floating inputs should be avoided to reduce current consumption in all other modes where the digital inputs are enabled (Reset, Active mode and
Idle mode).

TIMING
	5s BOOT window
	loop
		60us/2ms ATMEGA328P Start-up time
		2ms ADC Wake Up
		3.72ms ADC readings
		6.96ms DS18B20 getting data
		3us W25Q64BV release from Power Down 
		90us Write 8 bytes to 4 pages
		3us W25Q64BV enter Power Down
		39.6ms EEPROM write 12 bytes every X cycles (EEPROM write 3.3ms); 'F' every 100 (25s), 'S' every 10 (80s) 
		0.25s/8s Watchdog Timer 'F'/'S' mode
		
CONSUMPTION (estimated)
	SLOW mode 0.014/8s (0.054/8s every 10th cycle) -> 0.056mA average current (0.150mA worst case - DS18B20 consumption duration uncertainty)
	FAST mode 0.014/0.25s (0.054/0.25s every 100th cycle) -> 0.057mA average current (0.432mA worst case - DS18B20 consumption duration uncertainty)

UART Commands in BOOT
'B' - enter BOOT within 5-10s after power up
'R' - read the whole chip (8MB)
'A' - ASCII output when READing
'N' - NORMAL output when READing (byte)
'E' - erase the whole chip, reset the address pointers, '0x77' response when done
'D' - serves for debugging
'Q' - exit BOOT and start normal logging operation
'1' - DS18B20 9-bit resolution
'2' - DS18B20 10-bit resolution
'3' - DS18B20 11-bit resolution
'4' - DS18B20 12-bit resolution
'F' - Fast Logging Mode (0.25s watchdog, 9-bit DS18B20 resolution)
'S' - Slow Logging Mode (8s watchdog, 12-bit DS18B20 resolution)
*/

#define F_CPU 8000000UL // Arduino MINI PRO 3.3V 8MHz

#define USART_BAUDRATE 250000 // must be fast or reading the FLASH will take forever (250000 max for 4MHz)
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL))) - 1) // when F_CPU changes, 8UL must change as well

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // for UART RX
#include <avr/eeprom.h> // for W25Q64BV pointers
#include <avr/sleep.h> // for battery saving
#include <avr/power.h> // for battery saving
#include <avr/wdt.h> // for battery saving


//BLINK function ----------------------------------------------------------------------------------------------------
void blink_LED(void)
{
	PORTB |= (1 << PORTB5);
	_delay_ms(50);
	PORTB &= ~(1 << PORTB5);
}


//UART functions ----------------------------------------------------------------------------------------------------
#define WD_S8 0b00100001 // 8s Watchdog Timer
#define WD_S1 0b00000110 // 1s Watchdog Timer
#define WD_S05 0b00000101 // 0.5s Watchdog Timer
#define WD_S025 0b00000100 // 0.25s Watchdog Timer
#define TEMP_RES_9  0b00011111
#define TEMP_RES_10 0b00111111
#define TEMP_RES_11 0b01011111
#define TEMP_RES_12 0b01111111
#define UART_BUFFER_SIZE 32
#define UARTTO 65535
uint8_t UARTbuffer[UART_BUFFER_SIZE];
uint8_t UARTbuffer_current = 0;
uint8_t W25Q64BVerase = 0;
uint8_t W25Q64BVread = 0;
uint8_t W25Q64BVdebug = 0;
uint32_t UARTtimeout = UARTTO;
uint8_t boot = 0;
uint8_t NORMALch = 0;
uint8_t ASCIIch = 0;
uint8_t normalASCII = 0;
uint8_t watchDogMask = WD_S8;
uint8_t TEMPres9 = 0;
uint8_t TEMPres10 = 0;
uint8_t TEMPres11 = 0;
uint8_t TEMPres12 = 0;
uint8_t FASTlog = 0;
uint8_t SLOWlog = 0;
uint8_t TEMPRES = TEMP_RES_12;
uint8_t normalOperation = 0;

void UART_init(void)
{
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Turn on the UART function on pins 0 and 1
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
	UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate
	UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate
}

void UART_transmit(uint8_t data)
{
	UARTtimeout = UARTTO;
	while(!(UCSR0A & (1 << UDRE0)) && UARTtimeout) UARTtimeout--; // Wait until UDR0 is ready for new data
	if(UARTtimeout) UDR0 = data; // Write the data into the register
}

uint8_t UART_receive(void)
{
	UARTtimeout = UARTTO;
	while(!(UCSR0A & (1 << RXC0)) && UARTtimeout) UARTtimeout--; // Wait for the receive flag
	if(UARTtimeout) return UDR0;
	else return 0x00;
}

ISR(USART_RX_vect)
{
	uint8_t data = UDR0;
	if(data == 'B') boot = 1;
	if(data == 'E') W25Q64BVerase = 1;
	if(data == 'R') W25Q64BVread = 1;
	if(data == 'A') ASCIIch = 1;
	if(data == 'N') NORMALch = 1;
	if(data == 'D') W25Q64BVdebug = 1;
	if(data == 'Q') boot = 0;
	if(data == '1') TEMPres9 = 1;
	if(data == '2') TEMPres10 = 1;
	if(data == '3') TEMPres11 = 1;
	if(data == '4') TEMPres12 = 1;
	if(data == 'F') FASTlog = 1;
	if(data == 'S') SLOWlog = 1;
	data = 0x00;
}

void ASCII_16bit_transmit(uint16_t number)
{
	uint8_t ascii1 = 0; // 16bit numbers - 5 decimal places max
	uint8_t ascii2 = 0;
	uint8_t ascii3 = 0;
	uint8_t ascii4 = 0;
	uint8_t ascii5 = 0;
	uint16_t num = number;
	ascii1 = num / 10000;
	num = number;
	ascii2 = (num - (ascii1 * 10000)) / 1000;
	num = number;
	ascii3 = (num - (ascii1 * 10000) - (ascii2 * 1000)) / 100;
	num = number;
	ascii4 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) / 10;
	num = number;
	ascii5 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) % 10;
	ascii1 += '0';
	ascii2 += '0';
	ascii3 += '0';
	ascii4 += '0';
	ascii5 += '0';
	if(number >= 10000)UART_transmit(ascii1);
	if(number >= 1000)UART_transmit(ascii2);
	if(number >= 100)UART_transmit(ascii3);
	if(number >= 10)UART_transmit(ascii4);
	UART_transmit(ascii5);
	UART_transmit(0x0D); // '\r'
	UART_transmit(0x0A); // '\n'
}

void ASCII_32bit_transmit(uint32_t number)
{
	uint8_t ascii1 = 0; // 32bit numbers - 10 decimal places max
	uint8_t ascii2 = 0;
	uint8_t ascii3 = 0;
	uint8_t ascii4 = 0;
	uint8_t ascii5 = 0;
	uint8_t ascii6 = 0;
	uint8_t ascii7 = 0;
	uint8_t ascii8 = 0;
	uint8_t ascii9 = 0;
	uint8_t ascii10 = 0;
	uint16_t num1 = number % 1000;
	uint16_t num2 = ((number % 1000000) - num1) / 1000;
	uint16_t num3 = ((number % 1000000000) - num1 - num2) / 1000000;
	uint16_t num4 = number / 1000000000;
	ascii1 = num4;
	ascii2 = num3 / 100;
	ascii3 = (num3 - (ascii2 * 100)) / 10;
	ascii4 = (num3 - (ascii2 * 100)) % 10;
	ascii5 = num2 / 100;
	ascii6 = (num2 - (ascii5 * 100)) / 10;
	ascii7 = (num2 - (ascii5 * 100)) % 10;
	ascii8 = num1 / 100;
	ascii9 = (num1 - (ascii8 * 100)) / 10;
	ascii10 = (num1 - (ascii8 * 100)) % 10;
	ascii1 += '0';
	ascii2 += '0';
	ascii3 += '0';
	ascii4 += '0';
	ascii5 += '0';
	ascii6 += '0';
	ascii7 += '0';
	ascii8 += '0';
	ascii9 += '0';
	ascii10 += '0';
	if(number >= 1000000000)UART_transmit(ascii1);
	if(number >= 100000000)UART_transmit(ascii2);
	if(number >= 10000000)UART_transmit(ascii3);
	if(number >= 1000000)UART_transmit(ascii4);
	if(number >= 100000)UART_transmit(ascii5);
	if(number >= 10000)UART_transmit(ascii6);
	if(number >= 1000)UART_transmit(ascii7);
	if(number >= 100)UART_transmit(ascii8);
	if(number >= 10)UART_transmit(ascii9);
	UART_transmit(ascii10);
	UART_transmit(0x0D); // '\r'
	UART_transmit(0x0A); // '\n'
}


// SPI Functions ------------------------------------------------------------------------------------
#define SS PORTB2
#define MOSI PORTB3
#define MISO PORTB4
#define SCK PORTB5
#define SPITO 8192
uint32_t SPItimeout = SPITO;

void SPI_init_master(void)
{
	DDRB |= (1<<MOSI)|(1<<SCK)|(1<<SS); // MOSI, SCK, SS as OUTPUT - PB2/D51, PB1/D52, PB0/D53 (MEGA) - PB3/D11, PB5/D13, PB2/D10 (NANO)
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // Enable SPI, Set as Master, Prescaler: Fosc/16
	PORTB |= (1<<SS); // set SS HIGH
}

uint8_t SPI_master_transmit(uint8_t data)
{
	SPItimeout = SPITO;
	SPDR = data; // Start transmission
	while(!(SPSR & (1<<SPIF)) && SPItimeout) SPItimeout--; // Wait for transmission complete
	return SPDR; // return the answer
}


// W25Q64BV Functions -----------------------------------------------------------------------------------------
/*
7 - SRP0 (Status Register Protect 0)
6 - SEC (Sector Protect)
5 - TB (Top/Bottom Write Protect)
4 - BP2 (Block Protect)
3 - BP1 (Block Protect)
2 - BP0 (Block Protect)
1 - WEL (Write Enable Latch)
0 - BUSY (Erase or Wright in Progress)
*/
uint8_t W25Q64BVbufferW[256]; // 256 bytes = max bytes written at once
uint8_t W25Q64BVbuffer[256];
uint8_t W25Q64BVblock1buff[2]; // buffer for 16bit write to Block 1
uint8_t W25Q64BVblock2buff[2]; // buffer for 16bit write to Block 2
uint8_t W25Q64BVblock3buff[2]; // buffer for 16bit write to Block 3
uint8_t W25Q64BVblock4buff[2]; // buffer for 16bit write to Block 4
uint32_t W25Q64BVpointer;

bool W25Q64BV_busy(void)
{
	uint8_t r1;
	PORTB &= ~(1<<SS); // Set SS LOW
	SPI_master_transmit(0x05);
	r1 = SPI_master_transmit(0xff);
	PORTB |= (1<<SS); // Set SS HIGH
	if(r1 & 0x01)
	return true;
	return false;
}

void W25Q64BV_read_data(uint8_t buffer[], uint32_t address, uint32_t num_bytes) // address - 24 bits
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x03);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	for(uint32_t i = 0; i < num_bytes; i++)
	{
		buffer[i] = SPI_master_transmit(0x00);
	}
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_write_data(uint8_t buffer[], uint32_t address, uint8_t num_bytes) // address - 24 bits, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x02);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	for(uint8_t i = 0; i < num_bytes; i++)
	{
		SPI_master_transmit(buffer[i]);
	}
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_write_byte(uint8_t byte, uint32_t address) // address - 24 bits, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x02);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	SPI_master_transmit(byte);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_write_enable(void)
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x06);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_write_disable(void)
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x04);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_sector_erase(uint32_t address) // erase 4KB, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x20);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_block_erase32(uint32_t address) // erase 32KB, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0x52);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_block_erase64(uint32_t address) // erase 64KB, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0xD8);
	SPI_master_transmit(address >> 16);
	SPI_master_transmit(address >> 8);
	SPI_master_transmit(address);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_chip_erase(void) // erase the whole chip, WEL = 1
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0xC7);
	PORTB |= (1<<SS); // set SS HIGH
}

void W25Q64BV_power_down(void)
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0xB9);
	PORTB |= (1<<SS); // set SS HIGH
	_delay_us(3); // wait for the device to enter Power Down
}

void W25Q64BV_release_power_down(void)
{
	PORTB &= ~(1<<SS); // set SS LOW
	SPI_master_transmit(0xAB);
	PORTB |= (1<<SS); // set SS HIGH
	_delay_us(3); // wait for the device to restore normal operation
}


//EEPROM functions -------------------------------------------------------------------------------------------------
/*
8388608 FLASH addresses divided into 4 blocks
16bit numbers -> 1048576 values (write every 1s -> 12 days)
test of ATMEGA328P's EEPROM claims 1230163 write cycles until the first error (write every 60s -> 2.34 years)
EEPROM ADDRESS
0-3		W25Q64BVblock1pointer
4-7		W25Q64BVblock2pointer
8-11	W25Q64BVblock3pointer
12-15	W25Q64BVblock4pointer
16		FAST/SLOW LOGGING mode
17		DS18B20 resolution
18		ASCII/HEX output
*/
uint16_t EEPROMblock1add = 0; // address in EEPROM that holds the current W25Q64BVblock1pointer value (updated every Xs)
uint16_t EEPROMblock2add = 4; // address in EEPROM that holds the current W25Q64BVblock2pointer value (updated every Xs)
uint16_t EEPROMblock3add = 8; // address in EEPROM that holds the current W25Q64BVblock3pointer value (updated every Xs)
uint16_t EEPROMblock4add = 12; // address in EEPROM that holds the current W25Q64BVblock4pointer value (updated every Xs)
uint8_t EEPROMmodeadd = 16;
uint8_t EEPROMresolutionadd = 17;
uint8_t EEPROMoutputadd = 18;
uint32_t W25Q64BVblock1add = 0x000000; // start address of Block 1 (2097152 bytes)
uint32_t W25Q64BVblock2add = 0x200000; // start address of Block 2 (2097152 bytes)
uint32_t W25Q64BVblock3add = 0x400000; // start address of Block 3 (2097152 bytes)
uint32_t W25Q64BVblock4add = 0x600000; // start address of Block 4 (2097152 bytes)
uint32_t W25Q64BVblock1pointer, W25Q64BVblock2pointer, W25Q64BVblock3pointer, W25Q64BVblock4pointer;
uint8_t EEPROMupdate = 10;


//ADC functions ----------------------------------------------------------------------------------------------------
#define ADCSAMPLES 10
uint16_t FLASHvalue1, FLASHvalue2, FLASHvalue3, FLASHvalue4; // 16bit values to be written to the 4 FLASH blocks

void ADC_wake_up(void)
{
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, set prescaler 128 (16000000 / 128 = 125kHz)
	ADMUX = (1 << REFS1) | (1 << REFS0); // voltage reference internal 1.1V with external cap at AREF pin, MUX3:0 - 0 -> ADC0 input
	_delay_ms(2); // should be enough for the 100nF AREF cap to charge to 1.1V
}

void ADC_sleep(void)
{
	ADCSRA = 0;
}

uint16_t ADC_read_11ref(uint8_t channel)
{
	uint32_t temp = 0;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, set prescaler 128 (16000000 / 128 = 125kHz)
	channel &= 0b00000111; // channel value will always be between 0 and 7
	ADMUX = (1 << REFS1) | (1 << REFS0) | channel; // voltage reference internal 1.1V with external cap at AREF pin, MUX3:0 - 0 -> ADC0 input
	// one conversion after REF change should be discarded
	ADCSRA |= (1 << ADSC); // start single conversion for selected Analog Input
	while(ADCSRA & (1 << ADSC)); // wait for the end of conversion (ADSC = 0)
	temp = ADC;
	temp = 0;
	for(uint16_t y = 0; y < ADCSAMPLES; y++)
	{
		ADCSRA |= (1 << ADSC); // start single conversion for selected Analog Input
		while(ADCSRA & (1 << ADSC)); // wait for the end of conversion (ADSC = 0)
		temp += ADC;
	}
	return (temp / ADCSAMPLES);
}


//DS18B20 functions ----------------------------------------------------------------------------------------------------
/*
DS18B20 connections:
GND -> GND
DQ -> D7 (4.7k pull up to VCC)
VDD -> VCC (3.3V/5V)
*/
#define TEMP_PIN PORTD7 // pin D7, must be on PORTD

void DS18B20_init(void)
{
	DDRD &= ~(1 << TEMP_PIN); // input
	PORTD &= ~(1 << TEMP_PIN); // LOW
}

void DS18B20_OneWire_reset(void)
{
	DDRD |= (1 << TEMP_PIN); // output
	PORTD &= ~(1 << TEMP_PIN); // LOW
	_delay_us(500);
	DDRD &= ~(1 << TEMP_PIN); // input
	PORTD &= ~(1 << TEMP_PIN); // LOW
	_delay_us(500);
}

void DS18B20_OneWire_send_byte(uint8_t data)
{
	for(uint8_t i = 8; i > 0; i--)
	{
		if(data & 1)
		{
			DDRD |= (1 << TEMP_PIN); // output
			PORTD &= ~(1 << TEMP_PIN); // LOW
			_delay_us(5);
			DDRD &= ~(1 << TEMP_PIN); // input
			PORTD &= ~(1 << TEMP_PIN); // LOW
			_delay_us(60);
			}else{
			DDRD |= (1 << TEMP_PIN); // output
			PORTD &= ~(1 << TEMP_PIN); // LOW
			_delay_us(60);
			DDRD &= ~(1 << TEMP_PIN); // input
			PORTD &= ~(1 << TEMP_PIN); // LOW
		}
		data >>= 1;
	}
}

uint8_t DS18B20_OneWire_get_byte(void)
{
	uint8_t byte = 0, bit;
	for(uint8_t i = 0; i < 8; i++)
	{
		DDRD |= (1 << TEMP_PIN); // output
		PORTD &= ~(1 << TEMP_PIN); // LOW
		_delay_us(5);
		DDRD &= ~(1 << TEMP_PIN); // input
		PORTD &= ~(1 << TEMP_PIN); // LOW
		_delay_us(5);
		if(PIND & (1 << TEMP_PIN)) bit = 1;
		else bit = 0;
		byte = (byte >> 1) | (bit << 7);
		_delay_us(50);
	}
	return byte;
}

uint16_t DS18B20_get_temperature(void)
{
	uint16_t byte, data;
	DS18B20_OneWire_reset();
	DS18B20_OneWire_send_byte(0xCC); // skip ROM command
	DS18B20_OneWire_send_byte(0x44); // initiates temperature conversion
	DS18B20_OneWire_reset();
	DS18B20_OneWire_send_byte(0xCC); // skip ROM command
	DS18B20_OneWire_send_byte(0xBE); // reads bytes from scratchpad (first two bytes temperature data)
	data = DS18B20_OneWire_get_byte(); // Low Byte
	byte = DS18B20_OneWire_get_byte(); // High Byte
	data = data | (byte << 8);
	return data;
}

void DS18B20_resolution(uint8_t res) // 0b0xx11111 Bits 6:5 - 00 9-bit (93.75ms), 01 10-bit (187.5ms), 10 11-bit (375ms), 11 12-bit (750ms)
{
	DS18B20_OneWire_reset();
	DS18B20_OneWire_send_byte(0xCC); // skip ROM command
	DS18B20_OneWire_send_byte(0x4E); // write Scratchpad command
	DS18B20_OneWire_send_byte(0x00); // TH
	DS18B20_OneWire_send_byte(0x00); // TL
	DS18B20_OneWire_send_byte(res); // Config
	DS18B20_OneWire_reset();
	DS18B20_OneWire_send_byte(0xCC); // skip ROM command
	DS18B20_OneWire_send_byte(0xBE); // read Scratchpad command
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	uint8_t dat = DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	DS18B20_OneWire_get_byte();
	if(dat == res)
	{
		DS18B20_OneWire_reset();
		DS18B20_OneWire_send_byte(0xCC); // skip ROM command
		DS18B20_OneWire_send_byte(0x48); // copy Scratchpad command
		DS18B20_OneWire_reset();
		if(normalOperation)
		{
			UART_transmit('R');
			UART_transmit('E');
			UART_transmit('S');
			if(dat == 0b00011111) UART_transmit('9');
			else if(dat == 0b00111111) {UART_transmit('1'); UART_transmit('0');}
			else if(dat == 0b01011111) {UART_transmit('1'); UART_transmit('1');}
			else if(dat == 0b01111111) {UART_transmit('1'); UART_transmit('2');}
			else UART_transmit('?');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
		}
	}else{
		if(normalOperation)
		{
			UART_transmit('F');
			UART_transmit('A');
			UART_transmit('I');
			UART_transmit('L');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
		}
	}
}


//SLEEP & POWER DOWN functions -------------------------------------------------------------------------------------
void WatchDog_setup_and_sleep(void)
{
	cli();
	// TURN OFF MODULES
	UCSR0B = 0; // UART
	SPCR = 0; // SPI
	ADCSRA = 0; // ADC
	// INTERNAL PULL-UPS
	DDRB &= ~(0b00000011); // 0 input
	DDRC &= ~(0b00110100); // 0 input
	DDRD &= ~(0b01111100); // 0 input
	PORTB |= (1 << PORTB0) | (1 << PORTB1); // 1 internal pullup
	PORTC |= (1 << PORTC2) | (1 << PORTC4) | (1 << PORTC5); // 1 internal pullup
	PORTD |= (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5) | (1 << PORTD6); // 1 internal pullup
	// WATCHDOG SETUP
	MCUSR &= ~(1 << WDRF); // reset status flag
	WDTCSR |= (1 << WDCE) | (1 << WDE); // enable configuration changes
	WDTCSR = watchDogMask; // set the prescalar = 9 (8s); 1001 - 8s, 1000 - 4s, 0111 - 2s, 0110 - 1s, 0101 - 0.5s, 0100 - 0.25s, 0011 - 0.125s...
	WDTCSR |= (1 << WDIE); // enable interrupt mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // select the sleep mode
	sleep_enable(); // enable the sleep mode ready for use
	// TURN OFF BROWN OUT DETECTION
	MCUCR = (1 << BODS) | (1 << BODSE);
	MCUCR = (1 << BODS);
	sei();
	sleep_mode(); // trigger the sleep
	/* ...time passes ... */
	sleep_disable(); // prevent further sleeps
	cli();
	UART_init();
	SPI_init_master();
	DS18B20_init();
}

ISR(WDT_vect)
{
	wdt_disable();
}


//MAIN function ----------------------------------------------------------------------------------------------------
int main(void)
{
    uint8_t inboot = 0;
    uint8_t cycleCount = 0;
    
    UART_init();
    SPI_init_master();
    DS18B20_init();
	
	/*
	EEPROM write cycle endurance - 100 000 cycles (possibly 1 000 000 cycles)
	at 8 second sleep between cycles and updating values every 10 cycles (80s)
	these EEPROM addresses should work for 92.6 days (925.9 days for 1M cycles)
	*/
	// reading the last pointer addresses before power down
	W25Q64BVblock1pointer = eeprom_read_byte((uint8_t*)1);
	W25Q64BVblock1pointer <<= 8;
	W25Q64BVblock1pointer |= eeprom_read_byte((uint8_t*)2);
	W25Q64BVblock1pointer <<= 8;
	W25Q64BVblock1pointer |= eeprom_read_byte((uint8_t*)3);
	W25Q64BVblock2pointer = eeprom_read_byte((uint8_t*)5);
	W25Q64BVblock2pointer <<= 8;
	W25Q64BVblock2pointer |= eeprom_read_byte((uint8_t*)6);
	W25Q64BVblock2pointer <<= 8;
	W25Q64BVblock2pointer |= eeprom_read_byte((uint8_t*)7);
	W25Q64BVblock3pointer = eeprom_read_byte((uint8_t*)9);
	W25Q64BVblock3pointer <<= 8;
	W25Q64BVblock3pointer |= eeprom_read_byte((uint8_t*)10);
	W25Q64BVblock3pointer <<= 8;
	W25Q64BVblock3pointer |= eeprom_read_byte((uint8_t*)11);
	W25Q64BVblock4pointer = eeprom_read_byte((uint8_t*)13);
	W25Q64BVblock4pointer <<= 8;
	W25Q64BVblock4pointer |= eeprom_read_byte((uint8_t*)14);
	W25Q64BVblock4pointer <<= 8;
	W25Q64BVblock4pointer |= eeprom_read_byte((uint8_t*)15);
	
	// loading the last settings from EEPROM
	watchDogMask = eeprom_read_byte((uint8_t*)16);
	TEMPRES = eeprom_read_byte((uint8_t*)17);
	normalASCII = eeprom_read_byte((uint8_t*)18);
	if(watchDogMask == WD_S8) EEPROMupdate = 10;
	else if(watchDogMask == WD_S025) EEPROMupdate = 100;
	else watchDogMask = WD_S8;
	if(TEMPRES == TEMP_RES_9 || TEMPRES == TEMP_RES_10 || TEMPRES == TEMP_RES_11 || TEMPRES == TEMP_RES_12);
	else TEMPRES = TEMP_RES_12;
	DS18B20_resolution(TEMPRES);
	normalOperation = 1;
	
	UCSR0B |= (1 << RXCIE0); // enable RX complete interrupt
	sei();
	
	_delay_ms(5000); // time at startup to enter BOOT
	
	if(boot)
	{
		inboot = 1;
		UART_transmit('I');
		UART_transmit('N');
		UART_transmit(' ');
		UART_transmit('B');
		UART_transmit('O');
		UART_transmit('O');
		UART_transmit('T');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('R');
		UART_transmit('-');
		UART_transmit('r');
		UART_transmit('e');
		UART_transmit('a');
		UART_transmit('d');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('A');
		UART_transmit('-');
		UART_transmit('a');
		UART_transmit('s');
		UART_transmit('c');
		UART_transmit('i');
		UART_transmit('i');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('N');
		UART_transmit('-');
		UART_transmit('h');
		UART_transmit('e');
		UART_transmit('x');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('E');
		UART_transmit('-');
		UART_transmit('e');
		UART_transmit('r');
		UART_transmit('a');
		UART_transmit('s');
		UART_transmit('e');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('D');
		UART_transmit('-');
		UART_transmit('d');
		UART_transmit('e');
		UART_transmit('b');
		UART_transmit('u');
		UART_transmit('g');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('F');
		UART_transmit('-');
		UART_transmit('f');
		UART_transmit('a');
		UART_transmit('s');
		UART_transmit('t');
		UART_transmit(' ');
		UART_transmit('l');
		UART_transmit('o');
		UART_transmit('g');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('S');
		UART_transmit('-');
		UART_transmit('s');
		UART_transmit('l');
		UART_transmit('o');
		UART_transmit('w');
		UART_transmit(' ');
		UART_transmit('l');
		UART_transmit('o');
		UART_transmit('g');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('1');
		UART_transmit('-');
		UART_transmit('9');
		UART_transmit('b');
		UART_transmit('i');
		UART_transmit('t');
		UART_transmit(' ');
		UART_transmit('t');
		UART_transmit('e');
		UART_transmit('m');
		UART_transmit('p');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('2');
		UART_transmit('-');
		UART_transmit('1');
		UART_transmit('0');
		UART_transmit('b');
		UART_transmit('i');
		UART_transmit('t');
		UART_transmit(' ');
		UART_transmit('t');
		UART_transmit('e');
		UART_transmit('m');
		UART_transmit('p');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('3');
		UART_transmit('-');
		UART_transmit('1');
		UART_transmit('1');
		UART_transmit('b');
		UART_transmit('i');
		UART_transmit('t');
		UART_transmit(' ');
		UART_transmit('t');
		UART_transmit('e');
		UART_transmit('m');
		UART_transmit('p');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('4');
		UART_transmit('-');
		UART_transmit('1');
		UART_transmit('2');
		UART_transmit('b');
		UART_transmit('i');
		UART_transmit('t');
		UART_transmit(' ');
		UART_transmit('t');
		UART_transmit('e');
		UART_transmit('m');
		UART_transmit('p');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit('Q');
		UART_transmit('-');
		UART_transmit('q');
		UART_transmit('u');
		UART_transmit('i');
		UART_transmit('t');
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
		UART_transmit(0x0D); // '\r'
		UART_transmit(0x0A); // '\n'
	}
	
	while(inboot)
	{
		// checking for ERASE command
		if(W25Q64BVerase)
		{
			cli();
			W25Q64BV_release_power_down();
			W25Q64BV_write_enable();
			W25Q64BV_chip_erase();
			while(W25Q64BV_busy());
			W25Q64BVerase = 0;
			W25Q64BVblock1pointer = W25Q64BVblock1add; // setting the pointer to its initial FLASH address
			W25Q64BVblock2pointer = W25Q64BVblock2add; // setting the pointer to its initial FLASH address
			W25Q64BVblock3pointer = W25Q64BVblock3add; // setting the pointer to its initial FLASH address
			W25Q64BVblock4pointer = W25Q64BVblock4add; // setting the pointer to its initial FLASH address
			eeprom_write_byte((uint8_t*)1, W25Q64BVblock1pointer >> 16);
			eeprom_write_byte((uint8_t*)2, W25Q64BVblock1pointer >> 8);
			eeprom_write_byte((uint8_t*)3, W25Q64BVblock1pointer);
			eeprom_write_byte((uint8_t*)5, W25Q64BVblock2pointer >> 16);
			eeprom_write_byte((uint8_t*)6, W25Q64BVblock2pointer >> 8);
			eeprom_write_byte((uint8_t*)7, W25Q64BVblock2pointer);
			eeprom_write_byte((uint8_t*)9, W25Q64BVblock3pointer >> 16);
			eeprom_write_byte((uint8_t*)10, W25Q64BVblock3pointer >> 8);
			eeprom_write_byte((uint8_t*)11, W25Q64BVblock3pointer);
			eeprom_write_byte((uint8_t*)13, W25Q64BVblock4pointer >> 16);
			eeprom_write_byte((uint8_t*)14, W25Q64BVblock4pointer >> 8);
			eeprom_write_byte((uint8_t*)15, W25Q64BVblock4pointer);
			UART_transmit('E'); // signal the end of erase
			UART_transmit('R');
			UART_transmit('A');
			UART_transmit('S');
			UART_transmit('E');
			UART_transmit('D');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			cycleCount = 0;
			sei();
		}
		
		// checking for READ command
		if(W25Q64BVread)
		{
			cli();
			//uint32_t addNum = 8192; // to read the whole chip (8MB) 'x < 32768' in this 4 Block case 'x < 8192', takes about 12.5 minutes
			uint32_t addNum = (W25Q64BVblock1pointer / 256) + 1;
			if(addNum >= 8192) addNum = 8192;
			// read Block 1
			uint8_t evenodd = 0;
			uint16_t number = 0;
			W25Q64BV_release_power_down();
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			for(uint32_t x = 0; x < addNum; x++)
			{
				W25Q64BV_read_data(W25Q64BVbuffer, 0x000000 + (256 * x), 256);
				for(uint16_t i = 0; i <= 255; i++)
				{
					if(!evenodd)
					{
						number = W25Q64BVbuffer[i];
						number <<= 8;
						evenodd = 1;
						if(!normalASCII) UART_transmit(W25Q64BVbuffer[i]);
						}else{
						number |= W25Q64BVbuffer[i];
						evenodd = 0;
						if(normalASCII)
						{
							uint8_t ascii1 = 0; // 16bit numbers - 5 decimal places max
							uint8_t ascii2 = 0;
							uint8_t ascii3 = 0;
							uint8_t ascii4 = 0;
							uint8_t ascii5 = 0;
							uint16_t num = number;
							ascii1 = num / 10000;
							num = number;
							ascii2 = (num - (ascii1 * 10000)) / 1000;
							num = number;
							ascii3 = (num - (ascii1 * 10000) - (ascii2 * 1000)) / 100;
							num = number;
							ascii4 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) / 10;
							num = number;
							ascii5 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) % 10;
							ascii1 += '0';
							ascii2 += '0';
							ascii3 += '0';
							ascii4 += '0';
							ascii5 += '0';
							if(number >= 10000)UART_transmit(ascii1);
							if(number >= 1000)UART_transmit(ascii2);
							if(number >= 100)UART_transmit(ascii3);
							if(number >= 10)UART_transmit(ascii4);
							UART_transmit(ascii5);
							UART_transmit(0x0D); // '\r'
							UART_transmit(0x0A); // '\n'
						}else{
							UART_transmit(W25Q64BVbuffer[i]);
						}
						number = 0;
					}
				}
			}
			// read Block 2
			evenodd = 0;
			number = 0;
			for(uint32_t x = 0; x < addNum; x++)
			{
				W25Q64BV_read_data(W25Q64BVbuffer, 0x200000 + (256 * x), 256);
				for(uint16_t i = 0; i <= 255; i++)
				{
					if(!evenodd)
					{
						number = W25Q64BVbuffer[i];
						number <<= 8;
						evenodd = 1;
						if(!normalASCII) UART_transmit(W25Q64BVbuffer[i]);
						}else{
						number |= W25Q64BVbuffer[i];
						evenodd = 0;
						if(normalASCII)
						{
							uint8_t ascii1 = 0; // 16bit numbers - 5 decimal places max
							uint8_t ascii2 = 0;
							uint8_t ascii3 = 0;
							uint8_t ascii4 = 0;
							uint8_t ascii5 = 0;
							uint16_t num = number;
							ascii1 = num / 10000;
							num = number;
							ascii2 = (num - (ascii1 * 10000)) / 1000;
							num = number;
							ascii3 = (num - (ascii1 * 10000) - (ascii2 * 1000)) / 100;
							num = number;
							ascii4 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) / 10;
							num = number;
							ascii5 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) % 10;
							ascii1 += '0';
							ascii2 += '0';
							ascii3 += '0';
							ascii4 += '0';
							ascii5 += '0';
							if(number >= 10000)UART_transmit(ascii1);
							if(number >= 1000)UART_transmit(ascii2);
							if(number >= 100)UART_transmit(ascii3);
							if(number >= 10)UART_transmit(ascii4);
							UART_transmit(ascii5);
							UART_transmit(0x0D); // '\r'
							UART_transmit(0x0A); // '\n'
						}else{
							UART_transmit(W25Q64BVbuffer[i]);
						}
						number = 0;
					}
				}
			}
			// read Block 3
			evenodd = 0;
			number = 0;
			for(uint32_t x = 0; x < addNum; x++)
			{
				W25Q64BV_read_data(W25Q64BVbuffer, 0x400000 + (256 * x), 256);
				for(uint16_t i = 0; i <= 255; i++)
				{
					if(!evenodd)
					{
						number = W25Q64BVbuffer[i];
						number <<= 8;
						evenodd = 1;
						if(!normalASCII) UART_transmit(W25Q64BVbuffer[i]);
						}else{
						number |= W25Q64BVbuffer[i];
						evenodd = 0;
						if(normalASCII)
						{
							uint8_t ascii1 = 0; // 16bit numbers - 5 decimal places max
							uint8_t ascii2 = 0;
							uint8_t ascii3 = 0;
							uint8_t ascii4 = 0;
							uint8_t ascii5 = 0;
							uint16_t num = number;
							ascii1 = num / 10000;
							num = number;
							ascii2 = (num - (ascii1 * 10000)) / 1000;
							num = number;
							ascii3 = (num - (ascii1 * 10000) - (ascii2 * 1000)) / 100;
							num = number;
							ascii4 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) / 10;
							num = number;
							ascii5 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) % 10;
							ascii1 += '0';
							ascii2 += '0';
							ascii3 += '0';
							ascii4 += '0';
							ascii5 += '0';
							if(number >= 10000)UART_transmit(ascii1);
							if(number >= 1000)UART_transmit(ascii2);
							if(number >= 100)UART_transmit(ascii3);
							if(number >= 10)UART_transmit(ascii4);
							UART_transmit(ascii5);
							UART_transmit(0x0D); // '\r'
							UART_transmit(0x0A); // '\n'
						}else{
							UART_transmit(W25Q64BVbuffer[i]);
						}
						number = 0;
					}
				}
			}
			// read Block 4
			evenodd = 0;
			number = 0;
			for(uint32_t x = 0; x < addNum; x++)
			{
				W25Q64BV_read_data(W25Q64BVbuffer, 0x600000 + (256 * x), 256);
				for(uint16_t i = 0; i <= 255; i++)
				{
					if(!evenodd)
					{
						number = W25Q64BVbuffer[i];
						number <<= 8;
						evenodd = 1;
						if(!normalASCII) UART_transmit(W25Q64BVbuffer[i]);
						}else{
						number |= W25Q64BVbuffer[i];
						evenodd = 0;
						if(normalASCII)
						{
							uint8_t ascii1 = 0; // 16bit numbers - 5 decimal places max
							uint8_t ascii2 = 0;
							uint8_t ascii3 = 0;
							uint8_t ascii4 = 0;
							uint8_t ascii5 = 0;
							uint16_t num = number;
							ascii1 = num / 10000;
							num = number;
							ascii2 = (num - (ascii1 * 10000)) / 1000;
							num = number;
							ascii3 = (num - (ascii1 * 10000) - (ascii2 * 1000)) / 100;
							num = number;
							ascii4 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) / 10;
							num = number;
							ascii5 = (num - (ascii1 * 10000) - (ascii2 * 1000) - (ascii3 * 100)) % 10;
							ascii1 += '0';
							ascii2 += '0';
							ascii3 += '0';
							ascii4 += '0';
							ascii5 += '0';
							if(number >= 10000)UART_transmit(ascii1);
							if(number >= 1000)UART_transmit(ascii2);
							if(number >= 100)UART_transmit(ascii3);
							if(number >= 10)UART_transmit(ascii4);
							UART_transmit(ascii5);
							UART_transmit(0x0D); // '\r'
							UART_transmit(0x0A); // '\n'
						}else{
							UART_transmit(W25Q64BVbuffer[i]);
						}
						number = 0;
					}
				}
			}
			W25Q64BVread = 0;
			UART_transmit('R'); // signal the end of READ
			UART_transmit('E'); // signal the end of READ
			UART_transmit('A'); // signal the end of READ
			UART_transmit('D'); // signal the end of READ
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			sei();
		}
		
		// checking for DEBUG command
		if(W25Q64BVdebug)
		{
			cli();
			ADC_wake_up();
			FLASHvalue1 = ADC_read_11ref(1); // SOLAR
			FLASHvalue2 = ADC_read_11ref(0); // LTC3105 voltage
			FLASHvalue3 = ADC_read_11ref(3); // LTC3105 current
			ADC_sleep();
			FLASHvalue4 = DS18B20_get_temperature();
			if(normalASCII)
			{
				uint32_t add1 = eeprom_read_byte((uint8_t*)1);
				add1 <<= 16;
				add1 |= (eeprom_read_byte((uint8_t*)2) << 8);
				add1 |= (eeprom_read_byte((uint8_t*)3));
				uint32_t add2 = eeprom_read_byte((uint8_t*)5);
				add2 <<= 16;
				add2 |= (eeprom_read_byte((uint8_t*)6) << 8);
				add2 |= (eeprom_read_byte((uint8_t*)7));
				uint32_t add3 = eeprom_read_byte((uint8_t*)9);
				add3 <<= 16;
				add3 |= (eeprom_read_byte((uint8_t*)10) << 8);
				add3 |= (eeprom_read_byte((uint8_t*)11));
				uint32_t add4 = eeprom_read_byte((uint8_t*)13);
				add4 <<= 16;
				add4 |= (eeprom_read_byte((uint8_t*)14) << 8);
				add4 |= (eeprom_read_byte((uint8_t*)15));
				watchDogMask = eeprom_read_byte((uint8_t*)16);
				TEMPRES = eeprom_read_byte((uint8_t*)17);
				normalASCII = eeprom_read_byte((uint8_t*)18);
				UART_transmit('D');
				UART_transmit('E');
				UART_transmit('B');
				UART_transmit('U');
				UART_transmit('G');
				UART_transmit(0x0D); // '\r'
				UART_transmit(0x0A); // '\n'
				ASCII_32bit_transmit(add1);
				ASCII_32bit_transmit(add2);
				ASCII_32bit_transmit(add3);
				ASCII_32bit_transmit(add4);
				ASCII_32bit_transmit(W25Q64BVblock1pointer);
				ASCII_32bit_transmit(W25Q64BVblock2pointer);
				ASCII_32bit_transmit(W25Q64BVblock3pointer);
				ASCII_32bit_transmit(W25Q64BVblock4pointer);
				ASCII_16bit_transmit(FLASHvalue1);
				ASCII_16bit_transmit(FLASHvalue2);
				ASCII_16bit_transmit(FLASHvalue3);
				ASCII_16bit_transmit(FLASHvalue4);
				if(watchDogMask == WD_S8) // FAST or SLOW LOGGING mode
				{
					UART_transmit('8');
					UART_transmit('s');
				}else if(watchDogMask == WD_S025){
					UART_transmit('0');
					UART_transmit('.');
					UART_transmit('2');
					UART_transmit('5');
					UART_transmit('s');
				}else UART_transmit('?');
				UART_transmit(0x0D); // '\r'
				UART_transmit(0x0A); // '\n'
				if(TEMPRES == TEMP_RES_9) // DS18B20 resolution
				{
					UART_transmit('9');
					UART_transmit('-');
					UART_transmit('b');
					UART_transmit('i');
					UART_transmit('t');
				}else if(TEMPRES == TEMP_RES_10){
					UART_transmit('1');
					UART_transmit('0');
					UART_transmit('-');
					UART_transmit('b');
					UART_transmit('i');
					UART_transmit('t');
				}else if(TEMPRES == TEMP_RES_11){
					UART_transmit('1');
					UART_transmit('1');
					UART_transmit('-');
					UART_transmit('b');
					UART_transmit('i');
					UART_transmit('t');
				}else if(TEMPRES == TEMP_RES_12){
					UART_transmit('1');
					UART_transmit('2');
					UART_transmit('-');
					UART_transmit('b');
					UART_transmit('i');
					UART_transmit('t');
				}else UART_transmit('?');
				UART_transmit(0x0D); // '\r'
				UART_transmit(0x0A); // '\n'
				if(normalASCII) UART_transmit('A'); // ASCII or HEX output
				else UART_transmit('N');
				UART_transmit(0x0D); // '\r'
				UART_transmit(0x0A); // '\n'
			}else{
				UART_transmit(eeprom_read_byte((uint8_t*)1));
				UART_transmit(eeprom_read_byte((uint8_t*)2));
				UART_transmit(eeprom_read_byte((uint8_t*)3));
				UART_transmit(eeprom_read_byte((uint8_t*)5));
				UART_transmit(eeprom_read_byte((uint8_t*)6));
				UART_transmit(eeprom_read_byte((uint8_t*)7));
				UART_transmit(eeprom_read_byte((uint8_t*)9));
				UART_transmit(eeprom_read_byte((uint8_t*)10));
				UART_transmit(eeprom_read_byte((uint8_t*)11));
				UART_transmit(eeprom_read_byte((uint8_t*)13));
				UART_transmit(eeprom_read_byte((uint8_t*)14));
				UART_transmit(eeprom_read_byte((uint8_t*)15));
				UART_transmit(W25Q64BVblock1pointer >> 16);
				UART_transmit(W25Q64BVblock1pointer >> 8);
				UART_transmit(W25Q64BVblock1pointer);
				UART_transmit(W25Q64BVblock2pointer >> 16);
				UART_transmit(W25Q64BVblock2pointer >> 8);
				UART_transmit(W25Q64BVblock2pointer);
				UART_transmit(W25Q64BVblock3pointer >> 16);
				UART_transmit(W25Q64BVblock3pointer >> 8);
				UART_transmit(W25Q64BVblock3pointer);
				UART_transmit(W25Q64BVblock4pointer >> 16);
				UART_transmit(W25Q64BVblock4pointer >> 8);
				UART_transmit(W25Q64BVblock4pointer);
				UART_transmit(FLASHvalue1 >> 8);
				UART_transmit(FLASHvalue1);
				UART_transmit(FLASHvalue2 >> 8);
				UART_transmit(FLASHvalue2);
				UART_transmit(FLASHvalue3 >> 8);
				UART_transmit(FLASHvalue3);
				UART_transmit(FLASHvalue4 >> 8);
				UART_transmit(FLASHvalue4);
			}
			W25Q64BVdebug = 0;
			sei();
		}
		
		// checking for ASCII command
		if(ASCIIch)
		{
			UART_transmit('A');
			UART_transmit('S');
			UART_transmit('C');
			UART_transmit('I');
			UART_transmit('I');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			normalASCII = 1;
			eeprom_write_byte((uint8_t*)18, normalASCII);
			ASCIIch = 0;
		}
		
		// checking for NORMAL command
		if(NORMALch)
		{
			UART_transmit('H');
			UART_transmit('E');
			UART_transmit('X');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			normalASCII = 0;
			eeprom_write_byte((uint8_t*)18, normalASCII);
			NORMALch = 0;
		}
		
		// checking for QUIT BOOT command
		if(!boot)
		{
			UART_transmit('Q');
			UART_transmit('U');
			UART_transmit('I');
			UART_transmit('T');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			inboot = 0;
		}
		
		// checking for TEMP RES 9 command
		if(TEMPres9)
		{
			UART_transmit('1');
			UART_transmit('-');
			DS18B20_resolution(TEMP_RES_9); // 0b0xx11111 Bits 6:5 - 00 9-bit (93.75ms), 01 10-bit (187.5ms), 10 11-bit (375ms), 11 12-bit (750ms)
			TEMPRES = TEMP_RES_9;
			eeprom_write_byte((uint8_t*)17, TEMPRES);
			TEMPres9 = 0;
		}
		
		// checking for TEMP RES 10 command
		if(TEMPres10)
		{
			UART_transmit('2');
			UART_transmit('-');
			DS18B20_resolution(TEMP_RES_10); // 0b0xx11111 Bits 6:5 - 00 9-bit (93.75ms), 01 10-bit (187.5ms), 10 11-bit (375ms), 11 12-bit (750ms)
			TEMPRES = TEMP_RES_10;
			eeprom_write_byte((uint8_t*)17, TEMPRES);
			TEMPres10 = 0;
		}
		
		// checking for TEMP RES 11 command
		if(TEMPres11)
		{
			UART_transmit('3');
			UART_transmit('-');
			DS18B20_resolution(TEMP_RES_11); // 0b0xx11111 Bits 6:5 - 00 9-bit (93.75ms), 01 10-bit (187.5ms), 10 11-bit (375ms), 11 12-bit (750ms)
			TEMPRES = TEMP_RES_11;
			eeprom_write_byte((uint8_t*)17, TEMPRES);
			TEMPres11 = 0;
		}
		
		// checking for TEMP RES 12 command
		if(TEMPres12)
		{
			UART_transmit('4');
			UART_transmit('-');
			DS18B20_resolution(TEMP_RES_12); // 0b0xx11111 Bits 6:5 - 00 9-bit (93.75ms), 01 10-bit (187.5ms), 10 11-bit (375ms), 11 12-bit (750ms)
			TEMPRES = TEMP_RES_12;
			eeprom_write_byte((uint8_t*)17, TEMPRES);
			TEMPres12 = 0;
		}
		
		// checking for FAST LOG command
		if(FASTlog)
		{
			UART_transmit('F');
			UART_transmit('A');
			UART_transmit('S');
			UART_transmit('T');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			watchDogMask = WD_S025;
			EEPROMupdate = 100;
			eeprom_write_byte((uint8_t*)16, watchDogMask);
			FASTlog = 0;
		}
		
		// checking for SLOW LOG command
		if(SLOWlog)
		{
			UART_transmit('S');
			UART_transmit('L');
			UART_transmit('O');
			UART_transmit('W');
			UART_transmit(0x0D); // '\r'
			UART_transmit(0x0A); // '\n'
			watchDogMask = WD_S8;
			EEPROMupdate = 10;
			eeprom_write_byte((uint8_t*)16, watchDogMask);
			SLOWlog = 0;
		}
	}
	
	UCSR0B &= ~(1 << RXCIE0); // disable RX complete interrupt
	cli();
	
	while(1)
    {
		// when full, stop writing to FLASH
		if(W25Q64BVblock1pointer >= 0x200000 || W25Q64BVblock2pointer >= 0x400000 || W25Q64BVblock3pointer >= 0x600000 || W25Q64BVblock4pointer >= 0x800000) continue;
		
		// prepare the values / do the measurements
		ADC_wake_up();
		FLASHvalue1 = ADC_read_11ref(1); // SOLAR
		FLASHvalue2 = ADC_read_11ref(0); // LTC3105 voltage
		FLASHvalue3 = ADC_read_11ref(3); // LTC3105 current
		ADC_sleep();
		FLASHvalue4 = DS18B20_get_temperature();
		
		// Power-up the FLASH
		W25Q64BV_release_power_down();
		
		// write to FLASH Block 1
		W25Q64BVblock1buff[0] = (FLASHvalue1 >> 8);
		W25Q64BVblock1buff[1] = FLASHvalue1;
		W25Q64BV_write_enable();
		W25Q64BV_write_data(W25Q64BVblock1buff, W25Q64BVblock1pointer, 2);
		W25Q64BVblock1pointer += 2;
		while(W25Q64BV_busy());
		
		// write to FLASH Block 2
		W25Q64BVblock2buff[0] = (FLASHvalue2 >> 8);
		W25Q64BVblock2buff[1] = FLASHvalue2;
		W25Q64BV_write_enable();
		W25Q64BV_write_data(W25Q64BVblock2buff, W25Q64BVblock2pointer, 2);
		W25Q64BVblock2pointer += 2;
		while(W25Q64BV_busy());
		
		// write to FLASH Block 3
		W25Q64BVblock3buff[0] = (FLASHvalue3 >> 8);
		W25Q64BVblock3buff[1] = FLASHvalue3;
		W25Q64BV_write_enable();
		W25Q64BV_write_data(W25Q64BVblock3buff, W25Q64BVblock3pointer, 2);
		W25Q64BVblock3pointer += 2;
		while(W25Q64BV_busy());
		
		// write to FLASH Block 4
		W25Q64BVblock4buff[0] = (FLASHvalue4 >> 8);
		W25Q64BVblock4buff[1] = FLASHvalue4;
		W25Q64BV_write_enable();
		W25Q64BV_write_data(W25Q64BVblock4buff, W25Q64BVblock4pointer, 2);
		W25Q64BVblock4pointer += 2;
		while(W25Q64BV_busy());
		
		// Power-down the FLASH
		W25Q64BV_power_down();
		
		// every X cycles update the EEPROM pointer values
		cycleCount++;
		if(cycleCount >= EEPROMupdate) // 10 for 8s updates (every 80s - 1080 per day)
		{
			eeprom_write_byte((uint8_t*)1, W25Q64BVblock1pointer >> 16);
			eeprom_write_byte((uint8_t*)2, W25Q64BVblock1pointer >> 8);
			eeprom_write_byte((uint8_t*)3, W25Q64BVblock1pointer);
			eeprom_write_byte((uint8_t*)5, W25Q64BVblock2pointer >> 16);
			eeprom_write_byte((uint8_t*)6, W25Q64BVblock2pointer >> 8);
			eeprom_write_byte((uint8_t*)7, W25Q64BVblock2pointer);
			eeprom_write_byte((uint8_t*)9, W25Q64BVblock3pointer >> 16);
			eeprom_write_byte((uint8_t*)10, W25Q64BVblock3pointer >> 8);
			eeprom_write_byte((uint8_t*)11, W25Q64BVblock3pointer);
			eeprom_write_byte((uint8_t*)13, W25Q64BVblock4pointer >> 16);
			eeprom_write_byte((uint8_t*)14, W25Q64BVblock4pointer >> 8);
			eeprom_write_byte((uint8_t*)15, W25Q64BVblock4pointer);
			cycleCount = 0;
		}
		
		//_delay_ms(2); // to finish UART_transmit before SLEEP
		WatchDog_setup_and_sleep();
    }
}