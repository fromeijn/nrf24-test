//       _____ ____          //
//      |___  |  _ \         //
//         _| | |_) |        //
//        |_  |  _ <         //
//          |_|_| \_\        //
//                           //
// Written By Floris Romeijn //


#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <stdio.h>

#include "clksys_driver.h"
#define ENABLE_UART_F0    	1
#include "uart.h"
#include "usart_driver.h"
#include "nrf24_reg.h"

//#define PROTO 

#ifdef PROTO

#define LED_ROOD_ON PORTF.OUTSET = PIN0_bm
#define LED_GROEN_ON PORTC.OUTSET = PIN0_bm
#define LED_BLAUW_ON PORTF.OUTSET = PIN1_bm

#define LED_ROOD_OFF PORTF.OUTCLR = PIN0_bm
#define LED_GROEN_OFF PORTC.OUTCLR = PIN0_bm
#define LED_BLAUW_OFF PORTF.OUTCLR = PIN1_bm

#else

#define LED_ROOD_ON  TCF0.CCBBUF = 250//PORTF.OUTSET = PIN1_bm
#define LED_GROEN_ON TCF0.CCABUF = 250//PORTF.OUTSET = PIN0_bm
#define LED_BLAUW_ON TCC0.CCABUF = 1000//PORTC.OUTSET = PIN0_bm

#define LED_ROOD_OFF  TCF0.CCBBUF = 20//PORTF.OUTCLR = PIN1_bm
#define LED_GROEN_OFF TCF0.CCABUF = 20//PORTF.OUTCLR = PIN0_bm
#define LED_BLAUW_OFF TCC0.CCABUF = 50//PORTC.OUTCLR = PIN0_bm

#endif


#define BSCALE_SPI_BAUD		0
#define BSEL_SPI_BAUD		1


#define PAYLOAD_SIZE 1

#define CHECK_ROOD  0b10101010;
#define CHECK_GROEN 0b01010101;

void Init32MhzFrom16MhzExternal(void);
void EnableAllInterupts(void);

void Nrf24InitSpi(void);
void InitSpiUsart(void);
void Nrf24ReadWriteReg(uint8_t command, uint8_t reg, uint8_t* data, uint8_t len);
uint8_t Nrf24ReadReg(uint8_t reg);
void Nrf24WriteReg(uint8_t reg, uint8_t data);
uint8_t Nrf24CheckRegs();
void Nrf24RxState(void);
void Nrf24SetPayloadSize(uint8_t payloadsize);
uint8_t Nrf24RxDataReady();
void Nrf24FlushTX(void);
void Nrf24FlushRX(void);
void InitLEDS(void);


char str[256];

int main(void)
{
	Init32MhzFrom16MhzExternal();
	Nrf24InitSpi();
	InitLEDS();
	EnableAllInterupts();

	init_uart(&uartF0, &USARTF0, F_CPU, 230400, 0);
	sprintf(str, "\n\r\n\rnRF24 TestBoard\n\r");
  	uart_puts(&uartF0, str);

  	if(Nrf24CheckRegs()>=8)
  	{
  		LED_GROEN_ON;
  		Nrf24SetPayloadSize(PAYLOAD_SIZE);
  		Nrf24RxState();
  	}
  	else
  	{
  		while(1){
  			LED_ROOD_ON;
  			_delay_ms(250);
  			LED_ROOD_OFF;
  			_delay_ms(250);
  		}
  	}

	while(1)
	{
		LED_ROOD_ON;
		if(Nrf24RxDataReady())
			LED_BLAUW_ON;

		PORTA.OUT = CHECK_ROOD;
		PORTB.OUT = CHECK_ROOD;
		PORTD.OUT = CHECK_ROOD;
		PORTE.OUT = CHECK_ROOD;
		LED_GROEN_OFF;
		_delay_ms(250);

		PORTA.OUT = CHECK_GROEN;
		PORTB.OUT = CHECK_GROEN;
		PORTD.OUT = CHECK_GROEN;
		PORTE.OUT = CHECK_GROEN;
		LED_BLAUW_OFF;
		LED_GROEN_ON;
		_delay_ms(250);
	}
}

ISR(PORTF_INT0_vect) //nrf24 interrupt
{
	
}


void InitSpiUsart(void)
{
	
	RF24_USART_SPI_PORT.DIRSET = RF24_MOSI_PIN | RF24_CLK_PIN;
	RF24_USART_SPI_PORT.DIRCLR = RF24_MISO_PIN;
	
	USARTC0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	USARTC0.CTRLC = USART_CMODE_MSPI_gc;
	USARTC0.BAUDCTRLA = (BSEL_SPI_BAUD & USART_BSEL_gm);
	USARTC0.BAUDCTRLB = ((BSCALE_SPI_BAUD << USART_BSCALE_gp) & USART_BSCALE_gm) | ((BSEL_SPI_BAUD >> 8) & ~USART_BSCALE_gm);
	
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void Nrf24InitSpi(void)
{
	RF24_OTHERS_PORT.DIRSET = RF24_CE_PIN | RF24_NCS_PIN;
	RF24_OTHERS_PORT.OUTSET = RF24_CE_PIN | RF24_NCS_PIN;

	RF24_OTHERS_PORT.DIRCLR = RF24_IRQ_PIN;
	RF24_OTHERS_PORT.INT0MASK = RF24_IRQ_PIN;
	RF24_OTHERS_PORT.INTCTRL = PORT_INT0LVL_LO_gc;

	InitSpiUsart();
	RF24_SS_CLR;
}

uint8_t Nrf24RxDataReady()
{
	if(!(Nrf24ReadReg(NRF_FIFO_STATUS) & NRF_RX_EMPTY_bm))
		return 1;

	return 0;
}

void Nrf24RxState(void)
{
	Nrf24FlushTX();
	Nrf24FlushRX();

	Nrf24WriteReg(NRF_STATUS, NRF_RX_DR_bm | NRF_TX_DS_bm | NRF_MAX_RT_bm);
	Nrf24WriteReg(NRF_CONFIG, NRF_PRIM_RX_bm | NRF_PWR_UP_bm | NRF_EN_CRC_bm);
}

void Nrf24TxState(void)
{
	Nrf24FlushTX();
	Nrf24FlushRX();

	Nrf24WriteReg(NRF_STATUS, NRF_RX_DR_bm | NRF_TX_DS_bm | NRF_MAX_RT_bm);
	Nrf24WriteReg(NRF_CONFIG, NRF_PWR_UP_bm | NRF_EN_CRC_bm);
}

void Nrf24SetPayloadSize(uint8_t payloadsize)
{
	Nrf24WriteReg(NRF_RX_PW_P0, payloadsize);
	Nrf24WriteReg(NRF_RX_PW_P1, payloadsize);
}

void Nrf24FlushTX(void)
{
	uint8_t buffer[1];
	Nrf24ReadWriteReg(NRF_FLUSH_TX, 0x11, buffer, 0);
}

void Nrf24FlushRX(void)
{
	uint8_t buffer[1];
	Nrf24ReadWriteReg(NRF_FLUSH_RX, 0x11, buffer, 0);
}

uint8_t Nrf24CheckRegs()
{
	uint8_t default_reset_nrf24[10];
	default_reset_nrf24[0] = 0b1000;
	default_reset_nrf24[1] = 0b111111;
	default_reset_nrf24[2] = 0b11;
	default_reset_nrf24[3] = 0b11;
	default_reset_nrf24[4] = 0b11;
	default_reset_nrf24[5] = 0b10;
	default_reset_nrf24[6] = 0b1111;
	default_reset_nrf24[7] = 0b1110;
	default_reset_nrf24[8] = 0b0;
	default_reset_nrf24[9] = 0b0;

	//write default (if for somereason the chip was not in reset-state)
	for (int i = 0; i < 10; i++)
	{
		Nrf24WriteReg(i, default_reset_nrf24[i]);
	}

	//read values back
	uint8_t check = 0;
	for (int i = 0; i < 10; i++)
	{
		if(Nrf24ReadReg(i) == default_reset_nrf24[i])
			check++;
	}

	return check;
}


void Nrf24ReadWriteReg(uint8_t command, uint8_t reg, uint8_t* data, uint8_t len)
{
	uint8_t cnt = 0;
	RF24_SS_SET;

	USARTC0.DATA = command | (reg & 0x1F);

	while(!(USARTC0.STATUS & USART_RXCIF_bm));
	data[cnt] = USARTC0.DATA;
	cnt++;

	while(cnt<len)
	{
		USARTC0.DATA = data[cnt];
		while(!(USARTC0.STATUS & USART_RXCIF_bm));
		data[cnt] = USARTC0.DATA;
		cnt++;
	}

	RF24_SS_CLR;
}

uint8_t Nrf24ReadReg(uint8_t reg)
{
	uint8_t buffer[2];
	Nrf24ReadWriteReg(NRF_R_REGISTER, reg, buffer, 2);
	return buffer[1];
}

void Nrf24WriteReg(uint8_t reg, uint8_t data)
{
	uint8_t buffer[2];
	buffer[1] = data;
	Nrf24ReadWriteReg(NRF_W_REGISTER, reg, buffer, 2);
}

void InitLEDS(void)
{
	PORTC.DIRSET = PIN0_bm;
	PORTF.DIRSET = PIN0_bm | PIN1_bm;

	/* Init the timer/PWM and associated I/Os */
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_SINGLESLOPE_gc; /* PWM on CCA */
	/* CTRLC is of no interest to us */
	TCC0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc; /* No events */
	TCC0.CTRLE = TC_BYTEM_NORMAL_gc; /* No byte mode */
	TCC0.PER = 5000;
	TCC0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc; /* All timer interrupts off */
	TCC0.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCDINTLVL_OFF_gc; /* Disable Compare/Capture interrupts */
	TCC0.CNT = 0;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; /* Start the timer with a clock divider of 1 */

	/* Use TCF0 to drive the other two LED colours */
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC_WGMODE_SINGLESLOPE_gc; /* PWM on CCA and CCB */
	/* No need to modify CTRLC */
	TCF0.CTRLD = TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
	TCF0.CTRLE = TC_BYTEM_NORMAL_gc;
	TCF0.INTCTRLA = TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	TCF0.INTCTRLB = TC_CCAINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCDINTLVL_OFF_gc;
	TCF0.CTRLFCLR = TC0_DIR_bm;
	TCF0.PER = 5000;

	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; /* Start the timer with a clock divider of 1 */

	TCC0.CCABUF = 0;
	TCF0.CCABUF = 0;
	TCF0.CCBBUF = 0;

	PORTA.OUT = 0x00;
	PORTB.OUT = 0x00;
	PORTD.OUT = 0x00;
	PORTE.OUT = 0x00;

	PORTA.DIRSET = 0xFF;
	PORTB.DIRSET = 0xFF;
	PORTD.DIRSET = 0xFF;
	PORTE.DIRSET = 0xFF;

}

void Init32MhzFrom16MhzExternal(void)
{
	CLKSYS_XOSC_Config( OSC_FRQRANGE_12TO16_gc, 0, OSC_XOSCSEL_XTAL_16KCLK_gc );
	CLKSYS_Enable( OSC_XOSCEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );

	CLKSYS_PLL_Config( OSC_PLLSRC_XOSC_gc, 2);
	CLKSYS_Enable( OSC_PLLEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );

	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
	CLKSYS_Disable( OSC_RC2MEN_bm );
	CLKSYS_Disable( OSC_RC32MEN_bm );
}

void EnableAllInterupts(void)
{
  PMIC.CTRL |= PMIC_LOLVLEN_bm;
  PMIC.CTRL |= PMIC_MEDLVLEN_bm;
  PMIC.CTRL |= PMIC_HILVLEN_bm;
  sei();
}