/**
 * File: usci_spi.c - msp430 USCI SPI implementation
 *
 * Copyright (c) 2012 by Rick Kimball <rick@kimballsoftware.com>
 * spi abstraction api for msp430
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 */

#include <msp430.h>
#include <stdint.h>
#include "pga460_spi_430_g2553.h"
#include <Energia.h>

// AW: re-allocate SPI port from B0 pins
static const uint8_t PGA460_SS      = 8;
static const uint8_t PGA460_SCK     = 7;
static const uint8_t PGA460_MOSI    = 15;
static const uint8_t PGA460_MISO    = 14;

//#if defined(__MSP430_HAS_USCI_B0__) || defined(__MSP430_HAS_USCI_B1__) || defined(__MSP430_HAS_USCI__)

/**
 * USCI flags for various the SPI MODEs
 *
 * Note: The msp430 UCCKPL tracks the CPOL value. However,
 * the UCCKPH flag is inverted when compared to the CPHA
 * value described in Motorola documentation.
 */

#define SPI_MODE_0 (UCCKPH)			    /* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)                 	/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)    /* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)			    /* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

/**
 * spi_initialize_pga460() - Configure USCI UCB0 for SPI mode
 *
 * P2.0 - CS (active low)	// AW --> G2553
 * P1.5 - SCLK 		 	// AW --> G2553
 * P1.6 - MISO aka SOMI 	// AW --> G2553
 * P1.7 - MOSI aka SIMO 	// AW --> G2553
 *
 */

/* Calculate divisor to keep SPI clock close to 4MHz but never over */
#ifndef SPI_CLOCK_SPEED
#define SPI_CLOCK_SPEED 4000000L
#endif

#if F_CPU < 4000000L
#define SPI_CLOCK_DIV() 1 
#else
#define SPI_CLOCK_DIV() ((F_CPU / SPI_CLOCK_SPEED) + (F_CPU % SPI_CLOCK_SPEED == 0 ? 0:1))
#endif

#define SPI_CLOCK_DIV_DEFAULT (F_CPU / 4)

void spi_initialize_pga460_g2553(void)
{
	UCB0CTL1 = UCSWRST | UCSSEL_2;      // Put USCI in reset mode, source USCI clock from SMCLK
	UCB0CTL0 = SPI_MODE_0 | UCMSB | UCSYNC | UCMST;  // Use SPI MODE 0 - CPOL=0 CPHA=0

 	/* Set pins to SPI mode. */
	pinMode_int(PGA460_SCK, PORT_SELECTION0 | SPISCK_SET_MODE);//(PM_UCB0CLK << 8));//SPISCK_SET_MODE);
	pinMode_int(PGA460_MOSI, PORT_SELECTION0 | SPIMOSI_SET_MODE);//(PM_UCB0SDA << 8));//SPIMOSI_SET_MODE);
	pinMode_int(PGA460_MISO, PORT_SELECTION0 | SPIMISO_SET_MODE);//(PM_UCB0SCL << 8));//SPIMISO_SET_MODE);

	UCB0BR0 = SPI_CLOCK_DIV() & 0xFF;   // set initial speed to 4MHz
	UCB0BR1 = (SPI_CLOCK_DIV() >> 8 ) & 0xFF;

	UCB0CTL1 &= ~UCSWRST;			    // release USCI for operation
}

/**
 * spi_disable_pga460() - put USCI into reset mode
 */
void spi_disable_pga460_g2553(void)
{
    UCB0CTL1 |= UCSWRST;                // Put USCI in reset mode
}

/**
 * spi_send() - send a byte and recv response
 */
uint8_t spi_send_pga460_g2553(const uint8_t _data)
{
	UCB0TXBUF = _data; // setting TXBUF clears the TXIFG flag
	while (UCB0STAT & UCBUSY)
		; // wait for SPI TX/RX to finish

	return UCB0RXBUF; // reading clears RXIFG flag
}

/***SPI_MODE_0
 * spi_set_divisor_pga460() - set new clock divider for USCI
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1
 *
 */
void spi_set_divisor_pga460_g2553(const uint16_t clkdiv)
{
	UCB0CTL1 |= UCSWRST;		// go into reset state
	UCB0BR0 = clkdiv & 0xFF;
	UCB0BR1 = (clkdiv >> 8 ) & 0xFF;
	UCB0CTL1 &= ~UCSWRST;		// release for operation
}

/**
 * spi_set_bitorder_pga460(LSBFIRST=0 | MSBFIRST=1)
 */
void spi_set_bitorder_pga460_g2553(const uint8_t order)
{
    UCB0CTL1 |= UCSWRST;        // go into reset state
    UCB0CTL0 = (UCB0CTL0 & ~UCMSB) | ((order == 1 /*MSBFIRST*/) ? UCMSB : 0); /* MSBFIRST = 1 */
    UCB0CTL1 &= ~UCSWRST;       // release for operation
}

/**
 * spi_set_datamode_pga460() - mode 0 - 3
 */
void spi_set_datamode_pga460_g2553(const uint8_t mode)
{
    UCB0CTL1 |= UCSWRST;        // go into reset state
    switch(mode) {
    case 0: /* SPI_MODE0 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_0;
        break;
    case 1: /* SPI_MODE1 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_1;
        break;
    case 2: /* SPI_MODE2 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_2;
        break;
    case 4: /* SPI_MODE3 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_3;
        break;
    default:
        break;
    }
    UCB0CTL1 &= ~UCSWRST;       // release for operation
}
//#else
    //#error "Error! This device doesn't have a USCI peripheral"
//#endif
