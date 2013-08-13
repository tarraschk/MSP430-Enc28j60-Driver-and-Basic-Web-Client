#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "msp430.h"
#include "msp430f5529.h"
#include "spi.h"
#include "enc28j60.h"

/* Modified by Duncan Frost & adapted by Maxime Alay-Eddine
 *
 */

#define SCLK    BIT3
#define SDI     BIT2
#define SDO     BIT1
#define CS      BIT0

#define TRUE  1
#define FALSE 0

/** MACRO for selecting or deselecting chip select for the ENC28J60. Some HW dependancy.*/
#define SEL_MAC(x)  (x==TRUE) ? (P4OUT&=(~CS)) : (P4OUT|=CS)  

/***********************************************************************/
/** \brief Initilialise the SPI peripheral
 *
 * Description: init the SPI peripheral. 
 */
/**********************************************************************/
void initSPI(void)
{
  UCB1CTL1 = UCSWRST;
  // Most signigicant bit, Mode ( 0, 0 ), Master, Syncronous mode, two stop bits
  UCB1CTL0 = UCSYNC + UCMST + UCMSB + UCCKPH;
  // Use CPU clk (SMCLK)
  UCB1CTL1 |= UCSSEL_2;
  // No division of CPU clk
  UCB1BR0 = 0x1;
  UCB1BR1 = 0;
  // Using these ports for SPI
  /*P3SEL |= SCLK + SDO + SDI; // BIT3 + BIT1 + BIT2
  P3DIR |= SCLK + SDO; // BIT3 + BIT1
  P2DIR |= CS; // BIT0
  P3DIR &= ~SDI;*/
  
  // For UCB 0
  /*
  P2SEL &= ~BIT6;
  P3SEL |= BIT2 + BIT1 + BIT0;
  P3DIR |= BIT2 + BIT0;
  P2DIR |= BIT6;
  P3DIR &= ~BIT1;*/
  
  // For UCB 1
  P4SEL &= ~BIT0;
  P4SEL |= BIT3 + BIT2 + BIT1;
  P4DIR |= BIT3 + BIT1 + BIT0;
  P4DIR &= ~BIT2;
  
  // Start UCB1
  UCB1CTL1 &= ~UCSWRST;
}

/***********************************************************************/
/** \brief SPiWrite 
 *
 * Description: Writes bytes from buffer to SPI tx reg

 * \author Iain Derrington
 * \param ptrBuffer Pointer to buffer containing data.
 * \param ui_Len number of bytes to transmit.
 * \return uint Number of bytes transmitted.
 */
/**********************************************************************/
unsigned int SPIWrite(unsigned char * ptrBuffer, unsigned int ui_Len)
{
  unsigned int i;
  
  if (ui_Len == 0)
    return 0;

    for (i=0;i<ui_Len;i++)
    {
      UCB1TXBUF = *ptrBuffer++;
      while (! (UCB1IFG & UCTXIFG));
    }

    return i;
}

/***********************************************************************/
/** \brief SPIRead
 *
 * Description: Will read ui_Length bytes from SPI. Bytes placed into ptrbuffer
 *
 * \author Iain Derrington
 * \param ptrBuffer Buffer containing data.
 * \param ui_Len Number of bytes to read.
 * \return uint Number of bytes read.
 */
/**********************************************************************/
unsigned int SPIRead(unsigned char * ptrBuffer, unsigned int ui_Len)
{
  unsigned int i;

  for ( i=0;i<ui_Len;i++)
  {
      while (! (UCB1IFG & UCTXIFG));// Could remove this to speed up
      // Dummy byte sent to create read.
      UCB1TXBUF = 0x00;
      while (! (UCB1IFG & UCRXIFG));
      *ptrBuffer++ = UCB1RXBUF;
  }
  return i;
}
/***********************************************************************/
/** \brief Test SPI Link by sending message
 *
 * Description:
 * This program will test the SPI link by sending the ENC28J60 a message to turn on
 * the two LEDs that can be connected as outputs to the ENC28J60.
 *
 * To turn on the LEDs:
 * 1. Device is reset.
 * 2. The PHLCON register is updated to force on LEDs.
 *
 * To write to the PHLCON register:
 * 1. MIRREGADR register is changed to point to the PHLCON register
 * 2. Lower byte of message is written to MIWRL register
 * 3. Upper byte of message is written to MIWRH register
 *
 * As the MIREGADR, MIWRL and MIWRH registers are located in bank 2 ECON1 will have
 * to be updated to select the second bank.
 *
 * ECON1 Register
 * --7------6------5------4-------3------2------1-----0--
 * TXRST--RXRST--DMAST--CSUMEN--TXRTS--RXEN--BSEL1--BSEL0
 *
 * BSELX = Bank select bit X
 * All other bits can be looked up in data sheet.
 *
 *
 * MIREGADR Register
 * bit 4-0 = Phy register address
 *
 *
 * MIWRL & MIWRH Registers
 * All bits are message sent to Phy register.
 *
 *
 * PHLCON Register
 * Bits 15-12 and 0 are reserved set as 0.
 * --11-----10-----9------8-------7-----6------5------4------3-----2------1-----0--
 * LACFG3-LACFG2-LACFG1-LACFG0-LBCFG3-LBCFG2-LBCFG1-LBCFG0-LFRQ1-LFRQ0-STRCH-Reserved
 *
 * LACFGX = LED A configuration bit X
 * LBCFGX = LED B configuration bit X
 * LFRQX = LED pulse stretch
 * STRCH = Pulse stretch enable
 *
 *
 * LYCFGX configuration registers can be set to 0b1000 to turn on the LED.
 *
 * 0xFF is will reset the enc28j60
 * 0x40 is the write operation
 * ECON1 addr is 0x1F
 * MIREGADR addr is 0x14
 * MIWRL addr is 0x16
 * MIWRH addr is 0x17
 * PHLCON addr is 0x14
 *              
 *              
 * \author Duncan Frost & Maxime Alay-Eddine
 */
/**********************************************************************/
void enc_lightLeds(void)
{
  unsigned char op = RESET_OP; //Reset
  
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  //__delay_cycles(1600); 
  //Wait for enc28j60 to reset. 50us requried but best give it extra.
  op = ECON1 | WCR_OP; 
  //Write next message to ECON1 register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  
  op = ECON1_BSEL_BANK2; 
  //ECON1_BSEL_BANK2 (0x02) in ECON1 selects bank 2 registers.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  
  op = MIREGADR | WCR_OP; 
  //Write next message to MIREGADR register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  
  op = PHLCON; 
  //PHLCON (0x14) in MIREGADR selects the PHLCON register.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  op = MIWRL | WCR_OP; 
  //Write next message to MIWRL register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  op = 0x80; 
  //0x80 in MIWRL will turn on LEDB.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  op = MIWRH | WCR_OP; 
  //Write next message to MIWRH register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  op = 0x08; 
  //0x08 in MIWRH will turn on LEDA.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
}

void enc_shutdownLeds(void) {
  unsigned char op = RESET_OP;
  op = PHLCON; 
  //PHLCON (0x14) in MIREGADR selects the PHLCON register.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  op = MIWRL | WCR_OP; 
  //Write next message to MIWRL register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  op = 0x00; 
  //0x80 in MIWRL will turn on LEDB.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
  
  op = MIWRH | WCR_OP; 
  //Write next message to MIWRH register.
  SEL_MAC(TRUE);
  SPIWrite(&op,1);
  op = 0x00; 
  //0x08 in MIWRH will turn on LEDA.
  SPIWrite(&op,1);
  SEL_MAC(FALSE);
}
