#ifndef ENC28J60_H
#define ENC28J60_H

// initialisation routine
void initMAC(const unsigned char* deviceMAC);
// function to write and send a packet from the ENC28J60_H
unsigned int MACWrite(unsigned char* packet, unsigned int len);
// function to read a byte (if there) into a buffer
unsigned int MACRead(unsigned char* packet, unsigned int maxLen);


#define UIP_LLH_LEN             14
#define UIP_TCPIP_HLEN          28
// useful structures
typedef union {
    unsigned char v[7];
    struct {
        unsigned int     ByteCount;
        unsigned char    CollisionCount:4;
        unsigned char    CRCError:1;
        unsigned char    LengthCheckError:1;
        unsigned char    LengthOutOfRange:1;
        unsigned char    Done:1;
        unsigned char    Multicast:1;
        unsigned char    Broadcast:1;
        unsigned char    PacketDefer:1;
        unsigned char    ExcessiveDefer:1;
        unsigned char    MaximumCollisions:1;
        unsigned char    LateCollision:1;
        unsigned char    Giant:1;
        unsigned char    Underrun:1;
        unsigned int     BytesTransmittedOnWire;
        unsigned char    ControlFrame:1;
        unsigned char    PAUSEControlFrame:1;
        unsigned char    BackpressureApplied:1;
        unsigned char    VLANTaggedFrame:1;
        unsigned char    Zeros:4;
    } bits;
} TXSTATUS;

typedef union {
    unsigned char v[6];
    struct {
        unsigned int     NextPacket;
        unsigned int     ByteCount;
        unsigned char    LongEvent:1;
        unsigned char    Reserved:1;
        unsigned char    CarrierEvent:1;
        unsigned char    Reserved2:1;
        unsigned char    CRCError:1;
        unsigned char    LenChkError:1;
        unsigned char    LenOutofRange:1;
        unsigned char    RxOk:1;
        unsigned char    RxMultiCast:1;
        unsigned char    RxBroadCast:1;
        unsigned char    DribbleNibble:1;
        unsigned char    RxCntrlFrame:1;
        unsigned char    RxPauseFrame:1;
        unsigned char    RxUkwnOpcode:1;
        unsigned char    RxVlan:1;
        unsigned char    Zeros:1;
    } bits;
} RXSTATUS;


// Hardware specific contents
#define CS_ENC28J60             0x00000100


// constants used to set up memory
#define RXSTART                 0x0000
#define RXEND                   0x0fff
#define TXSTART                 0x1000
#define TXEND                   0x1fff
#define RXMAXBUFLEN             RXEND - RXSTART
#define TXMAXBUFLEN             TXEND - TXSTART

#define MAXFRAMELEN             1518

#define RBM_OP                  0x3a
#define WCR_OP                  0x40
#define WBM_OP                  0x7a
#define BFS_OP                  0x80
#define BFC_OP                  0xa0
#define RESET_OP                0xff

// Bank 0 registers --------
#define ERDPTL                  0x00
#define ERDPTH                  0x01
#define EWRPTL                  0x02
#define EWRPTH                  0x03
#define ETXSTL                  0x04
#define ETXSTH                  0x05
#define ETXNDL                  0x06
#define ETXNDH                  0x07
#define ERXSTL                  0x08
#define ERXSTH                  0x09
#define ERXNDL                  0x0A
#define ERXNDH                  0x0B
#define ERXRDPTL                0x0C
#define ERXRDPTH                0x0D
#define ERXWRPTL                0x0E
#define ERXWRPTH                0x0F
#define EDMASTL                 0x10
#define EDMASTH                 0x11
#define EDMANDL                 0x12
#define EDMANDH                 0x13
#define EDMADSTL                0x14
#define EDMADSTH                0x15
#define EDMACSL                 0x16
#define EDMACSH                 0x17
//#define N/A                   0x18
//#define N/A                   0x19
//#define Reserved              0x1A
#define EIE                     0x1B
#define EIR                     0x1C
#define ESTAT                   0x1D
#define ECON2                   0x1E
#define ECON1                   0x1F

// Bank 1 registers ---------
#define EHT0                    0x00
#define EHT1                    0x01
#define EHT2                    0x02
#define EHT3                    0x03
#define EHT4                    0x04
#define EHT5                    0x05
#define EHT6                    0x06
#define EHT7                    0x07
#define EPMM0                   0x08
#define EPMM1                   0x09
#define EPMM2                   0x0A
#define EPMM3                   0x0B
#define EPMM4                   0x0C
#define EPMM5                   0x0D
#define EPMM6                   0x0E
#define EPMM7                   0x0F
#define EPMCSL                  0x10
#define EPMCSH                  0x11
//#define N/A                   0x12
//#define N/A                   0x13
#define EPMOL                   0x14
#define EPMOH                   0x15
#define EWOLIE                  0x16
#define EWOLIR                  0x17
#define ERXFCON                 0x18
#define EPKTCNT                 0x19
//#define Reserved              0x1A                                                            //Already defined with Bank 0
//#define EIE                   0x1B                                                            //Already defined with Bank 0
//#define EIR                   0x1C                                                            //Already defined with Bank 0
//#define ESTAT                 0x1D                                                            //Already defined with Bank 0
//#define ECON2                 0x1E                                                            //Already defined with Bank 0
//#define ECON1                 0x1F                                                            //Already defined with Bank 0

// Bank 2 registers -----
#define MACON1                  0x00
//#define Reserved              0x01
#define MACON3                  0x02
#define MACON4                  0x03
#define MABBIPG                 0x04
//#define N/A                   0x05
#define MAIPGL                  0x06
#define MAIPGH                  0x07
#define MACLCON1                0x08
#define MACLCON2                0x09
#define MAMXFLL                 0x0A
#define MAMXFLH                 0x0B
//#define Reserved              0x0C
#define MAPHSUP                 0x0D
//#define Reserved              0x0E
//#define N/A                   0x0F
//#define Reserved              0x10
#define MICON                   0x11
#define MICMD                   0x12
//#define N/A                   0x13
#define MIREGADR                0x14
//#define Reserved              0x15
#define MIWRL                   0x16
#define MIWRH                   0x17
#define MIRDL                   0x18
#define MIRDH                   0x19
//#define Reserved              0x1A                                                            //Already defined with Bank 0
//#define EIE                   0x1B                                                            //Already defined with Bank 0
//#define EIR                   0x1C                                                            //Already defined with Bank 0
//#define ESTAT                 0x1D                                                            //Already defined with Bank 0
//#define ECON2                 0x1E                                                            //Already defined with Bank 0
//#define ECON1                 0x1F                                                            //Already defined with Bank 0

// Bank 3 registers -----
#define MAADR5                  0x00
#define MAADR6                  0x01
#define MAADR3                  0x02
#define MAADR4                  0x03
#define MAADR1                  0x04
#define MAADR2                  0x05
#define EBSTSD                  0x06
#define EBSTCON                 0x07
#define EBSTCSL                 0x08
#define EBSTCSH                 0x09
#define MISTAT                  0x0A
//#define N/A                   0x0B
//#define N/A                   0x0C
//#define N/A                   0x0D
//#define N/A                   0x0E
//#define N/A                   0x0F
//#define N/A                   0x10
//#define N/A                   0x11
#define EREVID                  0x12
//#define N/A                   0x13
//#define N/A                   0x14
#define ECOCON                  0x15
//#define Reserved              0x16
#define EFLOCON                 0x17
#define EPAUSL                  0x18
#define EPAUSH                  0x19
//#define Reserved              0x1A                                                            //Already defined with Bank 0
//#define EIE                   0x1B                                                            //Already defined with Bank 0
//#define EIR                   0x1C                                                            //Already defined with Bank 0
//#define ESTAT                 0x1D                                                            //Already defined with Bank 0
//#define ECON2                 0x1E                                                            //Already defined with Bank 0
//#define ECON1                 0x1F                                                            //Already defined with Bank 0

/******************************************************************************
* PHY Register Locations
******************************************************************************/
#define PHCON1                  0x00
#define PHSTAT1                 0x01
#define PHID1                   0x02
#define PHID2                   0x03
#define PHCON2                  0x10
#define PHSTAT2                 0x11
#define PHIE                    0x12
#define PHIR                    0x13
#define PHLCON                  0x14


/******************************************************************************
* Individual Register Bits
******************************************************************************/
// ETH/MAC/MII bits

// EIE bits ----------
#define EIE_INTIE               (1<<7)                                                          //Global INT interrupt enable bit - 1: Allow interrupt events to drive the /INT pin, 0: Disable all /INT pin activity (pin is continuously driven high)
#define EIE_PKTIE               (1<<6)                                                          //Receive Packet Pending Interrupt Enable bit - 1: Enable receive packet pending interrupt, 0: Disable receive packet pending interrupt
#define EIE_DMAIE               (1<<5)                                                          //DMA Interrupt Enable bit - 1: Enable DMA interrupt, 0: Disable DMA interrupt
#define EIE_LINKIE              (1<<4)                                                          //Link Status Change Interrupt Enable bit - 1: Enable link change interrupt from the PHY, 0: Disable link change interrupt
#define EIE_TXIE                (1<<3)                                                          //Transmit Enable bit - 1: Enable transmit interrupt, 0: Disable transmit interrupt
//#define Reserved              (1<<2)
#define EIE_TXERIE              (1<<1)                                                          //Transmit Error Interrupt Enable bit - 1: Enable transmit error interrupt, 0: Disable transmit error interrupt
#define EIE_RXERIE              (1)                                                             //Receive Error Interrupt Enable bit - 1: Enable receive error interrupt, 0: Disable receive error interrupt

// EIR bits ----------
//#define N/A                   (1<<7)
#define EIR_PKTIF               (1<<6)                                                          //Receive Packet Pending Interrupt Flag bit - 1: Receive buffer contains one or more unprocessed packets, cleared when PKTDEC is set, 0: Receive buffer is empty
#define EIR_DMAIF               (1<<5)                                                          //DMA Interrupt Flag bit - 1: DMA copy or checksum calculation has completed, 0: No DMA interrupt is pending
#define EIR_LINKIF              (1<<4)                                                          //Link Change Interrupt Flag bit - 1: PHY reports that the link status has changed, read PHIR register to clear, 0: Link status has not changed
#define EIR_TXIF                (1<<3)                                                          //Transmit Interrupt Flag bit - 1: Transmit request has ended, 0: No transmit interrupt is pending
//#define Reserved              (1<<2)
#define EIR_TXERIF              (1<<1)                                                          //Transmit Error Interrupt Flag bit - 1: A transmit error has occurred, 0: No transmit error has occurred
#define EIR_RXERIF              (1)                                                             //Receive Error Interrupt Flag bit - 1: A packet was aborted because there is insufficient buffer space or the packet count is 255, 0: No receive error interrupt is pending

// ESTAT bits ---------
#define ESTAT_INT               (1<<7)                                                          //INT interrupt Flag bit - 1: INT interrupt is pending, 0: No INT interrupt is pending
#define ESTAT_BUFER             (1<<6)                                                          //Ethernet Buffer Error Status bit - 1: An Ethernet read or write has generated a buffer error (overrun or underrun), 0: No buffer error has occurred
//#define Reserved              (1<<5)
#define    ESTAT_LATECOL        (1<<4)                                                          //Late Collision Error bit - 1: A collision occurred after 64 bytes had been transmitted, 0: No collisions after 64 bytes have occurred
//#define N/A                   (1<<3)
#define ESTAT_RXBUSY            (1<<2)                                                          //Receive Busy bit - 1: Receive logic is receiving a data packet, 0: Receive logic is Idle
#define ESTAT_TXABRT            (1<<1)                                                          //Transmit Abort Error bit - 1: The transmit request was aborted, 0: No transmit abort error
#define ESTAT_CLKRDY            (1)                                                             //Clock Ready bit - 1: OST has expired, PHY is ready, 0: OST is still counting, PHY is not ready

// ECON2 bits --------
#define ECON2_AUTOINC           (1<<7)                                                          //Automatic Buffer Pointer Increment Enable bit - 1: Automatically incr ERDPT & EWRPT when SPI RBM/WBM command is used, 0: Do not automatically change ERDPT & EWRPT after the buffer is accessed
#define ECON2_PKTDEC            (1<<6)                                                          //Packet Decrement bit - 1:Decrement the EPKTCNT register by one, 0:Leave EPKTCNT unchanged
#define ECON2_PWRSV             (1<<5)                                                          //Power Save Enable bit - 1: MAC, PHY and control logic are in Low-Power Sleep mode, 0: Normal operation
//#define Reserved              (1<<4)
#define ECON2_VRPS              (1<<3)                                                          //Voltage Regulator Power Save Enable bit - Ignored if PWRSV equals 0. If PWRSV equals 1, 1: Internal voltage regulator is in Low-Current mode, 0: Internal voltage regulator is in Normal Current mode. See values below.
//#define N/A                   (1<<2)
//#define N/A                   (1<<1)
//#define N/A                   (1)
// ECON2 values --------
#define ECON2_LOWCURRENT        (ECON2_PWRSV+ECON2_VRPS)                                        //Internal voltage regulator in Low-Current mode
#define ECON2_NORCURRENT        (ECON2_PWRSV)                                                   //Internal voltage regulator in Normal Current mode

// ECON1 bits --------
#define ECON1_TXRST             (1<<7)                                                          //Transmit logic Reset bit - 1: Held in reset, 0: Normal
#define ECON1_RXRST             (1<<6)                                                          //Receive logic Reset bit - 1: Held in reset, 0: Normal
#define ECON1_DMAST             (1<<5)                                                          //DMA Start and Busy bit - 1: DMA copy or checksum operation is in progress, 0: DMA hardware is Idle
#define ECON1_CSUMEN            (1<<4)                                                          //DMA Checksum Enable bit - 1: DMA hardware calculates checksums, 0: DMA hardware is Idle
#define ECON1_TXRTS             (1<<3)                                                          //Transmit Request to Send bit - 1: Transmit logic is attempting to transmit a packet, 0: Transmit logic is Idle
#define ECON1_RXEN              (1<<2)                                                          //Receive Enable bit - 1:Packets which pass current filter configuration will be wirtten into the receive buffer, 0: All packets received will be ignored
#define ECON1_BSEL1             (1<<1)                                                          //Bank Select bit, see values below
#define ECON1_BSEL0             (1)                                                             //Bank Select bit, see values below
// ECON1 values --------
#define ECON1_BSEL_BANK3        (ECON1_BSEL1+ECON1_BSEL0)                                       //SPI accesses registers in Bank3
#define ECON1_BSEL_BANK2        (ECON1_BSEL1)                                                   //SPI accesses registers in Bank2
#define ECON1_BSEL_BANK1        (ECON1_BSEL0)                                                   //SPI accesses registers in Bank1
#define ECON1_BSEL_BANK0        (0)                                                             //SPI accesses registers in Bank0
#define ECON1_BSEL              (ECON1_BSEL1+ECON1_BSEL0)

// ERDPTL bits --------
#define ERDPTL_ERDPT7           (1<<7)
#define ERDPTL_ERDPT6           (1<<6)
#define ERDPTL_ERDPT5           (1<<5)
#define ERDPTL_ERDPT4           (1<<4)
#define ERDPTL_ERDPT3           (1<<3)
#define ERDPTL_ERDPT2           (1<<2)
#define ERDPTL_ERDPT1           (1<<1)
#define ERDPTL_ERDPT0           (1)

// ERDPTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ERDPTH_ERDPT12          (1<<4)
#define ERDPTH_ERDPT11          (1<<3)
#define ERDPTH_ERDPT10          (1<<2)
#define ERDPTH_ERDPT9           (1<<1)
#define ERDPTH_ERDPT8           (1)

// EWRPTL bits --------
#define EWRPTL_EWRPT7           (1<<7)
#define EWRPTL_EWRPT6           (1<<6)
#define EWRPTL_EWRPT5           (1<<5)
#define EWRPTL_EWRPT4           (1<<4)
#define EWRPTL_EWRPT3           (1<<3)
#define EWRPTL_EWRPT2           (1<<2)
#define EWRPTL_EWRPT1           (1<<1)
#define EWRPTL_EWRPT0           (1)

// EWRPTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EWRPTH_EWRPT12          (1<<4)
#define EWRPTH_EWRPT11          (1<<3)
#define EWRPTH_EWRPT10          (1<<2)
#define EWRPTH_EWRPT9           (1<<1)
#define EWRPTH_EWRPT8           (1)

// ETXSTL bits --------
#define ETXSTL_ETXST7           (1<<7)
#define ETXSTL_ETXST6           (1<<6)
#define ETXSTL_ETXST5           (1<<5)
#define ETXSTL_ETXST4           (1<<4)
#define ETXSTL_ETXST3           (1<<3)
#define ETXSTL_ETXST2           (1<<2)
#define ETXSTL_ETXST1           (1<<1)
#define ETXSTL_ETXST0           (1)

// ETXSTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ETXSTH_ETXST12          (1<<4)
#define ETXSTH_ETXST11          (1<<3)
#define ETXSTH_ETXST10          (1<<2)
#define ETXSTH_ETXST9           (1<<1)
#define ETXSTH_ETXST8           (1)

// ETXNDL bits --------
#define ETXNDL_ETXND7           (1<<7)
#define ETXNDL_ETXND6           (1<<6)
#define ETXNDL_ETXND5           (1<<5)
#define ETXNDL_ETXND4           (1<<4)
#define ETXNDL_ETXND3           (1<<3)
#define ETXNDL_ETXND2           (1<<2)
#define ETXNDL_ETXND1           (1<<1)
#define ETXNDL_ETXND0           (1)

// ETXNDH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ETXNDH_ETXND12          (1<<4)
#define ETXNDH_ETXND11          (1<<3)
#define ETXNDH_ETXND10          (1<<2)
#define ETXNDH_ETXND9           (1<<1)
#define ETXNDH_ETXND8           (1)

// ERXSTL bits --------
#define ERXSTL_ERXST7           (1<<7)
#define ERXSTL_ERXST6           (1<<6)
#define ERXSTL_ERXST5           (1<<5)
#define ERXSTL_ERXST4           (1<<4)
#define ERXSTL_ERXST3           (1<<3)
#define ERXSTL_ERXST2           (1<<2)
#define ERXSTL_ERXST1           (1<<1)
#define ERXSTL_ERXST0           (1)

// ERXSTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ERXSTH_ERXST12          (1<<4)
#define ERXSTH_ERXST11          (1<<3)
#define ERXSTH_ERXST10          (1<<2)
#define ERXSTH_ERXST9           (1<<1)
#define ERXSTH_ERXST8           (1)

// ERXNDL bits --------
#define ERXNDL_ERXND7           (1<<7)
#define ERXNDL_ERXND6           (1<<6)
#define ERXNDL_ERXND5           (1<<5)
#define ERXNDL_ERXND4           (1<<4)
#define ERXNDL_ERXND3           (1<<3)
#define ERXNDL_ERXND2           (1<<2)
#define ERXNDL_ERXND1           (1<<1)
#define ERXNDL_ERXND0           (1)

// ERXNDH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ERXNDH_ERXND12          (1<<4)
#define ERXNDH_ERXND11          (1<<3)
#define ERXNDH_ERXND10          (1<<2)
#define ERXNDH_ERXND9           (1<<1)
#define ERXNDH_ERXND8           (1)

// ERXRDPTL bits --------
#define ERXRDPTL_ERXRDPT7       (1<<7)
#define ERXRDPTL_ERXRDPT6       (1<<6)
#define ERXRDPTL_ERXRDPT5       (1<<5)
#define ERXRDPTL_ERXRDPT4       (1<<4)
#define ERXRDPTL_ERXRDPT3       (1<<3)
#define ERXRDPTL_ERXRDPT2       (1<<2)
#define ERXRDPTL_ERXRDPT1       (1<<1)
#define ERXRDPTL_ERXRDPT0       (1)

// ERXRDPTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ERXRDPTH_ERXRDPT12      (1<<4)
#define ERXRDPTH_ERXRDPT11      (1<<3)
#define ERXRDPTH_ERXRDPT10      (1<<2)
#define ERXRDPTH_ERXRDPT9       (1<<1)
#define ERXRDPTH_ERXRDPT8       (1)

// ERXWRPTL bits --------
#define ERXWRPTL_ERXWRPT7       (1<<7)
#define ERXWRPTL_ERXWRPT6       (1<<6)
#define ERXWRPTL_ERXWRPT5       (1<<5)
#define ERXWRPTL_ERXWRPT4       (1<<4)
#define ERXWRPTL_ERXWRPT3       (1<<3)
#define ERXWRPTL_ERXWRPT2       (1<<2)
#define ERXWRPTL_ERXWRPT1       (1<<1)
#define ERXWRPTL_ERXWRPT0       (1)

// ERXWRPTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define ERXWRPTH_ERXWRPT12      (1<<4)
#define ERXWRPTH_ERXWRPT11      (1<<3)
#define ERXWRPTH_ERXWRPT10      (1<<2)
#define ERXWRPTH_ERXWRPT9       (1<<1)
#define ERXWRPTH_ERXWRPT8       (1)

// EDMASTL bits --------
#define EDMASTL_EDMAST7         (1<<7)
#define EDMASTL_EDMAST6         (1<<6)
#define EDMASTL_EDMAST5         (1<<5)
#define EDMASTL_EDMAST4         (1<<4)
#define EDMASTL_EDMAST3         (1<<3)
#define EDMASTL_EDMAST2         (1<<2)
#define EDMASTL_EDMAST1         (1<<1)
#define EDMASTL_EDMAST0         (1)

// EDMASTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EDMASTH_EDMAST12        (1<<4)
#define EDMASTH_EDMAST11        (1<<3)
#define EDMASTH_EDMAST10        (1<<2)
#define EDMASTH_EDMAST9         (1<<1)
#define EDMASTH_EDMAST8         (1)

// EDMANDL bits --------
#define EDMANDL_EDMAND7         (1<<7)
#define EDMANDL_EDMAND6         (1<<6)
#define EDMANDL_EDMAND5         (1<<5)
#define EDMANDL_EDMAND4         (1<<4)
#define EDMANDL_EDMAND3         (1<<3)
#define EDMANDL_EDMAND2         (1<<2)
#define EDMANDL_EDMAND1         (1<<1)
#define EDMANDL_EDMAND0         (1)

// EDMANDH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EDMANDH_EDMAND12        (1<<4)
#define EDMANDH_EDMAND11        (1<<3)
#define EDMANDH_EDMAND10        (1<<2)
#define EDMANDH_EDMAND9         (1<<1)
#define EDMANDH_EDMAND8         (1)

// EDMADSTL bits --------
#define EDMADSTL_EDMADST7       (1<<7)
#define EDMADSTL_EDMADST6       (1<<6)
#define EDMADSTL_EDMADST5       (1<<5)
#define EDMADSTL_EDMADST4       (1<<4)
#define EDMADSTL_EDMADST3       (1<<3)
#define EDMADSTL_EDMADST2       (1<<2)
#define EDMADSTL_EDMADST1       (1<<1)
#define EDMADSTL_EDMADST0       (1)

// EDMADSTH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EDMADSTH_EDMADST12      (1<<4)
#define EDMADSTH_EDMADST11      (1<<3)
#define EDMADSTH_EDMADST10      (1<<2)
#define EDMADSTH_EDMADST9       (1<<1)
#define EDMADSTH_EDMADST8       (1)

// EDMACSL bits --------
#define EDMACSL_EDMACS7         (1<<7)
#define EDMACSL_EDMACS6         (1<<6)
#define EDMACSL_EDMACS5         (1<<5)
#define EDMACSL_EDMACS4         (1<<4)
#define EDMACSL_EDMACS3         (1<<3)
#define EDMACSL_EDMACS2         (1<<2)
#define EDMACSL_EDMACS1         (1<<1)
#define EDMACSL_EDMACS0         (1)

// EDMACSH bits --------
#define EDMACSH_EDMACS15        (1<<7)
#define EDMACSH_EDMACS14        (1<<6)
#define EDMACSH_EDMACS13        (1<<5)
#define EDMACSH_EDMACS12        (1<<4)
#define EDMACSH_EDMACS11        (1<<3)
#define EDMACSH_EDMACS10        (1<<2)
#define EDMACSH_EDMACS9         (1<<1)
#define EDMACSH_EDMACS8         (1)

// EHT0 bits --------
#define EHT0_EHT7               (1<<7)
#define EHT0_EHT6               (1<<6)
#define EHT0_EHT5               (1<<5)
#define EHT0_EHT4               (1<<4)
#define EHT0_EHT3               (1<<3)
#define EHT0_EHT2               (1<<2)
#define EHT0_EHT1               (1<<1)
#define EHT0_EHT0               (1)

// EHT1 bits --------
#define EHT1_EHT15              (1<<7)
#define EHT1_EHT14              (1<<6)
#define EHT1_EHT13              (1<<5)
#define EHT1_EHT12              (1<<4)
#define EHT1_EHT11              (1<<3)
#define EHT1_EHT10              (1<<2)
#define EHT1_EHT9               (1<<1)
#define EHT1_EHT8               (1)

// EHT2 bits --------
#define EHT2_EHT23              (1<<7)
#define EHT2_EHT22              (1<<6)
#define EHT2_EHT21              (1<<5)
#define EHT2_EHT20              (1<<4)
#define EHT2_EHT19              (1<<3)
#define EHT2_EHT18              (1<<2)
#define EHT2_EHT17              (1<<1)
#define EHT2_EHT16              (1)

// EHT3 bits --------
#define EHT3_EHT31              (1<<7)
#define EHT3_EHT30              (1<<6)
#define EHT3_EHT29              (1<<5)
#define EHT3_EHT28              (1<<4)
#define EHT3_EHT27              (1<<3)
#define EHT3_EHT26              (1<<2)
#define EHT3_EHT25              (1<<1)
#define EHT3_EHT24              (1)

// EHT4 bits --------
#define EHT4_EHT39              (1<<7)
#define EHT4_EHT38              (1<<6)
#define EHT4_EHT37              (1<<5)
#define EHT4_EHT36              (1<<4)
#define EHT4_EHT35              (1<<3)
#define EHT4_EHT34              (1<<2)
#define EHT4_EHT33              (1<<1)
#define EHT4_EHT32              (1)

// EHT5 bits --------
#define EHT5_EHT47              (1<<7)
#define EHT5_EHT46              (1<<6)
#define EHT5_EHT45              (1<<5)
#define EHT5_EHT44              (1<<4)
#define EHT5_EHT43              (1<<3)
#define EHT5_EHT42              (1<<2)
#define EHT5_EHT41              (1<<1)
#define EHT5_EHT40              (1)

// EHT6 bits --------
#define EHT6_EHT55              (1<<7)
#define EHT6_EHT54              (1<<6)
#define EHT6_EHT53              (1<<5)
#define EHT6_EHT52              (1<<4)
#define EHT6_EHT51              (1<<3)
#define EHT6_EHT50              (1<<2)
#define EHT6_EHT49              (1<<1)
#define EHT6_EHT48              (1)

// EHT7 bits --------
#define EHT7_EHT63              (1<<7)
#define EHT7_EHT62              (1<<6)
#define EHT7_EHT61              (1<<5)
#define EHT7_EHT60              (1<<4)
#define EHT7_EHT59              (1<<3)
#define EHT7_EHT58              (1<<2)
#define EHT7_EHT57              (1<<1)
#define EHT7_EHT56              (1)

// EPMM0 bits --------
#define EPMM0_EPMM7             (1<<7)
#define EPMM0_EPMM6             (1<<6)
#define EPMM0_EPMM5             (1<<5)
#define EPMM0_EPMM4             (1<<4)
#define EPMM0_EPMM3             (1<<3)
#define EPMM0_EPMM2             (1<<2)
#define EPMM0_EPMM1             (1<<1)
#define EPMM0_EPMM0             (1)

// EPMM1 bits --------
#define EPMM1_EPMM15            (1<<7)
#define EPMM1_EPMM14            (1<<6)
#define EPMM1_EPMM13            (1<<5)
#define EPMM1_EPMM12            (1<<4)
#define EPMM1_EPMM11            (1<<3)
#define EPMM1_EPMM10            (1<<2)
#define EPMM1_EPMM9             (1<<1)
#define EPMM1_EPMM8             (1)

// EPMM2 bits --------
#define EPMM2_EPMM23            (1<<7)
#define EPMM2_EPMM22            (1<<6)
#define EPMM2_EPMM21            (1<<5)
#define EPMM2_EPMM20            (1<<4)
#define EPMM2_EPMM19            (1<<3)
#define EPMM2_EPMM18            (1<<2)
#define EPMM2_EPMM17            (1<<1)
#define EPMM2_EPMM16            (1)

// EPMM3 bits --------
#define EPMM3_EPMM31            (1<<7)
#define EPMM3_EPMM30            (1<<6)
#define EPMM3_EPMM29            (1<<5)
#define EPMM3_EPMM28            (1<<4)
#define EPMM3_EPMM27            (1<<3)
#define EPMM3_EPMM26            (1<<2)
#define EPMM3_EPMM25            (1<<1)
#define EPMM3_EPMM24            (1)

// EPMM4 bits --------
#define EPMM4_EPMM39            (1<<7)
#define EPMM4_EPMM38            (1<<6)
#define EPMM4_EPMM37            (1<<5)
#define EPMM4_EPMM36            (1<<4)
#define EPMM4_EPMM35            (1<<3)
#define EPMM4_EPMM34            (1<<2)
#define EPMM4_EPMM33            (1<<1)
#define EPMM4_EPMM32            (1)

// EPMM5 bits --------
#define EPMM5_EPMM47            (1<<7)
#define EPMM5_EPMM46            (1<<6)
#define EPMM5_EPMM45            (1<<5)
#define EPMM5_EPMM44            (1<<4)
#define EPMM5_EPMM43            (1<<3)
#define EPMM5_EPMM42            (1<<2)
#define EPMM5_EPMM41            (1<<1)
#define EPMM5_EPMM40            (1)

// EPMM6 bits --------
#define EPMM6_EPMM55            (1<<7)
#define EPMM6_EPMM54            (1<<6)
#define EPMM6_EPMM53            (1<<5)
#define EPMM6_EPMM52            (1<<4)
#define EPMM6_EPMM51            (1<<3)
#define EPMM6_EPMM50            (1<<2)
#define EPMM6_EPMM49            (1<<1)
#define EPMM6_EPMM48            (1)

// EPMM7 bits --------
#define EPMM7_EPMM63            (1<<7)
#define EPMM7_EPMM62            (1<<6)
#define EPMM7_EPMM61            (1<<5)
#define EPMM7_EPMM60            (1<<4)
#define EPMM7_EPMM59            (1<<3)
#define EPMM7_EPMM58            (1<<2)
#define EPMM7_EPMM57            (1<<1)
#define EPMM7_EPMM56            (1)

// EPMCSL bits --------
#define EPMCSL_EPMCS7           (1<<7)
#define EPMCSL_EPMCS6           (1<<6)
#define EPMCSL_EPMCS5           (1<<5)
#define EPMCSL_EPMCS4           (1<<4)
#define EPMCSL_EPMCS3           (1<<3)
#define EPMCSL_EPMCS2           (1<<2)
#define EPMCSL_EPMCS1           (1<<1)
#define EPMCSL_EPMCS0           (1)

// EPMCSH bits --------
#define EPMCSH_EPMCS15          (1<<7)
#define EPMCSH_EPMCS14          (1<<6)
#define EPMCSH_EPMCS13          (1<<5)
#define EPMCSH_EPMCS12          (1<<4)
#define EPMCSH_EPMCS11          (1<<3)
#define EPMCSH_EPMCS10          (1<<2)
#define EPMCSH_EPMCS9           (1<<1)
#define EPMCSH_EPMCS8           (1)

// EPMOL bits --------
#define EPMOL_EPMO7             (1<<7)
#define EPMOL_EPMO6             (1<<6)
#define EPMOL_EPMO5             (1<<5)
#define EPMOL_EPMO4             (1<<4)
#define EPMOL_EPMO3             (1<<3)
#define EPMOL_EPMO2             (1<<2)
#define EPMOL_EPMO1             (1<<1)
#define EPMOL_EPMO0             (1)

// EPMOH bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EPMOH_EPMO12            (1<<4)
#define EPMOH_EPMO11            (1<<3)
#define EPMOH_EPMO10            (1<<2)
#define EPMOH_EPMO9             (1<<1)
#define EPMOH_EPMO8             (1)

// EWOLIE bits --------
#define EWOLIE_UCWOLIE          (1<<7)
#define EWOLIE_AWOLIE           (1<<6)
//#define N/A                   (1<<5)
#define EWOLIE_PMWOLIE          (1<<4)
#define EWOLIE_MPWOLIE          (1<<3)
#define EWOLIE_HTWOLIE          (1<<2)
#define EWOLIE_MCWOLIE          (1<<1)
#define EWOLIE_BCWOLIE          (1)

// EWOLIR bits --------
#define EWOLIR_UCWOLIF          (1<<7)
#define EWOLIR_AWOLIF           (1<<6)
//#define N/A                   (1<<5)
#define EWOLIR_PMWOLIF          (1<<4)
#define EWOLIR_MPWOLIF          (1<<3)
#define EWOLIR_HTWOLIF          (1<<2)
#define EWOLIR_MCWOLIF          (1<<1)
#define EWOLIR_BCWOLIF          (1)

// ERXFCON bits ------
#define ERXFCON_UCEN            (1<<7)                                                          //Unicast Filter Enable bit - When ANDOR=1, 1: Packets not having a destination address matching the local MAC address will be discarded. 0: Filter disabled. - When ANDOR=0, 1:Packets with a destination address matching the local MAC address will be accepted. 0: Filter disabled.
#define ERXFCON_ANDOR           (1<<6)                                                          //AND/OR Filter Select bit - 1: AND, Packets will be rejected unless all enabled filters accept the packet, 0: OR, Packets will be accepted unless all enabled filters reject the packet
#define ERXFCON_CRCEN           (1<<5)                                                          //Post-Filter CRC Check Enable bit - 1: All packets with an invalid CRC will be discarded, 0: The CRC validity will be ignored
#define ERXFCON_PMEN            (1<<4)                                                          //Pattern Match Filter Enable bit - When ANDOR=1, 1: Packets must meet the Pattern Match criteria or they will be discarded. 0: Filter disabled. - When ANDOR=0, 1: Packets which meet the Pattern Match criteria will be accepted. 0: Filter disabled.
#define ERXFCON_MPEN            (1<<3)                                                          //Magic Packete Filter Enable bit - When ANDOR=1, 1: Packets must be Magic Packets for the local MAC address or they will be discarded. 0: Filter disabled. - When ANDOR=0, 1: Magic Packets for the local MAC address will be accepted. 0: Filter disabled.
#define ERXFCON_HTEN            (1<<2)                                                          //Hash Table Filter Enable bit - When ANDOR=1, 1: Packets must meet the Hash Table criteria or they will be discarded. 0: Filter disabled. - When ANDOR=0, 1: Packets which meet the Hash Table criteria will be accepted. 0: Filter disabled.
#define ERXFCON_MCEN            (1<<1)                                                          //Multicast Filter Enable bit - When ANDOR=1, 1: Packets must have the LSB set in the destination address or they will be discarded. 0: Filter disabled. - When ANDOR=0, 1: Packets which have the LSB set in the destination address will be accepted. 0: Filter disabled.
#define ERXFCON_BCEN            (1)                                                             //Broadcast Filter Enable bit - When ANDOR=1, 1: Packets must have a destination address of FF-FF-FF-FF-FF-FF or they will be discarded. 0: Filter disabled. - When ANDOR=0, 1: Packets which have a destination address of FF-FF-FF-FF-FF-FF will be accepted. 0: Filter disabled.

// EPKTCNT bits -------
// Not to be used

// MACON1 bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define Reserved              (1<<4)
#define MACON1_TXPAUS           (1<<3)                                                          //Pause Control Frame Transmission Enable bit - 1: Allow the MAC to transmit pause control frame (needed for flow control in full duplex), 0: Disallow pause frame transmissions
#define MACON1_RXPAUS           (1<<2)                                                          //Pause Control Frame Reception Enable bit - 1: Inhibit transmissions when pause control frames are received (normal operation), 0: Ignore pause control frames which are received
#define MACON1_PASSALL          (1<<1)                                                          //Pass all receive Frames Enable bit - 1: Control frames received by the MAC will be written into the receive buffer if not filtered out, 0: Control frames will be discarded after being processed by the MAC (normal operation)
#define MACON1_MARXEN           (1)                                                             //MAC Receive Enable bit - 1: Enable packets to be received by the MAC, 0: Disable packet reception

/* Reserved since Revision E of the DataSheet
// MACON2 bits --------
#define MACON2_MARST            (1<<7)
#define MACON2_RNDRST           (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
#define MACON2_MARXRST          (1<<3)
#define MACON2_RFUNRST          (1<<2)
#define MACON2_MATXRST          (1<<1)
#define MACON2_TFUNRST          (1)
*/

// MACON3 bits --------
#define MACON3_PADCFG2          (1<<7)                                                          //Automatic Pad and CRC COnfiguration bits - Please see below for values
#define MACON3_PADCFG1          (1<<6)                                                          //Automatic Pad and CRC COnfiguration bits - Please see below for values
#define MACON3_PADCFG0          (1<<5)                                                          //Automatic Pad and CRC COnfiguration bits - Please see below for values
#define MACON3_TXCRCEN          (1<<4)                                                          //Transmit CRC Enable bit - 1: MAC will append a valid CRC to all frames transmitted regardless of PADCFG bits. TXCRCEN must be set if the PADCFG bits specitfy that a valid CRC will be appended, 0: MAC will not append a CRC. The last 4 bytes will be checked and if it is an invalid CRC, it will bne reported in the transmit status vector.
#define MACON3_PHDREN           (1<<3)                                                          //Proprietary Header Enable bit - 1: Frames presented to the MAC contain a 4-byte proprietary header which will not be used when calculating the CRC, 0: No proprietary header is present. The CRC will cover all data (normal operation).
#define MACON3_HFRMEN           (1<<2)                                                          //Huge Frame Enable bit - 1: Frames of any size will be allowed to be transmitted and received, 0: Frames bigger than MAMFXL will be aborted when transmitted or received.
#define MACON3_FRMLNEN          (1<<1)                                                          //Frame Length Checking Enable bit - 1: The type/length field of transmitted and received frames will be checked. If it represents a length, the frame size will be compared and mismatches will be reported in the transmit/receive status vector, 0: Frame lengths will not be compared with the type/length field.
#define MACON3_FULDPX           (1)                                                             //MAC Full-Duplex Enable bit - 1: MAC will operate in Full-Duplex mode. PDPXMD bit must also be set, 0: MAC will operate in Half-Duplex mode. PDPXMD bit must also be clear.
// MACON3 values ------
#define MACON3_PADCFG_OPT111    (MACON3_PADCFG2+MACON3_PADCFG1+MACON3_PADCFG0)                  //All short frames will be zero-padded to 64bytes and a valid CRC will then be appended
#define MACON3_PADCFG_OPT110    (MACON3_PADCFG2+MACON3_PADCFG1)                                 //No automatic padding of short frames
#define MACON3_PADCFG_OPT101    (MACON3_PADCFG2+MACON3_PADCFG0)                                 //MAC will automatically detect VLAN protocol frames which have a 8100h type field and automatically pad to 64bytes. If the frame is not a VLAN frame, it will be padded to 60bytes. After padding, a valid CRC will be appended.
#define MACON3_PADCFG_OPT100    (MACON3_PADCFG2)                                                //No automatic padding of short frames
#define MACON3_PADCFG_OPT011    (MACON3_PADCFG1+MACON3_PADCFG0)                                 //All short frames will be zero-padded to 64bytes and a valid CRC will then be appended
#define MACON3_PADCFG_OPT010    (MACON3_PADCFG1)                                                //No automatic padding of short frames
#define MACON3_PADCFG_OPT001    (MACON3_PADCFG0)                                                //All short frames will be zero-padded to 60bytes and a valid CRC will then be appended
#define MACON3_PADCFG_OPT000    (0)                                                             //No automatic padding of short frames
#define MACON3_PADCFG           (MACON3_PADCFG2+MACON3_PADCFG1+MACON3_PADCFG0)

// MACON4 bits --------
//#define N/A                   (1<<7)
#define MACON4_DEFER            (1<<6)                                                          //Defer Transmission Enable bit (applies to half duplex only) - 1:When the medium is occupied, the MAC will wait indefinitely for it to become free when attempting to transmit (use this setting for IEEE 802.3 compliance), 0: When the medium is occupied, the MAC will abort the transmission after the excessive deferral limit is reached
#define MACON4_BPEN             (1<<5)                                                          //No Backoff During Backpressure Enable bit (applies to half duplex only) - 1: After incidentally causing a collision during backpressure, the MAC will immediately begin retransmitting, 0: After incidentally causing a collision during backpressure, the MAC will delay using the Binary Exponential Backoff algorithm before attempting to retransmit (normal operation)
#define MACON4_NOBKOFF          (1<<4)                                                          //No Backoff Enable bit (applies to half duplex only) - 1: After any collision, the MAC will immediately begin retransmitting, 0: After any collision, the MAC will delay using the Binary Exponential Backoff algorithm before attempting to retransmit (normal operation)
//#define N/A                   (1<<3)
//#define N/A                   (1<<2)
//#Reserved                     (1<<1)
//#Reserved                     (1)

// MABBIPG bits --------
//#define N/A                   (1<<7)
#define MABBIPG_BBIPG6          (1<<6)                                                          //Back-to-Back Inter-Packet Gap Delay Time bits...
#define MABBIPG_BBIPG5          (1<<5)                                                          //...Nibble time offset delay between the end of one tranmission and the beginning of the next...
#define MABBIPG_BBIPG4          (1<<4)                                                          //... in a back-to-back sequence. The register value should be programmed to the desired period in...
#define MABBIPG_BBIPG3          (1<<3)                                                          //...nibble times minus 3 if MACON3_FULDPX=1, minus 6 if MACON3_FULDPX=0.
#define MABBIPG_BBIPG2          (1<<2)                                                          //The recommended setting is 15h if MACON3_FULDPX=1, 12h if MACON3_FULDPX=0, which represent the minimum IEEE...
#define MABBIPG_BBIPG1          (1<<1)                                                          //...specified Inter-Packet Gap (OPG) of 9.6µs.
#define MABBIPG_BBIPG0          (1)                                                             //
// MABBIPG values ------
#define MABBIPG_RECOFULDPX1     (15)
#define MABBIPG_RECOFULDPX0     (12)
#define MABBIPG_BBIPG           (MABBIPG_BBIPG6+MABBIPG_BBIPG5+MABBIPG_BBIPG4+MABBIPG_BBIPG3+MABBIPG_BBIPG2+MABBIPG_BBIPG1+MABBIPG_BBIPG0)

// MAIPGL bits --------
//#define N/A                   (1<<7)
#define MAIPGL_MAIPGL6          (1<<6)
#define MAIPGL_MAIPGL5          (1<<5)
#define MAIPGL_MAIPGL4          (1<<4)
#define MAIPGL_MAIPGL3          (1<<3)
#define MAIPGL_MAIPGL2          (1<<2)
#define MAIPGL_MAIPGL1          (1<<1)
#define MAIPGL_MAIPGL0          (1)

// MAIPGH bits --------
//#define N/A                   (1<<7)
#define MAIPGH_MAIPGH6          (1<<6)
#define MAIPGH_MAIPGH5          (1<<5)
#define MAIPGH_MAIPGH4          (1<<4)
#define MAIPGH_MAIPGH3          (1<<3)
#define MAIPGH_MAIPGH2          (1<<2)
#define MAIPGH_MAIPGH1          (1<<1)
#define MAIPGH_MAIPGH0          (1)

// MACLCON1 bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
#define MACLCON1_RETMAX3        (1<<3)
#define MACLCON1_RETMAX2        (1<<2)
#define MACLCON1_RETMAX1        (1<<1)
#define MACLCON1_RETMAX0        (1)

// MACLCON2 bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
#define MACLCON2_COLWIN5        (1<<5)
#define MACLCON2_COLWIN4        (1<<4)
#define MACLCON2_COLWIN3        (1<<3)
#define MACLCON2_COLWIN2        (1<<2)
#define MACLCON2_COLWIN1        (1<<1)
#define MACLCON2_COLWIN0        (1)

// MAMXFLL bits --------
#define MAMXFLL_MAMXFL7         (1<<7)
#define MAMXFLL_MAMXFL6         (1<<6)
#define MAMXFLL_MAMXFL5         (1<<5)
#define MAMXFLL_MAMXFL4         (1<<4)
#define MAMXFLL_MAMXFL3         (1<<3)
#define MAMXFLL_MAMXFL2         (1<<2)
#define MAMXFLL_MAMXFL1         (1<<1)
#define MAMXFLL_MAMXFL0         (1)

// MAMXFLH bits --------
#define MAMXFLH_MAMXFL15        (1<<7)
#define MAMXFLH_MAMXFL14        (1<<6)
#define MAMXFLH_MAMXFL13        (1<<5)
#define MAMXFLH_MAMXFL12        (1<<4)
#define MAMXFLH_MAMXFL11        (1<<3)
#define MAMXFLH_MAMXFL10        (1<<2)
#define MAMXFLH_MAMXFL9         (1<<1)
#define MAMXFLH_MAMXFL8         (1)

// MAPHSUP bits --------
#define MAPHSUP_RSTINTFC        (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define Reserved              (1<<4)
#define MAPHSUP_RSTRMII         (1<<3)
//#define N/A                   (1<<2)
//#define N/A                   (1<<1)
//#define Reserved              (1)

// MICON bits --------
#define MICON_RSTMII            (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
//#define N/A                   (1<<3)
//#define N/A                   (1<<2)
//#define N/A                   (1<<1)
//#define N/A                   (1)

// MICMD bits ---------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
//#define N/A                   (1<<3)
//#define N/A                   (1<<2)
#define MICMD_MIISCAN           (1<<1)                                                          //MII Scan Enable bit, 1: PHY register at MIREGADR is continuously read and the data is placed in MIRD, 0: No MII Magement scan operation is in progress
#define MICMD_MIIRD             (1)                                                             //MII Read Enable bit, 1: PHY register at MIREGADR is read one and the data is placed in MIRD, 0: No MII Management read operation is in progress

// MIREGADR bits ---------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define MIREGADR_MIREGADR4      (1<<4)
#define MIREGADR_MIREGADR3      (1<<3)
#define MIREGADR_MIREGADR2      (1<<2)
#define MIREGADR_MIREGADR1      (1<<1)
#define MIREGADR_MIREGADR0      (1)

// MIWRL bits --------
#define MIWRL_MIWR7             (1<<7)
#define MIWRL_MIWR6             (1<<6)
#define MIWRL_MIWR5             (1<<5)
#define MIWRL_MIWR4             (1<<4)
#define MIWRL_MIWR3             (1<<3)
#define MIWRL_MIWR2             (1<<2)
#define MIWRL_MIWR1             (1<<1)
#define MIWRL_MIWR0             (1)

// MIWRH bits --------
#define MIWRH_MIWR15            (1<<7)
#define MIWRH_MIWR14            (1<<6)
#define MIWRH_MIWR13            (1<<5)
#define MIWRH_MIWR12            (1<<4)
#define MIWRH_MIWR11            (1<<3)
#define MIWRH_MIWR10            (1<<2)
#define MIWRH_MIWR9             (1<<1)
#define MIWRH_MIWR8             (1)

// MIRDL bits --------
#define MIRDL_MIRD7             (1<<7)
#define MIRDL_MIRD6             (1<<6)
#define MIRDL_MIRD5             (1<<5)
#define MIRDL_MIRD4             (1<<4)
#define MIRDL_MIRD3             (1<<3)
#define MIRDL_MIRD2             (1<<2)
#define MIRDL_MIRD1             (1<<1)
#define MIRDL_MIRD0             (1)

// MIRDH bits --------
#define MIRDH_MIRD15            (1<<7)
#define MIRDH_MIRD14            (1<<6)
#define MIRDH_MIRD13            (1<<5)
#define MIRDH_MIRD12            (1<<4)
#define MIRDH_MIRD11            (1<<3)
#define MIRDH_MIRD10            (1<<2)
#define MIRDH_MIRD9             (1<<1)
#define MIRDH_MIRD8             (1)

// MAADR5 bits --------
#define MAADR5_MAADR15          (1<<7)
#define MAADR5_MAADR14          (1<<6)
#define MAADR5_MAADR13          (1<<5)
#define MAADR5_MAADR12          (1<<4)
#define MAADR5_MAADR11          (1<<3)
#define MAADR5_MAADR10          (1<<2)
#define MAADR5_MAADR9           (1<<1)
#define MAADR5_MAADR8           (1)

// MAADR6 bits --------
#define MAADR6_MAADR7           (1<<7)
#define MAADR6_MAADR6           (1<<6)
#define MAADR6_MAADR5           (1<<5)
#define MAADR6_MAADR4           (1<<4)
#define MAADR6_MAADR3           (1<<3)
#define MAADR6_MAADR2           (1<<2)
#define MAADR6_MAADR1           (1<<1)
#define MAADR6_MAADR0           (1)

// MAADR3 bits --------
#define MAADR3_MAADR31          (1<<7)
#define MAADR3_MAADR30          (1<<6)
#define MAADR3_MAADR29          (1<<5)
#define MAADR3_MAADR28          (1<<4)
#define MAADR3_MAADR27          (1<<3)
#define MAADR3_MAADR26          (1<<2)
#define MAADR3_MAADR25          (1<<1)
#define MAADR3_MAADR24          (1)

// MAADR4 bits --------
#define MAADR4_MAADR23          (1<<7)
#define MAADR4_MAADR22          (1<<6)
#define MAADR4_MAADR21          (1<<5)
#define MAADR4_MAADR20          (1<<4)
#define MAADR4_MAADR19          (1<<3)
#define MAADR4_MAADR18          (1<<2)
#define MAADR4_MAADR17          (1<<1)
#define MAADR4_MAADR16          (1)

// MAADR1 bits --------
#define MAADR1_MAADR47          (1<<7)
#define MAADR1_MAADR46          (1<<6)
#define MAADR1_MAADR45          (1<<5)
#define MAADR1_MAADR44          (1<<4)
#define MAADR1_MAADR43          (1<<3)
#define MAADR1_MAADR42          (1<<2)
#define MAADR1_MAADR41          (1<<1)
#define MAADR1_MAADR40          (1)

// MAADR2 bits --------
#define MAADR2_MAADR39          (1<<7)
#define MAADR2_MAADR38          (1<<6)
#define MAADR2_MAADR37          (1<<5)
#define MAADR2_MAADR36          (1<<4)
#define MAADR2_MAADR35          (1<<3)
#define MAADR2_MAADR34          (1<<2)
#define MAADR2_MAADR33          (1<<1)
#define MAADR2_MAADR32          (1)

// EBSTSD bits -----
#define EBSTSD_EBSTSD7          (1<<7)
#define EBSTSD_EBSTSD6          (1<<6)
#define EBSTSD_EBSTSD5          (1<<5)
#define EBSTSD_EBSTSD4          (1<<4)
#define EBSTSD_EBSTSD3          (1<<3)
#define EBSTSD_EBSTSD2          (1<<2)
#define EBSTSD_EBSTSD1          (1<<1)
#define EBSTSD_EBSTSD0          (1)

// EBSTCON bits -----
#define EBSTCON_PSV2            (1<<7)                                                          //Pattern Shift Values bits, please see values below for details
#define EBSTCON_PSV1            (1<<6)                                                          //Pattern Shift Values bits, please see values below for details
#define EBSTCON_PSV0            (1<<5)                                                          //Pattern Shift Values bits, please see values below for details
#define EBSTCON_PSEL            (1<<4)                                                          //Port Select bit - 1: DMA and BIST modules will swap ports when accessing the memory, 0: Normal configuration
#define EBSTCON_TMSEL1          (1<<3)                                                          //Test Mode Select bits, please see values below for details
#define EBSTCON_TMSEL0          (1<<2)                                                          //Test Mode Select bits, please see values below for details
#define EBSTCON_TME             (1<<1)                                                          //Test Mode Enable bit - 1: Enable Test mode, 0: Disable Test mode
#define EBSTCON_BISTST          (1)                                                             //Built-In Self-Test Start/Busy bit - 1: Test in progress, cleared automatically when test is done, 0: No test running
// EBSTCON values ---
#define EBSTCON_PSV_ON          (EBSTCON_PSV2)                                                  //The bits in EBSTSD will shift left by this amount after writing to each memory location
#define EBSTCON_PSV_OFF         (0)                                                             //This value is ignored
#define EBSTCON_PSV             (EBSTCON_PSV2+EBSTCON_PSV1+EBSTCON_PSV0)
#define EBSTCON_TMSEL_PSHIFT    (EBSTCON_TMSEL1)                                                //Pattern shift fill
#define EBSTCON_TMSEL_ADDRES    (EBSTCON_TMSEL0)                                                //Address fill
#define EBSTCON_TMSEL_RANDOM    (0)                                                             //Random data fill
#define EBSTCON_TMSEL           (EBSTCON_TMSEL1+EBSTCON_TMSEL0)

// EBSTCSL bits --------
#define EBSTCSL_EBSTCS7         (1<<7)
#define EBSTCSL_EBSTCS6         (1<<6)
#define EBSTCSL_EBSTCS5         (1<<5)
#define EBSTCSL_EBSTCS4         (1<<4)
#define EBSTCSL_EBSTCS3         (1<<3)
#define EBSTCSL_EBSTCS2         (1<<2)
#define EBSTCSL_EBSTCS1         (1<<1)
#define EBSTCSL_EBSTCS0         (1)

// EBSTCSH bits --------
#define EBSTCSH_EBSTCS15        (1<<7)
#define EBSTCSH_EBSTCS14        (1<<6)
#define EBSTCSH_EBSTCS13        (1<<5)
#define EBSTCSH_EBSTCS12        (1<<4)
#define EBSTCSH_EBSTCS11        (1<<3)
#define EBSTCSH_EBSTCS10        (1<<2)
#define EBSTCSH_EBSTCS9         (1<<1)
#define EBSTCSH_EBSTCS8         (1)

// MISTAT bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
//#define Reserved              (1<<3)
#define MISTAT_NVALID           (1<<2)                                                          //MII Management Read Data Not Valid bit, 1: Contents of MIRD not valid yet, 0: MII Management read cycle has completed and MIRD has been updated
#define MISTAT_SCAN             (1<<1)                                                          //MII Management Scan Operation bit, 1: MII Management scan operation is in progress, 0: no scan operation is in progress
#define MISTAT_BUSY             (1)                                                             //MII Management Busy bit, 1: A PHY register is currently being read or written to, 0: The MII Management interface is Idle

// EREVID bits --------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
#define EREVID_EREVID4          (1<<4)
#define EREVID_EREVID3          (1<<3)
#define EREVID_EREVID2          (1<<2)
#define EREVID_EREVID1          (1<<1)
#define EREVID_EREVID0          (1)

// ECOCON bits -------
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
//#define N/A                   (1<<3)
#define ECOCON_COCON2           (1<<2)
#define ECOCON_COCON1           (1<<1)
#define ECOCON_COCON0           (1)
// ECOCON values -------
//#define Reserved              (ECOCON_COCON2+ECOCON_COCON1+ECOCON_COCON0)                     //Factory tests
//#define Reserved              (ECOCON_COCON2+ECOCON_COCON1)                                   //Factory tests
#define ECOCON_CLKOUT_8         (ECOCON_COCON2+ECOCON_COCON0)                                   //CLKOUT outputs main clock divided by 8 (3.125MHz)
#define ECOCON_CLKOUT_4         (ECOCON_COCON2)                                                 //CLKOUT outputs main clock divided by 4 (6.25MHz)
#define ECOCON_CLKOUT_3         (ECOCON_COCON1+ECOCON_COCON0)                                   //CLKOUT outputs main clock divided by 3 (8.333333MHz)
#define ECOCON_CLKOUT_2         (ECOCON_COCON1)                                                 //CLKOUT outputs main clock divided by 2 (12.5MHz)
#define ECOCON_CLKOUT_1         (ECOCON_COCON0)                                                 //CLKOUT outputs main clock divided by 1 (25MHz)
#define ECOCON_CLKOUT_0         (0)                                                             //CLKOUT disabled
#define ECOCON_COCON            (ECOCON_COCON2+ECOCON_COCON1+ECOCON_COCON0)

// EFLOCON bits -----
//#define N/A                   (1<<7)
//#define N/A                   (1<<6)
//#define N/A                   (1<<5)
//#define N/A                   (1<<4)
//#define N/A                   (1<<3)
#define EFLOCON_FULDPXS         (1<<2)                                                          //Read-Only MAC Full-Duplex Shadow bit - 1: MAC is configured for Full-Duplex mode, FULDPX (MACON<3>) is set, 0: MAC is configured for Half-Duplex mode, FULDPX is clear
#define EFLOCON_FCEN1           (1<<1)                                                          //Flow Control Enable bits - Please see values below
#define EFLOCON_FCEN0           (1)                                                             //Flow Control Enable bits - Please see values below
// EFLOCON values ---
#define EFLOCON_FCEN11          (3)                                                             //If FULDPXS=1, Send one pause frame with a '0' timer value and then turn flow control off. If FULDPXS=0, Flow control on.
#define EFLOCON_FCEN10          (2)                                                             //If FULDPXS=1, Send pause frames periodically. If FULDPXS=0, Flow control off.
#define EFLOCON_FCEN01          (1)                                                             //If FULDPXS=1, Send one pause frame then turn flow control off. If FULDPXS=0, Flow control on.
#define EFLOCON_FCEN00          (0)                                                             //If FULDPXS=1, Flow control off. If FULDPXS=0, Flow control off.
#define EFLOCON_FCEN            (EFLOCON_FCEN1+EFLOCON_FCEN0)

// EPAUSL bits --------
#define EPAUSL_EPAUS7           (1<<7)
#define EPAUSL_EPAUS6           (1<<6)
#define EPAUSL_EPAUS5           (1<<5)
#define EPAUSL_EPAUS4           (1<<4)
#define EPAUSL_EPAUS3           (1<<3)
#define EPAUSL_EPAUS2           (1<<2)
#define EPAUSL_EPAUS1           (1<<1)
#define EPAUSL_EPAUS0           (1)

// EPAUSH bits --------
#define EPAUSH_EPAUS15          (1<<7)
#define EPAUSH_EPAUS14          (1<<6)
#define EPAUSH_EPAUS13          (1<<5)
#define EPAUSH_EPAUS12          (1<<4)
#define EPAUSH_EPAUS11          (1<<3)
#define EPAUSH_EPAUS10          (1<<2)
#define EPAUSH_EPAUS9           (1<<1)
#define EPAUSH_EPAUS8           (1)


// *************************************
// PHY bits

// PHCON1 bits ----------
#define PHCON1_PRST             (1ul<<15)                                                       //PHY Software Reset bit - 1: PHY is processing a Software Reset (automatically resets to '0' when done), 0: Normal operation
#define PHCON1_PLOOPBK          (1ul<<14)                                                       //PHY Loopback bit - 1: All data transmitted will be returned to the MAC. The twited-pair interface will be disabled.
//#define N/A                   (1ul<<13)
//#define N/A                   (1ul<<12)
#define PHCON1_PPWRSV           (1ul<<11)                                                       //PHY Power-Down bit - 1: PHY is shut down, 0: Normal operation
//#define Reserved              (1ul<<10)
//#define N/A                   (1ul<<9)
#define PHCON1_PDPXMD           (1ul<<8)                                                        //PHY Duplex Mode bit - 1: PHY operates in Full-Duplex Mode, 0: PHY operates in Half-Duplex mode.
//#define Reserved              (1ul<<7)
//#define N/A                   (1ul<<6)
//#define N/A                   (1ul<<5)
//#define N/A                   (1ul<<4)
//#define N/A                   (1ul<<3)
//#define N/A                   (1ul<<2)
//#define N/A                   (1ul<<1)
//#define N/A                   (1ul)

// PHSTAT1 bits --------
//#define N/A                   (1ul<<15)
//#define N/A                   (1ul<<14)
//#define N/A                   (1ul<<13)
#define PHSTAT1_PFDPX           (1ul<<12)                                                       //PHY Full-Duplex Capable bit, 1: PHY is capable of operating at 10 Mbps in Full-Duplex mode (this bit is always set)
#define PHSTAT1_PHDPX           (1ul<<11)                                                       //PHY Half-Duplex Capable bit, 1: PHY is capable of operating at 10 Mbps in Half-Duplex mode (this bit is always set)
//#define N/A                   (1ul<<10)
//#define N/A                   (1ul<<9)
//#define N/A                   (1ul<<8)
//#define N/A                   (1ul<<7)
//#define N/A                   (1ul<<6)
//#define N/A                   (1ul<<5)
//#define N/A                   (1ul<<4)
//#define N/A                   (1ul<<3)
#define PHSTAT1_LLSTAT          (1ul<<2)                                                        //PHY Latching Link Status bit, 1: Link is up and has been up continuously since PHSTAT1 was last read, 0: Link is down or was down for a period since PHSTAT1 was last read
#define PHSTAT1_JBSTAT          (1ul<<1)                                                        //PHY Latching Jabber Status bit, 1: PHY has detected a transmission meeting the jabber criteria since PHSTAT1 was last read, 0: PHY has not detected any jabbering transmissions since PHSTAT1 was last read
//#define N/A                   (1ul)

// PHID1 bits --------
#define PHID1_PID18             (1ul<<15)
#define PHID1_PID17             (1ul<<14)
#define PHID1_PID16             (1ul<<13)
#define PHID1_PID15             (1ul<<12)
#define PHID1_PID14             (1ul<<11)
#define PHID1_PID13             (1ul<<10)
#define PHID1_PID12             (1ul<<9)
#define PHID1_PID11             (1ul<<8)
#define PHID1_PID10             (1ul<<7)
#define PHID1_PID9              (1ul<<6)
#define PHID1_PID8              (1ul<<5)
#define PHID1_PID7              (1ul<<4)
#define PHID1_PID6              (1ul<<3)
#define PHID1_PID5              (1ul<<2)
#define PHID1_PID4              (1ul<<1)
#define PHID1_PID3              (1ul)

// PHID2 bits --------
#define PHID2_PID24             (1ul<<15)
#define PHID2_PID23             (1ul<<14)
#define PHID2_PID22             (1ul<<13)
#define PHID2_PID21             (1ul<<12)
#define PHID2_PID20             (1ul<<11)
#define PHID2_PID19             (1ul<<10)
#define PHID2_PPN5              (1ul<<9)
#define PHID2_PPN4              (1ul<<8)
#define PHID2_PPN3              (1ul<<7)
#define PHID2_PPN2              (1ul<<6)
#define PHID2_PPN1              (1ul<<5)
#define PHID2_PPN0              (1ul<<4)
#define PHID2_PREV3             (1ul<<3)
#define PHID2_PREV2             (1ul<<2)
#define PHID2_PREV1             (1ul<<1)
#define PHID2_PREV0             (1ul)

// PHCON2 bits ----------
//#define N/A                   (1ul<<15)
#define PHCON2_FRCLNK           (1ul<<14)                                                       //PHY Force Linkup bit - 1: Force linkup even when no link partner is detected, 0: Normal operation
#define PHCON2_TXDIS            (1ul<<13)                                                       //Twisted-Pair Transmitter Disable bit - 1: Disable twisted-pair transmitter, 0: Normal operation
//#define Reserved              (1ul<<12)
//#define Reserved              (1ul<<11)
#define PHCON2_JABBER           (1ul<<10)                                                       //Jabber Correction Disable bit - 1: Disable jabber correction, 0: Normal operation
//#define Reserved              (1ul<<9)
#define PHCON2_HDLDIS           (1ul<<8)                                                        //PHY Half-Duplex Loopback Disable bit - When PHCON1<8>=1 or PHCON1<14>=1 this bit is ignored, otherwise : 1: Transmitted data will only be sent out on the twisted-pair interface, 0: Transmitted data will be looped back to the MAC and sent out on the twisted-pair interface
//#define Reserved              (1ul<<7)
//#define Reserved              (1ul<<6)
//#define Reserved              (1ul<<5)
//#define Reserved              (1ul<<4)
//#define Reserved              (1ul<<3)
//#define Reserved              (1ul<<2)
//#define Reserved              (1ul<<1)
//#define Reserved              (1ul)

// PHSTAT2 bits --------
//#define N/A                   (1ul<<15)
//#define N/A                   (1ul<<14)
#define PHSTAT2_TXSTAT          (1ul<<13)                                                       //PHY Transmit Status bit, 1: PHY is transmitting data, 0: PHY is not transmitting data
#define PHSTAT2_RXSTAT          (1ul<<12)                                                       //PHY Receive Status bit, 1:PHY is receiving data, 0: PHY is not receiving data
#define PHSTAT2_COLSTAT         (1ul<<11)                                                       //PHY Collision Status bit, 1: A collision is occuring, 0: A collision is not occuring
#define PHSTAT2_LSTAT           (1ul<<10)                                                       //PHY Link Status bit (non-latching), 1: Link is up, 0: Link is down
#define PHSTAT2_DPXSTAT         (1ul<<9)                                                        //PHY Duplex Status bit, 1:PHY is configured for full-duplex operation (PHCON<8> set), 0: PHY is configured for half-duplex operation (PHCON<8> clear)
//#define N/A                   (1ul<<8)
//#define N/A                   (1ul<<7)
//#define N/A                   (1ul<<6)
#define PHSTAT2_PLRITY          (1ul<<5)                                                        //Polarity Status bit, 1: polarity of the signal on TPIN+/TPIN- is reversed, 0: polarity of the signal on TPIN+/TPIN- is correct
//#define N/A                   (1ul<<4)
//#define N/A                   (1ul<<3)
//#define N/A                   (1ul<<2)
//#define N/A                   (1ul<<1)
//#define N/A                   (1ul)

// PHIE bits -----------
//#define Reserved              (1ul<<15)
//#define Reserved              (1ul<<14)
//#define Reserved              (1ul<<13)
//#define Reserved              (1ul<<12)
//#define Reserved              (1ul<<11)
//#define Reserved              (1ul<<10)
//#define Reserved              (1ul<<9)
//#define Reserved              (1ul<<8)
//#define Reserved              (1ul<<7)
//#define Reserved              (1ul<<6)
//#define Reserved              (1ul<<5)
#define PHIE_PLNKIE             (1ul<<4)                                                        //PHY Link Change Interrupt Enable bit - 1: PHY link change interrupt is enabled, 0: PHY link change interrupt is disabled
//#define Reserved              (1ul<<3)
//#define Reserved              (1ul<<2)
#define PHIE_PGEIE              (1ul<<1)                                                        //PHY Global Interrupt Enable bit - 1: PHY interrupts are enabled, 0: PHY interrupts are disabled
//#define Reserved              (1ul)

// PHIR bits -----------
//#define Reserved              (1ul<<15)
//#define Reserved              (1ul<<14)
//#define Reserved              (1ul<<13)
//#define Reserved              (1ul<<12)
//#define Reserved              (1ul<<11)
//#define Reserved              (1ul<<10)
//#define Reserved              (1ul<<9)
//#define Reserved              (1ul<<8)
//#define Reserved              (1ul<<7)
//#define Reserved              (1ul<<6)
//#define Reserved              (1ul<<5)
#define PHIR_PLNKIF             (1ul<<4)                                                        //PHY Link Change Interrupt Flag bit - 1: PHY link status has changed since PHIR was last read, resets to 0 when read, 0: PHY link status has not changed since PHIR was last read
//#define Reserved              (1ul<<3)
#define PHIR_PGIF               (1ul<<2)                                                        //PHY Global Interrupt Flag bit - 1: One or more enabled PHY interrupts have occurred since PHIR was last read, resets to 0 when read, 0: No PHY interrupts have occurred
//#define Reserved              (1ul<<1)
//#define Reserved              (1ul)

// PHLCON bits -------
//#define Reserved              (1ul<<15)
//#define Reserved              (1ul<<14)
//#define Reserved              (1ul<<13)
//#define Reserved              (1ul<<12)
#define PHLCON_LACFG3           (1ul<<11)
#define PHLCON_LACFG2           (1ul<<10)
#define PHLCON_LACFG1           (1ul<<9)
#define PHLCON_LACFG0           (1ul<<8)
#define PHLCON_LBCFG3           (1ul<<7)
#define PHLCON_LBCFG2           (1ul<<6)
#define PHLCON_LBCFG1           (1ul<<5)
#define PHLCON_LBCFG0           (1ul<<4)
#define PHLCON_LFRQ1            (1ul<<3)
#define PHLCON_LFRQ0            (1ul<<2)
#define PHLCON_STRCH            (1ul<<1)
//#define Reserved              (1ul)
// PHLCON values LEDA -------
//#define Reserved              (0)                                                             //Reserved
#define PHLCON_LA_TA            (PHLCON_LACFG0)                                                 //Display transmit activity (stretchable)
#define PHLCON_LA_RA            (PHLCON_LACFG1)                                                 //Display receive activity (stretchable)
#define PHLCON_LA_CA            (PHLCON_LACFG1+PHLCON_LACFG0)                                   //Display collision activity (stretchable)
#define PHLCON_LA_LS            (PHLCON_LACFG2)                                                 //Display link status
#define PHLCON_LA_DS            (PHLCON_LACFG2+PHLCON_LACFG0)                                   //Display duplex status
//#define Reserved              (PHLCON_LACFG2+PHLCON_LACFG1)                                   //Reserved
#define PHLCON_LA_TRA           (PHLCON_LACFG2+PHLCON_LACFG1+PHLCON_LACFG0)                     //Display transmit and receive activity (stretchable)
#define PHLCON_LA_ON            (PHLCON_LACFG3)                                                 //On
#define PHLCON_LA_OFF           (PHLCON_LACFG3+PHLCON_LACFG0)                                   //Off
#define PHLCON_LA_BF            (PHLCON_LACFG3+PHLCON_LACFG1)                                   //Blink fast
#define PHLCON_LA_BS            (PHLCON_LACFG3+PHLCON_LACFG1+PHLCON_LACFG0)                     //Blink slow
#define PHLCON_LA_LSRA          (PHLCON_LACFG3+PHLCON_LACFG2)                                   //Display link status and receive activity (always stretched)
#define PHLCON_LA_LSTRA         (PHLCON_LACFG3+PHLCON_LACFG2+PHLCON_LACFG0)                     //Display link status and transmit/receive activity (always stretched)
#define PHLCON_LA_DSCA          (PHLCON_LACFG3+PHLCON_LACFG2+PHLCON_LACFG1)                     //Display duplex status and collision activity (always stretched)
//#define Reserved              (PHLCON_LACFG3+PHLCON_LACFG2+PHLCON_LACFG1+PHLCON_LACFG0)       //Reserved
#define PHLCON_LACFG            (PHLCON_LACFG3+PHLCON_LACFG2+PHLCON_LACFG1+PHLCON_LACFG0)
// PHLCON values LEDB -------
//#define Reserved              (0)                                                             //Reserved
#define PHLCON_LB_TA            (PHLCON_LBCFG0)                                                 //Display transmit activity (stretchable)
#define PHLCON_LB_RA            (PHLCON_LBCFG1)                                                 //Display receive activity (stretchable)
#define PHLCON_LB_CA            (PHLCON_LBCFG1+PHLCON_LBCFG0)                                   //Display collision activity (stretchable)
#define PHLCON_LB_LS            (PHLCON_LBCFG2)                                                 //Display link status
#define PHLCON_LB_DS            (PHLCON_LBCFG2+PHLCON_LBCFG0)                                   //Display duplex status
//#define Reserved              (PHLCON_LBCFG2+PHLCON_LBCFG1)                                   //Reserved
#define PHLCON_LB_TRA           (PHLCON_LBCFG2+PHLCON_LBCFG1+PHLCON_LBCFG0)                     //Display transmit and receive activity (stretchable)
#define PHLCON_LB_ON            (PHLCON_LBCFG3)                                                 //On
#define PHLCON_LB_OFF           (PHLCON_LBCFG3+PHLCON_LBCFG0)                                   //Off
#define PHLCON_LB_BF            (PHLCON_LBCFG3+PHLCON_LBCFG1)                                   //Blink fast
#define PHLCON_LB_BS            (PHLCON_LBCFG3+PHLCON_LBCFG1+PHLCON_LBCFG0)                     //Blink slow
#define PHLCON_LB_LSRA          (PHLCON_LBCFG3+PHLCON_LBCFG2)                                   //Display link status and receive activity (always stretched)
#define PHLCON_LB_LSTRA         (PHLCON_LBCFG3+PHLCON_LBCFG2+PHLCON_LBCFG0)                     //Display link status and transmit/receive activity (always stretched)
#define PHLCON_LB_DSCA          (PHLCON_LBCFG3+PHLCON_LBCFG2+PHLCON_LBCFG1)                     //Display duplex status and collision activity (always stretched)
//#define Reserved              (PHLCON_LBCFG3+PHLCON_LBCFG2+PHLCON_LBCFG1+PHLCON_LBCFG0)       //Reserved
#define PHLCON_LBCFG           (PHLCON_LBCFG3+PHLCON_LBCFG2+PHLCON_LBCFG1+PHLCON_LBCFG0)
// PHLCON values LED Pulse Stretch Time -------
//#define Reserved              (PHLCON_LFRQ1+PHLCON_LFRQ0)                                     //Reserved
#define PHLCON_LFRQ_140         (PHLCON_LFRQ1)                                                  //Stretch LED events to apprx 140ms
#define PHLCON_LFRQ_70          (PHLCON_LFRQ0)                                                  //Stretch LED events to apprx 70ms
#define PHLCON_LFRQ_40          (0)                                                             //Stretch LED events to apprx 40ms
#define PHLCON_LFRQ             (PHLCON_LFRQ1+PHLCON_LFRQ0)
// PHLCON values LED Pulse Stretching Activation -------
#define PHLCON_STRCH_ON         (PHLCON_STRCH)                                                  //Stretchable LED events will cause lengthened LED pulses based on the LFRQ configuration
#define PHLCON_STRCH_OFF        (0)                                                             //Stretchable LED events will only be displayed while they are occurring

#endif
