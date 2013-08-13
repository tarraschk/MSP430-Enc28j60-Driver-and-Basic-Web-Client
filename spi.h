#ifndef SPI_H
#define SPI_H

// Function Prototypes
void initSPI(void);
unsigned int SPIWrite(unsigned char*, unsigned int);
unsigned int SPIRead(unsigned char*, unsigned int);
void enc_lightLeds(void);
void enc_shutdownLeds(void);

#endif /* SPI_H */
