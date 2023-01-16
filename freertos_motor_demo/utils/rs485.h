/*
 * rs485.h
 *
 *  Created on: Sep 10, 2019
 *      Author: domya
 */

#ifndef UTILS_RS485_H_
#define UTILS_RS485_H_

#include <stdarg.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//

//maybe try forcing buffered operation for RS-485
//needed due to half duplex hardware - so maybe add in a define for half or full duplex -DY
//*****************************************************************************

#ifndef RS485_RX_BUFFER_SIZE
#define RS485_RX_BUFFER_SIZE     128

#endif

#ifndef MAX_FIFO_SIZE
#define MAX_FIFO_SIZE            16

#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void RS485Config(uint32_t ui32PortNum, uint32_t ui32Baud, uint32_t ui32SrcClock,
                        uint32_t ui32ContrPort, uint8_t ui8ContrPin);
extern int RS485read(char *pcBuf, uint32_t ui32Len);
extern int RS485avail(void);
extern unsigned char RS485getc(void);
extern int RS485write(char *pcBuf, uint32_t ui32Len);
extern int RS485Peek(unsigned char ucChar);
extern void RS485IntHandler(void);
void RS485FlushRx(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* UTILS_RS485_H_ */
