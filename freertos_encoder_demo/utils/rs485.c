/*
 * rs485.c - a modified uartstdio.c for controlling a half duplex RS-485 driver.
 *
 *  Created on: Sep 10, 2019
 *      Author: domya
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/rs485.h"
//#include "FreeRTOS.h"

//*****************************************************************************
//
//! \addtogroup rs485_api
//! @{
//
//*****************************************************************************


//*****************************************************************************
//
// Input ring buffer.  Buffer is full if g_ui32RS485TxReadIndex is one ahead of
// g_ui32RS485TxWriteIndex.  Buffer is empty if the two indices are the same.
//
//*****************************************************************************
static unsigned char g_pcRS485RxBuffer[RS485_RX_BUFFER_SIZE];
static volatile uint32_t g_ui32RS485RxWriteIndex = 0;
static volatile uint32_t g_ui32RS485RxReadIndex = 0;

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the receive buffer.
//
//*****************************************************************************
#define RX_BUFFER_USED          (GetBufferCount(&g_ui32RS485RxReadIndex,  \
                                                &g_ui32RS485RxWriteIndex, \
                                                RS485_RX_BUFFER_SIZE))
#define RX_BUFFER_FREE          (RS485_RX_BUFFER_SIZE - RX_BUFFER_USED)
#define RX_BUFFER_EMPTY         (IsBufferEmpty(&g_ui32RS485RxReadIndex,   \
                                               &g_ui32RS485RxWriteIndex))
#define RX_BUFFER_FULL          (IsBufferFull(&g_ui32RS485RxReadIndex,  \
                                              &g_ui32RS485RxWriteIndex, \
                                              RS485_RX_BUFFER_SIZE))
#define ADVANCE_RX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % RS485_RX_BUFFER_SIZE


//*****************************************************************************
//
// The base address of the chosen UART.
//
//*****************************************************************************
static uint32_t g_ui32RS485Base = 0;

//*****************************************************************************
//
// The base address of the chosen Control Pin.
//
//*****************************************************************************
static uint32_t g_ui32ContrBase = 0;

//*****************************************************************************
//
// The pin number of the chosen control pin.
//
//*****************************************************************************
static uint8_t g_ui8ContrPin = 0;


//*****************************************************************************
//
// The default high value of the control pin.
//
//*****************************************************************************
static uint8_t g_ui8ContrHIGH = 0;

//*****************************************************************************
//
// The default low value of the control pin.
//
//*****************************************************************************
static uint8_t g_ui8ContrLOW = 0;

//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
//static const char * const g_pcHex = "0123456789abcdef";



//*****************************************************************************
//
// The list of possible base addresses for the console UART.
//
//*****************************************************************************
static const uint32_t g_ui32UARTBase[8] =
{
    UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE, UART4_BASE, UART5_BASE,
    UART6_BASE, UART7_BASE
};

//*****************************************************************************
//
// The list of possible interrupts for the console UART.
//
//*****************************************************************************
static const uint32_t g_ui32UARTInt[8] =
{
    INT_UART0, INT_UART1, INT_UART2, INT_UART3, INT_UART4, INT_UART5, INT_UART6, INT_UART7
};

//*****************************************************************************
//
// The port number in use.
//
//*****************************************************************************
static uint32_t g_ui32PortNum;

//*****************************************************************************
//
// The list of UART peripherals.
//
//*****************************************************************************
static const uint32_t g_ui32UARTPeriph[8] =
{
    SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2,
    SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART4, SYSCTL_PERIPH_UART5,
    SYSCTL_PERIPH_UART6, SYSCTL_PERIPH_UART7
};

//*****************************************************************************
//
// The list of GPIO bases.
//
//*****************************************************************************
static const uint32_t g_ui32GPIOBase[8] =
{
     GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE,
     GPIO_PORTD_BASE, GPIO_PORTE_BASE, GPIO_PORTF_BASE,
     GPIO_PORTG_BASE, GPIO_PORTH_BASE
};

//*****************************************************************************
//
// The list of GPIO peripherals.
//
//*****************************************************************************
static const uint32_t g_ui32GPIOPeriph[8] =
{
    SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC,
    SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF,
    SYSCTL_PERIPH_GPIOG, SYSCTL_PERIPH_GPIOH
};

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pui32Read points to the read index for the buffer.
//! \param pui32Write points to the write index for the buffer.
//! \param ui32Size is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is full or \b false otherwise.
//
//*****************************************************************************
static bool
IsBufferFull(volatile uint32_t *pui32Read,
             volatile uint32_t *pui32Write, uint32_t ui32Size)
{
    uint32_t ui32Write;
    uint32_t ui32Read;

    ui32Write = *pui32Write;
    ui32Read = *pui32Read;

    return((((ui32Write + 1) % ui32Size) == ui32Read) ? true : false);
}

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pui32Read points to the read index for the buffer.
//! \param pui32Write points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is empty or \b false otherwise.
//
//*****************************************************************************
static bool
IsBufferEmpty(volatile uint32_t *pui32Read,
              volatile uint32_t *pui32Write)
{
    uint32_t ui32Write;
    uint32_t ui32Read;

    ui32Write = *pui32Write;
    ui32Read = *pui32Read;

    return((ui32Write == ui32Read) ? true : false);
}

//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pui32Read points to the read index for the buffer.
//! \param pui32Write points to the write index for the buffer.
//! \param ui32Size is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************
static uint32_t
GetBufferCount(volatile uint32_t *pui32Read,
               volatile uint32_t *pui32Write, uint32_t ui32Size)
{
    uint32_t ui32Write;
    uint32_t ui32Read;

    ui32Write = *pui32Write;
    ui32Read = *pui32Read;

    return((ui32Write >= ui32Read) ? (ui32Write - ui32Read) :
           (ui32Size - (ui32Read - ui32Write)));
}

//*****************************************************************************
//
//! Writes a string of characters to the UART output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ui32Len is the length of the string to transmit.
//!
//! This function will transmit the string to the UART output.  The number of
//! characters transmitted is determined by the \e ui32Len parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to a UART, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ui32Len parameter to stop transmitting the string, if
//! a null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output FIFO.  In buffered mode,
//! the characters are written to the UART transmit buffer and the call returns
//! immediately.  If insufficient space remains in the transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
int
RS485write(char *pcBuf, uint32_t ui32Len)
{
    unsigned int uIdx;

    //
    // Check for valid UART base address, and valid arguments.
    //
    ASSERT(g_ui32RS485Base != 0);
    ASSERT(pcBuf != 0);

    //
    //Write control pin high.
    //
    MAP_GPIOPinWrite(g_ui32ContrBase, g_ui8ContrPin, g_ui8ContrHIGH);

    //
    //Is message short enough?
    //
    if (ui32Len <= MAX_FIFO_SIZE && MAP_UARTSpaceAvail(g_ui32RS485Base))
    {
        //
        // Send the characters
        //
        for(uIdx = 0; uIdx < ui32Len; uIdx++)
        {
            //
            // Send the character to the UART output.
            //
            MAP_UARTCharPutNonBlocking(g_ui32RS485Base, *pcBuf);
            pcBuf++;
        }

        MAP_UARTCharPut(g_ui32RS485Base, '\n');

        //
        //If everything has been transmitted write control pin Low.
        //
        while(MAP_UARTBusy(g_ui32RS485Base)){
        }
        MAP_GPIOPinWrite(g_ui32ContrBase,g_ui8ContrPin,g_ui8ContrLOW);

        //
        // Return the number of characters written.
        //
        return(uIdx);
    }
    else
    {
        return -1;
    }
}

//*****************************************************************************
//
//! Configures the RS485 driver.
//!
//! \param ui32PortNum is the number of UART port to use for RS485 comms
//! (0-7)
//! \param ui32Baud is the bit rate that the UART is to be configured to use.
//! \param ui32SrcClock is the frequency of the source clock for the UART
//! module.
//! \param ui32ContrPort is the GPIO Port number (A=0, B=1, etc) to use for RS485 flow control
//! \param ui8ContrPin is the GPIO Pin number to use for RS485 flow control
//!
//! This function will configure the specified serial port to be used as a
//! serial console.  The serial parameters are set to the baud rate
//! specified by the \e ui32Baud parameter and use 8 bit, no parity, and 1 stop
//! bit.
//!
//! This function must be called prior to using any of the other UART console
//! functions: UARTprintf() or UARTgets().  This function assumes that the
//! caller has previously configured the relevant UART pins for operation as a
//! UART rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
void
RS485Config(uint32_t ui32PortNum, uint32_t ui32Baud, uint32_t ui32SrcClock, uint32_t ui32ContrPort, uint8_t ui8ContrPin)
{
    //
    // Check the arguments.
    //
    ASSERT((ui32PortNum == 0) || (ui32PortNum == 1) ||
           (ui32PortNum == 2) || (ui32PortNum == 3) ||
           (ui32PortNum == 4) || (ui32PortNum == 5) ||
           (ui32PortNum == 6) || (ui32PortNum == 7));

    //
    // We only allow a single instance to be opened.
    // For safety reasons for now. Not really an issue with the AVATAR Project Hardware.
    //
    ASSERT(g_ui32RS485Base == 0);

    //
    // Check to make sure the UART peripheral is present.
    //
    if(!MAP_SysCtlPeripheralPresent(g_ui32UARTPeriph[ui32PortNum]))
    {
        return;
    }

    //
    // Select the base address of the UART.
    //
    g_ui32RS485Base = g_ui32UARTBase[ui32PortNum];

    //
    //Select the base and pin number of the control pin
    //
    g_ui32ContrBase = g_ui32GPIOBase[ui32ContrPort];
    g_ui8ContrPin = ui8ContrPin;

    g_ui8ContrHIGH = ui8ContrPin;
    g_ui8ContrLOW = 0;

    //
    // Enable the UART peripheral for use.
    //
    MAP_SysCtlPeripheralEnable(g_ui32UARTPeriph[ui32PortNum]);

    //
    // Enable the GPIO peripheral for use.
    //
    MAP_SysCtlPeripheralEnable(g_ui32GPIOPeriph[ui32ContrPort]);

    MAP_GPIOPinWrite(g_ui32ContrBase, g_ui8ContrPin, g_ui8ContrLOW);

    //
    // Configure the UART for 115200, n, 8, 1
    //
    MAP_UARTConfigSetExpClk(g_ui32RS485Base, ui32SrcClock, ui32Baud,
                            (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_WLEN_8));

    //
    // Register RS485 Interrupt
    //
    UARTIntRegister(g_ui32RS485Base, RS485IntHandler);

    //
    // Set the UART to interrupt whenever the TX FIFO is almost empty or
    // when any character is received.
    //
    MAP_UARTFIFOLevelSet(g_ui32RS485Base, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    //
    // Flush both the buffers.
    //
    RS485FlushRx();

    //
    // Remember which interrupt we are dealing with.
    //
    g_ui32PortNum = ui32PortNum;

    //
    // We are configured for buffered output so enable the master interrupt
    // for this UART and the receive interrupts.  We don't actually enable the
    // transmit interrupt in the UART itself until some data has been placed
    // in the transmit buffer.
    //
    MAP_UARTIntDisable(g_ui32RS485Base, 0xFFFFFFFF);
    MAP_UARTIntEnable(g_ui32RS485Base, UART_INT_RX | UART_INT_RT);
    MAP_IntEnable(g_ui32UARTInt[ui32PortNum]);

    //
    // Enable the UART operation.
    //
    MAP_UARTEnable(g_ui32RS485Base);
    MAP_IntMasterEnable();

}

//*****************************************************************************
//
//! A simple function to see if there is data in the Rx Buffer
//!
//! \return Returns the count of characters that are in the buffer.
//!  0 if buffer is empty
//
//*****************************************************************************
int
RS485avail(void)
{
    if(!IsBufferEmpty(g_ui32RS485RxReadIndex,g_ui32RS485RxWriteIndex))
    {
        return RX_BUFFER_USED;
    }
    else
        return 0;
}



//*****************************************************************************
//
//! A simple UART based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the UART.
//! \param ui32Len is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the UART input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to UARTPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling UARTgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************
int
RS485read(char *pcBuf, uint32_t ui32Len)
{
    uint32_t ui32Count = 0;
    int8_t cChar;

    //
    // Check the arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(ui32Len != 0);
    ASSERT(g_ui32RS485Base != 0);

    //
    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    //
    ui32Len--;

    //
    // Process characters until a newline is received.
    //
    while(!RX_BUFFER_EMPTY)
    {
        //
        // Read the next character from the receive buffer.
        //
        if(!RX_BUFFER_EMPTY)
        {
            cChar = g_pcRS485RxBuffer[g_ui32RS485RxReadIndex];
            ADVANCE_RX_BUFFER_INDEX(g_ui32RS485RxReadIndex);

            //
            // See if a newline or escape character was received.
            //
            if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
            {
                //
                // Stop processing the input and end the line.
                //
                break;
            }

            //
            // Process the received character as long as we are not at the end
            // of the buffer.  If the end of the buffer has been reached then
            // all additional characters are ignored until a newline is
            // received.
            //
            if(ui32Count < ui32Len)
            {
                //
                // Store the character in the caller supplied buffer.
                //
                pcBuf[ui32Count] = cChar;

                //
                // Increment the count of characters received.
                //
                ui32Count++;
            }
        }
    }

    //
    // Add a null termination to the string.
    //
    pcBuf[ui32Count] = 0;

    //
    // Return the count of int8_ts in the buffer, not counting the trailing 0.
    //
    return(ui32Count);
}

//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************
unsigned char
RS485getc(void)
{
    unsigned char cChar;

    //
    // Wait for a character to be received.
    //
    if(RX_BUFFER_EMPTY)
    {
        //
        // Block waiting for a character to be received (if the buffer is
        // currently empty).
        //
        return('/0');
    }

    //
    // Read a character from the buffer.
    //
    cChar = g_pcRS485RxBuffer[g_ui32RS485RxReadIndex];
    ADVANCE_RX_BUFFER_INDEX(g_ui32RS485RxReadIndex);

    //
    // Return the character to the caller.
    //
    return(cChar);

}


//*****************************************************************************
//
//! Returns the number of bytes available in the receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to determine the number
//! of bytes of data currently available in the receive buffer.
//!
//! \return Returns the number of available bytes.
//
//*****************************************************************************

int
RS485RxBytesAvail(void)
{
    return(RX_BUFFER_USED);
}



//*****************************************************************************
//
//! Looks ahead in the receive buffer for a particular character.
//!
//! \param ucChar is the character that is to be searched for.
//!
//! This function may be used to look ahead in the
//! receive buffer for a particular character and report its position if found.
//! It is typically used to determine whether a complete message of user input is
//! available, in which case ucChar should be set to CR ('\\r') which is used
//! as the line end marker in the receive buffer.
//!
//! \return Returns -1 to indicate that the requested character does not exist
//! in the receive buffer.  Returns a non-negative number if the character was
//! found in which case the value represents the position of the first instance
//! of \e ucChar relative to the receive buffer read pointer.
//
//*****************************************************************************
int
RS485Peek(unsigned char ucChar)
{
    int iCount;
    int iAvail;
    uint32_t ui32ReadIndex;

    //
    // How many characters are there in the receive buffer?
    //
    iAvail = (int)RX_BUFFER_USED;
    ui32ReadIndex = g_ui32RS485RxReadIndex;

    //
    // Check all the unread characters looking for the one passed.
    //
    for(iCount = 0; iCount < iAvail; iCount++)
    {
        if(g_pcRS485RxBuffer[ui32ReadIndex] == ucChar)
        {
            //
            // We found it so return the index
            //
            return(iCount);
        }
        else
        {
            //
            // This one didn't match so move on to the next character.
            //
            ADVANCE_RX_BUFFER_INDEX(ui32ReadIndex);
        }
    }

    //
    // If we drop out of the loop, we didn't find the character in the receive
    // buffer.
    //
    return(-1);
}

//*****************************************************************************
//
//! Flushes the receive buffer.
//!
//! \return None.
//
//*****************************************************************************

void
RS485FlushRx(void)
{
    uint32_t ui32Int;

    //
    // Temporarily turn off interrupts.
    //
    ui32Int = MAP_IntMasterDisable();

    //
    // Flush the receive buffer.
    //
    g_ui32RS485RxReadIndex = 0;
    g_ui32RS485RxWriteIndex = 0;

    //
    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    //
    if(!ui32Int)
    {
        MAP_IntMasterEnable();
    }
}





//*****************************************************************************
//
//! Handles UART interrupts.
//!
//! This function handles interrupts from the UART. It
//! will copy data from the UART receive FIFO to the receive buffer if data is
//! available.
//!
//! \return None.
//
//*****************************************************************************
void
RS485IntHandler(void)
{
    uint32_t ui32Ints;
    int32_t i32Char;
    uint8_t cChar;

    //
    // Get and clear the current interrupt source(s)
    //
    ui32Ints = MAP_UARTIntStatus(g_ui32RS485Base, true);
    MAP_UARTIntClear(g_ui32RS485Base, ui32Ints);

    //
    // Are we being interrupted because the TX FIFO has space available?
    //
    if(ui32Ints & UART_INT_TX)
    {
        //
        // Disabled and never re-enabled.
        // Probably okay because we're transmitting as soon as we can.
        // Might be more useful to interrupt when TX FIFO is full.
        //

        MAP_UARTIntDisable(g_ui32RS485Base, UART_INT_TX); //disabled and never re-enabled.
    }

    //
    // Are we being interrupted due to a received character?
    //
    if(ui32Ints & (UART_INT_RX | UART_INT_RT))
    {
        //
        // Get all the available characters from the UART.
        //
        while(MAP_UARTCharsAvail(g_ui32RS485Base))
        {
            //
            // Read a character
            //
            i32Char = MAP_UARTCharGetNonBlocking(g_ui32RS485Base);
            cChar = (unsigned char)(i32Char & 0xFF);

            //
            // If there is space in the receive buffer, put the character
            // there, otherwise throw it away.
            //
            if(!RX_BUFFER_FULL)
            {
                //
                // Store the new character in the receive buffer
                //
                g_pcRS485RxBuffer[g_ui32RS485RxWriteIndex] =
                    (unsigned char)(i32Char & 0xFF);
                ADVANCE_RX_BUFFER_INDEX(g_ui32RS485RxWriteIndex);

            }
            else{
                RS485FlushRx();
            }
        }
    }
}


