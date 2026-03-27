/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2023 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*/

#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RTT buffer lock mechanism (not used in single-threaded). */
#define SEGGER_RTT_LOCK()
#define SEGGER_RTT_UNLOCK()

/* Initialize RTT: sets up memory buffer and metadata. */
void SEGGER_RTT_Init(void);

/* Send string to RTT terminal (channel 0). */
void SEGGER_RTT_WriteString(const char *s);

/* Send formatted string (like sprintf). */
int SEGGER_RTT_printf(const char *fmt, ...);

/* Send single character. */
void SEGGER_RTT_PutChar(char c);

#ifdef __cplusplus
}
#endif

#endif  /* SEGGER_RTT_H */
