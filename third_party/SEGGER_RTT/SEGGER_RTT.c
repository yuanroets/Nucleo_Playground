/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*/

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* RTT up-buffer descriptor. */
typedef struct {
  char *pBuffer;
  unsigned int BufferSize;
  unsigned int Pos;
  unsigned int Flags;
  int tid;
} SEGGER_RTT_BUFFER_UP;

/* RTT down-buffer descriptor. */
typedef struct {
  char *pBuffer;
  unsigned int BufferSize;
  unsigned int Pos;
  unsigned int Flags;
} SEGGER_RTT_BUFFER_DOWN;

/* RTT control block. */
typedef struct {
  char acID[16];
  unsigned int MaxNumUpBuffers;
  unsigned int MaxNumDownBuffers;
  SEGGER_RTT_BUFFER_UP aUp[NUM_UP_BUFFERS];
  SEGGER_RTT_BUFFER_DOWN aDown[NUM_DOWN_BUFFERS];
} SEGGER_RTT_CB;

/* RTT magic number for identification. */
#define RTT_CTRL_MAGIC_NUMBER (0x19681122)

/* Static control block placed in memory where debugger can find it. */
__attribute__((section(".rtt_control")))
static SEGGER_RTT_CB _SEGGER_RTT = {
  .acID = "SEGGER RTT",
  .MaxNumUpBuffers = NUM_UP_BUFFERS,
  .MaxNumDownBuffers = NUM_DOWN_BUFFERS,
};

/* Up buffer (terminal output) storage. */
static char _aUpBuffer0[BUFFER_SIZE_UP];

/* Down buffer (input) storage. */
static char _aDownBuffer0[BUFFER_SIZE_DOWN];

/* Initialize RTT buffers and control structure. */
void SEGGER_RTT_Init(void) {
  if (_SEGGER_RTT.aUp[0].pBuffer == NULL) {
    _SEGGER_RTT.aUp[0].pBuffer = _aUpBuffer0;
    _SEGGER_RTT.aUp[0].BufferSize = BUFFER_SIZE_UP;
    _SEGGER_RTT.aUp[0].Pos = 0;
    _SEGGER_RTT.aUp[0].Flags = 0;

    _SEGGER_RTT.aDown[0].pBuffer = _aDownBuffer0;
    _SEGGER_RTT.aDown[0].BufferSize = BUFFER_SIZE_DOWN;
    _SEGGER_RTT.aDown[0].Pos = 0;
    _SEGGER_RTT.aDown[0].Flags = 0;
  }
}

/* Send single character to RTT terminal. */
void SEGGER_RTT_PutChar(char c) {
  SEGGER_RTT_BUFFER_UP *pUp = &_SEGGER_RTT.aUp[0];

  if (pUp->pBuffer == NULL) {
    return;  /* Not initialized. */
  }

  SEGGER_RTT_LOCK();

  pUp->pBuffer[pUp->Pos] = c;
  pUp->Pos = (pUp->Pos + 1) % pUp->BufferSize;

  SEGGER_RTT_UNLOCK();
}

/* Send string to RTT terminal. */
void SEGGER_RTT_WriteString(const char *s) {
  if (s == NULL) {
    return;
  }

  while (*s) {
    SEGGER_RTT_PutChar(*s++);
  }
}

/* Formatted printf output to RTT. */
int SEGGER_RTT_printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  SEGGER_RTT_WriteString(buf);

  return len;
}
