#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H
#include "stm32f4xx.h"
void FlashProgramedata (u16 data);
void FlashAllErase (void);
void FlashProgramedataBoot (u16 data);
void FlashAllEraseBoot (void);
int FlashWriteK(u8 *data);
int FlashReadK(u8 *data);
#endif
