#ifndef RC_H
#define RC_H

#define RC_NVIC 4
typedef signed short int int16_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
