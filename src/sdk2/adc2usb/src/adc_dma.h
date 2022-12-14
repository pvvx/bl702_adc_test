/*
 * adc_dma.h
 *
 *  Created on: 17 нояб. 2022 г.
 *      Author: pvvx
 */

#ifndef ADC_DMA_H_
#define ADC_DMA_H_
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "ring_buffer.h"

#define ADC_DATA_BLK_CNT	(127-2) // (31-2)
//#define USB_RX_RINGBUFFER_SIZE (4 * 1024)
#define USB_TX_RINGBUFFER_SIZE (4 * 1024)

//extern uint8_t usb_rx_mem[USB_RX_RINGBUFFER_SIZE];
//extern uint8_t usb_tx_mem[USB_TX_RINGBUFFER_SIZE];

//extern Ring_Buffer_Type usb_rx_rb;
extern Ring_Buffer_Type usb_tx_rb;

extern struct bflb_device_s *adc;
//extern struct bflb_device_s *dma0_ch0;

extern volatile uint32_t adc_read_cnt;
extern volatile uint32_t adc_overflow_cnt; // overflow

void adc_dma_init(uint32_t sps);

#endif /* ADC_DMA_H_ */
