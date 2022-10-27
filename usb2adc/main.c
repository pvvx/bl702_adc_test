/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "../usb2adc/dev_cfg.h"
#include "bflb_platform.h"
#include "hal_usb.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
//#include "uart_interface.h"
#include "bl702_ef_ctrl.h"
#include "bl702_glb.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_dma.h"

#define CDC_IN_EP  0x82
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x02, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'B', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'u', 0x00,                  /* wcChar2 */
    'f', 0x00,                  /* wcChar3 */
    'f', 0x00,                  /* wcChar4 */
    'a', 0x00,                  /* wcChar5 */
    'l', 0x00,                  /* wcChar6 */
    'o', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x20,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'B', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'u', 0x00,                  /* wcChar2 */
    'f', 0x00,                  /* wcChar3 */
    'f', 0x00,                  /* wcChar4 */
    'a', 0x00,                  /* wcChar5 */
    'l', 0x00,                  /* wcChar6 */
    'o', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'S', 0x00,                  /* wcChar9 */
    'e', 0x00,                  /* wcChar10 */
    'r', 0x00,                  /* wcChar11 */
    'i', 0x00,                  /* wcChar13 */
    'a', 0x00,                  /* wcChar14 */
    'l', 0x00,                  /* wcChar15 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x30,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'F', 0x00,                  /* wcChar0 */
    'a', 0x00,                  /* wcChar1 */
    'c', 0x00,                  /* wcChar2 */
    't', 0x00,                  /* wcChar3 */
    'o', 0x00,                  /* wcChar4 */
    'r', 0x00,                  /* wcChar5 */
    'y', 0x00,                  /* wcChar6 */
    'A', 0x00,                  /* wcChar7 */
    'I', 0x00,                  /* wcChar8 */
    'O', 0x00,                  /* wcChar9 */
    'T', 0x00,                  /* wcChar10 */
    ' ', 0x00,                  /* wcChar11 */
    'P', 0x00,                  /* wcChar12 */
    'r', 0x00,                  /* wcChar13 */
    'o', 0x00,                  /* wcChar14 */
    'g', 0x00,                  /* wcChar15 */
    ' ', 0x00,                  /* wcChar16 */
    'S', 0x00,                  /* wcChar17 */
    'e', 0x00,                  /* wcChar18 */
    'r', 0x00,                  /* wcChar19 */
    'i', 0x00,                  /* wcChar20 */
    'a', 0x00,                  /* wcChar21 */
    'l', 0x00,                  /* wcChar22 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};
struct device *usb_fs;

extern struct device *usb_dc_init(void);

#define USB_RX_RINGBUFFER_SIZE (4 * 1024)
#define USB_TX_RINGBUFFER_SIZE (4 * 1024)
uint8_t usb_rx_mem[USB_RX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));
uint8_t usb_tx_mem[USB_TX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));

Ring_Buffer_Type usb_rx_rb;
Ring_Buffer_Type usb_tx_rb;

#define ADC_DATA_BLK_CNT	16

adc_channel_t posChList[] = { ADC_CHANNEL1 };
adc_channel_t negChList[] = { ADC_CHANNEL_GND };

struct device *adc_test;
struct device *dma_ch0;

uint32_t adc_buffer[ADC_DATA_BLK_CNT];
struct {
	uint16_t id;
	uint16_t data[ADC_DATA_BLK_CNT];
} adc_blk;

volatile uint32_t adc_read_cnt;
uint32_t adc_overflow_cnt; // overflow

adc_data_parse_t data_parse;


static void hexarr2string(uint8_t *hexarray, int length, uint8_t *string)
{
    unsigned char num2string_table[] = "0123456789ABCDEF";
    int i = 0;

    while (i < length) {
        *(string++) = num2string_table[((hexarray[i] >> 4) & 0x0f)];
        *(string++) = num2string_table[(hexarray[i] & 0x0f)];
        i++;
    }
}

void usbd_cdc_acm_bulk_out(uint8_t ep)
{
    usb_dc_receive_to_ringbuffer(usb_fs, &usb_rx_rb, ep);
}

void usbd_cdc_acm_bulk_in(uint8_t ep)
{
    usb_dc_send_from_ringbuffer(usb_fs, &usb_tx_rb, ep);
}

void usbd_cdc_acm_set_line_coding(uint32_t baudrate, uint8_t databits, uint8_t parity, uint8_t stopbits)
{
    //uart1_config(baudrate, databits, parity, stopbits);
}

void usbd_cdc_acm_set_dtr(bool dtr)
{
	if(dtr) {
		adc_read_cnt = 0;
		adc_overflow_cnt = 0;
		adc_channel_start(adc_test);
	} else {
		adc_channel_stop(adc_test);
	}
    //dtr_pin_set(!dtr);
}

void usbd_cdc_acm_set_rts(bool rts)
{
    //rts_pin_set(!rts);
}

usbd_class_t cdc_class;
usbd_interface_t cdc_cmd_intf;
usbd_interface_t cdc_data_intf;

usbd_endpoint_t cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

usbd_endpoint_t cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

void dma_ch0_irq_callback(struct device *dev, void *args, uint32_t size, uint32_t state)
{
	if(Ring_Buffer_Get_Empty_Length(&usb_tx_rb) >= ADC_DATA_BLK_CNT*2) {
	    for (uint8_t i = 0; i < ADC_DATA_BLK_CNT; i++) {
	    	adc_blk.data[i] = (uint16_t)adc_buffer[i];
	    }
	    adc_blk.id = 0x0A00 + ADC_DATA_BLK_CNT*2;
	    Ring_Buffer_Write(&usb_tx_rb, (uint8_t *)&adc_blk, sizeof(adc_blk));
		adc_read_cnt++;
	} else {
		adc_overflow_cnt++;
	}
}

int adc_init(void)
{
    adc_channel_cfg_t adc_channel_cfg;

    adc_channel_cfg.pos_channel = posChList;
    adc_channel_cfg.neg_channel = negChList;
    adc_channel_cfg.num = 1;

    adc_register(ADC0_INDEX, "adc");

    adc_test = device_find("adc");

    if (adc_test) {
    	// ADC_CLOCK_DIV_32, ADC_DATA_WIDTH_16B_WITH_256_AVERAGE = 3906.25 sps
    	// ADC_CLOCK_DIV_16, ADC_DATA_WIDTH_16B_WITH_128_AVERAGE = 15625 sps
    	// ADC_CLOCK_DIV_16, ADC_DATA_WIDTH_14B_WITH_64_AVERAGE = 31250 sps
    	// ADC_CLOCK_DIV_16, ADC_DATA_WIDTH_14B_WITH_16_AVERAGE = 125000 sps
    	ADC_DEV(adc_test)->clk_div = ADC_CLOCK_DIV_32;
    	ADC_DEV(adc_test)->vref = ADC_VREF_3V2;
    	ADC_DEV(adc_test)->continuous_conv_mode = ENABLE;
    	ADC_DEV(adc_test)->differential_mode = DISABLE;
        ADC_DEV(adc_test)->data_width = ADC_DATA_WIDTH_14B_WITH_16_AVERAGE;
        ADC_DEV(adc_test)->fifo_threshold = ADC_FIFO_THRESHOLD_16BYTE;
    	ADC_DEV(adc_test)->gain = ADC_GAIN_1;
        device_open(adc_test, DEVICE_OFLAG_DMA_RX);
        if (device_control(adc_test, DEVICE_CTRL_ADC_CHANNEL_CONFIG, &adc_channel_cfg) == ERROR) {
            MSG("ADC channel config error , Please check the channel corresponding to IO is initial success by board system or Channel is invaild \r\n");
            BL_CASE_FAIL;
            return 1;
        }
        MSG("adc device find success\r\n");
    } else
    	return 1;
    dma_register(DMA0_CH0_INDEX, "dma_ch0");
    dma_ch0 = device_find("dma_ch0");

    if (dma_ch0) {
        DMA_DEV(dma_ch0)->direction = DMA_PERIPH_TO_MEMORY;
        DMA_DEV(dma_ch0)->transfer_mode = DMA_LLI_ONCE_MODE;
        DMA_DEV(dma_ch0)->src_req = DMA_REQUEST_ADC0;
        DMA_DEV(dma_ch0)->dst_req = DMA_REQUEST_NONE;
        DMA_DEV(dma_ch0)->src_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
        DMA_DEV(dma_ch0)->dst_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
        DMA_DEV(dma_ch0)->src_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_ch0)->dst_burst_size = DMA_BURST_1BYTE;
        DMA_DEV(dma_ch0)->src_width = DMA_TRANSFER_WIDTH_32BIT;
        DMA_DEV(dma_ch0)->dst_width = DMA_TRANSFER_WIDTH_32BIT;
        device_open(dma_ch0, 0);
        device_set_callback(dma_ch0, dma_ch0_irq_callback);
        device_control(dma_ch0, DEVICE_CTRL_SET_INT, NULL);
    }

    /* connect dac device and dma device */
    device_control(adc_test, DEVICE_CTRL_ATTACH_RX_DMA, dma_ch0);

    data_parse.input = adc_buffer;
    data_parse.output = NULL;
    data_parse.num = ADC_DATA_BLK_CNT;

//    adc_read_cnt = 0;
//    adc_overflow_cnt = 0;

//    adc_channel_start(adc_test);
    return DEVICE_EOK;
}

void _ringbuffer_lock()
{
    cpu_global_irq_disable();
}
void _ringbuffer_unlock()
{
    cpu_global_irq_enable();
}

void main_ringbuffer_init(void)
{
    /* init mem for ring_buffer */
    memset(usb_rx_mem, 0, USB_RX_RINGBUFFER_SIZE);
    memset(usb_tx_mem, 0, USB_TX_RINGBUFFER_SIZE);

    /* init ring_buffer */
    Ring_Buffer_Init(&usb_rx_rb, usb_rx_mem, USB_RX_RINGBUFFER_SIZE, _ringbuffer_lock, _ringbuffer_unlock);
    Ring_Buffer_Init(&usb_tx_rb, usb_tx_mem, USB_TX_RINGBUFFER_SIZE, _ringbuffer_lock, _ringbuffer_unlock);
}

void main_read_command(void)
{
    if (Ring_Buffer_Get_Length(&usb_rx_rb)) {
#if 1
    	Ring_Buffer_Reset(&usb_rx_rb);
#else
    	uint32_t avalibleCnt = Ring_Buffer_Read(&usb_rx_rb, src_buffer, USB_TX_BLK_SIZE);
    	if (avalibleCnt) {

    	}
#endif
    }
}

int main(void)
{
    uint8_t chipid[8];
    uint8_t chipid2[6];

    bflb_platform_init(0);
    main_ringbuffer_init();

    usbd_desc_register(cdc_descriptor);

    EF_Ctrl_Read_Chip_ID(chipid);
    hexarr2string(&chipid[2], 3, chipid2);
    // bflb_platform_dump(chipid,8);
    // bflb_platform_dump(chipid2,6);
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24] = chipid2[0];
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24 + 2] = chipid2[1];
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24 + 4] = chipid2[2];
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24 + 6] = chipid2[3];
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24 + 8] = chipid2[4];
    cdc_descriptor[0x12 + USB_CONFIG_SIZE + 0x04 + 0x12 + 0x20 + 0x24 + 10] = chipid2[5];

    usbd_cdc_add_acm_interface(&cdc_class, &cdc_cmd_intf);
    usbd_cdc_add_acm_interface(&cdc_class, &cdc_data_intf);
    usbd_interface_add_endpoint(&cdc_data_intf, &cdc_out_ep);
    usbd_interface_add_endpoint(&cdc_data_intf, &cdc_in_ep);

    usb_fs = usb_dc_init();

    if (usb_fs) {
        device_control(usb_fs, DEVICE_CTRL_SET_INT, (void *)(USB_EP1_DATA_OUT_IT | USB_EP2_DATA_IN_IT));
    }

    while (!usb_device_is_configured()) {
    }

    adc_init();

    while (1) {
    	main_read_command();
        if (!dma_channel_check_busy(dma_ch0)) {
            device_read(adc_test, 0, adc_buffer, sizeof(adc_buffer) / sizeof(uint8_t));  /* size need convert to uint8_t*/
        }
    }
}
