#include "usbh_core.h"
//#include "bflb_mtimer.h"
#include "board.h"
#include "bl702_common.h"
#include "adc_dma.h"


extern void cdc_acm_init(void);
extern void dac_init(void);
extern void usb_dc_send_from_ringbuffer(Ring_Buffer_Type *rb);


int main(void)
{
    board_init();
    adc_dma_init(100000); // If used USB2.0FS -> max 166666 sps!
    cdc_acm_init(); // BL702 USB2.0FS max output 1000 kbytes/s
    dac_init();

    while (1) {
    	usb_dc_send_from_ringbuffer(&usb_tx_rb);
    }
}
