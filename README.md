# bl702_adc_test

USB ADC 16 bits (averaging) test BL702


#XT-ZB1Kit
To operate USB BL702C, it is necessary to cut the wire in the area of contact (16) CH340C and solder the wire connecting contact (2) AMS1117-3.3 and contacts (4, 16) CH340C.
![CH340C_xtzb1](https://github.com/pvvx/bl702_adc_test/blob/main/img/CH340C_xtzb1.jpg)
ADC - GPIO_9 (mark D9)
DAC - GPIO_17 (mark D17)

---

#XT-ZB1Kit Tests USB ADC-DAC, SDK v2.0

src/sdk2/project_bl702.bin - Test Firmware
TestsTools/adc/wso_adcs.html - ADC data reception via USB

![usb-adc-dac-sdkv2.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/usb-adc-dac-sdkv2.png)


---

Comparison with a linear function
![cmplinf.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/cmplinf.png)

Nonlinearity and Noise
![nlinn.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/nlinn.png)


