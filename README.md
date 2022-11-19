# bl702_adc_test

USB ADC 16 bits (averaging) test BL702


### XT-ZB1Kit

To operate USB BL702C, it is necessary to cut the wire in the area of contact (16) CH340C and solder the wire connecting contact (2) AMS1117-3.3 and contacts (4, 16) CH340C.

![CH340C_xtzb1](https://github.com/pvvx/bl702_adc_test/blob/main/img/CH340C_xtzb1.jpg)

* ADC - GPIO_9 (mark D9)
* DAC - GPIO_17 (mark D17)

---

### XT-ZB1Kit Tests USB ADC-DAC, SDK v2.0

src/sdk2/project_bl702.bin - Test Firmware

TestsTools/adc/wso_adcs.html - ADC data reception via USB

![usb-adc-dac-sdkv2.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/usb-adc-dac-sdkv2.png)

---

DAC steps test on ADC (20 ksps, real 15 bits):

![dac-adc-20ksps.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/dac-adc-20ksps.png)

ADC scale 16 bits (0..65535)

---

Checking the ADC (20 ksps 16 bits) to GND through a resistor with a 1 uF electrolytic capacitor and checking with switching on to the INA199A1 output:

![adc-snr-20kps16bits.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/adc-snr-20kps16bits.png)

ADC scale 16 bits (0..65535)

To INA199A1

ENOB: 14,93	Bits

SNR: 91,63	dB

To GND

ENOB: 15.16	Bits

SNR:  93.02	dB

---

Checking the ADC (100 ksps 14 bits) to GND through a resistor with a 1 uF electrolytic capacitor:

![adc-snr-gnd-100kps14bits.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/adc-snr-gnd-100kps14bits.png)

ADC scale 14 bits (0..16383)

ENOB: 13.89	Bits

SNR:  85.38	dB

---

Comparison with a linear function
![cmplinf.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/cmplinf.png)

Nonlinearity and Noise
![nlinn.png](https://github.com/pvvx/bl702_adc_test/blob/main/img/nlinn.png)


