# esp8266_ledStrip_audioI
Audio-controlled led strip. The leds stip creates visual effect based on the sounds.
The purpose is to create a decoration lighting for a playing music.

#### Parts

* ESP266 placed on the Node MCU board
* Led Strip WS2812B
* Microphone

#### Pinout
| Periphery                     | ESP8266        | Node MCU PCB |
| ----------------------------- | -------------- | -------------|
| Microphone Analog output      | ADC0 			 | A1 			|
| WS2812B						| GPIO4		 | D2			|
| Debug pin output      		| GPIO5		 | D1			|



