
# ADS130B04-Q1 LIB

An ESP32 library for setting up and reading from TI's ADS130B04-Q1 four
channel 16 bit Analog to Digital Converter.

For platformio, add this to your platformio.ini file:

```
https://github.com/racleave/ads130b04
```

Then

```
#include "ads130b04.h"

ADS130B04Q1 adc(ADC_CS_PIN, ADC_DATA_READY_PIN, 25000000);

// Setup SPI.
SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, ADC_CS_PIN);
    
// Start the adc at a certain frequency: 250, 500, 1000, 2000, 4000, 8000, 16000, 32000 Hz.
adc.begin(500);
  
adc.enableChannels(true, false, true, false);

int16_t data[4] = {0, 0, 0, 0};
adc.read(data);


```

## To do

## Status

 - reset is currently not working.
 
