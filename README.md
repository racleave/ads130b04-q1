
# ADS130B04 LIB

An ESP32 library for setting up and reading from TI's ADS130B04 four
channel 16 bit Analog to Digital Converter.

For platformio, add this to your platformio.ini file:

```
https://github.com/racleave/ads130b04
```

Then

```
#include "ads130b04.h"

ADS130B04 adc(ADC_CS_PIN, ADC_DATA_READY_PIN, 25000000);

adc.enableChannels(true, false, true, false);

int16_t data[4];
adc.read(data);


```

## Status


 - reset is currently not working.
 
 - still a fair bit of ironing out to do...
 
