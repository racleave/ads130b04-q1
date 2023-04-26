
#include <SPI.h>

#include "ads130b04-q1.h"

/****************************

MOSI GPIO13

MISO GPIO12

SCLK GPIO14

CS GPIO15

DRDY GPIO04

 ***************************/

#define MOSI_PIN 13 // DIN
#define MISO_PIN 12 // DOUT
#define SCLK_PIN 14 // SCLK
#define ADC_CS_PIN 15 // CS
#define ADC_DATA_READY_PIN 4 // DRDY
#define CLK_OUT_CONTROL_PIN 17 // CLKIN

#define CLK_OUT_PWM_CH 0
#define CLK_OUT_PWM_RES 1
#define CLK_OUT_PWM_FREQ 8192000

// Instantiate library.
ADS130B04Q1 adc(ADC_CS_PIN, ADC_DATA_READY_PIN, 25000000);

// Where to put incoming data.
int16_t data[4] = {0, 0, 0, 0};

void setup()
{
  
  Serial.begin(115200);
  Serial.println(__FILE__);
    
  Serial.print("ADS130B04Q1_LIB_VERSION: ");
  Serial.println(ADS130B04Q1_LIB_VERSION);

  ledcSetup(CLK_OUT_PWM_CH, CLK_OUT_PWM_FREQ, CLK_OUT_PWM_RES);
  ledcAttachPin(CLK_OUT_CONTROL_PIN, CLK_OUT_PWM_CH);
  ledcWrite(CLK_OUT_PWM_CH, 1);
  
  pinMode(GPIO_NUM_27, OUTPUT);  
  digitalWrite(GPIO_NUM_27, HIGH);

  // Setup SPI.
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, ADC_CS_PIN);

  // Start the adc at a certain frequency: 250, 500, 1000, 2000, 4000, 8000, 16000, 32000 Hz.
  adc.begin(500);

  //delay(500);

  // The following is not necessary but shows things are working:
  unsigned int idData = adc.readRegister(0x01, 2);
  Serial.print("ID ");
  Serial.println(idData);
  delay(1);
  idData = adc.readRegister(0x01, 2);
  Serial.print("ID ");
  Serial.println(idData);
  
  adc.status();
  adc.read(data);
  adc.read(data);

  // Set up the channels we actually want.
  adc.enableChannels(true, false, true, false);
    
  adc.read(data);
  for (int i = 0; i < 4; i++) {
    Serial.print("\t");
    Serial.print(data[i]);
  }
  Serial.println();
}

unsigned long t = 0;

void loop()
{

  if (digitalRead(ADC_DATA_READY_PIN) == LOW) {
    Serial.print("dt=");
    Serial.print(micros() - t);
    Serial.print("us\t");
    adc.read(data);
    for (int i = 0; i < 4; i++) {
      Serial.print("\t");
      Serial.print(data[i]);
    }
    Serial.println();
    t = micros();
  }
  
}


