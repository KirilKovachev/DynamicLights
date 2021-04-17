
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#define ADC_BAT_PIN 33

#define NO_OF_SAMPLES   8          //Multisampling


esp_adc_cal_characteristics_t *adc_chars = new esp_adc_cal_characteristics_t;//adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));

adc1_channel_t get_adc1_chanel(uint8_t pin) {
  adc1_channel_t chan;
  switch (pin) {
    case 32:
      chan = ADC1_CHANNEL_4;
      break;
    case 33:
      chan = ADC1_CHANNEL_5;
      break;
    case 34:
      chan = ADC1_CHANNEL_6;
      break;
    case 35:
      chan = ADC1_CHANNEL_7;
      break;
    case 36:
      chan = ADC1_CHANNEL_0;
      break;
    case 37:
      chan = ADC1_CHANNEL_1;
      break;
    case 38:
      chan = ADC1_CHANNEL_2;
      break;
    case 39:
      chan = ADC1_CHANNEL_3;
      break;
  }
  return chan;
}

uint32_t read_bat_adc_idf() {
  uint32_t adc_reading = 0;

  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)get_adc1_chanel(ADC_BAT_PIN));
  }
  adc_reading /= NO_OF_SAMPLES;
  return adc_reading;
}

void adc_setup() {
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(get_adc1_chanel(ADC_BAT_PIN), ADC_ATTEN_11db );
}
//

void setup() {
  Serial.begin(115200);
  adc_setup();

}

void loop() {
  // put your main code here, to run repeatedly:
 uint32_t bat_test = read_bat_adc_idf();
 Serial.println(bat_test);

}
