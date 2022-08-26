#include <BLEDevice.h>
#include <driver/adc.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Adafruit_AHTX0.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define ESP32

//Number of Client WiFi Lamps
#define NUMBER_OF_CLIENTS 1

//GPIO Ports
#define RF_RECEIVE_PORT 35
#define RF_TRANSMIT_PORT 15
#define OPTIONS_KEY_PORT 19
#define SOUND_ANALOG_PORT 33
const uint16_t IR_TRANSMIT = 4;
const uint16_t IR_RECEIVE = 34;

//Infrared Sender/Receiver init
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;  // Milli-Seconds
const uint16_t kFrequency = 38000;  // Frequency modulation in Hz
// The IR transmitter.
IRsend irsend(IR_TRANSMIT);
// The IR receiver.
IRrecv irrecv(IR_RECEIVE, kCaptureBufferSize, kTimeout, false);
// Somewhere to store the captured message.
decode_results results;

//BLE UUIDs
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID                         "e03d88aa-6ed5-11eb-9439-0242ac130002"
#define CHARACTERISTIC_UUID_RF               "f9357db8-6ed5-11eb-9439-0242ac130002"
#define CHARACTERISTIC_UUID_RF_RAW           "a19dba56-ea31-11ec-8fea-0242ac120002"
#define CHARACTERISTIC_UUID_TEMP             "2506d436-6e72-11ec-90d6-0242ac120003"
#define CHARACTERISTIC_UUID_HUMIDITY         "37aa7c46-6e72-11ec-90d6-0242ac120003"
#define CHARACTERISTIC_UUID_BLE_IR_RAW       "0abab4b8-ea31-11ec-8fea-0242ac120002"

BLECharacteristic *pCharacteristicTemp;
BLECharacteristic *pCharacteristicHumidity;

//MIC ADC settings
#define NO_OF_SAMPLES   1        //Multisampling

//Audio LED variables
int checkDelay = 5000;
int numOpModes = 6;
uint32_t brightness = 100;

//Button variables
uint8_t pushButtonPreviousState = LOW;

//Temperature and humidity sensor variables
#define AHTX0_I2CADDR_DEFAULT 0x38
#define AHT10_CHECK_INTERVAL 5000
uint32_t aht10_last_checked = 0; 
Adafruit_AHTX0 aht;

WiFiUDP UDP;
AsyncWebServer server(80);

struct led_command {
  uint8_t opmode;
  uint32_t data;
};

//Define here instead in header file to save space
struct heartbeat_message {
  uint8_t client_id;
  IPAddress IP;
};

struct heartbeat {
  IPAddress IP;
  bool connected;
  uint32_t lastchecked;
};

struct heartbeat heartbeats[NUMBER_OF_CLIENTS];

int opMode = 0;

//Sound analog raw value
uint32_t analogRaw = 0;

sensors_event_t humidity, temp;

// 433mhz RF Codes
bool rf_wall_buttonOn[] = {1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1};
bool rf_wall_buttonOff[] = {1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1};
bool rf_wall_led_on = false;

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

bool str_to_uint16(const char *str, uint16_t *res) {
    char *end;
    errno = 0;
    long val = strtol(str, &end, 10);
    if (errno || end == str || *end != '\0' || val < 0 || val >= UINT16_MAX) {
        return false;
    }
    *res = (uint16_t)val;
    return true;
}

uint32_t read_adc_idf() {
  uint32_t adc_reading = 0;

  //Multisampling audio signal read
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)get_adc1_chanel(SOUND_ANALOG_PORT));
  }
  adc_reading /= NO_OF_SAMPLES;
  return adc_reading;
}

void adc_setup() {
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(get_adc1_chanel(SOUND_ANALOG_PORT),  ADC_ATTEN_11db );
}

void SendRF(const bool button[], uint16_t count, uint8_t pulse_lenght, uint8_t repeat) {
  for (int i=1;i<=repeat;i++) {
     //for (bool n : button) {
      for(uint16_t a=0;a<count;a++) {
       if (button[a] == 0) digitalWrite(RF_TRANSMIT_PORT,LOW);
       if (button[a] == 1) digitalWrite(RF_TRANSMIT_PORT,HIGH);
       delayMicroseconds(pulse_lenght);   
    }
    delayMicroseconds(41000);
    Serial.println("SendRF Called");
      for(uint16_t a = 0;a < count;a++) {
        Serial.print(button[a]);
      }
    Serial.println("SendRF End");
  }
}

void wallLedPowerOn() {
  SendRF(rf_wall_buttonOn,583,64,4);
}

void wallLedPowerOff() {
  SendRF(rf_wall_buttonOff,583,64,4);
}

byte bleCommand = 0;
uint32_t bleData;
bool bleDeviceConnected = false;
bool bleDeviceConnectedLastState = false;

//Callback BLE connect/disconnect
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
          bleDeviceConnected = true;
    }
 
    void onDisconnect(BLEServer* pServer) {
          bleDeviceConnected = false;
    }
};

//Callback BLE on data
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string rxCharacteristicID = characteristic->getUUID().toString();
    std::string rxValue = characteristic->getValue();
    Serial.printf("Received callback for Characteristics: %s \n", rxCharacteristicID.c_str());
    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }
      Serial.println();
      if (rxCharacteristicID == CHARACTERISTIC_UUID_RF) {
        Serial.println("Processing RF received by BLE");
        if (rxValue.find("L1") != -1) bleCommand = 1; //PowerOn Window LED Stripe
        if (rxValue.find("L2") != -1) bleCommand = 2; //PowerOff Window LED Stripe
        if (rxValue.find("L3") != -1) bleCommand = 3;
      }
      if (rxCharacteristicID == CHARACTERISTIC_UUID_RF_RAW) {
        Serial.println("Processing RAW RF received by BLE");
        bool rf_data[583];
        char *token, *str, *tofree;
        tofree = str = strdup(rxValue.c_str());  // We own str's memory now.
        int rf_index=0; uint16_t rf_data_value=0; bool rf_convert_error = false;
        while ((token = strsep(&str, ","))) {
          if(str_to_uint16(token, &rf_data_value))
            rf_data[rf_index] = rf_data_value; 
          else
            rf_convert_error = true;
        rf_index++;
        }
        free(tofree);
        if (!rf_convert_error) {
          for(int i=0;i<rf_index;i++) Serial.printf("Sending RF data: %f \n", rf_data[rf_index]);
          Serial.println();
          SendRF(rf_data,583,64,4);
        }
        else
          Serial.printf("Error converting RF data: %s \n", rxValue);
      }
      //Handle IR Send commands
      //BLE IR RAW
      if (rxCharacteristicID == CHARACTERISTIC_UUID_BLE_IR_RAW) {
        Serial.println("Processing RAW IR received by BLE");
        bleCommand = 20;
        uint16_t ir_data[300];
        char *token, *str, *tofree;
        tofree = str = strdup(rxValue.c_str());  // We own str's memory now.
        int ir_index=0; uint16_t ir_data_value=0; bool ir_convert_error = false;
        while ((token = strsep(&str, ","))) {
          if(str_to_uint16(token, &ir_data_value))
            ir_data[ir_index] = ir_data_value; 
          else
            ir_convert_error = true;
        ir_index++;
        }
        free(tofree);
        if (!ir_convert_error) {
          for(int i=0;i<ir_index;i++) Serial.printf("Sending IR data: %f \n", ir_data[ir_index]);
          Serial.println();
          irsend.sendRaw(ir_data, ir_index , kFrequency);
        }
        else
          Serial.printf("Error converting IR data: %s \n", rxValue);
      }
    }
  }//onWrite
};

void setup() {

  //Set serial baud rate
  Serial.begin(115200);

  //WiFi setup
  Serial.println("Setting soft-AP ... ");
  WiFi.persistent(false);
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP("dynamicLights", "k0k0mqvka1");
  Serial.print("Soft-AP IP address is: ");
  Serial.println(WiFi.softAPIP());
  UDP.begin(7171);

  //Initialize web serial logging
  WebSerial.begin(&server);
  server.begin();
  
  //OTA Config
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname("DynamicLights-Server");
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA init ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //Initialize GPIO ports
  pinMode(RF_RECEIVE_PORT, INPUT);
  pinMode(RF_TRANSMIT_PORT, OUTPUT);
  pinMode(OPTIONS_KEY_PORT, INPUT);
  pinMode(SOUND_ANALOG_PORT, INPUT);
  pinMode(IR_RECEIVE, INPUT);
  pinMode(IR_TRANSMIT, OUTPUT);

  //Mic ADC Setup
  adc_setup();

  irrecv.enableIRIn();  // Start up the IR receiver.
  irsend.begin();       // Start up the IR sender.

  //BLE Setup
  BLEDevice::init("dynamicLights");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristicRF = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_RF,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE | 
                                          BLECharacteristic::PROPERTY_WRITE_NR
  );

  BLECharacteristic *pCharacteristicRF_RAW = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_RF_RAW,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE | 
                                          BLECharacteristic::PROPERTY_WRITE_NR
  );

  BLECharacteristic *pCharacteristicIR = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_BLE_IR_RAW,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE | 
                                          BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristicTemp = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_TEMP,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristicHumidity = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_HUMIDITY,
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_NOTIFY
  );

  BLEDescriptor pDescriptorTemp(BLEUUID((uint16_t)0x2902));
  pDescriptorTemp.setValue("Temperature Celsius");
  pCharacteristicTemp->addDescriptor(&pDescriptorTemp);

  BLEDescriptor pDescriptorHumidity(BLEUUID((uint16_t)0x2903));
  pDescriptorHumidity.setValue("Humidity % rH");
  pCharacteristicHumidity->addDescriptor(&pDescriptorHumidity);

  // set temperature BLE characteristic
  pService->addCharacteristic(pCharacteristicTemp);
  
  // set humidity  BLE characteristic
  pService->addCharacteristic(pCharacteristicHumidity);

  pCharacteristicRF->setCallbacks(new CharacteristicCallbacks());
  pCharacteristicRF_RAW->setCallbacks(new CharacteristicCallbacks());
  pCharacteristicIR->setCallbacks(new CharacteristicCallbacks());
  
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  //HeartBeats initialization, zero all clients
  initHeartBeats();

  //init aht
   while (! aht.begin()) {
    WebSerial.println("Could not find AHT. Check wiring...");
    delay(1000);
  }
  WebSerial.println("AHT10 found");
  
}

void loop() {

  //Handle OTA
  ArduinoOTA.handle();

  // bluetooth commands
  if (bleDeviceConnected) {
     if(bleCommand == 1) {wallLedPowerOn();bleCommand=0;rf_wall_led_on=true;WebSerial.println("Led On");}
     if(bleCommand == 2) {wallLedPowerOff();bleCommand=0;rf_wall_led_on=false;WebSerial.println("Led Off");}
     if(bleCommand == 20) { }
  }

  // Check if an IR message has been received.
  if (irrecv.decode(&results)) {  // We have captured something.
      // The capture has stopped at this point.
      decode_type_t protocol = results.decode_type;
      uint16_t size = results.bits;
      bool success = true;
      uint16_t *raw_array = resultToRawArray(&results);
      size = getCorrectedRawLength(&results);
      WebSerial.print("Received IR Raw data: ");
      for(int i = 0; i < size; i++) {
          WebSerial.print(raw_array[i]);if (i != (size-1)) WebSerial.print(",");
      }
      WebSerial.println();
    irrecv.resume();
  }

  // handle push button state
  int pushButtonState = digitalRead(OPTIONS_KEY_PORT);
  if ( pushButtonState != pushButtonPreviousState) { 
      if(pushButtonState == HIGH) buttonClicked();
      pushButtonPreviousState = pushButtonState;
  }

  // temperature and humidity sensor
  if(aht10_last_checked < millis() - AHT10_CHECK_INTERVAL) {
    //read temp and humidity
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //Print current values to console
    WebSerial.print("Temperature: "); WebSerial.print(temp.temperature); WebSerial.println(" degrees C");
    WebSerial.print("Humidity: "); WebSerial.print(humidity.relative_humidity); WebSerial.println("% rH");
    //Update temperature and humidity over BLE
    if(bleDeviceConnected) {
      if (bleDeviceConnected != bleDeviceConnectedLastState) WebSerial.println("BLE device connected");
      static char temperatureCTemp[6];
      dtostrf(temp.temperature, 6, 2, temperatureCTemp);
      pCharacteristicTemp->setValue(temperatureCTemp);
      pCharacteristicTemp->notify();
      static char humidityRH[6];
      dtostrf(humidity.relative_humidity, 6, 2, humidityRH);
      pCharacteristicHumidity->setValue(humidityRH);
      pCharacteristicHumidity->notify();
    } else if(bleDeviceConnected != bleDeviceConnectedLastState) { 
        Serial.println("BLE device disconnected");
        BLEDevice::startAdvertising();
    }
    aht10_last_checked = millis();
    bleDeviceConnectedLastState = bleDeviceConnected;
  }
    
  switch (opMode) {
    case 0:
      //background lightning only
      invalidateHeartBeats();
      readHeartBeats();
      sendLedData(brightness, opMode);
      break;    
    case 1:
      //turn off lamps and do not send data
      break;
    case 2:
      //dynamic lights
      invalidateHeartBeats();
      readHeartBeats();
      analogRaw = analogRead(SOUND_ANALOG_PORT);
      sendLedData(analogRaw, opMode);
      break;
    //light modes - opMode 3-6
    case 3:
      invalidateHeartBeats();
      readHeartBeats();
      sendLedData(brightness, opMode);
      break;
    case 4:
      invalidateHeartBeats();
      readHeartBeats();
      sendLedData(brightness, opMode);
      break;
    case 5:
      invalidateHeartBeats();
      readHeartBeats();
      sendLedData(brightness, opMode);
      break;
    case 6:
      invalidateHeartBeats();
      readHeartBeats();
      sendLedData(brightness, opMode);
      break;
  }
  delay(4);
}

void sendLedData(uint32_t data, uint8_t op_mode) 
{
 struct led_command send_data;
 send_data.opmode = op_mode; 
 send_data.data = data; 
 for (int i = 0; i < NUMBER_OF_CLIENTS - 1; i++) 
 {
    if(heartbeats[i].connected == true) { 
      UDP.beginPacket(heartbeats[i].IP, 7001); 
      UDP.write((const uint8_t*)&send_data,sizeof(send_data));
      UDP.endPacket();
    }
 }
}

void initHeartBeats() {
  IPAddress ip(0,0,0,0);
  for (int i = 0; i < NUMBER_OF_CLIENTS; i++) {
   heartbeats[i].IP = ip;
   heartbeats[i].connected = false;
   heartbeats[i].lastchecked = millis();
  }
}

void readHeartBeats() {
  struct heartbeat_message hbm;
  int packetSize = UDP.parsePacket();
  if (packetSize == sizeof(hbm)) {
    UDP.read((char *)&hbm, sizeof(hbm));
    if (hbm.client_id > NUMBER_OF_CLIENTS - 1) {
      Serial.printf("Error: invalid client_id received: %d", hbm.client_id);
    } else {
      if (heartbeats[hbm.client_id].connected == false) Serial.printf("Client with ID: %d and IP: %s is now connected.\n", hbm.client_id, heartbeats[hbm.client_id].IP.toString());
      heartbeats[hbm.client_id].IP = hbm.IP;
      heartbeats[hbm.client_id].connected = true;
      heartbeats[hbm.client_id].lastchecked = millis();
    }
  }
}

void invalidateHeartBeats() {
  for (int i = 0; i < NUMBER_OF_CLIENTS - 1; i++) {
    // Invalidate heartbeat if checkDelay is passed
    if ((millis() - heartbeats[i].lastchecked > checkDelay) && heartbeats[i].connected == true)  {
        Serial.printf("Invalidating HB. Current time is: %d. HB last received for Client with ID: %d and IP: %s at: %d \n",millis(), i, heartbeats[i].IP.toString(), heartbeats[i].lastchecked);
        heartbeats[i].connected = false;
    }
  }
}

void buttonClicked() {
  if (opMode == numOpModes)
    opMode = 0;
  else
    opMode++;
  Serial.printf("Setting opmode %d \n", opMode);
}
