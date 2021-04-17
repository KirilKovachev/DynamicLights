/*
  Example for receiving
  
  https://github.com/sui77/rc-switch/
  
  If you want to visualize a telegram copy the raw data and 
  paste it into http://test.sui.li/oszi/
*/

#include <RCSwitch.h>

static int receiverPin = 32;

RCSwitch mySwitch = RCSwitch();


void setup() {
  Serial.begin(9600);
  pinMode(15, OUTPUT);
  digitalWrite(15,0);
  pinMode(receiverPin, INPUT);
  static const RCSwitch::Protocol customprotocol = { 82, { 15, 15 }, { 2, 8 }, { 7, 3 }, false };
  mySwitch.setProtocol(customprotocol);
  mySwitch.enableReceive(digitalPinToInterrupt(receiverPin));
}

void loop() {
  if (mySwitch.available()) {
    output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    mySwitch.resetAvailable();
  }
}
