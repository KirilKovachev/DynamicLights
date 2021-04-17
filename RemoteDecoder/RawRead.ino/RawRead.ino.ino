#define SAMPLESIZE 500
#define TIMING_THRESHOLD 0

static unsigned int timings[SAMPLESIZE];
static unsigned int pos = 0;
static unsigned long lastTime = 0;

static int receiverPin = 32;
static int interruptPin = 0;

void setup() {
  pinMode(15, OUTPUT);
  digitalWrite(15,0);
  pinMode(receiverPin, INPUT);
  interruptPin = digitalPinToInterrupt(receiverPin);
  Serial.begin(9600); 
  attachInterrupt(interruptPin, handleInterrupt, CHANGE);
}

void loop() {
    for (int i = 5; i>0; i--) {
      Serial.print(i);
      Serial.print("... ");
      delay(900);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    }
    Serial.println();
      
    detachInterrupt(interruptPin);
    int finalstate = digitalRead(receiverPin);
    
    for (unsigned int i = pos + finalstate; i< SAMPLESIZE; i++) {
        Serial.print( timings[i] );
        Serial.print(",");
    }
 
    for (unsigned int i = 0; i < pos; i++) {
        Serial.print( timings[i] );
        Serial.print(",");
    }

    Serial.println("");
    Serial.println("Reset your Arduino to scan again...");

    while(true) {} 
  
}

void handleInterrupt() {
  const long time = micros();
  timings[pos] = time - lastTime;
  lastTime = time;
  if (++pos > SAMPLESIZE-1) {
    pos = 0;
  }
}
