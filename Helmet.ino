#include <VirtualWire.h>
#include <IRremote.h>

char *controller;



int PIN_IR = 3;
int headswitch = 5;
int BtnCancel = 7;
int AlcoholSensor = 15;
int vibrationSensor = 9;
int BtnCancelState = 0;
int BtnheadState = 0;

unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 1000;

void setup() {
  pinMode(PIN_IR, OUTPUT);
  digitalWrite(PIN_IR, HIGH);

  pinMode(BtnCancel, INPUT);
  pinMode(headswitch, INPUT);
  digitalWrite(BtnCancel, HIGH);
  digitalWrite(headswitch, HIGH);

  pinMode(AlcoholSensor, INPUT);
  pinMode(vibrationSensor, INPUT);

  Serial.begin(9600); //Set serial baud rate to 9600 bps
  vw_set_ptt_inverted(true); //
  vw_set_tx_pin(11);
  vw_setup(4000);// speed of data transfer Kbps
}

long TP_init() {
  long measurement = pulseIn (vibrationSensor, HIGH); //wait for the pin to get HIGH and returns measurement
  return measurement;
}


void loop() {
  long measurement = TP_init();
  BtnCancelState = digitalRead(BtnCancel);
  BtnheadState = digitalRead(headswitch);
  int val;

  val = analogRead(0); //Read Gas value from analog 0
  Serial.println(val);//Print the value to serial port
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    if (BtnheadState == LOW) {
      Serial.println("wearing helmet");//Wearing Helmet
      controller = "P"  ;
      vw_send((uint8_t *)controller, strlen(controller));
      vw_wait_tx(); // Wait until the whole message is gone
    }
    previousMillis = currentMillis;
  }
  if (measurement > 9000) {
    Serial.println("Crash detected");//Print the value to serial port
    controller = "C"  ; //helmet Crash/bump
    vw_send((uint8_t *)controller, strlen(controller));
    vw_wait_tx(); // Wait until the whole message is gone
  }

  if (BtnCancelState == LOW) {
    Serial.println("Message Canceled");//Print the value to serial port
    controller = "B"  ; // Cancel button pressed
    vw_send((uint8_t *)controller, strlen(controller));
    vw_wait_tx(); // Wait until the whole message is gone
  }

  if (val > 190) {
    Serial.println("Drunk");//Print the value to serial port
    controller = "A"; // Drunk Driver
    vw_send((uint8_t *)controller, strlen(controller));
    vw_wait_tx(); // Wait until the whole message is gone
  }

}






