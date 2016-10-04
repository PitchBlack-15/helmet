#include <VirtualWire.h>
#include <IRremote.h>

char *controller;



int PIN_IR = 3;
int LowBatOut = 5;
int BtnCancel = 7;
int AlcoholSensor = 15;
int vibrationSensor = 9;
int BtnCancelState = 0; 

void setup() {
  pinMode(PIN_IR, OUTPUT);
  pinMode(BtnCancel, INPUT);
  pinMode(LowBatOut, INPUT);
  pinMode(AlcoholSensor, INPUT);
  pinMode(vibrationSensor, INPUT);
  Serial.begin(9600); //Set serial baud rate to 9600 bps
  vw_set_ptt_inverted(true); //
  vw_set_tx_pin(12);
  vw_setup(4000);// speed of data transfer Kbps
}

void loop() {
  long measurement = TP_init();
  BtnCancelState = digitalRead(BtnCancel);
  int val;
  val=analogRead(0);//Read Gas value from analog 0
  Serial.println(val,DEC);//Print the value to serial port
  
  
  delay(50);
  if (measurement > 1000) {
    controller = "C"  ; //helmet Crash/bump
    vw_send((uint8_t *)controller, strlen(controller));
    vw_wait_tx(); // Wait until the whole message is gone
    delay(1000);
  }

  if (BtnCancelState == HIGH) {
      controller = "B"  ; // Cancel button pressed
      vw_send((uint8_t *)controller, strlen(controller));
      vw_wait_tx(); // Wait until the whole message is gone
      delay(1000);
  }


  controller = "L"  ; //Low Battery
  vw_send((uint8_t *)controller, strlen(controller));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(2000);



  controller = "A"  ; // Drunk Driver
  vw_send((uint8_t *)controller, strlen(controller));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(2000);

}


long TP_init() {
  delay(10);
  long measurement = pulseIn (vibrationSensor, HIGH); //wait for the pin to get HIGH and returns measurement
  return measurement;
}
