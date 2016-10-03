#include <VirtualWire.h>
#include <IRremote.h>
#define PIN_IR 3
char *controller;
IRsend irsend;
int LowBatOut = 5;
int BtnCancel = 7;
int AlcoholSensor = 15;

void setup() {
  irsend.enableIROut(38);
  irsend.mark(0);
  vw_set_ptt_inverted(true); //
  vw_set_tx_pin(12);
  vw_setup(4000);// speed of data transfer Kbps
}

void loop() {
  controller = "L"  ;
  vw_send((uint8_t *)controller, strlen(controller));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(2000);
  
  controller = "C"  ;
  vw_send((uint8_t *)controller, strlen(controller));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(2000);
  
  controller = "B"  ;
  vw_send((uint8_t *)controller, strlen(controller));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(2000);

}
