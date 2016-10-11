// receiver.pde
//
// Simple example of how to use VirtualWire to receive messages
// Implements a simplex (one-way) receiver with an Rx-B1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: receiver.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

const unsigned int RF_rcPin = 12; // RF recieve PIN
char str[4];

void setup()
{
  Serial.begin(9600);	// Debugging only
  Serial.println("setup");
  mySerial.begin(9600);
  // Initialise the IO and ISR
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_rx_pin(RF_rcPin); // Set recieve pin
  vw_setup(4000);  // Bits per sec
  vw_rx_start();   // Start the receiver PLL running

}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen)) {
    Serial.println(F(" has message"));
    Serial1.write(buf[0]);
  }
}
