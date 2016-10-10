#include <VirtualWire.h>
#include <IRremote.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

char Phone1[] =  "09157047148";
char Phone2[] =  "09157047148";
char Phone3[] =  "09157047148";

//LCD circuit:
// * 5V to Arduino 5V pin
// * GND to Arduino GND pin
// * CLK to Analog #5
// * DAT to Analog #4

//GSM pin
#define FONA_RX 2
#define FONA_TX 10
#define FONA_RST 4
const unsigned int SENSORPIN = 52;

int sensorState = 0;

const unsigned int buzzpin = 6; //Pin For Buzzer
const unsigned int RF_rcPin = 12; // RF recieve PIN
const unsigned int relayPin = 5;

unsigned int SendCounter = 0;
unsigned int ShutOffCounter = 0;
unsigned int ShutOffTimer = 5;

bool crashed = false;
bool drunk = false;
bool wearing_helmet = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

unsigned long previousMillis = 0;
void setup() {
  pinMode(SENSORPIN, INPUT);
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  pinMode(relayPin, OUTPUT);
  pinMode(buzzpin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  digitalWrite(buzzpin, HIGH);

  Serial.begin(115200);
  Serial.println(F("Motor cycle starting"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  lcd.init();
  lcd.init();
  lcd.backlight();
  LCD_MSG("Initializing...", "");

  fonaSerial->begin(4800);  // make it slow so its easy to read!
  if (! fona.begin(*fonaSerial)) {
    LCD_MSG("Error 130", "");
    while (1);
  }
  fona.enableGPS(true);
  LCD_MSG("   GPS & GSM   ", "      OK!       ");

  vw_set_ptt_inverted(true); // Required for DR3100
  vw_set_rx_pin(RF_rcPin); // Set recieve pin
  vw_setup(4000);  // Bits per sec
  vw_rx_start();   // Start the receiver PLL running


  lcd.backlight();
  buzz(200); //Beep
  buzz(200); //Beep
  LCD_MSG("  Connecting... ", "              ");
  RF_recieve();
  delay(1000);
  if (drunk) {
    digitalWrite(relayPin, LOW);
  }
  LCD_MSG("   Anti-Drunk   ", " Driving Helmet");
}

void loop() {
 // Serial.println(F("Looping"));
  RF_recieve();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    IR_Connect();
    if (drunk) {
      Serial.println(F("Looping drunk"));
      ShutOffCounter++;
      if (ShutOffTimer <= 0) {
        LCD_MSG(" Shutting down! ", "");
        buzz(200);
        delay(1000);
        digitalWrite(relayPin, HIGH);
      } else if (ShutOffCounter > 30) {
        ShutOffTimer--;
        LCD_MSG("Motorcycle Will", "Shut-Off in " + String(ShutOffTimer));
        buzz(200);
      }
    }

    if (crashed) {
      Serial.println(F("Looping crash"));
      LCD_MSG("Press Helmet", "Button to cancel");
      SendCounter++;
      buzz(200);
      if (SendCounter > 30) {
        LCD_MSG("Sending Message", "");
        SendSMS();
        SendCounter = 0;
      }
    }
    previousMillis = currentMillis;
  }
}


/////////////////////////////////////////////////////////////////////
//Method: RF_recieve()
//Functionality: For Helmet and motorcycle data connection
////////////////////////////////////////////////////////////////////
String RF_recieve() {
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen)) {
    Serial.println(F(" has message"));
     Serial.println(buf[0]);
//     String data = buf[0];
    LCD_MSG("   RF_recieve  ", "    message!   ");
    if (buf[0] == 67) {
      crashed = true;
      LCD_MSG("   Helmet Bump  ", "    Detected!   ");
      buzz(1000);
      Serial.println(F("crash"));
      return "Crash";
    } else if (buf[0] == 65) {
      drunk = true;
      buzz(1000);
      LCD_MSG("Drunk Driver", "    Detected!   ");
      Serial.println(F("Drunk has message"));
      return "Drunk";
    } else if (buf[0] == 66) {
      crashed = false;
      SendCounter = 0;
      buzz(1000);
      LCD_MSG("  Cancel Button ", "    Pressed!   ");
      Serial.println(F("Cancel button has message"));
      return "OFFBTN";
    } else if (buf[0] == 80) {
      wearing_helmet = true;
      Serial.println(F("wear helmet"));
      buzz(1000);
      return "Lowbat";
    } else if (buf[0] == 78) {
      wearing_helmet = false;
      Serial.println(F("not wear helmet"));
      LCD_MSG("Please Wear", "Your Helmet");
      buzz(1000);
      return "Lowbat";
    } else {
      return "OK";
    }
  }

  return "OFF";
}





/////////////////////////////////////////////////////////////////////
//Method: IR_Connect()
//Functionality: For Detecting if the Driver is Wearing the helmet
//Infrared blocking detection
////////////////////////////////////////////////////////////////////
void IR_Connect() {
  sensorState = digitalRead(SENSORPIN);
  if (sensorState == LOW ) {
    Serial.println(F("IR_Connect cannot detect helmet"));
    ShutOffCounter++;
    LCD_MSG("No Helmet", String(ShutOffCounter)); // debug
    if (ShutOffTimer <= 0) {
      LCD_MSG(" Shutting down! ", "");
      buzz(200);
      delay(1000);
      digitalWrite(relayPin, HIGH);
      while (1) {
        LCD_MSG("................", "................");
      }
    } else if (ShutOffCounter > 15) {
      ShutOffTimer--;
      LCD_MSG("Motorcycle Will", "Shut-Off in " + String(ShutOffTimer));
      buzz(200);
    } else if ((ShutOffCounter > 10)) {
      LCD_MSG("    Please!!!   ", "Wear your helmet");
      buzz(200);
    } else {
      // LCD_MSG("No Helmet", "Nothing happen"); //debug
    }
  } else {
  //  Serial.println(F("IR_Connect detect helmet"));
    if (!drunk) {
      digitalWrite(relayPin, LOW);
      ShutOffCounter = 0;
      ShutOffTimer = 5;
      LCD_MSG("NOT Drunk", "Very Good"); // debug

    }
  }
}



/////////////////////////////////////////////////////////////////////
//Method: SendSMS()
//Functionality: For Sending Help Notification
//Send Help with location coordinates
//Limitaion: cant send msg if 0 Load balance
////////////////////////////////////////////////////////////////////
void SendSMS() {
  float latitude, longitude, speed_kph;
  boolean gps_success = false;
  char LAT[10];
  char LONG[10];
  char msg[140];
  gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &speed_kph, &speed_kph);
  while (!gps_success) {
    gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &speed_kph, &speed_kph);

    if (fona.getNetworkStatus() == 1) {
      gps_success = fona.getGSMLoc(&latitude, &longitude);
      if (!gps_success) {
        fona.enableGPRS(false);
        if (!fona.enableGPRS(true)) {
        }
      }
    }

  }
  // construct the message
  dtostrf(latitude, 4, 6, LAT);
  dtostrf(longitude, 4, 6, LONG);
  strcpy(msg, "");
  strcat(msg, "Im ADDH, Please help me! my Location is -> latitude: ");
  strcat(msg, LAT);
  strcat(msg, " longitude: ");
  strcat(msg, LONG);

  //Send message to the recorded number
  if (!fona.sendSMS(Phone1, msg)) {
    LCD_MSG("Not Sent to:", Phone1);
    delay(500);
  } else {
    LCD_MSG("Message Sent", "to:" + String(Phone1));
    delay(500);
  }
  if (!fona.sendSMS(Phone2, msg)) {
    LCD_MSG("Not Sent to:", Phone2);
    delay(500);
  } else {
    LCD_MSG("Message Sent", "to:" + String(Phone2));
    delay(500);
  }
  if (!fona.sendSMS(Phone3, msg)) {
    LCD_MSG("Not Sent to:", Phone3);
    delay(500);
  } else {
    LCD_MSG("Message Sent", "to:" + String(Phone3));
    delay(500);
  }
}


/////////////////////////////////////////////////////////////////////
//Method: buzz(unsigned char time)
//Functionality: For Sound Notification
//parameter: Length of Beeping sound
////////////////////////////////////////////////////////////////////
void buzz(unsigned char time) {
  digitalWrite(buzzpin, LOW);
  delay(400);
  digitalWrite(buzzpin, HIGH);
  delay(400);
}


/////////////////////////////////////////////////////////////////////
//Method: LCD_MSG(String MSG1, String MSG2)
//Functionality: For visual Notification
//parameter: (message on first line, message on second line)
/////////////////////////////////////////////////////////////////
void LCD_MSG(String MSG1, String MSG2) {
  lcd.clear();
  lcd.setCursor(0, 0); //set to second Line
  lcd.print(MSG1); // Print a message to the LCD.
  lcd.setCursor(0, 1); //set to second Line
  lcd.print(MSG2); // Print a message to the LCD.
}
