/*
 * Token Stich
 *
 * Stichies protocol for a Token Ring version
 * of the stichies, to be used at the mid-scale
 * experiment where we will wear interactive
 * necklesses to check how we interact with each other
 *
 * (c) 2013 onescaleone, Tony, Andreas, Stahle, David
 */

#include <SoftwareSerial.h>

#define STICHIE_ID 4

#define rxPin 0
#define txPin 2
#define ledPin 2
#define touchPin PB4
#define moto 1
#define motoKick 30

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
int intensity=0; 
int on=1;
int steadypin = 3; // the led that's on while you're touching the input pin
long timeCheck=millis()+1000;
int calibration = 0;
int persistance=0;

int count=0;

void setup()
{
  // declare pins' functionalities
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(steadypin, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(moto, OUTPUT);

  // init the touchPin to be off
  digitalWrite(touchPin,LOW);

  // configure the software serial port
  mySerial.begin(9600);

  // wait my friend
  delay(1000);

  // calibrate the touch
  for (int i = 0; i < 8; i++) {
    calibration += chargeTime(touchPin);
    delay(20);
  }
  calibration = (calibration + 4) / 8;

  ledBlink();
}

void loop()
{
  updateMoto();
  updateTouch();
}

// this will only work for pins on the
// PB port of the processor, BEWARE!!
byte chargeTime(byte pin)
{
  byte mask = (1 << pin);
  byte i;

  DDRB &= ~mask; // input
  PORTB |= mask; // pull-up on

  for (i = 0; i < 16; i++) {
    if (PINB & mask) break;
  }
  PORTB &= ~mask; // pull-up off
  DDRB |= mask; // discharge
  return i;
}

void ledBlink(){
 digitalWrite(3,HIGH);
 delay(500);
 digitalWrite(3,LOW);
}

void updateMoto() {
  if(mySerial.available()>=4) {
    int serdata = mySerial.read();
    if (serdata == 255) count++;
    else count = 0;
    if(count>0) {
      int sendId = mySerial.read();
      int receiveId = mySerial.read();
      intensity = mySerial.read();
      if(receiveId == STICHIE_ID) {
        //ledBlink();
        analogWrite(moto,intensity);
        delay(motoKick);
      }else {
        mySerial.write(255);
        mySerial.write(sendId);
        mySerial.write(receiveId);
        mySerial.write(intensity);
        
      }
    }
  }
}

// send information about the touch pin to the
// serial port
void updateTouch() {
  
  if (chargeTime(touchPin) > calibration) {
        persistance+=40; 
    if(persistance>=254){
      persistance=254;
    }

    mySerial.write(255);
    mySerial.write(STICHIE_ID);
    mySerial.write(STICHIE_ID);
    mySerial.write(persistance);  // by default, make the motor go full blast
     analogWrite(moto,persistance);
    ledBlink();
  }else if(persistance>0){
    persistance=0; 
    mySerial.write(255);
    mySerial.write(STICHIE_ID);
    mySerial.write(STICHIE_ID);
    mySerial.write(persistance);  // by default, make the motor go full blast
     analogWrite(moto,persistance);
  }
}
