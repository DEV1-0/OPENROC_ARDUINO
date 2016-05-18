/*******************************************************************************
* Projet    :  OpenRoc
* @author   : eric Papet <e.papet@dev1-0.com>
* Objet     : integration du protocol Mysensors
* Board     : Aduino Uno
* RF        : RF24L01+
* Protocol  : Mysensors
* *****************************************************************************
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************************************************************
* Example sketch showing how to request time from controller.
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
*/

#include <SPI.h>
#include <MySensor.h>
#include <Time.h>

MySensor gw;
boolean timeReceived = false;
unsigned long lastUpdate=0, lastRequest=0;

char convBuf[MAX_PAYLOAD*2+1];

void setup()
{
  gw.begin(incomingMessage,AUTO,false);
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Clock", "1.0");
  // Request time from controller.
  gw.requestTime(receiveTime);
}

// This is called when a new time value was received
void receiveTime(unsigned long time) {
  // Ok, set incoming time
  Serial.print("Got Time");
  Serial.println(time);
  setTime(time);
  timeReceived = true;
}

void loop()
{
  unsigned long now = millis();
  gw.process();

   // If no time has been received yet, request it every 10 second from controller
  // When time has been received, request update every hour
  if ((!timeReceived && now-lastRequest > 20*100)
    || (timeReceived && now-lastRequest > 60*1000*60)) {
    // Request time from controller.
    Serial.println("requesting time");
    gw.requestTime(receiveTime);
    lastRequest = now;
  }

  // Print time every second
  if (timeReceived && now-lastUpdate > 1000) {
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
    lastUpdate = now;
  }
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void incomingMessage(const MyMessage &message) {
//  if (mGetCommand(message) == C_PRESENTATION && inclusionMode) {
//	gw.rxBlink(3);
//   } else {
//	gw.rxBlink(1);
//   }
   // Pass along the message from sensors to serial line
   Serial.println("Recieve Incomming Message from RF24");
   Serial.print("Sender :");
   Serial.print(message.sender,DEC);
   Serial.print(";");
   Serial.print("Sensor :");
   Serial.print(message.sensor,DEC);
    Serial.print(";");
   Serial.print("Command :");
   Serial.print(mGetCommand(message),DEC);
    Serial.print(";");
   Serial.print("Ack :");
   Serial.print(mGetAck(message),DEC);
    Serial.print(";");
   Serial.print("Type :");
   Serial.print(mGetCommand(message),DEC);
    Serial.print(";");
   Serial.print("Payload:");
   Serial.println(message.getString(convBuf));

   //Serial.print(PSTR("%d;%d;%d;%d;%d;%s\n"),message.sender, message.sensor, mGetCommand(message), mGetAck(message), message.type, message.getString(convBuf));
}
