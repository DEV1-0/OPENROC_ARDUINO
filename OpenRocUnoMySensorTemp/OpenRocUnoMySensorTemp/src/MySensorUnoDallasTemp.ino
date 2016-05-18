/**
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
 *******************************
 *
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 */

#include <MySensor.h>
#include <SPI.h>
//#include <DallasTemperature.h>
//#include <OneWire.h>

//#define COMPARE_TEMP 0 // Send temperature only if changed? 1 = Yes 0 = No

//#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected
//#define MAX_ATTACHED_DS18B20 16
//
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)
//OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
MySensor gw;
//float lastTemperature[MAX_ATTACHED_DS18B20];
//int numSensors=0;
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message
MyMessage msg(0,V_TEMP);
//
//epa
char convBuf[MAX_PAYLOAD*2+1];
unsigned long timer;
//unsigned long time;

void setup()
{
  // EPA DEBUG
  Serial.begin(115200);
  delay(100);
  //while(!Serial.available());
  Serial.println("Serial Started");
              // Startup up the OneWire library
//  sensors.begin();
          // requestTemperatures() will not block current thread
//  sensors.setWaitForConversion(false);

  // Startup and initialize MySensors library. Set callback for incoming messages.
  gw.begin(incomingMessage,AUTO,false);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Fake Temperature Sensor", "1.1");

  // Fetch the number of attached temperature sensors
  //numSensors = sensors.getDeviceCount();

   gw.present(0, S_TEMP);

  // Present all sensors to controller
//  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
//     gw.present(i, S_TEMP);
//  }
}


void loop()
{
//  if ((millis()-timer) > 3000) {  // sync every few minutes
Serial.println("Request Time to GateWay");
gw.requestTime(receiveTime); // request time from network
  //timer=millis();
//}
  // Process incoming messages (like config from server)
  gw.process();
  delay(500);

  // Fetch temperatures from Dallas sensors
  //sensors.requestTemperatures();

      // query conversion time and sleep until conversion completed
//  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
//int16_t conversionTime = 1000;
      // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
//  gw.sleep(conversionTime);

  // Read temperatures and send them to controller
  //for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

    // Fetch and round temperature to one decimal
    //epa
  //  float temperature;
  //  float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

    // Only send data if temperature has changed and no error
  //  #if COMPARE_TEMP == 1
  //  if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
//    #else
//    if (temperature != -127.00 && temperature != 85.00) {
//    #endif

      // Send in the new temperature
//      Serial.println("Send Température to GateWay");
  //    gw.send(msg.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
    //  lastTemperature[i]=temperature;
  //  }
//  }
  //epa

  gw.send(msg.setSensor(0).set(12.20,1));
  //gw.sleep(SLEEP_TIME);
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

//Calback time
void receiveTime(unsigned long time) {
  // Ok, set incoming time
 Serial.print("Recieve Time :");
 Serial.println(time,DEC);
}
