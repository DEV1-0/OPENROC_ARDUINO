/*******************************************************************************
 * Projet    :  OpenRoc
 * @author   : eric Papet <e.papet@dev1-0.com>
 * Objet     : integration du protocol Mysensors
 * Board     : Aduino Uno
 * RF        : RF24L01+
 * Protocol  : Mysensors
 * Sensors   : Temperature and Humidity
 *             DHT22 + VCC (Batery Level)
 * *****************************************************************************
 *
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
 *
 */

#include <MySensor.h>
#include <SPI.h>
#include <Stream.h>

#include <Vcc.h>
//const float VccExpected   = 3.0;
//const float VccCorrection = 2.860/2.92;  // Measured Vcc by multimeter divided by reported Vcc

const float VccMin   = 0.0;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 5.0;           // Maximum expected Vcc level, in Volts.
const float VccCorrection = 1.0/1.0;
float v;
float p;
float previous_p;
Vcc vcc(VccCorrection);



unsigned long SLEEP_TIME = 5000; // Sleep time between reads (in milliseconds)

MySensor gw;
//
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message 0=sensor_id,V_TEMP=Metric Type
MyMessage msg_temp(0,V_TEMP);
MyMessage msg_hum(1,V_HUM);
//MyMessage msg_var(255,V_VAR1);
//MyMessage msg_press(255,S_ARDUINO_NODE);

//
//epa
char convBuf[MAX_PAYLOAD*2+1];
unsigned long timer;
//unsigned long time;

//Timer pour les messages entrant incomming(&Message)
unsigned long timer_sleep;

unsigned int NB_SENSORS = 2;

const char MODEL_0 [] ="DALLAS";
const char MODEL_1 [] ="DHT";
const char API_VERSION [] ="1.5.4";




void presentation(){
  gw.sendSketchInfo("TEST Sensor", "1.1");
  gw.present(255, S_ARDUINO_NODE,API_VERSION);
  gw.present(0, S_TEMP,MODEL_0);
  gw.present(1,S_HUM,MODEL_1);
}

void setup()
{
  // EPA DEBUG
  Serial.begin(115200);
  delay(100);

  Serial.println("Serial Started");

  // Startup and initialize MySensors library. Set callback for incoming messages.
  gw.begin(incomingMessage,AUTO,false);
  //
  presentation();
  //present();
  // Send the sketch version information to the gateway and Controller
  // Send 3 messages to Controlor :
  //C_INTERNAL:: I_CONFIG <- sender:5,sensor:255,type:6,payload:0, ack:0 [<- 5;255;3;0;6;0]
  //C_INTERNAL:: I_SKETCH_NAME <- sender:5,sensor:255,type:11,payload:Fake Temperature Sensor, ack:0 [<- 5;255;3;0;11;Fake Temperature Sensor]
  //C_INTERNAL:: I_SKETCH_VERSION <- sender:5,sensor:255,type:12,payload:1.1, ack:0,[<- 5;255;3;0;12;1.1]
//  gw.sendSketchInfo("TEST Sensor", "1.1");

  // Fetch the number of attached temperature sensors
  //numSensors = sensors.getDeviceCount();
  //C_PESENTATION:Sensor <- sender:5,sensor:0,type:6,payload:, ack:0 [<- 5;0;0;0;6;]
  // gw.present(0, S_TEMP);

  // Present all sensors to controller
//  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
//     gw.present(i, S_TEMP);
//  }
}


void loop()
{
  //

//  if ((millis()-timer) > 3000) {  // sync every few minutes
//Serial.println("Request Time to GateWay");
//gw.requestTime(receiveTime); // request time from network
  //timer=millis();
//}
  // Process incoming messages (like config from server)
  //Tant que tempo_process

  gw.process();


//  delay(200);

  // Fetch temperatures from Dallas sensors
  //sensors.requestTemperatures();

      // query conversion time and sleep until conversion completed
//  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
//int16_t conversionTime = 1000;
      // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
//  gw.sleep(conversionTime);

  // Read temperatures and send them to controller
  //for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

      // Send in the new temperature
//      Serial.println("Send Température to GateWay");
  //    gw.send(msg.setSensor(i).set(temperature,1));
      // Save new temperatures for next compare
    //  lastTemperature[i]=temperature;
  //  }
//  }
  //epa
  if((millis() - timer) > 5000){
  //  gw.send(msg_temp.setSensor(0).set(12.20,1));
  //  gw.send(msg_hum.setSensor(1).set(59));
    //
    v = vcc.Read_Volts();
    Serial.print("VCC = ");
    Serial.print(v);
    Serial.println(" Volts");

   p = vcc.Read_Perc(VccMin, VccMax);
   Serial.print("VCC = ");
   Serial.print(p);
   Serial.println(" %");
//Send Battery Level if chage
if(p !=previous_p ){
   previous_p = p;
   gw.sendBatteryLevel(p);
}
//debug
gw.sendBatteryLevel(p);
    //
    //delay(500);
    timer=millis();
  }
  //
  if((millis() - timer_sleep) > 10000){
    gw.sleep(SLEEP_TIME);
    timer_sleep=millis();
  }


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
   Serial.print(message.type,DEC);
    Serial.print(";");
   Serial.print("Payload:");
   Serial.println(message.getString(convBuf));
   //Req PRESETATION from Controler
   if(mGetCommand(message) == C_REQ && message.type == V_VAR1){
    presentation();
    }


   //Serial.print(PSTR("%d;%d;%d;%d;%d;%s\n"),message.sender, message.sensor, mGetCommand(message), mGetAck(message), message.type, message.getString(convBuf));
}

//Calback time
void receiveTime(unsigned long time) {
  // Ok, set incoming time
 Serial.print("Recieve Time :");
 Serial.println(time,DEC);
}
