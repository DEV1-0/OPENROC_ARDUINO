/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Temperature Sensor Node
 * 			Protocol Openroc JSON
 * Sensor: DHT21
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <EEPROM.h>

#include <RF24Mesh_config.h>
#include <RF24Mesh.h>

#include <RF24Network_config.h>
#include <RF24Network.h>
#include <Sync.h>

#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <printf.h>

#include <SPI.h>

#include "DHT.h"

#define MESH_NOMASTER 0

#define DHTPIN 2     // what digital pin we're connected to

//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

/**** Configure the nrf24l01 CE and CS pins ****/
uint8_t CE = 9 ;
uint8_t CS = 10;

//uint8_t CE = 7 ;
//uint8_t CS = 8;

//uint8_t CE = 3 ;
//uint8_t CS = 7;
//
RF24 radio(CE,CS);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

/**
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 * This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
 *
 **/
//#define nodeID 1
uint8_t nodeID=-1;
uint32_t displayTimer=0;
uint32_t sensorTimer=0;
uint32_t pingTimer=0;
boolean init=true;
//
 struct TemperatureHumidity {
  uint8_t id;
  boolean b;
  float t;
  float r;
  float h;
  char model[10];
} th;



void readSensor(){
	sprintf(th.model,"%s","DHT21");
	//typedef struct TemperatureHumididy payload;
	 // Reading temperature or humidity takes about 250 milliseconds!
	  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	  float h = dht.readHumidity();

	  // Read temperature as Celsius (the default)
	  float t = dht.readTemperature();

	  // Read temperature as Fahrenheit (isFahrenheit = true)
	  float f = dht.readTemperature(true);

	  // Check if any reads failed and exit early (to try again).
	  if (isnan(h) || isnan(t) || isnan(f)) {
	    Serial.println("Failed to read from DHT sensor!");
	    th.b=false;

	  }else{
		  // Compute heat index in Fahrenheit (the default)
		 	  float hif = dht.computeHeatIndex(f, h);
		 	  // Compute heat index in Celsius (isFahreheit = false)
		 	  float hic = dht.computeHeatIndex(t, h, false);
		 	  //
		 	  Serial.print("Humidity: ");
		 	  Serial.print(h);
		 	  Serial.print(" %\t");
		 	  Serial.print("Temperature: ");
		 	  Serial.print(t);
		 	  Serial.print(" *C ");
		 	  Serial.print(f);
		 	  Serial.print(" *F\t");
		 	  Serial.print("Heat index: ");
		 	  Serial.print(hic);
		 	  Serial.print(" *C ");
		 	  Serial.print(hif);
		 	  Serial.println(" *F");
		 	  //
		 	  th.b=true;
		 	  th.t=t;
		 	  th.r=hic;
		 	  th.h=h;
	  }
	  //
	  th.id=nodeID;
	  if(!mesh.write(&th,'H',sizeof(th))){
	       // If a write fails, check connectivity to the mesh network
	       Serial.println("NODE --> temp/humidity Write error Check Connection ");
	       if( !mesh.checkConnection() ){
	         //refresh the network address
	         Serial.println("NODE --> temp/humidity Renewing Address");
	         mesh.renewAddress();

	       }else{
	         Serial.println("NODE --> temp/humidity Send fail, Test OK");
	       }
	     }else{
	     	//radio.printDetails();
	       Serial.print("NODE --> temp/humidity Send OK: ");
	       //Serial.println(displayTimer);
	     }
}

void pingMaster(){
	//uint8_t p = 0;
	if(!mesh.write(&nodeID,'P',sizeof(nodeID))){
		       // If a write fails, check connectivity to the mesh network
		if( !mesh.checkConnection() ){
		   //refresh the network address
			mesh.renewAddress();
		}
	}
}


//
void setup() {
 //delay(2000);
  Serial.begin(57600);
   printf_begin();
   delay(100);
 // Set the nodeID by console
   while (!mesh.getNodeID()) {
       // Wait for the nodeID to be set via Serial
       if (Serial.available()) {
    	   nodeID= getNodeIdFromSerial();
    	   mesh.setNodeID(nodeID);
    	   Serial.print("nodeID : ");
    	   Serial.println(nodeID);
    	   delay(100);
       }
     }
 Serial.println(F("OPEN-ROC NODE_SENSOR try to connecting to the Open-Roc mesh network..."));
  mesh.begin();
  //sensor
 dht.begin();
  delay(100);

}


uint8_t getNodeIdFromSerial() {
	uint8_t ret =0;
	uint8_t cpt=0;
	char r;
	char buf[3] = {'@','@','@'} ;
	Serial.print("Set NodeID: ");
	while (cpt <=3 && r !=13){
		if (Serial.available()) {
			r=Serial.read();
			if (r != 13){
				buf[cpt]=r;
				Serial.print(buf[cpt]);
			}
			cpt++;
		}
	}
	//
	if(buf[1] =='@' && buf[2] =='@')
		ret= (buf[0]-'0');
	else if (buf[1] !='@' &&  buf[2] =='@')
		ret = (10 * (buf[0]-'0'))+ (buf[1]-'0');
	else
		ret = ( (100 * (buf[0]-'0')) + (10 * (buf[1]-'0')) + (buf[2]-'0') );
	//
	Serial.print("\n");
	return ret;
}



void loop() {
	// loop
	mesh.update();
	//Serial.println("Sleep ..");
	//delay(200);
  // Send to the master node every second
 //
	if(init){
		if(!mesh.write(&nodeID,'I',sizeof(nodeID))){
		      // If a write fails, check connectivity to the mesh network
		      Serial.println("NODE --> Write error Check Connection ");
		      if( !mesh.checkConnection() ){
		        //refresh the network address
		        Serial.println("NODE --> Renewing Address");
		        mesh.renewAddress();

		      }else{
		        Serial.println("NODE --> Send init, Test OK");
		      }
		    }else{
		    	init=false;
		    }

	}

  //

  if(millis() - sensorTimer >= 5000){
	  	readSensor();
	  	sensorTimer =  millis();
  }
  //
  if(millis() - pingTimer >= 30000){
  	  	pingMaster();
  	  	pingTimer =  millis();
    }

}
