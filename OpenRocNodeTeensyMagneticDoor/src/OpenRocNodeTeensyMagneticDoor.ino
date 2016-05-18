/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Temperature and Humidity Sensor Node
 * 			Protocol Openroc JSON
 * Sensor: Magnetic Door
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
#include"magneticDoor.h"


#define MESH_NOMASTER 0

/**** Configure the nrf24l01 CE and CS pins ****/
uint8_t CE = 9;
uint8_t CS = 10;

//uint8_t CE = 7 ;
//uint8_t CS = 8;

//uint8_t CE = 3 ;
//uint8_t CS = 7;
//
RF24 radio(CE, CS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

//
const int SERIAL_BAUD = 57600;

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
uint8_t nodeID = 10;
uint32_t displayTimer = 0;
uint32_t sensorTimer = 0;
uint32_t pingTimer = 0;
boolean init = true;
uint16_t delaySensor = 300;
uint16_t delayPing = 50000;
//
//- Magnetic pin
// one in ping gnd
// other in pin 2 (3 pin after teensy GND)
const int switchPin = 2;
boolean prevState = false;

//typedef struct Door Door;

//









//
void setup() {
	//delay(2000);
	Serial.begin(SERIAL_BAUD);
	printf_begin();
	//
	mesh.setNodeID(nodeID);
	// Set the nodeID by console
	while (!mesh.getNodeID()) {
		// Wait for the nodeID to be set via Serial
		if (Serial.available()) {
			mesh.setNodeID(Serial.read());
			Serial.print("Set NodeID: ");
			Serial.println(mesh.getNodeID());
		}
	}
	//-- Magnetic DOOR
	pinMode(switchPin, INPUT);
	digitalWrite(switchPin, HIGH);
	//--
	Serial.println(
			F(
					"OPEN-ROC NODE_SENSOR try to connecting to the Open-Roc mesh network..."));
	mesh.begin();
	delay(100);

}

void loop() {
	//struct Door door;
	// loop
	mesh.update();

	if (init) {
		if (!mesh.write(&nodeID, 'I', sizeof(nodeID))) {
			// If a write fails, check connectivity to the mesh network
			Serial.println("NODE --> Write error Check Connection ");
			if (!mesh.checkConnection()) {
				//refresh the network address
				Serial.println("NODE --> Renewing Address");
				mesh.renewAddress();

			} else {
				Serial.println("NODE --> Send init, Test OK");
			}
		} else {
			init = false;
		}

	}

	//

	if (millis() - sensorTimer >= delaySensor) {
		readSensor(&door);
		sensorTimer = millis();
	}
	//
	if (millis() - pingTimer >= delayPing) {
		pingMaster();
		pingTimer = millis();
	}

}

void pingMaster() {
	//uint8_t p = 0;
	if (!mesh.write(&nodeID, 'P', sizeof(nodeID))) {
		// If a write fails, check connectivity to the mesh network
		if (!mesh.checkConnection()) {
			//refresh the network address
			mesh.renewAddress();
		}
	}
}

void send2Master(void * s,char type, int size){
	//Serial.print("size : ");
	//Serial.println(sizeof(*s));
	 if(!mesh.write(s,type,size)){
		       // If a write fails, check connectivity to the mesh network
		       Serial.println("send2Matser -->  Write error Check Connection ");
		       if( !mesh.checkConnection() ){
		         //refresh the network address
		         Serial.println("send2Matser --> Renewing Address");
		         mesh.renewAddress();
		       }else{
		         Serial.println("send2Matser --> Send fail, Test OK");
		       }
		     }else{
		     	//radio.printDetails();
		       Serial.println("send2Matser --> Send OK: ");
		       //Serial.println(displayTimer);
		     }
}

void readSensor(struct Door* d) {
	sprintf(d->model,"%s","DOOR");
	boolean  currentState = (digitalRead(switchPin));
	if ( currentState != prevState){
		prevState = currentState;//transition
		Serial.print("Status is :");
		Serial.println((currentState) ? true:false);
		d->s=currentState;
		Serial.println(d->model);
		send2Master(d,type,sizeof(*d));
	}

}
