/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Sensor Node Template for Mysensors protocol
 * 			Node Presentation
 * 			Sensor Presentation
 * 			Timer and send data sensors

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


#include "OpenRocNodeTeensy.h"

void reqNodeId();
void presentationNode();
void presentationSensor();
void setValue(int type_value,void*);
void pingMaster();





uint32_t displayTimer = 0;
uint32_t sensorTimer = 0;
uint32_t pingTimer = 0;
boolean init = true;
uint8_t delaySensor =3000;
uint8_t delayPing =50000;




//
void setup() {
	//delay(2000);
	Serial.begin(57600);
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
	Serial.println(
			F(
					"OPEN-ROC NODE_SENSOR try to connecting to the Open-Roc mesh network..."));
	mesh.begin();
	delay(100);
	presentationNode();

}

void loop() {
	// loop
	mesh.update();
	//Serial.println("Sleep ..");
	//delay(200);
	// Send to the master node every second
	//
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

		sensorTimer = millis();
	}
	//
	if (millis() - pingTimer >= delayPing) {
		//pingMaster();
		pingTimer = millis();
	}

}
