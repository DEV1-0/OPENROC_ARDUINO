/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Temperature and Humidity Sensor Node
 * 			Protocol Openroc JSON
 * Sensor: Pir Sensor
 * @see http://www.seeedstudio.com/wiki/Grove_-_PIR_Motion_Sensor
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



#define MESH_NOMASTER

#define pirPIN 4    // what digital pin we're connected to



//Pir Adfruit Sensors V&.2
// http://www.seeedstudio.com/wiki/Grove_-_PIR_Motion_Sensor
///////////////////////////////////////////////////////////////////////
//the time when the sensor outputs a low impulse
long unsigned int lowIn;

//the amount of milliseconds the sensor has to be low
//before we assume all motion has stopped
long unsigned int pause = 5000;

boolean lockLow = true;
boolean takeLowTime;

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
uint8_t nodeID = 8;
uint8_t calibrationTime = 5;
uint32_t displayTimer = 0;
uint32_t sensorTimer = 0;
uint32_t tempTimer = 0;
uint32_t pingTimer = 0;
boolean init = true;
//

//
#pragma pack(1)
typedef struct Pir{
	uint32_t time;
	bool m;
	char model[10] ;
} Pir,*PirPtr;


void calibrationPir(){
	pinMode(pirPIN, INPUT);
	digitalWrite(pirPIN, LOW);

	  //give the sensor some time to calibrate
	  Serial.print("calibrating sensor ");
	    for(int i = 0; i < calibrationTime; i++){
	      Serial.print(".");
	      delay(1000);
	      }
	    Serial.println(" done");
	    Serial.println("PIR SENSOR ACTIVE");
	    delay(50);
}

void  detection(){
	Pir pir;
	sprintf(pir.model,"%s","PIRMS");
	//
	 if(digitalRead(pirPIN) == HIGH){
	       if(lockLow){
	         //makes sure we wait for a transition to LOW before any further output is made:
	         lockLow = false;
	         Serial.println("---");
	         Serial.print("motion detected at ");
	         Serial.print(millis()/1000);
	         Serial.println(" sec");
	         delay(50);
	         pir.time=millis()/1000;
	         pir.m=true;
	         if(!mesh.write(&pir,'D',sizeof(pir))){
	         	    	       // If a write fails, check connectivity to the mesh network
	         	    	       Serial.println("NODE --> Pir Sensor Motion  Write error Check Connection ");
	         	    	       if( !mesh.checkConnection() ){
	         	    	         //refresh the network address
	         	    	         Serial.println("NODE --> Pir Sensor Motion Renewing Address");
	         	    	         mesh.renewAddress();
	         	    	       }else{
	         	    	    	   //@ TODO implements send mess to master
	         	    	         Serial.println("NODE -->Pir Sensor Motion Send fail, Test OK");
	         	    	       }
	         	    	     }else{
	         	    	     	//radio.printDetails();
	         	    	       Serial.print("NODE --> Pir Sensor Motion Send OK: ");
	         	    	       //Serial.println(displayTimer);
	         	    	     }
	         }
	         takeLowTime = true;
	       }

	     if(digitalRead(pirPIN) == LOW){
	       if(takeLowTime){
	        lowIn = millis();          //save the time of the transition from high to LOW
	        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
	        }
	       //if the sensor is low for more than the given pause,
	       //we assume that no more motion is going to happen
	       if(!lockLow && millis() - lowIn > pause){
	           //makes sure this block of code is only executed again after
	           //a new motion sequence has been detected
	           lockLow = true;
	           Serial.print("motion ended at ");      //output
	           Serial.print((millis() - pause)/1000);
	           Serial.println(" sec");
	           delay(50);
	           pir.time=millis()/1000;
	           pir.m=false;
	           if(!mesh.write(&pir,'D',sizeof(pir))){
	           	    	       // If a write fails, check connectivity to the mesh network
	           	    	       Serial.println("NODE --> Pir Sensor Motion  Write error Check Connection ");
	           	    	       if( !mesh.checkConnection() ){
	           	    	         //refresh the network address
	           	    	         Serial.println("NODE --> Pir Sensor Motion Renewing Address");
	           	    	         mesh.renewAddress();
	           	    	       }else{
	           	    	    	   //@ TODO implements send mess to master
	           	    	         Serial.println("NODE -->Pir Sensor Motion Send fail, Test OK");
	           	    	       }
	           	    	     }else{
	           	    	     	//radio.printDetails();
	           	    	       Serial.print("NODE --> Pir Sensor Motion Send OK: ");
	           	    	       //Serial.println(displayTimer);
	           	    	     }
	           }
	       }
	// return pir;

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
//
/**
 * Saisie du NodeId
 */
uint8_t getNodeIdFromSerial() {
	uint8_t ret =0;
	uint8_t cpt=0;
	char r =' ';
	char buf[3] = {'@','@','@'} ;
	Serial.print("Set NodeID: ");
	while (cpt <=3 && r !=13){//CRLF
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
	    	   nodeID= getNodeIdFromSerial();
	    	   mesh.setNodeID(nodeID);
	    	   Serial.print("nodeID : ");
	    	   Serial.println(nodeID);
	    	   delay(100);
	       }
	     }
	   //
	Serial.println(F("OPEN-ROC NODE_SENSOR try to connecting to the Open-Roc mesh network..."));
	mesh.begin();
	delay(100);
}

void loop() {

	// loop
	mesh.update();
	//Serial.println("Sleep ..");
	//delay(200);
	// Send to the master node every second
	//
	if (init) {
		// First connect message send to master Type 'I' value : nodeID
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
		calibrationPir();
	}

	//
	detection();
	if (millis() - sensorTimer >= 1000) {
		//

		/*
		if(digitalRead(pirPIN) == HIGH){
			Serial.println("HIGH");
		}else{
			Serial.println("LOW");
		}
		*/
		//sendPir(_pir);
		sensorTimer = millis();
	}

//
	if (millis() - tempTimer >= 3000) {
		//

		tempTimer = millis();
	}
	//
	if (millis() - pingTimer >= 30000) {
		//pingMaster();
		pingTimer = millis();
	}

}
