/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Temperature and Humidity Sensor Node
 * 			Protocol Openroc JSON
 * Sensor: DHT21 and DS18B20
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

#include "OneWire.h"
#include "DallasTemperature.h"

#include <stdio.h>

#define MESH_NOMASTER 0

#define DHTPIN 2     // what digital pin we're connected to

//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT21   // DHT 21 (AM2301)

// AM2301DHT Sensors
///////////////////////////////////////////////////////////////////////
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
DHT sensorDHT(DHTPIN, DHTTYPE);

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
uint8_t nodeID = 5;
uint32_t displayTimer = 0;
uint32_t sensorTimer = 0;
uint32_t tempTimer = 0;
uint32_t pingTimer = 0;
boolean init = true;
//
#pragma pack(1)
struct TemperatureHumidity {
	uint8_t id;
	boolean b;
	float t;
	float r;
	float h;
	char model[10];
} th;
//
#pragma pack(1)
struct Temperature {
	uint8_t id;
	float t;
	char model[10];
} temperature;

//
//DALLAS TEMPERATURE
///////////////////////////////////////////////////////
// Connect Gnd (black or blueue  teensy Gnd
// Vcc (red cable)     to     teensy +3.3V pin
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 3 of the sensor to whatever your DHTPIN is
// Connect a 10K resistor from pin 3 (data) to pin 1 (power) of the sensor (pull up)
#define DS18B20 0x28     // Adresse 1-Wire du DS18B20
#define BROCHE_ONEWIRE 3 // Broche utilisée pour le bus 1-Wire
//
OneWire ds(BROCHE_ONEWIRE); // Création de l'objet OneWire ds
//
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensorsDS18B20(&ds);
/////////////////////////////////////////////////////////
/**
 * Lecture de la sont de température étanche DS18B20 avec seulement la library OneWire
 * Fonction récupérant la température depuis le DS18B20
 * Retourne true si tout va bien, ou false en cas d'erreur
 * Ref : https://skyduino.wordpress.com/2012/04/26/arduino-capteur-de-temperature-ds18b20/
 */
boolean getTemperatureDS18B20(float *temp) {
	byte data[9], addr[8];
	// data : Données lues depuis le scratchpad
	// addr : adresse du module 1-Wire détecté

	if (!ds.search(addr)) { // Recherche un module 1-Wire
		ds.reset_search();    // Réinitialise la recherche de module
		return false;         // Retourne une erreur
	}

	if (OneWire::crc8(addr, 7) != addr[7]) // Vérifie que l'adresse a été correctement reçue
		return false;       // Si le message est corrompu on retourne une erreur

	if (addr[0] != DS18B20) // Vérifie qu'il s'agit bien d'un DS18B20
		return false;         // Si ce n'est pas le cas on retourne une erreur

	ds.reset();             // On reset le bus 1-Wire
	ds.select(addr);        // On sélectionne le DS18B20

	ds.write(0x44, 1);      // On lance une prise de mesure de température
	delay(800);             // Et on attend la fin de la mesure

	ds.reset();             // On reset le bus 1-Wire
	ds.select(addr);        // On sélectionne le DS18B20
	ds.write(0xBE);         // On envoie une demande de lecture du scratchpad

	for (byte i = 0; i < 9; i++) // On lit le scratchpad
		data[i] = ds.read();       // Et on stock les octets reçus

	// Calcul de la température en degré Celsius
	*temp = ((data[1] << 8) | data[0]) * 0.0625;

	// Pas d'erreur
	return true;
}

void printSerialTempDS18B20(float temp){
	// Affiche la température
				Serial.print("SENSORS DALLAS DS18B20 ::");
				Serial.print("Temperature : ");
				Serial.print(temp,DEC);
				//Serial.write(176); // caractère °
				Serial.println("°C");
}

/**
 * Lecture de la sont de température étanche DS18B20 avec ls library OneWire & DallasTemperature
 * La librairie DallasTemperature offre une API avec getstion des alarmes
 *
 */
void readSensorDS18B20 (){
	   // call sensors.requestTemperatures() to issue a global temperature
	  // request to all devices on the bus
	float temp;
	//
	//sprintf(sensor_model,"DS18B20");
	sprintf(temperature.model,"%s","DS18B20");
	//th.model = {'D','S'};
	  Serial.print("Requesting temperatures for DS18B20 by Dallas Library...");
	  sensorsDS18B20.requestTemperatures(); // Send the command to get temperatures
	  Serial.println("DONE");
	  temp=sensorsDS18B20.getTempCByIndex(0);// Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
	  Serial.print("Temperature for Device 1 is: ");
	  Serial.println(temp);
	  temperature.id=nodeID;
	  temperature.t=temp;
	 // temperature.model= (char[])  sensor_model;
	  if (!mesh.write(&temperature, 'T', sizeof(temperature))) {
	  		// If a write fails, check connectivity to the mesh network
	  		Serial.println("SensorDS18B20 Send Error --> temperature RF Write error Check Connection ");
	  		if (!mesh.checkConnection()) {
	  			//refresh the network address
	  			Serial.println("SensorDS18B20 Send Error -->  Renewing Address..");
	  			mesh.renewAddress();

	  		} else {
	  			Serial.println("SensorDS18B20 Send Error --> Send by RF fail After RenewAddresse..");
	  		}
	  	} else {
	  		Serial.println("SensorDS18B20 --> Data temperature Send OK.. ");
	  	}


}

void readSensorDHT() {
	//
	 sprintf(th.model,"%s","DHT21");
	//temperature.model = {'D','H','2','1'};
	//typedef struct TemperatureHumididy payload;
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float h = sensorDHT.readHumidity();

	// Read temperature as Celsius (the default)
	float t = sensorDHT.readTemperature();

	// Read temperature as Fahrenheit (isFahrenheit = true)
	float f = sensorDHT.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
		Serial.println("Failed to read from DHT sensor!");
		th.b = false;

	} else {
		// Compute heat index in Fahrenheit (the default)
		float hif = sensorDHT.computeHeatIndex(f, h);
		// Compute heat index in Celsius (isFahreheit = false)
		float hic = sensorDHT.computeHeatIndex(t, h, false);
		//
		Serial.print("SENSORS AM2301 DHT21 :: ");
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
		th.b = true;
		th.t = t;
		th.r = hic;
		th.h = h;
	}
	//
	th.id = nodeID;
	//
	if (!mesh.write(&th, 'H', sizeof(th))) {
		// If a write fails, check connectivity to the mesh network
		Serial.println("SensorDHTSend Error --> temp/humidity RF Write error Check Connection ");
		if (!mesh.checkConnection()) {
			//refresh the network address
			Serial.println("SensorDHT Send Error -->  Renewing Address..");
			mesh.renewAddress();

		} else {
			Serial.println("SensorDHT Send Error --> Send by RF fail After RenewAddresse..");
		}
	} else {
		Serial.println("SensorDHT --> Data temp/humidity Send OK.. ");
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
	mesh.setNodeID(nodeID);
	// Set the nodeID by console
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
	//sensor DTF
	sensorDHT.begin();
	//
	sensorsDS18B20.begin();
	//
	delay(100);
}

void loop() {
	float temp;
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

	}

	//

	if (millis() - sensorTimer >= 5000) {
		readSensorDHT();
		sensorTimer = millis();
	}
//
	if (millis() - tempTimer >= 3000) {
		//
		readSensorDS18B20();
		tempTimer = millis();
		/**
		 * Test de lecture bas niveau du bus Wire
		 * sans la lib DALLAS
		if (getTemperatureDS18B20(&temp)) {
			printSerialTempDS18B20(temp);
		}else{
			Serial.println("getTemperatureDS18B20 Failed ....");
		}
		delay(200);
		**/
		// by library
	}
	//
	//if (millis() - pingTimer >= 30000) {
		//pingMaster();
		//pingTimer = millis();
	//}

}
