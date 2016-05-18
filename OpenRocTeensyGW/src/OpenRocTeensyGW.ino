/*******************************************************************************
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Gate Way (GW) entre le réseaux mesh des node et le routeur Web
 * 			Gestion du réseau Mesh
 * 			gestion des messages envoyés par les Nodes Sensors
 * 			Envoie des messages au routeur HTTP par la sortie série
 * Messages :
 * 			header 'I' attachement d'un nouveau Node au réseau Mesh payload: NodeID (int):AddressMesh (Octal) ex: {type:'N',id:1,add::02}
 * 			Header 'L' Topologie du réseau Mesh payload : {type:'L',[{id:1,add::02},{id:2,add::03},{id:1,add::021}]}
 * 			header 'T' Température en Celsus + humidité en % payload: {type:'H',temp:25,6,hum:30}
 *******************************************************************************
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
*******************************************************************************/

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

#include <Time.h>

#include <ArduinoJson.h>

#define MASTER_NODE 0
#define BAUD 115200
#define PING_DELAY 2000
#define KBPS_DELAY 2000

/***** Configure the chosen CE,CS pins *****/
uint8_t CE = 9;
uint8_t CS = 10;
//
//
//float temperature;
//
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
// Pir Motion sensor
//
#pragma pack(1)
struct Pir{
	uint32_t time;
	bool m;
	char model[10] ;
} pir;
//
// Magnetic Door
//
#pragma pack(1)
struct Door {
    bool s;
    char model[10];
 } door;

//
#pragma pack(1)
struct InputCommand {
	uint8_t id;
	uint8_t mess;
} command;
// Receive Ping to test Node
#pragma pack(1)
struct Test {
	uint8_t id;
	uint16_t cpt_send_t;
	uint16_t cpt_send_OK;
	uint16_t cpt_rcv;
	uint16_t cpt_send_err;
	uint16_t cpt_send_renew;
	uint16_t rcv_counter;
	uint16_t time;
} test;
//
#pragma pack(1)
struct payload_t {
	unsigned long ms;
	unsigned long counter;
} payload_t;
uint16_t cpt_rcv_ping = 0;
unsigned long rcv_ping_counter = 0;
//
// Ping node
uint8_t nodeCounter;
uint16_t failID = 0;
uint32_t kbTimer = 0, pingTimer = 0;
uint8_t boldID = 0;
//
float kbCount_send = 0;
float kbCount_rcv = 0;
unsigned long mess_send = 0;
unsigned long mess_rcv = 0;
//
//

//
void processIncomingNodeMessage();
//
void processIncomingSerialMessage();
//
void pingNode(uint8_t cpt);
//
void calculateKbps(uint32_t time_s);

//
RF24 radio(CE, CS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
//
//
void setup() {
	//delay(10000);
	Serial.begin(BAUD);
	 while (!Serial) {
	    // wait serial port initialization
	  }
	///printf_begin();
	// Set the nodeID to 0 for the master node
	mesh.setNodeID(0);
	//Serial.print("SETUP Node ID:");
	//Serial.println(mesh.getNodeID());
	//printf(" SETUP Node ID: %d",mesh.getNodeID());
	// Connect to the mesh
	mesh.begin();
	//radio.printDetails();
	delay(1000);
	//monitoring
	pingTimer = millis();
}
//
bool sateAdressTable = true;
uint32_t checkListTimer = 0;
uint32_t pullTimer = 0;
uint8_t c = 1;
//int cpt = 0;
//
void loop() {
	// Call mesh.update to keep the network updated
	mesh.update();
	// In addition, keep the 'DHCP service' running on the master node so addresses will
	// be assigned to the sensor nodes
	mesh.DHCP();
	// Check for incoming RF data from the sensors and send to WEB ROUTEUR
	if (network.available()) {
		processIncomingNodeMessage();
	}
	//  Check for incoming Serial data from WEB ROUTEUR
	if (Serial.available()) {
		processIncomingSerialMessage();
	}
	// Ping each connected node, one per PING_DELAY ms
	if (millis() - pingTimer > PING_DELAY && sizeof(mesh.addrList) > 0) {
		pingTimer = millis();
		if (nodeCounter == mesh.addrListTop) { // if(mesh.addrMap.size() > 1){ it=mesh.addrMap.begin(); } continue;}
			nodeCounter = 0;
		}
		if (mesh.addrListTop > 0) {
			pingNode(nodeCounter);
			nodeCounter++;
		}
	}
	//send Sum of (header + payload) size send by second
	if (millis() - kbTimer > KBPS_DELAY && (kbCount_rcv + kbCount_send) > 0) {
		kbTimer = millis();
		// ms to s
		calculateKbps(((millis() - kbTimer) * 0,0001));
	}
}

//
void processIncomingNodeMessage() {
	// Memory pool for JSON object tree.
	  //
	  // Inside the brackets, 200 is the size of the pool in bytes.
	  // If the JSON object is more complex, you need to increase that value.
	  StaticJsonBuffer<200> jsonBuffer;

	  // StaticJsonBuffer allocates memory on the stack, it can be
	  // replaced by DynamicJsonBuffer which allocates in the heap.
	  // It's simpler but less efficient.
	  //
	  // DynamicJsonBuffer  jsonBuffer;

	  // Create the root of the object tree.
	  //
	  // It's a reference to the JsonObject, the actual bytes are inside the
	  // JsonBuffer with all the other nodes of the object tree.
	  // Memory is freed when jsonBuffer goes out of scope.
	  JsonObject& root_message = jsonBuffer.createObject();

	//
	RF24NetworkHeader header_rcv;
	RF24NetworkHeader header_send;
	network.peek(header_rcv);
	//
	uint8_t nodeId = getNodeId(header_rcv);
	uint8_t nodeId2;
	//
	mess_rcv ++;
	kbCount_rcv += sizeof(header_rcv);
	//
	switch (header_rcv.type) {
	case 'I': // New Node connection
		network.read(header_rcv, &nodeId2, sizeof(nodeId));
		kbCount_rcv += sizeof(nodeId);
		root_message["type"] = "I";
		root_message["node_id"] = nodeId;
		root_message["net_adr"] = getNodeAdr(nodeId);

		break;
		//
	case 'H':
		network.read(header_rcv, &th, sizeof(th));
		//
		kbCount_rcv += sizeof(th);
		//
		root_message["type"] = "S";
		root_message["node_id"] = nodeId;
		root_message["t"] = th.t;
		root_message["r"] = th.r;
		root_message["h"] = th.h;
		root_message["model"]=th.model;

		break;
	case 'D':
			network.read(header_rcv, &pir, sizeof(pir));
			//
			kbCount_rcv += sizeof(pir);
			//
			root_message["type"] = "S";
			root_message["node_id"] = nodeId;
			root_message["p"] = pir.m;
			root_message["t"] = pir.time;
			root_message["model"]=pir.model;
			break;
	case 'O':
				//struct Door door;
				network.read(header_rcv, &door, sizeof(door));

				//
				kbCount_rcv += sizeof(door);
				//
				root_message["type"] = "S";
				root_message["node_id"] = nodeId;
				root_message["m"] = door.s;
				root_message["model"] = door.model;
				break;
	case 'T':
		network.read(header_rcv, &temperature, sizeof(temperature));
		//String mess1 = String("{\"type\":\"")+ header.type+ "\",\"id\":" + header.id + ",\"t\":" + temperature+"}");
		//Serial.print(mess);
		kbCount_rcv += sizeof(temperature);
		root_message["type"] = "S";
		root_message["node_id"] = nodeId;
		root_message["t"] = temperature.t;
		root_message["model"]=temperature.model;

		break;
	case 'M': // Mode test node
		//{"type":"M","id":2,"net_addr":"00,"nb_send":1000,"nb_rcv":0,"nb_err":0,"nb_renew":0,"time":99,"gw_rcv_ping":1000,"gw_rcv_counter":0"}
		network.read(header_rcv, &test, sizeof(test));
		kbCount_rcv += sizeof(test);
		root_message["type"]="M";
		root_message["node_id"]=nodeId ;
		root_message["net_adr"]=getNodeAdr(header_rcv);
		root_message["cpt_node_send_t"]=test.cpt_send_t ;
		root_message["cpt_node_send_ok"]=test.cpt_send_OK ;
		root_message["cpt_node_send_err"]=test.cpt_send_err ;
		root_message["cpt_node_renew_ok"]=test.cpt_send_renew ;
		root_message["cpt_node_rcv"]=test.cpt_rcv ;
		root_message["node_time"]=test.time ;
		root_message["gw_rcv_ping"]= cpt_rcv_ping;
		root_message["gw_rcv_counter"]=rcv_ping_counter ;

		//
		cpt_rcv_ping = 0;
		rcv_ping_counter = 0;
		break;
		//
	case 'P':// mode test
		network.read(header_rcv, &payload_t, sizeof(payload_t));
		cpt_rcv_ping ++;
		rcv_ping_counter = payload_t.counter;
		kbCount_rcv += sizeof(payload_t);
		header_send = RF24NetworkHeader(getNodeAdr(header_rcv),10); //Constructing a header && The type of message which follows. Only 0-127 are allowed for user messages. Types 1-64 will not receive a network acknowledgement.
		network.write(header_send, &payload_t, sizeof(payload_t));
		kbCount_send += sizeof(header_send);
		kbCount_send += sizeof(payload_t);
		mess_send ++;
		break;
	default:
		//network.read(header, 0, 0);
		root_message["type"]="F";
		break;
	}
	// send to Node.js
	if (root_message.containsKey("type")){
		root_message.printTo(Serial);
		Serial.println();
	}

}

//
void processIncomingSerialMessage() {
	getSerialCommandFromRouteur();
	switch ((char) command.mess) {
	case 'P': //ping node
		if (!mesh.write(&c, 'P', sizeof(c), command.id)) {
			Serial.print("{");
			Serial.print("\"type\":");
			Serial.print("\"R\""); // retour de command
			Serial.print(",");
			Serial.print("\"id\":");
			Serial.print(command.id);
			Serial.print(",");
			Serial.print("\"mess\":");
			Serial.print("\"P\":");
			Serial.print("\"result\":");
			Serial.print(1);
			Serial.println("\"}");
		} else {
			Serial.print("{");
			Serial.print("\"type\":");
			Serial.print("\"R\""); // retour de command
			Serial.print(",");
			Serial.print("\"id\":");
			Serial.print(command.id);
			Serial.print(",");
			Serial.print("\"mess\":");
			Serial.print("\"P\":");
			Serial.print("\"result\":");
			Serial.print(0);
			Serial.println("\"}");
		}
	}
}

//
void getSerialCommandFromRouteur() {
	bool stop = false;
	byte b;
	//byte [2] command;
	int cpt = -1;
	while (!stop) {
		b = 0;
		if (Serial.available()) {
			b = Serial.read();

			if (b == 64 && cpt > 0) // End Message '@'
				stop = true;

			if (b == 64) // begin message '@'
				cpt = 0;
			else if (cpt >= 0)
				cpt++;

			switch (cpt) {
			case 1:
				command.id = b;
				break;
			case 2:
				command.mess = b;

				break;

			}
			//Serial.print("CPT :");
			//Serial.println(cpt);
			//Serial.print("ECHO :");
			//Serial.println((char) b);
		}
	}

}

//
void pingNode(uint8_t listNo) {
	RF24NetworkHeader headers(mesh.addrList[listNo].address, NETWORK_PING);
	uint32_t pingtime = millis();
	//
	StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root_message = jsonBuffer.createObject();
    //
	bool ok;
	bool state = true;
	if (headers.to_node) {
		ok = network.write(headers, 0, 0);
		if (ok && (failID == mesh.addrList[listNo].nodeID)) {
			failID = 0;
			//state = true;
		}
		if (!ok) {
			failID = mesh.addrList[listNo].nodeID;
			state = false;
		}else{
			kbCount_send =+ sizeof(headers);
			mess_send ++;
		}
	}
	pingtime = millis() - pingtime;
	//
	root_message["type"]="P";
	root_message["nb_node"]=mesh.addrListTop;
	root_message["node_id"]=mesh.addrList[listNo].nodeID;
	root_message["net_adr"]=mesh.addrList[listNo].address;

	if (ok) {
		root_message["time_ping"]=pingtime;
		root_message["state"]=true;
	} else {
		root_message["time_ping"]=pingtime;
		root_message["state"]=false;
	}
	//Serial.println("}");
	 root_message.printTo(Serial);
	  // This prints:
	  // {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}
	 Serial.println();
	//end of message
}

//
void calculateKbps(uint32_t time_s){
	//Serial.println(kbps);
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject& root_message = jsonBuffer.createObject();
	root_message["type"] = "K";
	root_message["kbps_rcv"] = ((kbCount_rcv /1024) / time_s);
	root_message["kbps_send"] = ((kbCount_send /1024) / time_s);
	root_message["nb_message_rcv"] = mess_rcv;
	root_message["nb_message_send"] = mess_send;
	//
	root_message.printTo(Serial);
	Serial.println();
//
	mess_send=0;
	mess_rcv = 0;
	kbCount_rcv = 0;
	kbCount_send = 0;
}

void sendToGateway(byte* message, int length) {
//
	Serial.write(message, length);
	Serial.flush();
}



/**
 *
 */
void checkChildList() {
// Display the currently assigned addresses and nodeIDs
	if (millis() - checkListTimer > 5000) {

		checkListTimer = millis();
		if (mesh.addrListTop <= 0 && sateAdressTable) {
			Serial.print("{");
			Serial.print("type:");
			Serial.print("L");
			Serial.print(",");
			Serial.print("list:");
			Serial.println("[]}");
			sateAdressTable = false;
		} else {
			//Serial.println(F("\n\n****Check Master Table Assigned Addresses****"));
			Serial.print("{");
			Serial.print("type:");
			Serial.print("L");
			Serial.print(",");
			Serial.print("list:");
			Serial.print("[");
			for (int i = 0; i < mesh.addrListTop; i++) {
				//
				if (i != 0)
					Serial.print(",");
				//
				Serial.print("{");
				Serial.print("\"id\":");
				Serial.print(mesh.addrList[i].nodeID);
				Serial.print(",");
				Serial.print("add:0");
				Serial.print(mesh.addrList[i].address, OCT);
				Serial.print("}");
			}
			Serial.println("]}");
		}
		sateAdressTable = true;
	}
}

/**
 *
 *
 */
void checkMessage(RF24NetworkHeader header, uint32_t data) {
	Serial.println("Header: ");
//
	Serial.print("\t type:");
	Serial.println(header.type);
//
	Serial.print("\t to_node:");
	Serial.println(header.to_node, OCT);
//
	Serial.print("\t from_node:");
	Serial.println(header.from_node, OCT);
//
	Serial.print("recieve data:");
	Serial.println(data);
//radio.printDetails();
}

uint8_t getNodeId(RF24NetworkHeader header) {
	uint8_t ret = 0;
	for (int i = 0; i < mesh.addrListTop; i++) {
		if (mesh.addrList[i].address == header.from_node)
			ret = mesh.addrList[i].nodeID;
	}
	return ret;
}

//
uint16_t getNodeAdr(RF24NetworkHeader header) {
	uint16_t ret = 0;
	for (int i = 0; i < mesh.addrListTop; i++) {
		if (mesh.addrList[i].address == header.from_node)
			ret = mesh.addrList[i].address;
	}
	return ret;
}
