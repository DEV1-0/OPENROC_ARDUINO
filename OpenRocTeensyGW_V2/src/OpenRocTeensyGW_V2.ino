/*
 * PROJET : OpenROC
 * @author 2015-2016 Eric Papet <e.papet@dev1-0.com.com>
 * @see The GNU Public License (GPL)
 * Board : Teensy 3.1
 * RF : RF24L01+
 * Objet: Teensy Gate Way Integration protocol JSON to MySensor
 * Protocol :  Openroc JSON
 *
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
#include "OpenRocMasterTeensy.h"


char message[50];
char m_node_id[4];
char m_sensor_id[4];
char m_type[2];
char m_type_value[3];
char m_value[50];


//
void processIncomingNodeMessage();
//
void processIncomingSerialMessage();
//
void pingNode(uint8_t cpt);
//
void calculateKbps(uint32_t time_s);


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
	delay(200);
	//
	//presentationNode();
	//monitoring
	pingTimer = millis();
}
//

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
	root_message["node_id"]=mesh.getNodeID(header_rcv.from_node);
	//
	uint8_t nodeId = getNodeId(header_rcv);
	uint8_t nodeId2;
	//
	mess_rcv ++;
	kbCount_rcv += sizeof(header_rcv);
	kbCount_rcv += sizeof(or_mess);
	//

	switch (header_rcv.type) {

		case OR_PRESENTATION:
			root_message["type"] = 0;
		break;

		case OR_SEQ:
			root_message["type"] = 1;
		break;

 	  case OR_REQ:
     root_message["type"] = 2;
    break;

	case OR_INTERNAL : // New Node connection
		root_message["type"] = 3;
		break;
		//
		default:
		//network.read(header, 0, 0);
		//root_message["type"]="F";
		break;
	}
		// récupération du message RF
		network.read(header_rcv, &or_mess, sizeof(or_mess));
		root_message["sensor_id"]=or_mess.sensor_id;
		root_message["type_value"]=or_mess.type_value;
		root_message["value"]=or_mess.value;
	// send to Node.js
	if (root_message.containsKey("type")){
		sprintf(m_node_id,"%i;",root_message["node_id"]);
		sprintf(m_sensor_id,"%i;",or_mess.sensor_id);
		// m_type[2];
		//m_type_value[3];
		//m_value[50];
		//Serial.print(m_node_id);
		//Serial.print(m_sensor_id);
		//Serial.print("\n");
		root_message.printTo(Serial);
		Serial.print("\n");
	}

}

//
void processIncomingSerialMessage() {
}

//
void getSerialCommandFromRouteur() {
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
