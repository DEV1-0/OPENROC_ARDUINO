
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
#include <ArduinoJson.h>

#define MASTER_NODE 0
#define BAUD 115200
#define PING_DELAY 2000
#define KBPS_DELAY 2000


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
uint8_t nodeID = MASTER_NODE;
/**** Configure the nrf24l01 CE and CS pins ****/
uint8_t CE = 9;
uint8_t CS = 10;

bool sateAdressTable = true;
uint32_t checkListTimer = 0;
uint32_t pullTimer = 0;
uint8_t c = 1;

//uint8_t CE = 7 ;
//uint8_t CS = 8;

//uint8_t CE = 3 ;
//uint8_t CS = 7;
//
RF24 radio(CE, CS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);
//Message Types: User message types 1 through 64 will NOT be acknowledged by the network, while message types 65 through 127 will receive a network ACK.
//System message types 192 through 255 will NOT be acknowledged by the network. Message types 128 through 192 will receive a network ACK.
//Type of the packet. 0-127 are user-defined types, 128-255 are reserved for system
//--
//Type de message, on ce serre du Hedaer MessAGE POUR LE TYPE (COMMAND)
//--
#define OR_PRESENTATION 100
		//Sent by a node when they present attached sensors. This is usually done in setup() at startup.
#define OR_SEQ 101
	//This message is sent from or to a sensor when a sensor value should be updated
#define OR_REQ 102
	// Requests a variable value (usually from an actuator destined for controller).
#define OR_INTERNAL 103
	//This is a special internal message. See table below for the details

//--
// presentation type de sensors
//--
#define S_DOOR	0	//Door and window sensors	V_TRIPPED, V_ARMED
#define S_MOTION	1	//Motion sensors	V_TRIPPED, V_ARMED
#define S_SMOKE	2	//Smoke sensor	V_TRIPPED, V_ARMED
#define S_LIGHT	3	// Light Actuator (on/off)	V_STATUS (or V_LIGHT), V_WATT
#define S_BINARY	3	// Binary device (on/off), Alias for S_LIGHT 	V_STATUS (or V_LIGHT), V_WATT
#define S_DIMMER	4	// Dimmable device of some kind	V_STATUS (on/off), V_DIMMER (dimmer level 0-100), V_WATT
#define S_COVER	5	// Window covers or shades	V_UP, V_DOWN, V_STOP, V_PERCENTAGE
#define S_TEMP	6	//Temperature sensor	V_TEMP, V_ID
#define S_HUM	7	//Humidity sensor	V_HUM

//--
// Set/Get Value, type of Value
//--
#define V_TEMP	0	//Temperature 	S_TEMP, S_HEATER, S_HVAC
#define V_HUM	1	//Humidity	S_HUM

//--
// Internal value type
//-
#define I_BATTERY_LEVEL	0	//Use this to report the battery level (in percent 0-100).
#define I_TIME	1	//Sensors can request the current time from the Controller using this message. The time will be reported as the seconds since 1970
#define I_VERSION	2	//Used to request gateway version from controller.
#define I_ID_REQUEST	3	//Use this to request a unique node id from the controller.
#define I_ID_RESPONSE	4	//Id response back to node. Payload contains node id.

void send2Master(void* ,char , int );

#pragma pack(1)
 struct OpenRocMessage {
  int sensor_id;
  int type_value;
  char value[50];
} or_mess;

void reqNodeId(){


}

void presentationNode(){

}

void presentationSensor(){

}
void setValue(int type_value,void*){

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
