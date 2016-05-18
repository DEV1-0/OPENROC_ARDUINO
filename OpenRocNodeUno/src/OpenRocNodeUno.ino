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

/**** Configure the nrf24l01 CE and CS pins ****/
uint8_t CE = 9;
uint8_t CS = 10;
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
#define nodeID 3
//
uint32_t displayTimer = 0;
//
#pragma pack(1)
struct payload_t {
	unsigned long ms;
	unsigned long counter;
} payload_send,payload_rcv;
//
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
int NB_SEND = 1000;
uint16_t cpt_send_total = 0;
uint16_t cpt_send_error = 0;
uint16_t cpt_send_ok = 0;
uint16_t cpt_rcv_total = 0;
uint16_t counter_rcv = 0;
uint16_t cpt_renew = 0;
uint16_t start_time = 0;
bool start = true;

void setup() {
	Serial.begin(57600);
	//printf_begin();
	// Set the nodeID manually
	mesh.setNodeID(nodeID);
	// Connect to the mesh
	Serial.println("Connecting to the mesh...");
	mesh.begin();
	start_time = millis();
}
void loop() {
	mesh.update();
	// Send to the master
	if (millis() - displayTimer >= 50 && start) {
		displayTimer = millis();
		// Send an 'M' type message containing the current millis()
			cpt_send_total ++;
			payload_send.counter = cpt_send_total;
			payload_send.ms=displayTimer;
			if (!mesh.write(&payload_send, 'P', sizeof(payload_send))) {
				// If a write fails, check connectivity to the mesh network
				if (!mesh.checkConnection()) {
					//refresh the network address
					Serial.println("Renewing Address");
					cpt_renew++;
					mesh.renewAddress();
				} else {
					cpt_send_error ++;
					Serial.println("Send fail, Test OK");

				}
			} else {
				cpt_send_ok ++;
				Serial.print("Send OK: ");
				Serial.println(displayTimer);
			}
	}
	//
	while (network.available()) {
		RF24NetworkHeader header;
		network.read(header, &payload_rcv, sizeof(payload_rcv));
		cpt_rcv_total ++;
		counter_rcv = payload_rcv.counter,
		Serial.print("Received packet #");
		Serial.print(payload_rcv.counter);
		Serial.print(" at ");
		Serial.println(payload_rcv.ms);
	}
	//
	if( cpt_send_total >= NB_SEND && start){
		start = false;
		test.id = nodeID;
		test.cpt_send_t = cpt_send_total;
		test.cpt_send_OK = cpt_send_ok;
		test.cpt_rcv = cpt_rcv_total;
		test.cpt_send_err = cpt_send_error;
		test.cpt_send_renew = cpt_renew;
		test.rcv_counter = counter_rcv;
		test.time = (millis() - start_time);
		if (!mesh.write(&test, 'M', sizeof(test))) {
			Serial.println(" Monitoring end send Failed ");
		} else{
			Serial.println(" Monitoring end send OK ");
		}

	}
}
