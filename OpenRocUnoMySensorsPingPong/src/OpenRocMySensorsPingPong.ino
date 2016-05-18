/*******************************************************************************
 * Projet    :  OpenRoc
 * @author   : eric Papet <e.papet@dev1-0.com>
 * Objet     : integration du protocol Mysensors
 * Board     : Aduino Uno
 * RF        : RF24L01+
 * Protocol  : Mysensors
 * Test      : Sensors Node Monitoring
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
 * DESCRIPTION
 *  This is a simple sketch used to demenstrate and test node-to-node MySensor's communication.
 *  To use this sketch, assemble MySensors nodes - they need nothing more than a radio
 *  1.  Flash each node with the same sketch, open the console and type either 0 or 1 to the respective nodes to set thei ID
 *  2.  You only need to set the node id once, and restart the nodes
 *  3.  To being a ping-pong test, simply type T in the console for one of the nodes.
 *
 *  2015-05-25 Bruce Lacey v1.0
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensor.h>
#include "MYSLog.h"

#define VSN "v1.0"

// Define two generic nodes with a single child
#define YING 200
#define YANG 201
#define CHILD 1

MySensor gw;

MyMessage mPing(CHILD, V_VAR1);   //Ping message
MyMessage mPong(CHILD, V_VAR2);   //Pong message

const char * msgTypeAsCharRepresentation( mysensor_data mType ) ;
const char * nodeTypeAsCharRepresentation( uint8_t node );
void setNodeId(byte nodeID);
void sendPingOrPongResponse( MyMessage msg );
void receive(const MyMessage &message) ;

void setup() {
    gw.begin(receive,AUTO,false);
}

void presentation()  {
  gw.present(CHILD, S_CUSTOM);  //

  gw.sendSketchInfo( nodeTypeAsCharRepresentation( gw.getNodeId() ), VSN );
  LOG(F("\n%sReady.\n"), nodeTypeAsCharRepresentation(gw.getNodeId()));
}

void loop() {

gw.process();
  // Interactive command and control
  // Entering a number from 0 or 1 will write the node 200 (YING) or 201 (YANG) to EEPROM
  // Entering T on either node will initiatve a ping-pong test.
  if (Serial.available()) {
    byte inChar = Serial.read();
    uint8_t node = gw.getNodeId();

    // Manual Test Mode
    if (inChar == 'T' || inChar == 't') {
      LOG(F("T received - starting test...\n"));
      MyMessage msg = mPong;
      msg.sender = (node == YING ? YANG : YING);
      sendPingOrPongResponse( msg );
    }
    else if (inChar == '0' or inChar == '1') {
      byte nodeID = 200 + (inChar - '0');
      setNodeId(nodeID);
    }
    else {
      LOG("Invalid input\n");
    }
  }
}

void receive(const MyMessage &message) {

  LOG(F("Received %s from %s\n"), msgTypeAsCharRepresentation((mysensor_data)message.type), nodeTypeAsCharRepresentation(message.sender));

  delay(250);
  sendPingOrPongResponse( message );
}

void sendPingOrPongResponse( MyMessage msg ) {

  MyMessage response = (msg.type == V_VAR1 ? mPong : mPing);

  LOG(F("Sending %s to %s\n"), msgTypeAsCharRepresentation( (mysensor_data)response.type ), nodeTypeAsCharRepresentation(msg.sender));

  // Set payload to current time in millis to ensure each message is unique
  response.set( millis() );
  response.setDestination(msg.sender);
  gw.send(response);
}

void setNodeId(byte nodeID) {
  LOG(F("Setting node id to: %i.\n***Please restart the node for changes to take effect.\n"), nodeID);
  eeprom_write_byte((uint8_t*)EEPROM_NODE_ID_ADDRESS, (byte)nodeID);
}

const char * msgTypeAsCharRepresentation( mysensor_data mType ) {
  return mType == V_VAR1 ? "Ping" : "Pong";
}

const char * nodeTypeAsCharRepresentation( uint8_t node ) {
  return node == YING ? "Ying Node" : "Yang Node";
}