/*******************************************************************************
 * Projet    :  OpenRoc
 * @author   : eric Papet <e.papet@dev1-0.com>
 * Objet     : integration du protocol Mysensors
 * Board     : DevDuino v3.0  Solar
 * RF        : RF24L01+
 * Protocol  : Mysensors
 * Sensor    : MQ2
 * *****************************************************************************
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
 *
 * DESCRIPTION
 *
 * Connect the MQ2 sensor as follows :
 *
 *   A H A   >>> 5V
 *   B       >>> A0
 *   H       >>> GND
 *   B       >>> 10K ohm >>> GND
 *
 * Contribution: epierre
 * Based on http://sandboxelectronics.com/?p=165
 * License: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
 * Modified by HEK to work in 1.4
 *
 */

#include <SPI.h>
#include <MySensor.h>
#include <Wire.h>

#define   CHILD_ID_LPG                 0
#define   CHILD_ID_CO                  1
#define   CHILD_ID_SMOKE               2
#define   CHILD_ID_DIG                 4 // 0 || 1 if gaz detection

/************************Hardware Related Macros************************************/
#define   MQ_SENSOR_ANALOG_PIN         (0)  //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in
                                                     //normal operation
                                                     #define         GAS_LPG                      (0)
                                                     #define         GAS_CO                       (1)
/**********************Application Related Macros**********************************/
#define         GAS_SMOKE                    (2)
/*****************************Globals***********************************************/
unsigned long SLEEP_TIME = 5000; // Sleep time between reads (in milliseconds)
//VARIABLES
//float Ro = 4700.0;
float Ro = 10;// this has to be tuned 10K Ohm
uint16_t val = 0;
uint16_t last_co = 0;
uint16_t last_smoke = 0;
uint16_t last_lpg = 0;
int last_dig = 0;
// variable to store the value coming from the sensor

float valMQ =0.0;
float LPGCurve[3]  =  {2.3,0.21,-0.47};             //two points are taken from the curve.
float lastMQ =0.0;
//to the original curve.
//with these two points, a line is formed which is "approximately equivalent"
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
//with these two points, a line is formed which is "approximately equivalent"
float COCurve[3]  =  {2.3,0.72,-0.34};              //two points are taken from the curve.
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float SmokeCurve[3] ={2.3,0.53,-0.44};              //two points are taken from the curve.
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2:(lg10000,-0.22)

//epa message definition
MyMessage msg_dig(CHILD_ID_DIG, V_ARMED);
MyMessage msg_co(CHILD_ID_CO, V_LEVEL);
MyMessage msg_lpg(CHILD_ID_LPG, V_LEVEL);
MyMessage msg_smoke(CHILD_ID_SMOKE, V_LEVEL);
MySensor gw;

#include <Vcc.h>
//const float VccExpected   = 3.0;
//const float VccCorrection = 2.860/2.92;  // Measured Vcc by multimeter divided by reported Vcc

const float VccMin   = 0.0;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 5.0;           // Maximum expected Vcc level, in Volts.
const float VccCorrection = 1.0/1.0;
float v;
float p;
float previous_p;
Vcc vcc(VccCorrection);
//
boolean receivedConfig = false;
boolean metric = true;
//epa payload entrant
char convBuf[MAX_PAYLOAD*2+1];
//epa this is the digital pin who change whem gaz is detected
//TODO mount a interrupt in this pin
int pin_d = 7; //Digital pin
void setup()
{
        gw.begin(incomingMessage,AUTO,false);
        // epa digital pin sensors
        pinMode(pin_d, INPUT);
        //
        presentation();
        // Calibartion
        Ro = MQCalibration(MQ_SENSOR_ANALOG_PIN);
        Serial.print("Ro=");
        Serial.println(Ro,DEC);  //Calibrating the sensor. Please make sure the sensor is in clean air
        //when you perform the calibration
}

void loop()
{
        //
        getBatteryLevel();
        // Gas Sensors
        readGazSensor();
        // EPA Mise en lowpower
        gw.sleep(SLEEP_TIME); //sleep for: sleepTime
}

void readGazSensor(){
        // Lecture du senseur sur l'entree digital
        int valeur_digital  = digitalRead(pin_d);
        if (valeur_digital != last_dig) {
                gw.send(msg_dig.set(valeur_digital));
                last_dig = valeur_digital;
        }
        // CO
        val = MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_CO);
        if (val != last_co) {
                gw.send(msg_co.set((int)ceil(val)));
                last_co = val;
                val=0;
        }
        // LPG
        val = MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_LPG);
        if (val != last_co) {
                gw.send(msg_lpg.set((int)ceil(val)));
                last_lpg = val;
                val=0;
        }
        // SMOKE
        val = MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/Ro,GAS_SMOKE);
        if (val != last_smoke) {
                gw.send(msg_smoke.set((int)ceil(val)));
                last_smoke = val;
                val=0;
        }
        Serial.print("Digital Read:");
        Serial.print(valeur_digital);
        Serial.print("\t");
        Serial.print("LPG:");
        Serial.print(last_lpg );
        Serial.print( "ppm" );
        Serial.print("\t");
        Serial.print("CO:");
        Serial.print(last_co );
        Serial.print( "ppm" );
        Serial.print("\t");
        Serial.print("SMOKE:");
        Serial.print(last_smoke);
        Serial.print( "ppm" );
        Serial.print("\n");
}

/**
 * epa
 *
 */
void getBatteryLevel(){
        v = vcc.Read_Volts();
        Serial.print("VCC = ");
        Serial.print(v);
        Serial.println(" Volts");

        p = vcc.Read_Perc(VccMin, VccMax);
        Serial.print("VCC = ");
        Serial.print(p);
        Serial.println(" %");
//Send Battery Level if chage
        if(p !=previous_p ) {
                previous_p = p;
                gw.sendBatteryLevel(p);
        }
//debug
//gw.sendBatteryLevel(p);
}
/**
 * epa
 * Gestion de la présnation du node et des sensors
 *
 */
void presentation(){
        // Send the sketch version information to the gateway and Controller
        gw.sendSketchInfo("Air Quality Sensor", "1.0");

        // Register all sensors to gateway (they will be created as child devices)
        gw.present(CHILD_ID_SMOKE, S_AIR_QUALITY,"FUMÉE");
        gw.present(CHILD_ID_CO, S_AIR_QUALITY,"CO");
        gw.present(CHILD_ID_LPG, S_AIR_QUALITY,"LPG");
        gw.present(CHILD_ID_DIG, S_SMOKE,"DETECTION GAZ");
}


/**
 * Epa
 * Gestion des messages entrant
 *
 */
void incomingMessage(const MyMessage &message) {

        // Pass along the message from sensors to serial line
        Serial.println("Recieve Incomming Message from RF24");
        Serial.print("Sender :");
        Serial.print(message.sender,DEC);
        Serial.print(";");
        Serial.print("Sensor :");
        Serial.print(message.sensor,DEC);
        Serial.print(";");
        Serial.print("Command :");
        Serial.print(mGetCommand(message),DEC);
        Serial.print(";");
        Serial.print("Ack :");
        Serial.print(mGetAck(message),DEC);
        Serial.print(";");
        Serial.print("Type :");
        Serial.print(message.type,DEC);
        Serial.print(";");
        Serial.print("Payload:");
        Serial.println(message.getString(convBuf));
        //Req PRESETATION from Controler
        if(mGetCommand(message) == C_REQ && message.type == V_VAR1) {
                presentation();
        }


        //Serial.print(PSTR("%d;%d;%d;%d;%d;%s\n"),message.sender, message.sensor, mGetCommand(message), mGetAck(message), message.type, message.getString(convBuf));
}

/****************** MQResistanceCalculation ****************************************
   Input:   raw_adc - raw value read from adc, which represents the voltage
   Output:  the calculated sensor resistance
   Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
 ************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
        return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
   Input:   mq_pin - analog channel
   Output:  Ro of the sensor
   Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
        int i;
        float val=0;

        for (i=0; i<CALIBARAION_SAMPLE_TIMES; i++) {    //take multiple samples
                val += MQResistanceCalculation(analogRead(mq_pin));
                delay(CALIBRATION_SAMPLE_INTERVAL);
        }
        val = val/CALIBARAION_SAMPLE_TIMES;             //calculate the average value

        val = val/RO_CLEAN_AIR_FACTOR;                  //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                                        //according to the chart in the datasheet

        return val;
}
/*****************************  MQRead *********************************************
   Input:   mq_pin - analog channel
   Output:  Rs of the sensor
   Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
 ************************************************************************************/
float MQRead(int mq_pin)
{
        int i;
        float rs=0;

        for (i=0; i<READ_SAMPLE_TIMES; i++) {
                rs += MQResistanceCalculation(analogRead(mq_pin));
                delay(READ_SAMPLE_INTERVAL);
        }

        rs = rs/READ_SAMPLE_TIMES;

        return rs;
}

/*****************************  MQGetGasPercentage **********************************
   Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
   Output:  ppm of the target gas
   Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
        if ( gas_id == GAS_LPG ) {
                return MQGetPercentage(rs_ro_ratio,LPGCurve);
        } else if ( gas_id == GAS_CO ) {
                return MQGetPercentage(rs_ro_ratio,COCurve);
        } else if ( gas_id == GAS_SMOKE ) {
                return MQGetPercentage(rs_ro_ratio,SmokeCurve);
        }

        return 0;
}

/*****************************  MQGetPercentage **********************************
   Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
   Output:  ppm of the target gas
   Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
 ************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
        return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
