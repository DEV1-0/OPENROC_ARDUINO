/*******************************************************************************
 * Projet    :  OpenRoc
 * @author   : eric Papet <e.papet@dev1-0.com>
 * Objet     : integration du protocol Mysensors
 * Board     : Aduino Uno
 * RF        : RF24L01+
 * Protocol  : Mysensors
 * Sensor    : DHT22
 ******************************************************************************/
/**
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
 * John Main added dewpoint code from : http://playground.arduino.cc/main/DHT11Lib
 * Also added DegC output for Heat Index.
 * dewPoint function NOAA
 * reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
 * reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
 */

  #include <MySensor.h>
  #include <SPI.h>
  #include <Stream.h>

  #include <SPI.h>
  #include <DHT.h>
  #define DHT_SENSOR_PIN 2
  #define DHT_SENSOR_TYPE DHT22
// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
float temperature;
float dewpoint;
float humidity;
float heatindex;

double dewPoint(double celsius, double humidity)
{
        // (1) Saturation Vapor Pressure = ESGG(T)
        double RATIO = 373.15 / (273.15 + celsius);
        double RHS = -7.90298 * (RATIO - 1);
        RHS += 5.02808 * log10(RATIO);
        RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO ))) - 1);
        RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
        RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
        double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
        double T = log(VP / 0.61078); // temp var
        return (241.88 * T) / (17.558 - T);
}


  #include <Vcc.h>
//const float VccExpected   = 3.0;
//const float VccCorrection = 2.860/2.92;  // Measured Vcc by multimeter divided by reported Vcc
const float VccMin   = 0.0;             // Minimum expected Vcc level, in Volts.
const float VccMax   = 5.0;             // Maximum expected Vcc level, in Volts.
const float VccCorrection = 1.0/1.0;
float v;
float p;
float previous_p;
Vcc vcc(VccCorrection);


//Low Power
unsigned long SLEEP_TIME = 15000;   // Sleep time between reads (in milliseconds)

MySensor node;
//
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message 0=sensor_id,V_TEMP=Metric Type
MyMessage msg_temp(0,V_TEMP);
MyMessage msg_hum(1,V_HUM);
MyMessage msg_dp(2,V_TEMP);
MyMessage msg_hi(4,V_TEMP);
//MyMessage msg_var(255,V_VAR1);
//MyMessage msg_press(255,S_ARDUINO_NODE);

//
//epa payloar entrant
char convBuf[MAX_PAYLOAD*2+1];
unsigned long timer;
//unsigned long time;

//Timer pour les messages entrant incomming(&Message)
unsigned long timer_sleep;

unsigned int NB_SENSORS = 2;

const char MODEL_0 [] ="DHT22";
const char MODEL_1 [] ="Dew Point";
const char MODEL_2 [] ="Heat Index";
//const char MODEL_1 [] ="DHT22";
const char API_VERSION [] ="1.5.4";




void presentation(){
        node.sendSketchInfo("DHT22 Node", "1.1");
        node.present(255, S_ARDUINO_NODE,API_VERSION);
        delay(100);
        node.present(0, S_TEMP,MODEL_0);
        delay(100);
        node.present(1,S_HUM,MODEL_0);
        delay(100);
        node.present(2, S_TEMP,MODEL_1);
        delay(100);
        node.present(4, S_TEMP,MODEL_2);
}

void setup()
{
        // EPA DEBUG
        Serial.begin(115200);
        delay(100);
        Serial.println("Serial Started");
        //Initialisatio
        dht.begin();
        // Startup and initialize MySensors library. Set callback for incoming messages.
        node.begin(incomingMessage,AUTO,false);
        //
        presentation();
        //
        metric = node.getConfig().isMetric;
}


void loop()
{
        //

        node.process();

        // Reading temperature or humidity takes about 250 milliseconds!
        //delay(dht.getMinimumSamplingPeriod());
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        humidity = dht.readHumidity();
        // Read temperature as Celsius
        temperature = dht.readTemperature();
        // Read temperature as Fahrenheit
        float f = dht.readTemperature(true);

        // Check if any reads failed and exit early (to try again).
        if (isnan(humidity) || isnan(temperature) || isnan(f)) {
                Serial.println("Failed to read from DHT sensor!");
                return;
        }

        // Compute heat index
        // Must send in temp in Fahrenheit!
        float hi = dht.computeHeatIndex(f, humidity);
        heatindex = dht.convertFtoC(hi);
        dewpoint = dewPoint(temperature, humidity);
        //send
        node.send(msg_temp.set(temperature, 1));
        delay(100);
        node.send(msg_hum.set(humidity, 1));
        delay(100);
        node.send(msg_dp.set(dewpoint, 1));
        delay(100);
        //msg_hi.setSensor(3);
        node.send(msg_hi.set(heatindex, 1));

        //debug
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.print(" %\t");
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" *C ");
        Serial.print(f);
        Serial.print(" *F\t");
        Serial.print("Heat index: ");
        Serial.print(heatindex);
        Serial.print(" *C ");
        Serial.print(hi);
        Serial.print(" *F ");
        Serial.print("Dew Point (*C): ");
        Serial.println(dewpoint);
        //
        //epa
        if((millis() - timer) > 10000) {
                timer=millis();
        }
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
                node.sendBatteryLevel(p);
        }
        //

        //
        //  if((millis() - timer_sleep) > 10000){
        node.sleep(SLEEP_TIME);
        //timer_sleep=millis();
        //  }

}

void incomingMessage(const MyMessage &message) {
        //  if (mGetCommand(message) == C_PRESENTATION && inclusionMode) {
        //	gw.rxBlink(3);
        //   } else {
        //	gw.rxBlink(1);
        //   }
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

//Calback time
void receiveTime(unsigned long time) {
        // Ok, set incoming time
        Serial.print("Recieve Time :");
        Serial.println(time,DEC);
}
