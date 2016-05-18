#include <Arduino.h>

























#include <MySensor.h>
#include <SPI.h>
#include <Stream.h>

#include <SPI.h>
#include <DHT.h>
#define DHT_SENSOR_PIN 2
#define DHT_SENSOR_TYPE DHT22




double dewPoint(double celsius, double humidity);




void presentation();

void setup();


void loop();

void incomingMessage(const MyMessage &message);
void receiveTime(unsigned long time);
#line 38 "MySensorUnoDHT.ino"
DHT dht(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
float temperature;
float dewpoint;
float humidity;
float heatindex;






double dewPoint(double celsius, double humidity)
{

double RATIO = 373.15 / (273.15 + celsius);
double RHS = -7.90298 * (RATIO - 1);
RHS += 5.02808 * log10(RATIO);
RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO ))) - 1);
RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
RHS += log10(1013.246);


double VP = pow(10, RHS - 3) * humidity;


double T = log(VP / 0.61078); // temp var
return (241.88 * T) / (17.558 - T);
}


#include <Vcc.h>


const float VccMin   = 0.0;             // Minimum expected Vcc level, in Volts.
const float VccMax   = 5.0;             // Maximum expected Vcc level, in Volts.
const float VccCorrection = 1.0/1.0;
float v;
float p;
float previous_p;
Vcc vcc(VccCorrection);



unsigned long SLEEP_TIME = 15000;   // Sleep time between reads (in milliseconds)

MySensor node;

boolean receivedConfig = false;
boolean metric = true;

MyMessage msg_temp(0,V_TEMP);
MyMessage msg_hum(1,V_HUM);
MyMessage msg_dp(2,V_TEMP);
MyMessage msg_hi(4,V_TEMP);





char convBuf[MAX_PAYLOAD*2+1];
unsigned long timer;



unsigned long timer_sleep;

unsigned int NB_SENSORS = 2;

const char MODEL_0 [] ="DHT22";
const char MODEL_1 [] ="Dew Point";
const char MODEL_2 [] ="Heat Index";

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

Serial.begin(115200);
delay(100);
Serial.println("Serial Started");

dht.begin();

node.begin(incomingMessage,AUTO,false);

presentation();

metric = node.getConfig().isMetric;
}


void loop()
{


node.process();




humidity = dht.readHumidity();

temperature = dht.readTemperature();

float f = dht.readTemperature(true);


if (isnan(humidity) || isnan(temperature) || isnan(f)) {
Serial.println("Failed to read from DHT sensor!");
return;
}



float hi = dht.computeHeatIndex(f, humidity);
heatindex = dht.convertFtoC(hi);
dewpoint = dewPoint(temperature, humidity);

node.send(msg_temp.set(temperature, 1));
delay(100);
node.send(msg_hum.set(humidity, 1));
delay(100);
node.send(msg_dp.set(dewpoint, 1));
delay(100);

node.send(msg_hi.set(heatindex, 1));


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

if(p !=previous_p ) {
previous_p = p;
node.sendBatteryLevel(p);
}




node.sleep(SLEEP_TIME);



}

void incomingMessage(const MyMessage &message) {






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

if(mGetCommand(message) == C_REQ && message.type == V_VAR1) {
presentation();
}



}


void receiveTime(unsigned long time) {

Serial.print("Recieve Time :");
Serial.println(time,DEC);
}
