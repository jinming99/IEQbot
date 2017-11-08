#include "Adafruit_SI1145.h"
#include <Arduino.h>
#include "kSeries.h"
#include <Adafruit_AM2315.h>
#include "Adafruit_MCP9808.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi101.h>


// This is the No. of your sensor, which will show on dweet
#define Numbering 1 

// Put 1 if your board is conneted to the type of sensor, 0 otherwise
// The current seeting shows an example
#define K_30_ES 1
#define TGS_2620_ES 1
#define SEN_0177_ES 0
#define AM_2315_ES 0
#define MCP_9808_ES 0
#define SI1145_ES 1
#define DS_18S20_ES 0

WiFiClient client;
char ssid[] = "Put your wifi here";

// The structure to store sensor readings
struct Message {
  int numbering;
  int Visibility;
  int IR;
  int UV_index;
  int PM01; 
  int PM25; 
  int PM10; 
  double CO2;
  float Organic_vapor;
  float Temperature;
  float Humidity;
};
Message data;

#if TGS_2620_ES
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
#define TGS2620_PIN A1
unsigned char buf[LENG];
float rh65[] = {1.72, 1.72, 1.3, 1.3, 1, 0.74, 0.74, 0.62, 0.62};
float Organic_vapor;
#endif

#if SI1145_ES
int Visibility;
int IR;
int UV_index;
Adafruit_SI1145 uv = Adafruit_SI1145();
#endif

#if SEN_0177_ES
int PM01;
int PM25;
int PM10;
#endif

#if K_30_ES
float CO2;
kSeries K_30(12, 13);
#endif

#if AM_2315_ES
float Temperature;
float Humidity;
Adafruit_AM2315 am2315;
#endif

#if MCP_9808_ES
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
float Temperature;
#endif

#if DS_18S20_ES
int ONE_WIRE_BUS = 2;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float Temperature;
#endif

#if (!DS_18S20_ES && !MCP_9808_ES && !AM_2315_ES)
float Temperature = 20.0;
#endif

void setup() {
  setupSerial();
  setupConnectionObjects();
  setupConnection();
  setupSensors();
  setupPins();
}

void loop() {
  read_All();
  prepare_message();
  show_stats();
  uploadData();
  delay(5000);
}

void uploadData() {
  while (WiFi.status() != WL_CONNECTED) {
      setupConnection();
  }
  client.connect("dweet.io", 80);
  delay(3000);
  client.println("GET /dweet/for/CrestResearchData-" + String(data.numbering) + 
                  "?Visibility=" + String(data.Visibility) +
                  "&IR=" + String(data.IR) +
                  "&UV_index=" + String(data.UV_index) +
                  "&PM01=" + String(data.PM01) +
                  "&PM25=" + String(data.PM25) +
                  "&PM10=" + String(data.PM10) +
                  "&CO2=" + String(data.CO2) +
                  "&Organic_vapor=" + String(data.Organic_vapor) +
                  "&Temperature=" + String(data.Temperature) +
                  "&Humidity=" + String(data.Humidity) +
                  " HTTP/1.0");
  client.println();
  client.stop();
}

void read_All() {
  #if SEN_0177_ES
  read_SEN0177();
  #endif

  #if MCP_9808_ES
  read_MCP9808();
  #endif

  #if AM_2315_ES
  read_AM2315();
  #endif

  #if DS_18S20_ES
  read_DS18S20();
  #endif

  #if SI1145_ES
  read_SI1145();
  #endif

  #if K_30_ES
  read_K30();
  #endif

  #if TGS_2620_ES
  read_TGS2620();
  #endif
}

void prepare_message() {
   data.numbering = Numbering;
   #if SI1145_ES
   data.Visibility = Visibility;
   data.IR = IR; 
   data.UV_index = UV_index;
   #else
   data.Visibility = 0;
   data.IR = 0; 
   data.UV_index = 0;
   #endif

   #if SEN_0177_ES
   data.PM01 = PM01;
   data.PM25 = PM25;
   data.PM10 = PM10; 
   #else
   data.PM01 = 0;
   data.PM25 = 0;
   data.PM10 = 0; 
   #endif
  
   #if K_30_ES
   data.CO2 = CO2;
   #else
   data.CO2 = 0.0;
   #endif

   #if TGS_2620_ES
   data.Organic_vapor = Organic_vapor;
   #else
   data.Organic_vapor = 0.0;
   #endif

   #if AM_2315_ES
   data.Temperature = Temperature;
   data.Humidity = Humidity;
   #elif MCP_9808_ES
   data.Temperature = Temperature;
   data.Humidity = 0.0;
   #elif DS_18S20_ES
   data.Temperature = Temperature;
   data.Humidity = 0.0;
   #endif
}

void show_stats() {
  Serial.println("\nVisibility=" + String(data.Visibility) +
                "\nIR=" + String(data.IR) +
                "\nUV_index=" + String(data.UV_index) +
                "\nPM01=" + String(data.PM01) +
                "\nPM25=" + String(data.PM25) +
                "\nPM10=" + String(data.PM10) +
                "\nCO2=" + String(data.CO2) +
                "\nOrganic_vapor=" + String(data.Organic_vapor) +
                "\nTemperature=" + String(data.Temperature) +
                "\nHumidity=" + String(data.Humidity));
  Serial.println("--------------------------------------------------------------------------------------------------");
}

#if DS_18S20_ES
void read_DS18S20() {
  sensors.requestTemperatures();
  Temperature = sensors.getTempCByIndex(0);
}
#endif

#if MCP_9808_ES
void read_MCP9808() {
  tempsensor.shutdown_wake(0);
  Temperature = tempsensor.readTempC();
  delay(250);
  tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere
}
#endif

#if AM_2315_ES
void read_AM2315() {
  Temperature = am2315.readTemperature();
  Humidity = am2315.readHumidity();
}
#endif

#if TGS_2620_ES
void read_TGS2620() { // get the gas value and corrective it
  Organic_vapor = analogRead(TGS2620_PIN);
  if (Temperature > 10.0 && Temperature < 25.0) {
    float gasCorrection = getGasCorrection();
    Organic_vapor = Organic_vapor / gasCorrection;
  }
}

float getGasCorrection () { // get the gas value and corrective it
  float tempRHLow, tempRHHigh;
  for (int i = 0; i < 10; i = i + 1) {
    tempRHLow = i * 2.5 + (i - 1) * 2.5;
    tempRHHigh = (i + 1) * 2.5 + i * 2.5;
    if (tempRHLow <= Temperature && Temperature <= tempRHHigh) {
      return rh65[i];
    }
  }
}
#endif

#if K_30_ES
void read_K30() {
  double co2 = K_30.getCO2('p');
  CO2 = co2;
}
#endif

#if SEN_0177_ES
void read_SEN0177() {
  if (Serial1.find(0x42)) {   //start to read when detect 0x42
    Serial1.readBytes(buf, LENG);
    if (buf[0] == 0x4d) {
      if (checkValue(buf, LENG)) {
        PM01 = transmitPM01(buf); //count PM1.0 value of the air detector module
        PM25 = transmitPM2_5(buf); //count PM2.5 value of the air detector module
        PM10 = transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }
}

char checkValue(unsigned char *thebuf, char leng) {
  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }
  receiveSum = receiveSum + 0x42;

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1])) { //check the serial data
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf) {
  int PM01Val;
  PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf) {
  int PM2_5Val;
  PM2_5Val = ((thebuf[5] << 8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM2_5Val;
}

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf) {
  int PM10Val;
  PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}
#endif

#if SI1145_ES
void read_SI1145() {
  Visibility = uv.readVisible();
  IR = uv.readIR();
  UV_index = uv.readUV();
}
#endif

double read_voltage(int pin) {
  return analogRead(pin) / 1023.0 * readVcc();
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

char *f2s(float f, int p){
  char * pBuff;             // use to remember which part of the buffer to use for dtostrf
  const int iSize = 10;           // number of bufffers, one for each float before wrapping around
  static char sBuff[iSize][20];         // space for 20 characters including NULL terminator for each float
  static int iCount = 0;          // keep a tab of next place in sBuff to use
  pBuff = sBuff[iCount];          // use this buffer
  if(iCount >= iSize -1){         // check for wrap
    iCount = 0;             // if wrapping start again and reset
  }
  else{
    iCount++;               // advance the counter
  }
  return dtostrf(f, 0, p, pBuff);       // call the library function
}

void setupSensors() {
  #if SI1145_ES
  setupUV();
  #endif

  #if AM_2315_ES
  setupAM();
  #endif

  #if MCP_9808_ES
  setupMCP();
  #endif
}

#if AM_2315_ES
void setupAM() {
  if (! am2315.begin()) {
    Serial.println("Sensor not found, check wiring & pullups!");
    while (1);
  }
}
#endif

#if SI1145_ES
void setupUV() {
  if (! uv.begin()) {
    Visibility = 0;
    IR = 0;
    UV_index = 0;
    while (1);
  }
}
#endif

#if MCP_9808_ES
void setupMCP() {
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
}
#endif

void setupSerial() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void setupPins() {
  #if TGS_2620_ES
  pinMode(TGS2620_PIN, INPUT);
  #endif
}


void setupConnectionObjects() {
  WiFi.begin(ssid);
}

void setupConnection() {
  // attempt to connect to Wifi network:
  delay(3000);
  while (WiFi.status() != WL_CONNECTED) {
    // wait 10 seconds for connection:
    ET.sendData();
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(ssid);
    delay(10000);
  }
}

