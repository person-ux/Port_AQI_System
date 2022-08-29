//code for NodeMCU ESP8266
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <DHT_U.h>
#include "MQ135.h"
#include <Arduino.h>
#include "SH1106Wire.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>

#define LENG 31        //starting byte is 0x42 + 31 bytes = 32 bytes
#define DHTPIN 2       //ESP D4 pin
#define DHTTYPE DHT11 

#define lora_ss 15
#define lora_rst -1
#define lora_dio0 16

union{
  int32_t ival;
  float fval;
  unsigned char bytes[4];
}conv;

sensors_event_t DHT_event;        //DHT 11 setup
MQ135 gasSensor = MQ135(A0);      //MQ135 setup
//SoftwareSerial PMS5003_Serial;  //Software Serial for PMS5003
DHT_Unified dht(DHTPIN, DHTTYPE); //make dht object
SH1106Wire display(0x3c,SCL,SDA); //D1 -> SCL & D2 -> SDA

float LOC[2] = {0,0}; //[long,latt] //location of node
float Gas_con;               //MQ135 Vlaues
float Humidity;              //Humidity from DHT11
int PM01Value=0;             //1.0 um particles
int PM10Value=0;             //10 um particles
int PM2_5Value=0;            //2.5 um particles
float Temprature;            //Temprature from DHT11
unsigned char buff[LENG];    //buffer for PMS5003 uart packet
unsigned long next_upload=0; //Time for next upload
unsigned char LoraPacket[25];//Packet for LoRa

void setup()
{
  Serial.begin(9600);// serial port setup
  //PMS5003_Serial.begin(9600,SWSERIAL_8N1,0,15); //EspRx=0:D3, EspTx=15:D8
  
  //DHT 11 Setup
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  //Display setup
  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.clear();

  LoRa.setPins(lora_ss, lora_rst, lora_dio0);
  //LoRa.setSpreadingFactor(12);
  if (!LoRa.begin(865E6)){
    Serial.println("Starting LoRa failed!");
	//while (1);
  }
}
 
void loop()
{
  int32_t iPrevVal = 0 , AlertSend = 0;
  float fPrevVal = 0;
  //////////////////////////////////////////////PMS5003//////////////////////////////////////////////////
  if(Serial.find(0x42)){         //start of the transmission
    Serial.readBytes(buff,LENG); // store 31 bytes of data in buff
  
    if(buff[0] == 0x4d){
      if(verify_checksum(buff,LENG)){
        PM01Value = (buff[3] << 8) | buff[4];  //get PM1.0 value
        PM2_5Value = (buff[5] << 8) | buff[6]; //get PM2.5 value
        PM10Value = (buff[7] << 8) | buff[8];  //get PM10 value
      }
    }
  }
  
  conv.bytes[0] = LoraPacket[4];
  conv.bytes[1] = LoraPacket[5];
  conv.bytes[2] = LoraPacket[6];
  conv.bytes[3] = LoraPacket[7];
  iPrevVal = conv.ival;
  if((iPrevVal < 125) && (PM2_5Value > 125)){AlertSend = 1;}

  conv.bytes[0] = LoraPacket[8];
  conv.bytes[1] = LoraPacket[9];
  conv.bytes[2] = LoraPacket[10];
  conv.bytes[3] = LoraPacket[11];
  iPrevVal = conv.ival;
  if((iPrevVal < 60) && (PM10Value > 60)){AlertSend = 1;}
  
  /*Serial.print("PM1.0: ");  
  Serial.print(PM01Value);
  Serial.println("  ug/m3");            

  Serial.print("PM2.5: ");  
  Serial.print(PM2_5Value);
  Serial.println("  ug/m3");     
  
  Serial.print("PM10 : ");  
  Serial.print(PM10Value);
  Serial.println("  ug/m3");*/
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  
  //////////////////////////////////////////////DHT11////////////////////////////////////////////////////
  // Get temperature event and print its value.
  dht.temperature().getEvent(&DHT_event);
  if (isnan(DHT_event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else { Temprature = DHT_event.temperature; }
  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&DHT_event);
  if (isnan(DHT_event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else { Humidity = DHT_event.relative_humidity; }

  //Serial.print(F("Temperature: "));
  //Serial.print(Temprature);
  //Serial.println(F("°C"));
  
  //Serial.print(F("Humidity: "));
  //Serial.print(Humidity);
  //Serial.println(F("%"));
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  
  //////////////////////////////////////////////MQ135////////////////////////////////////////////////////
  Gas_con = gasSensor.getPPM();

  conv.bytes[0] = LoraPacket[12];
  conv.bytes[1] = LoraPacket[13];
  conv.bytes[2] = LoraPacket[14];
  conv.bytes[3] = LoraPacket[15];
  fPrevVal = conv.fval;
  if((fPrevVal < 4) && (Gas_con > 4)){AlertSend = 1;}
  /*Serial.print("Air Quality: ");  
  Serial.print(Gas_con);
  Serial.println("  PPM"); 
  Serial.println();*/
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  
  /////////////////////////////////////////////Display///////////////////////////////////////////////////
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0,0,("PM1.0: " + String(PM01Value) + "ug/m3"));
  display.drawString(0,10,("PM2.5: " + String(PM2_5Value) + "ug/m3"));
  display.drawString(0,20,("PM10: " + String(PM10Value) + "ug/m3"));
  display.drawString(0,30,("Harmful Gas: " + String(Gas_con) + "ppm"));
  display.drawString(0,40,("Temprature: " + String(Temprature) + "°C"));
  display.drawString(0,50,("Humidity: " + String(Humidity) + "%"));
  display.display();
  ///////////////////////////////////////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////Upload to Database/////////////////////////////////////////////
  unsigned long now_time = millis();
  
//  if((now_time >= next_upload)&&(now_time > 299999)){
//    next_upload = millis() + 300000;
  if((now_time >= next_upload)&&(now_time > 59999)){
    next_upload = millis() + 60000;
    SendData();
  }
  else if(AlertSend){
    SendData();
    AlertSend = 0;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  delay(2000); //Wait for 2 Seconds for next scan and upload
}

char verify_checksum(unsigned char *thebuff, char leng)
{  
  char receiveflag = 0;
  int receiveSum = 0;
 
  for(int i=0; i<(leng-2); i++){
    receiveSum = receiveSum + thebuff[i];
  }
  receiveSum = receiveSum + 0x42;
 
  if(receiveSum == ((thebuff[leng - 2] << 8) + thebuff[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

void SendData(void){
  
  conv.ival = PM01Value;
  LoraPacket[0] = conv.bytes[0];
  LoraPacket[1] = conv.bytes[1];
  LoraPacket[2] = conv.bytes[2];
  LoraPacket[3] = conv.bytes[3];
  
  conv.ival = PM2_5Value;
  LoraPacket[4] = conv.bytes[0];
  LoraPacket[5] = conv.bytes[1];
  LoraPacket[6] = conv.bytes[2];
  LoraPacket[7] = conv.bytes[3];
  
  conv.ival = PM10Value;
  LoraPacket[8] = conv.bytes[0];
  LoraPacket[9] = conv.bytes[1];
  LoraPacket[10] = conv.bytes[2];
  LoraPacket[11] = conv.bytes[3];
  
  conv.fval = Gas_con;
  LoraPacket[12] = conv.bytes[0];
  LoraPacket[13] = conv.bytes[1];
  LoraPacket[14] = conv.bytes[2];
  LoraPacket[15] = conv.bytes[3];
  
  conv.fval = Temprature;
  LoraPacket[16] = conv.bytes[0];
  LoraPacket[17] = conv.bytes[1];
  LoraPacket[18] = conv.bytes[2];
  LoraPacket[19] = conv.bytes[3];
  
  conv.fval = Humidity;
  LoraPacket[20] = conv.bytes[0];
  LoraPacket[21] = conv.bytes[1];
  LoraPacket[22] = conv.bytes[2];
  LoraPacket[23] = conv.bytes[3];

  conv.fval = LOC[0];
  LoraPacket[24] = conv.bytes[0];
  LoraPacket[25] = conv.bytes[1];
  LoraPacket[26] = conv.bytes[2];
  LoraPacket[27] = conv.bytes[3];

  conv.fval = LOC[1];
  LoraPacket[28] = conv.bytes[0];
  LoraPacket[29] = conv.bytes[1];
  LoraPacket[30] = conv.bytes[2];
  LoraPacket[31] = conv.bytes[3];
  
  LoRa.beginPacket(); //start packet
  
  LoRa.write(LoraPacket, 32); //Tx Packet
  
  LoRa.endPacket(); //end packet
  
  Serial.print("Data Posted");
}
