#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Arduino.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>

#define lora_ss 15
#define lora_rst 16
#define lora_dio0 4

time_t now;
tm timeinfo;
StaticJsonDocument<384> doc;      //make json object
float LOC[2] = {0,0}; //[long,latt]

JsonObject loc  = doc.createNestedObject("Location");
JsonArray coor  = loc.createNestedArray("coordinates");

float Humidity;              //Humidity from DHT11
int32_t PM01Value=0;             //1.0 um particles
int32_t PM10Value=0;             //10 um particles
int32_t PM2_5Value=0;            //2.5 um particles
float Temprature;            //Temprature from DHT11
float Gas_con;               //MQ135 Vlaues
unsigned long next_upload=0; //Time for next upload
uint8_t LoRaData[32] = {0};
char Post = 0;

void setup()
{
  Serial.begin(9600);// serial port setup

  //LoRa init
  LoRa.setPins(lora_ss, lora_rst, lora_dio0);
  //LoRa.setSpreadingFactor(12);
  if (!LoRa.begin(865E6)) { //using 865Mhz
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  /////////////////////////////////////////Connect to WIFI/////////////////////////////////////////////////
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager WifiManage;
  WifiManage.autoConnect("AutoConnectAP","password");
   
  //Clock setup
  configTime(0,0,"pool.ntp.org","time.nist.gov");
  setenv("TZ","ST-5:30",1);

  getClock(10); //set system clock through NTP
  
  loc["type"] = "Point";
  //coor.add(LOC1[0]); coor.add(LOC1[1]);
  coor.add(LOC[0]); coor.add(LOC[1]);
  
  doc["PM1"] = 0;
  doc["PM2"] = 0;
  doc["PM10"] = 0;
  doc["Gas"] = 0;
  doc["Temperature"] = 0;
  doc["Humidity"] = 0;
  doc["Year"] = 0;
  doc["Month"] = 0;
  doc["Date"] = 0;
  doc["Hours"] = 0;
  doc["Minutes"] = 0;
}
 
void loop()
{

  //////////////////////////////////////////////Reading LoRa//////////////////////////////////////////////
  int packetSize = LoRa.parsePacket(), N = 0; //check received packet size
  
  //check if paket size is greater than 0
  if (packetSize){
    for(int i = 0; i < packetSize; i++){
      while(!LoRa.available());
      //LoRaData = LoRa.readString();
      LoRaData[i] = LoRa.read();
    }
    Serial.println("LoRa Packet Received");
    
    //Parse data
    union{
      int32_t ival;
      float fval;
      unsigned char bytes[4];
    }conv;
    
    conv.bytes[0] = LoRaData[0];
    conv.bytes[1] = LoRaData[1];
    conv.bytes[2] = LoRaData[2];
    conv.bytes[3] = LoRaData[3];
    PM01Value = conv.ival;
    
    conv.bytes[0] = LoRaData[4];
    conv.bytes[1] = LoRaData[5];
    conv.bytes[2] = LoRaData[6];
    conv.bytes[3] = LoRaData[7];
    PM2_5Value = conv.ival;
    
    conv.bytes[0] = LoRaData[8];
    conv.bytes[1] = LoRaData[9];
    conv.bytes[2] = LoRaData[10];
    conv.bytes[3] = LoRaData[11];
    PM10Value = conv.ival;
    
    conv.bytes[0] = LoRaData[12];
    conv.bytes[1] = LoRaData[13];
    conv.bytes[2] = LoRaData[14];
    conv.bytes[3] = LoRaData[15];
    Gas_con = conv.fval;
    
    conv.bytes[0] = LoRaData[16];
    conv.bytes[1] = LoRaData[17];
    conv.bytes[2] = LoRaData[18];
    conv.bytes[3] = LoRaData[19];
    Temprature = conv.fval;
    
    conv.bytes[0] = LoRaData[20];
    conv.bytes[1] = LoRaData[21];
    conv.bytes[2] = LoRaData[22];
    conv.bytes[3] = LoRaData[23];
    Humidity = conv.fval;
    
	conv.bytes[0] = LoRaData[24];
    conv.bytes[1] = LoRaData[25];
    conv.bytes[2] = LoRaData[26];
    conv.bytes[3] = LoRaData[27];
    LOC[0] = conv.fval;
  
    conv.bytes[0] = LoRaData[28];
    conv.bytes[1] = LoRaData[29];
    conv.bytes[2] = LoRaData[30];
    conv.bytes[3] = LoRaData[31];
    LOC[1] = conv.fval;
    
    Post = 1;
    
    Serial.print("PM1.0: ");  
    Serial.print(PM01Value);
    Serial.println("  ug/m3");            
    
    Serial.print("PM2.5: ");  
    Serial.print(PM2_5Value);
    Serial.println("  ug/m3");     
    
    Serial.print("PM10 : ");  
    Serial.print(PM10Value);
    Serial.println("  ug/m3");   
    
    Serial.print(F("Temperature: "));
    Serial.print(Temprature);
    Serial.println(F("°C"));
    
    Serial.print(F("Humidity: "));
    Serial.print(Humidity);
    Serial.println(F("%"));
    
    Serial.print("Air Quality: ");  
    Serial.print(Gas_con);
    Serial.println("  PPM"); 
    Serial.println();
   }
  
  ////////////////////////////////////////Upload to Database/////////////////////////////////////////////
  if(Post){
    getClock(10); //set system clock through NTP
	
    coor[0] = LOC[0];
	coor[1] = LOC[1];
    doc["PM1"] = PM01Value;
    doc["PM2"] = PM2_5Value;
    doc["PM10"] = PM10Value;
    doc["Gas"] = Gas_con;
    doc["Temperature"] = Temprature;
    doc["Humidity"] = Humidity;
    doc["Year"] = 1900 + timeinfo.tm_year;
    doc["Month"] = timeinfo.tm_mon + 1;
    doc["Date"] = timeinfo.tm_mday;
    doc["Hours"] = timeinfo.tm_hour;
    doc["Minutes"] = timeinfo.tm_min;
    
    POSTData();
    Serial.println("Data Posted");
    Serial.println();
    Post = 0;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
}

bool getClock(unsigned char sec){
  uint32_t start = millis();  
  do {
    delay(10);
    time(&now);
    localtime_r(&now,&timeinfo);
  } while (((millis() - start) <= (1000 * sec)) && (timeinfo.tm_year < (2016 - 1900)));
  if (timeinfo.tm_year <= (2016 - 1900)) return false;  // the NTP call was not successful
  return true;
}

void POSTData(){
  BearSSL::WiFiClientSecure client; // creat bearssl instance
  BearSSL::X509List ApiCert(api_root_cert); //initialize cirtificate
  client.setTrustAnchors(&ApiCert); //give cert for verification
  
  HTTPClient https; //creat https client for GET and POST methods
  
  if (https.begin(client, ApiHost)){  // connect to host with secure HTTPS connection
    https.addHeader("Content-Type", "application/json");// het http header
    String json;
    serializeJson(doc, json);//store the JSON object as tring in json
    https.POST(json);//POST the http packet
    Serial.print(json);
  }
  https.end();// close HTTPS connection
}
