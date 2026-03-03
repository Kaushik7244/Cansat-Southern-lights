#include <Arduino.h>
#include <Arduino_BHY2Host.h>
#include <list>
#include <ctime>
#include <TinyGPS++.h>
#include "Altitude.h"
#include "Go_to_checkpoint.h"
#include "mbed.h"


Sensor barometer(SENSOR_ID_BARO);
Sensor temprature(SENSOR_ID_TEMP);
SensorQuaternion ori(SENSOR_ID_ORI);
SensorXYZ gyro(SENSOR_ID_GYRO);
Altitude alt;
TinyGPSPlus gps;
// GPS on custom pins: RX=A0, TX=A1 (BufferedSerial expects TX, RX)
mbed::BufferedSerial gpsDev(digitalPinToPinName(A1), digitalPinToPinName(A0), 9600);

struct checkpoint {
  double x, y;
};

checkpoint arr[100] = 
{
  {.x = 1, .y = 1},{.x = 1, .y = 2}
}; // checkpoint long lat coords


double pressure;
double temp;
double gyro_val; 
bool alt_init = false;
int counter = 0;

double altitude; // height in meters

double yaw;
double yaw_deg;
double w_ori;
double x_ori;     // swap
double y_ori;    // swap + invert
double z_ori; 




void setup() {
  Serial.begin(115200);
  while(!Serial);
  while(!BHY2Host.begin());
  barometer.begin();
  temprature.begin();
  ori.begin();
  gyro.begin();

  //Her lager vi en overskrift over hvilke data som skrives i de ulike kolonnene.
  
  Serial.print("callsign"); //navn på CanSat'en
  Serial.print("; "); //delimiter - tegn som separerer individuelle data
  
  Serial.print("time (ms)"); //tid siden oppstart i millisekund
  Serial.print("; ");
  
  Serial.print("counter"); //teller som viser antall ganger løkken har blitt kjørt
  Serial.print("; ");
  
  Serial.print("ntc"); //temperatursensor
  Serial.print("; ");
  
  Serial.print("pressure"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Alt(m)"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Longatude"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Latitude"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Paretschute status"); //lufttrykk 
  Serial.println("; "); //NB - legg merke til linjeskift

   

    

};

void loop() {
  std::time_t now = std::time(nullptr);
  // read quaternion from game rotation vector
  // read quaternion values from game rotation vector
  w_ori = ori.w();
  x_ori = ori.x();
  y_ori = ori.y();
  z_ori = ori.z();

  yaw = atan2(2*(w_ori*z_ori + x_ori*y_ori), 1 - 2*(y_ori*y_ori + z_ori*z_ori));
  yaw_deg = yaw * 180.0 / PI;

  temp =  temprature.value();
  pressure = barometer.value();
  

  
  //checkpoint extractor

  if (alt_init == false){
    alt = Altitude(temp, 0, pressure);
    alt_init = true;
  }
  altitude = alt.get_alt(temp, pressure);

  // read from BufferedSerial
  while (gpsDev.readable()) {
    char c;
    if (gpsDev.read(&c, 1) > 0) gps.encode(c);
  }

  double longitude = 0; 
  if (gps.location.isValid() == true){
    
    longitude = gps.location.lng();
  }
  else
    longitude = -1;

  double latitude = 0; // setter latitude til 0 når programmet starter
  if (gps.location.isValid() == true){
    
    latitude = gps.location.lat();
  }
  else
    latitude = -1;

  Serial.print("Fallschirmjäger"); //navn skrives som tekst
  Serial.print("; "); //delimiter skrives som tekst
  
  Serial.print(std::ctime(&now)); //funksjon som viser tid siden oppstart 
  Serial.print("; "); 
  
  Serial.print(counter++); //legg merke til ++, øker verdien på variablen med 1 for hver runde løkken har blitt kjørt
  Serial.print("; ");
  
  Serial.print(temp); //verdien fra analog temperatursensor
  Serial.print("; ");

  Serial.print(pressure); //lufttrykk i pascal
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(altitude); //høyde
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(longitude); //longitude kordinaten
  Serial.print("; "); 
  Serial.print(latitude); //latitude kordinaten
  Serial.print("; "); //NB - legg merke til linjeskift

  

}

