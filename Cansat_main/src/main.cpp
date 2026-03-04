#include <Arduino.h>
#include <Arduino_BHY2Host.h>
#include <list>
#include <ctime>
#include <memory>
#include <algorithm>
#include <Servo.h>
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
Servo break_right;
Servo break_left;
// GPS on custom pins: RX=A0, TX=A1 (BufferedSerial expects TX, RX)
mbed::BufferedSerial gpsDev(digitalPinToPinName(A1), digitalPinToPinName(A0), 9600);


//---------------------------------------------------------------------------------------------------------------------------------------------

//checkpoint system init

struct checkpoints {
  double x, y;
};
const checkpoints arr[100] = 
{
  {.x = 1, .y = 1},{.x = 1, .y = 2}
}; // checkpoint long lat coords
auto Target = std::make_unique<Go_to_checkpoint>(arr[0].x,arr[0].y);

//---------------------------------------------------------------------------------------------------------------------------------------------

double pressure;
double temp;
double gyro_val; 
bool alt_init = false;
int counter = 0;
int checkpoints_counter = 0;
int checkpoint_reached = false;
const double truning_rate = 0.8;
double altitude; // height in meters
double yaw;
double pitch;
double roll;
double yaw_deg;
double pitch_deg;
double roll_deg;
double w_ori;
double x_ori;    
double y_ori;    
double z_ori; 

void setup() {
  Serial.begin(115200);
  while(!Serial);
  while(!BHY2Host.begin());
  barometer.begin();
  temprature.begin();
  ori.begin();
  gyro.begin();
  break_left.attach(4);
  break_right.attach(5);

//---------------------------------------------------------------------------------------------------------------------------------------------

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
//---------------------------------------------------------------------------------------------------------------------------------
  //casat oirentation 
  //------------------------------------------------
  // MAKE SURE TO ASSIGN ORIENTATIONS CORRECTL!!!
  // IF YOU DONT KNOW HOW ASK CHAT!!!
  w_ori = ori.w();
  x_ori = ori.x();
  y_ori = ori.y();
  z_ori = ori.z();
  //-------------------------------------------------

  double norm = sqrt(w_ori*w_ori + x_ori*x_ori + y_ori*y_ori + z_ori*z_ori);
  w_ori /= norm;
  x_ori /= norm;
  y_ori /= norm;
  z_ori /= norm;

  yaw = atan2(2*(w_ori*z_ori + x_ori*y_ori), 1 - 2*(y_ori*y_ori + z_ori*z_ori));
  yaw_deg   = yaw   * 180.0 / PI;

  temp =  temprature.value();
  pressure = barometer.value();
  
  
//---------------------------------------------------------------------------------------------------------------------------------
  //checkpoint extractor

  if (alt_init == false){
    alt = Altitude(temp, 0, pressure);
    alt_init = true;
  }
  altitude = alt.get_alt(temp, pressure);

//---------------------------------------------------------------------------------------------------------------------------------
  //gps reader

  // read from BufferedSerial
  while (gpsDev.readable()) {
    char c;
    if (gpsDev.read(&c, 1) > 0) gps.encode(c);
  }

  double longitude = 0; 

  if (gps.location.isValid() == true){
    
    longitude = gps.location.lng();
  }
  else{
    longitude = -1;
  }

  double latitude = 0; // setter latitude til 0 når programmet starter

  if (gps.location.isValid() == true){
    
    latitude = gps.location.lat();
  }
  else{
    latitude = -1;
  }

//---------------------------------------------------------------------------------------------------------------------------------------------
  // glider controll

  auto target_heading = Target->Calc_desiered_heading(yaw_deg,longitude,latitude);
  auto dist = Target->Calc_dist(longitude,latitude);

  if (dist < 10){
    checkpoints_counter++;
    Target = std::make_unique<Go_to_checkpoint>(arr[checkpoints_counter].x,arr[checkpoints_counter].y);
  }
  else{

    if (target_heading<-100) target_heading = -100;
    if (target_heading>100)  target_heading = 100;

    target_heading = target_heading*truning_rate;

    if (target_heading < 0) break_left.write(0 + target_heading);
    if (target_heading < 0) break_left.write(180 - target_heading);
  }


  
//---------------------------------------------------------------------------------------------------------------------------------------------
  //csv printer
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