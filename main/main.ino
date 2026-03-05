#include <Arduino.h>
#include <Arduino_BHY2Host.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include "Altitude.h"
#include "Go_to_checkpoint.h"

// ---------------- Sensors ----------------
Sensor barometer(SENSOR_ID_BARO);
Sensor temprature(SENSOR_ID_TEMP);
SensorQuaternion ori(SENSOR_ID_ORI);

Altitude alt;

// ---------------- GPS ----------------
UART gpsUART(PA_9, PA_10);   // TX, RX
TinyGPSPlus gps;
#define gpsDev gpsUART

// ---------------- Servos ----------------
Servo break_right;
Servo break_left;

const int SERVO_NEUTRAL = 90;
const int SERVO_MAX_PULL = 40;

// ---------------- Checkpoints ----------------
struct Checkpoint {
  float x;
  float y;
};

const Checkpoint arr[100] = {
  {59.80796,10.44519},
  {59.80634,10.45074},
  {59.80694,10.45174}
};

Go_to_checkpoint Target(arr[0].x, arr[0].y);
int checkpoints_counter = 0;

// ---------------- Variables ----------------
float pressure;
float temp;
bool alt_init = false;
int counter = 0;

const float turning_rate = 0.8;

float altitude;
float yaw_deg;

// ---------------- Setup ----------------
void setup() {

  Serial.begin(115200);
  gpsDev.begin(9600);

  BHY2Host.begin();

  while (!barometer.begin());
  temprature.begin();
  ori.begin();

  break_left.attach(4);
  break_right.attach(5);

  Serial.println("callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude");
  delay(500);
}

// ---------------- Loop ----------------
void loop() {
  BHY2Host.update();

  if (BHY2Host.availableSensorData()) {

    // -------- Orientation --------
    float w = ori.w();
    float x = ori.x();
    float y = ori.y();
    float z = ori.z();

    float num = 2.0f * (w * z + x * y);
    float den = 1.0f - 2.0f * (y * y + z * z);
    yaw_deg = atan2(num, den) * 180.0 / PI;

    // -------- Sensors --------
    temp = temprature.value();
    pressure = barometer.value();

    if (!alt_init) {
      alt = Altitude(temp, 0, pressure);
      alt_init = true;
    }
  }

  altitude = alt.get_alt(temp, pressure);

  double longitude = -1;
  double latitude = -1;

  // -------- GPS --------
  while (gpsDev.available()) {
    gps.encode(gpsDev.read());
  }


  if (gps.location.isValid()) {
    longitude = gps.location.lng();
    latitude = gps.location.lat();
  }

  // -------- Navigation --------
  float target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);
  float dist = Target.Calc_dist(longitude, latitude);

  if (dist < 10 && checkpoints_counter < 99) {
    checkpoints_counter++;
    Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
  }

  // limit turn command
  if (target_heading < -100) target_heading = -100;
  if (target_heading > 100) target_heading = 100;

  target_heading *= turning_rate;

  int leftServo = SERVO_NEUTRAL;
  int rightServo = SERVO_NEUTRAL;

  if (target_heading > 0) {
    rightServo = SERVO_NEUTRAL - map(target_heading, 0, 100, 0, SERVO_MAX_PULL);
  }

  if (target_heading < 0) {
    leftServo = SERVO_NEUTRAL - map(abs(target_heading), 0, 100, 0, SERVO_MAX_PULL);
  }

  break_left.write(leftServo);
  break_right.write(rightServo);

  // -------- Telemetry --------
  Serial.print("Fallschirmjäger;");
  Serial.print(millis()); Serial.print(";");
  Serial.print(counter++); Serial.print(";");
  Serial.print(temp); Serial.print(";");
  Serial.print(pressure); Serial.print(";");
  Serial.print(altitude); Serial.print(";");
  Serial.print(longitude,5); Serial.print(";");
  Serial.print(latitude,5); Serial.print(";");
  Serial.print(target_heading); Serial.print(";");
  Serial.println(yaw_deg);

  delay(1);
}
