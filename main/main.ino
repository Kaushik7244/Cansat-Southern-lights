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
SensorXYZ gyro(SENSOR_ID_GYRO);

Altitude alt;

// ---------------- GPS ----------------
TinyGPSPlus gps;
#define gpsDev Serial2

// ---------------- Servos ----------------
Servo break_right;
Servo break_left;

const int SERVO_NEUTRAL = 90;
const int SERVO_MAX_PULL = 40;

// ---------------- Checkpoints ----------------
struct Checkpoint {
  double x;
  double y;
};

const Checkpoint arr[100] = {
  {1,1},
  {1,2}
};

Go_to_checkpoint Target(arr[0].x, arr[0].y);

int checkpoints_counter = 0;

// ---------------- Variables ----------------
double pressure;
double temp;
bool alt_init = false;
int counter = 0;

const double turning_rate = 0.8;

double altitude;
double yaw;
double yaw_deg;

double w_ori;
double x_ori;
double y_ori;
double z_ori;

// ---------------- Setup ----------------
void setup() {

  Serial.begin(115200);
  gpsDev.begin(9600);

  BHY2Host.begin();

  barometer.begin();
  temprature.begin();
  ori.begin();
  gyro.begin();

  break_left.attach(4);
  break_right.attach(5);

  Serial.println("callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude");
}

// ---------------- Loop ----------------
void loop() {

  unsigned long now = millis();

  // Update IMU
  BHY2Host.update();

  // Keep GPS buffer empty
  while (gpsDev.available()) {
    char c = gpsDev.read();
    gps.encode(c);
  }

  // -------- Orientation --------
  w_ori = ori.w();
  x_ori = ori.x();
  y_ori = ori.y();
  z_ori = ori.z();

  double norm = sqrt(w_ori*w_ori + x_ori*x_ori + y_ori*y_ori + z_ori*z_ori);

  if (norm != 0) {
    w_ori /= norm;
    x_ori /= norm;
    y_ori /= norm;
    z_ori /= norm;
  }

  yaw = atan2(2*(w_ori*z_ori + x_ori*y_ori), 1 - 2*(y_ori*y_ori + z_ori*z_ori));
  yaw_deg = yaw * 180.0 / PI;

  // -------- Sensors --------
  temp = temprature.value();
  pressure = barometer.value();

  if (!alt_init) {
    alt = Altitude(temp, 0, pressure);
    alt_init = true;
  }

  altitude = alt.get_alt(temp, pressure);

  // -------- GPS --------
  double longitude = -1;
  double latitude = -1;

  if (gps.location.isValid()) {
    longitude = gps.location.lng();
    latitude = gps.location.lat();
  }

  // -------- Navigation --------
  double target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);
  double dist = Target.Calc_dist(longitude, latitude);

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
    rightServo = SERVO_NEUTRAL - map(target_heading,0,100,0,SERVO_MAX_PULL);
  }

  if (target_heading < 0) {
    leftServo = SERVO_NEUTRAL - map(abs(target_heading),0,100,0,SERVO_MAX_PULL);
  }

  break_left.write(leftServo);
  break_right.write(rightServo);

  // -------- Telemetry --------
  Serial.print("Fallschirmjäger;");
  Serial.print(now); Serial.print(";");
  Serial.print(counter++); Serial.print(";");
  Serial.print(temp); Serial.print(";");
  Serial.print(pressure); Serial.print(";");
  Serial.print(altitude); Serial.print(";");
  Serial.print(longitude); Serial.print(";");
  Serial.println(latitude);
}