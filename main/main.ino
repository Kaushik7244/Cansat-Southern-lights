#include <Arduino.h>
#include <Arduino_BHY2Host.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include "Altitude.h"
#include "Go_to_checkpoint.h"
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"

#define USE_CAMERA 0

#if USE_CAMERA
#include "camera.h"
#include "hm0360.h"
HM0360 himax;
Camera cam(himax);
FrameBuffer frameBuffer;

#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 320
#define IMAGE_MODE CAMERA_GRAYSCALE
#define BITS_PER_PIXEL 8
#define PALETTE_COLORS_AMOUNT (1 << BITS_PER_PIXEL)
#define PALETTE_SIZE (PALETTE_COLORS_AMOUNT * 4)
#define IMAGE_PATH "/fileSystem/image.bmp"
#define BITMAP_FILE_HEADER_SIZE 14
#define DIB_HEADER_SIZE 40
#define HEADER_SIZE (BITMAP_FILE_HEADER_SIZE + DIB_HEADER_SIZE)
#endif

Sensor barometer(SENSOR_ID_BARO);
Sensor temprature(SENSOR_ID_TEMP);
SensorQuaternion ori(SENSOR_ID_ORI);

Altitude alt;
SDMMCBlockDevice blockDevice;
mbed::FATFileSystem fileSystem("fileSystem");

UART gpsUART(PA_9, PA_10);
TinyGPSPlus gps;
#define gpsDev gpsUART

Servo break_right;
Servo break_left;

const int SERVO_NEUTRAL = 90;
const int SERVO_MAX_PULL = 40;

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

float pressure = 0;
float temp = 0;
bool alt_init = false;
bool flight_end = false;
int counter = 0;
bool crossed_500 = false;
bool paratschute_deployed = false;

const float turning_rate = 0.8;

float altitude;
float yaw_deg;

FILE *telemetryFile = NULL;

void setup() {
  Serial.begin(115200);
  gpsDev.begin(9600);
  BHY2Host.begin();
  mountSDCard();
  pinMode(LEDG, OUTPUT);

  barometer.begin();
  ori.begin();
  temprature.begin();

#if USE_CAMERA
  cam.begin(CAMERA_R320x240, IMAGE_MODE, 30);
#endif

  break_left.attach(4);
  break_right.attach(5);
  break_left.write(180);
  break_right.write(0);

  telemetryFile = fopen("/fileSystem/telemetry.txt", "a");
  Serial.println("callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude");
  if (telemetryFile) {
    fprintf(telemetryFile, "callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude\r\n");
    fflush(telemetryFile);
  }
}

void loop() {
  digitalWrite(LEDG, LOW);
  BHY2Host.update();

  if (BHY2Host.availableSensorData()) {
    float w = ori.w();
    float x = ori.x();
    float y = ori.y();
    float z = ori.z();

    float num = 2.0f * (w * z + x * y);
    float den = 1.0f - 2.0f * (y * y + z * z);
    yaw_deg = atan2(num, den) * 180.0 / PI;

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

  if (altitude > 480) crossed_500 = true;
  if (crossed_500 && altitude < 60 && !paratschute_deployed) { 
    paratschute_deployed = true;
    break_left.write(SERVO_NEUTRAL);
    break_right.write(SERVO_NEUTRAL);
  }

#if USE_CAMERA
  static unsigned long lastCapture = 0;
  if (!(flight_end && altitude < 6) && millis() - lastCapture >= 100) {
    lastCapture = millis();
    if (cam.grabFrame(frameBuffer, 3000) == 0) {
      saveImage(frameBuffer.getBuffer(), IMAGE_PATH);
    }
  }
#endif

  while (gpsDev.available()) gps.encode(gpsDev.read());
  if (gps.location.isValid()) {
    longitude = gps.location.lng();
    latitude = gps.location.lat();
  }

  float target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);
  float dist = Target.Calc_dist(longitude, latitude);

  if (paratschute_deployed) {
    if (!flight_end && checkpoints_counter < 99 &&
        arr[checkpoints_counter].x != 0 && arr[checkpoints_counter].y != 0) {

      if (dist <= 5) checkpoints_counter++;

      Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
    }
    else if (!flight_end) {
      checkpoints_counter -= 1;
      Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
      flight_end = true;
    }

    if (target_heading < -100) target_heading = -100;
    if (target_heading > 100) target_heading = 100;

    target_heading *= turning_rate;

    int leftServo = SERVO_NEUTRAL;
    int rightServo = SERVO_NEUTRAL;

    if (target_heading > 0)
      rightServo = SERVO_NEUTRAL - map(target_heading, 0, 100, 0, SERVO_MAX_PULL);

    if (target_heading < 0)
      leftServo = SERVO_NEUTRAL - map(abs(target_heading), 0, 100, 0, SERVO_MAX_PULL);

    break_left.write(leftServo);
    break_right.write(rightServo);
  }

  String msg = "Fallschirmjäger;";
  msg += millis(); msg += ";";
  msg += counter++; msg += ";";
  msg += temp; msg += ";";
  msg += pressure; msg += ";";
  msg += altitude; msg += ";";
  msg += longitude; msg += ";";
  msg += latitude; msg += ";";
  msg += target_heading; msg += ";";
  msg += yaw_deg;

  Serial.println(msg);

  if (telemetryFile) {
    fprintf(telemetryFile, "%s\r\n", msg.c_str());
    if (counter % 20 == 0) fflush(telemetryFile);
  }

  if (flight_end && altitude < 6) {
    if (telemetryFile) {
      fclose(telemetryFile);
      telemetryFile = NULL;
    }
    fileSystem.unmount();
  }

  digitalWrite(LEDG, HIGH);
  delay(1);
}

void mountSDCard() {
  int error = fileSystem.mount(&blockDevice);
  if (error) {
    if (fileSystem.reformat(&blockDevice)) {
      Serial.println("No SD Card found");
    }
  }
}

#if USE_CAMERA
void setFileHeaders(unsigned char *bitmapFileHeader, unsigned char *bitmapDIBHeader, int fileSize) {
  memset(bitmapFileHeader, 0, BITMAP_FILE_HEADER_SIZE);
  memset(bitmapDIBHeader, 0, DIB_HEADER_SIZE);

  bitmapFileHeader[0] = 'B';
  bitmapFileHeader[1] = 'M';
  bitmapFileHeader[2] = (unsigned char)(fileSize);
  bitmapFileHeader[3] = (unsigned char)(fileSize >> 8);
  bitmapFileHeader[4] = (unsigned char)(fileSize >> 16);
  bitmapFileHeader[5] = (unsigned char)(fileSize >> 24);
  bitmapFileHeader[10] = (unsigned char)(HEADER_SIZE + PALETTE_SIZE);

  bitmapDIBHeader[0] = (unsigned char)(DIB_HEADER_SIZE);
  bitmapDIBHeader[4] = (unsigned char)(IMAGE_WIDTH);
  bitmapDIBHeader[5] = (unsigned char)(IMAGE_WIDTH >> 8);
  bitmapDIBHeader[8] = (unsigned char)(IMAGE_HEIGHT);
  bitmapDIBHeader[9] = (unsigned char)(IMAGE_HEIGHT >> 8);
  bitmapDIBHeader[14] = (unsigned char)(BITS_PER_PIXEL);
}

void setColorMap(unsigned char *colorMap) {
  memset(colorMap, 0, PALETTE_SIZE);
  for (int i = 0; i < PALETTE_COLORS_AMOUNT; i++) {
    colorMap[i * 4] = i;
    colorMap[i * 4 + 1] = i;
    colorMap[i * 4 + 2] = i;
  }
}

void saveImage(unsigned char *imageData, const char* imagePath) {
  int fileSize = BITMAP_FILE_HEADER_SIZE + DIB_HEADER_SIZE + IMAGE_WIDTH * IMAGE_HEIGHT;
  FILE *file = fopen(imagePath, "w");

  unsigned char bitmapFileHeader[BITMAP_FILE_HEADER_SIZE];
  unsigned char bitmapDIBHeader[DIB_HEADER_SIZE];
  unsigned char colorMap[PALETTE_SIZE];

  setFileHeaders(bitmapFileHeader, bitmapDIBHeader, fileSize);
  setColorMap(colorMap);

  fwrite(bitmapFileHeader, 1, BITMAP_FILE_HEADER_SIZE, file);
  fwrite(bitmapDIBHeader, 1, DIB_HEADER_SIZE, file);
  fwrite(colorMap, 1, PALETTE_SIZE, file);
  fwrite(imageData, 1, IMAGE_HEIGHT * IMAGE_WIDTH, file);

  fclose(file);
}
#endif
