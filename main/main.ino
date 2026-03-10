#include <Arduino.h>
#include <Arduino_BHY2Host.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include "Altitude.h"
#include "Go_to_checkpoint.h"
#include "SDMMCBlockDevice.h" // Multi Media Card APIs
#include "FATFileSystem.h"    // API to run operations on a FAT file system

#include "camera.h" // Arduino Mbed Core Camera APIs
#include "hm0360.h" // API to read from the Himax camera found on the Portenta Vision Shield Rev.2




// ---------------- Sensors ----------------
Sensor barometer(SENSOR_ID_BARO);
Sensor temprature(SENSOR_ID_TEMP);
SensorQuaternion ori(SENSOR_ID_ORI);
HM0360 himax;
Camera cam(himax);

Altitude alt;
FrameBuffer frameBuffer;
SDMMCBlockDevice blockDevice;
mbed::FATFileSystem fileSystem("fs");


// ---------------- GPS ----------------
UART gpsUART(PA_9, PA_10);   // TX, RX
TinyGPSPlus gps;
#define gpsDev gpsUART

// ----Settings for our camera setup---
#define IMAGE_HEIGHT (unsigned int)240
#define IMAGE_WIDTH (unsigned int)320
#define IMAGE_MODE CAMERA_GRAYSCALE
#define BITS_PER_PIXEL (unsigned int)8
#define PALETTE_COLORS_AMOUNT (unsigned int)(pow(2, BITS_PER_PIXEL))
#define PALETTE_SIZE  (unsigned int)(PALETTE_COLORS_AMOUNT * 4) // 4 bytes = 32bit per color (3 bytes RGB and 1 byte 0x00)
#define IMAGE_PATH "/fs/image.bmp"

// Headers info
#define BITMAP_FILE_HEADER_SIZE (unsigned int)14 // For storing general information about the bitmap image file
#define DIB_HEADER_SIZE (unsigned int)40 // For storing information about the image and define the pixel format
#define HEADER_SIZE (BITMAP_FILE_HEADER_SIZE + DIB_HEADER_SIZE)

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
}; // map shit. put in checkpoints in this array

Go_to_checkpoint Target(arr[0].x, arr[0].y);
int checkpoints_counter = 0;

// ---------------- Variables ----------------
float pressure = 0;
float temp = 0;
bool alt_init = false;
bool flight_end = false;
int counter = 0;


const float turning_rate = 0.8;

float altitude;
float yaw_deg;

// ---------------- Setup ----------------
void setup() {

  Serial.begin(115200);
  gpsDev.begin(9600);
  BHY2Host.begin();
  mountSDCard();

  while (!barometer.begin());
  ori.begin();
  temprature.begin();

  if (!cam.begin(CAMERA_R320x240, IMAGE_MODE, 30)){
    Serial.println("Unable to find the camera");
  }
  

  break_left.attach(4);
  break_right.attach(5);
  FILE *file = fopen("/fileSystem/telemetry.txt", "a");
  Serial.println("callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude");
  fprintf(file,"callsign;time(ms);counter;ntc;pressure;Alt(m);Longitude;Latitude"); // write data to file
  fprintf(file,"/r/n");
  fclose(file);
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

  //-------- camera ---------

  if (millis() % 10 == 0){
    unsigned char *imageData = captureImage();
    saveImage(imageData, IMAGE_PATH);
  }

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

  while (flight_end == false){
    if (dist > 5 && checkpoints_counter < 99 && arr[checkpoints_counter].x != 0 &&  arr[checkpoints_counter].y != 0) {
      checkpoints_counter++;
      Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
    }
    else{
      fileSystem.unmount();
      checkpoints_counter -= 1;
      Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
      flight_end = true;
    }
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

  String msg = "Fallschirmjäger;";
  msg += millis();
  msg += ";";
  msg += counter++;
  msg += ";";
  msg += temp;
  msg += ";";
  msg += pressure;
  msg += ";";
  msg += altitude;
  msg += ";";
  msg += longitude;
  msg += ";";
  msg += latitude;
  msg += ";";
  msg += target_heading;
  msg += ";";
  msg += yaw_deg;

  int str_length = msg.length();
  char telemtry_packet[str_length + 1];
  strcpy(telemtry_packet, msg.c_str());

  Serial.println(msg);
  FILE *file = fopen("/fileSystem/telemetry.txt", "a");
  fprintf(file,telemtry_packet); // write data to file
  fprintf(file,"/r/n");
  fclose(file);

  delay(1);
}

// Mount File system block
void mountSDCard(){
    int error = fileSystem.mount(&blockDevice);
    if (error){
        Serial.println("Trying to reformat...");
        int formattingError = fileSystem.reformat(&blockDevice);
        if (formattingError) {            
            Serial.println("No SD Card found");
            while (1);
        }
    }
}

// Get the raw image data (8bpp grayscale)
unsigned char * captureImage(){
    if (cam.grabFrame(frameBuffer, 3000) == 0){
        return frameBuffer.getBuffer();
    } else {
        Serial.println("could not grab the frame");
        while (1);
    }
}

// Set the headers data
void setFileHeaders(unsigned char *bitmapFileHeader, unsigned char *bitmapDIBHeader, int fileSize){
    // Set the headers to 0
    memset(bitmapFileHeader, (unsigned char)(0), BITMAP_FILE_HEADER_SIZE);
    memset(bitmapDIBHeader, (unsigned char)(0), DIB_HEADER_SIZE);

    // File header
    bitmapFileHeader[0] = 'B';
    bitmapFileHeader[1] = 'M';
    bitmapFileHeader[2] = (unsigned char)(fileSize);
    bitmapFileHeader[3] = (unsigned char)(fileSize >> 8);
    bitmapFileHeader[4] = (unsigned char)(fileSize >> 16);
    bitmapFileHeader[5] = (unsigned char)(fileSize >> 24);
    bitmapFileHeader[10] = (unsigned char)HEADER_SIZE + PALETTE_SIZE;

    // Info header
    bitmapDIBHeader[0] = (unsigned char)(DIB_HEADER_SIZE);
    bitmapDIBHeader[4] = (unsigned char)(IMAGE_WIDTH);
    bitmapDIBHeader[5] = (unsigned char)(IMAGE_WIDTH >> 8);
    bitmapDIBHeader[8] = (unsigned char)(IMAGE_HEIGHT);
    bitmapDIBHeader[9] = (unsigned char)(IMAGE_HEIGHT >> 8);
    bitmapDIBHeader[14] = (unsigned char)(BITS_PER_PIXEL);
}

void setColorMap(unsigned char *colorMap){
    //Init the palette with zeroes
    memset(colorMap, (unsigned char)(0), PALETTE_SIZE);
    
    // Gray scale color palette, 4 bytes per color (R, G, B, 0x00)
    for (int i = 0; i < PALETTE_COLORS_AMOUNT; i++) {
        colorMap[i * 4] = i;
        colorMap[i * 4 + 1] = i;
        colorMap[i * 4 + 2] = i;
    }
}

// Save the headers and the image data into the .bmp file
void saveImage(unsigned char *imageData, const char* imagePath){
    int fileSize = BITMAP_FILE_HEADER_SIZE + DIB_HEADER_SIZE + IMAGE_WIDTH * IMAGE_HEIGHT;
    FILE *file = fopen(imagePath, "w");

    // Bitmap structure (Head + DIB Head + ColorMap + binary image)
    unsigned char bitmapFileHeader[BITMAP_FILE_HEADER_SIZE];
    unsigned char bitmapDIBHeader[DIB_HEADER_SIZE];
    unsigned char colorMap[PALETTE_SIZE]; // Needed for <= 8bpp grayscale bitmaps    

    setFileHeaders(bitmapFileHeader, bitmapDIBHeader, fileSize);
    setColorMap(colorMap);

    // Write the bitmap file
    fwrite(bitmapFileHeader, 1, BITMAP_FILE_HEADER_SIZE, file);
    fwrite(bitmapDIBHeader, 1, DIB_HEADER_SIZE, file);
    fwrite(colorMap, 1, PALETTE_SIZE, file);
    fwrite(imageData, 1, IMAGE_HEIGHT * IMAGE_WIDTH, file);

    // Close the file stream
    fclose(file);
}

void countDownBlink(){
    for (int i = 0; i < 6; i++){
        digitalWrite(LEDG, i % 2);
        delay(500);
    }
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
}
