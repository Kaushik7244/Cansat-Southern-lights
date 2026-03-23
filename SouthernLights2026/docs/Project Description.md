### CanSat project SouthernLights 2026.
## Deadline March 22nd 2026
The competition week is 23rd to 27th of March. Monday and friday are travel days. 

Report submitted and approved.

## Team members
# Teo
Programming, Cad, 3d printing
# Kaushik 
Programming, paraglider, gps
# Alexander
Cansat encapsulation, cad, 3d print
# Aurora
Paraglider
# Emily
Paraglider
# Sarah (not participating at the event)

## Goals set by the team:
Continous registration of temperature and pressure from independent sensors
Collect and interpret image data
Monitor parachute / paraglider deployment
Control flight path
Store sensory data locally
Transmit data to base station

### Primary mission - Data collection
## Data collection
Collect Temperature and pressure data from separate sources / sensors. Store locally and transmit to ground station. See Telemetry.

### Secondary mission - 
## Capture image data
We aim to collect image data from two cameras. One pointing down and one pointing to the side of the cansat
Image capture must not interfere with primary mission and needs a system that operates independently from the rest of the software.

### Extra (ambitious) goals:
## navigate the CanSat back to base.
By using a paraglider and controlling that with servos.
# Equipment / sources:
GPS (Grove Air530)
Altitude calculated from pressure
Image 
BHI260AP for movement

### Planned equipment

## Microcontroller
Arduino Portenta H7

# H7 pinout
Port	TX	 RX	    Used for
Serial1	PA9	 PA10	APC220
Serial2	PA15 PF6	GPS Air530 (tomorrow)
Serial3	PJ8	 PJ9	SerialLoRa → Vision Shield

## Sensors
Nicla Sense
BHI260AP motion
BMM150 magnetometer.
BMP390 atmospheric pressure
BME688 gas-sensor
GY-91
BMP-280 pressure
NTCLE203E3103JBO NTC - temperature
Modulino thermo temperature

## Telemetry
APC220 radio
LoRa (Vision Shield LoRa)

## Servos
To be determined

## Power delivery
2 x 18650 Cells + voltage regulator to provide 5V
1 x LiPo 3.7V cell directly connected to H7

## Encapsulation
3d printed cansat box

### Software design

## Portenta M4 core to act as a "realtime" core with a very limited set of responsibilities.
The M4 core should collect sensor data, store or transmit but forward the findings to the M7 Core for processing. 
The design of the code for the M4 chip has "reliability" as highest goal.

## Portenta M7 core to do all non essential tasks and receive data regularly over RPC from the M4 core
The M7 core will handle anything related to images, parachute / paraglider operation, GPS and other nice-to-have operations.

## Considerations:
Our microcontrollers do not support true RealTime OS operations so we must apply steps to avoid situations where single functions can block the main thread and prevent data gathering.
A max runtime for each procedure and strict checking of loops as part of the code.
Avoid repetition of expensive «GetData» functions. Store result in local variable and reuse.
Attempt to get a real time clock and tag data with a timestamp.
