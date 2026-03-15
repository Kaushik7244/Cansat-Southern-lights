#include "Go_to_checkpoint.h"
#include <Arduino.h>
#include <math.h>

Go_to_checkpoint::Go_to_checkpoint(double longitude, double latitude)
{
    desierd_long = longitude;
    desierd_lat  = latitude;
}

double Go_to_checkpoint::Calc_desiered_heading(double current_heading, double current_long, double current_lat)
{
    double rel_x = desierd_long - current_long;   // east-west
    double rel_y = desierd_lat  - current_lat;    // north-south

    // Compass-style heading (0° = North)
    double angle_rad = atan2(rel_x, rel_y);
    double desired_heading = angle_rad * 180.0 / PI;

    // Convert to 0–360 range
    if (desired_heading < 0)
        desired_heading += 360.0;

    // Signed turn error — returns how much and which direction the CanSat must turn
    double turn = desired_heading - current_heading;

    // Wrap into [-180, 180]
    while (turn >  180.0) turn -= 360.0;
    while (turn < -180.0) turn += 360.0;

    return turn;
}

double Go_to_checkpoint::Calc_dist(double current_long, double current_lat)
{
    double long_dif = abs(current_long - desierd_long);
    double lat_dif  = abs(current_lat  - desierd_lat);

    double lat_m  = lat_dif  * 111111.0;
    double long_m = long_dif * 111111.0 * cos(current_lat * PI / 180.0);

    return hypot(long_m, lat_m);   // distance to checkpoint in metres
}
