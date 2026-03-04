
#include <Go_to_checkpoint.h>
#include <cmath>
#include <algorithm>

Go_to_checkpoint::Go_to_checkpoint(double longitude, double latitude){
    this -> desierd_long = longitude;
    this -> desierd_lat = latitude;
};

double Go_to_checkpoint::Calc_desiered_heading(double current_heading, double current_long, double current_lat)
{
    double rel_x = desierd_long - current_long;   // east-west
    double rel_y = desierd_lat  - current_lat;    // north-south

    // Compass-style heading (0° = North)
    double angle_rad = std::atan2(rel_x, rel_y);
    double desired_heading = angle_rad * 180.0 / M_PI;

    // Convert to 0–360 range
    if (desired_heading < 0)
        desired_heading += 360.0;

    // Signed turn error
    double turn = desired_heading - current_heading;

    // Wrap into [-180, 180]
    while (turn > 180.0)  turn -= 360.0;
    while (turn < -180.0) turn += 360.0;

    return turn;
    //gir ut hvor mye og hvilket vei cansaten må snu
}

double Go_to_checkpoint::Calc_dist(double current_long, double current_lat)
{
    double long_dif = std::abs(current_long-desierd_long);
    double lat_dif = std::abs(current_lat-desierd_lat);

    double lat_m = lat_dif*111111; //overfører lat til meter
    double long_m = long_dif*111111*std::cos(lat_m);

    return std::hypot(long_m,lat_m);

    //gir ut distanse fra checkpoint i meter

}

    

  