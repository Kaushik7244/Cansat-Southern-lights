#pragma once

class Go_to_checkpoint {
public:
    Go_to_checkpoint(double longitude, double latitude);
    Go_to_checkpoint() = default;
    double Calc_desiered_heading(double current_heading, double current_long, double current_lat);
    double Calc_dist(double current_long, double current_lat);

private:
    bool   location_reached;
    double desierd_long;
    double desierd_lat;
};
