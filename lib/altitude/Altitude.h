#pragma once

const float temprature_gradient_KelvinperM = -0.0065;
const double Gravitational_acceleration_M_per_s2 = 9.81;
const double Specific_gas_constant = 287.06;

class Altitude{
    public:
    Altitude(float drop_start_temp, float drop_start_hight, float drop_start_presure);
    float get_alt(float temprature, float pressure);


    private:
    float d_s_t;
    float d_s_h;
    float d_s_p;

};


