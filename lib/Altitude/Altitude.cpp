#include "Altitude.h"
#include <math.h>


Altitude::Altitude(float drop_start_temp, float drop_start_hight, float drop_start_presure){
    this -> d_s_t = drop_start_temp+273.15;
    this -> d_s_h = drop_start_hight;
    this -> d_s_p = drop_start_presure;
};

float Altitude::get_alt(float temprature, float pressure){
    int hight = (d_s_t/temprature_gradient_KelvinperM)
    *(pow((pressure/d_s_p),(-(temprature_gradient_KelvinperM*Specific_gas_constant)/Gravitational_acceleration_M_per_s2)) -1)
    + d_s_h;
    return hight;
}





