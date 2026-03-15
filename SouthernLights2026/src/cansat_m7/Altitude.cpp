#include "Altitude.h"
#include <cmath>

Altitude::Altitude(float drop_start_temp_C, float drop_start_height_m, float drop_start_pressure_hPa) {
    // Convert Celsius to Kelvin for internal calculations
    this->d_s_t = drop_start_temp_C + 273.15f;
    this->d_s_h = drop_start_height_m;
    // Convert hPa to Pa
    this->d_s_p = drop_start_pressure_hPa * 100.0f;
}

float Altitude::get_alt(float temperature_C, float pressure_hPa) {
    // Convert temperature to Kelvin
    float temp_K = temperature_C + 273.15f;
    // Convert pressure to Pa
    float pressure_Pa = pressure_hPa * 100.0f;

    float h = (d_s_t / temprature_gradient_KelvinperM) *
              (pow((pressure_Pa / d_s_p),
                  (-(temprature_gradient_KelvinperM * Specific_gas_constant) / Gravitational_acceleration_M_per_s2))
              - 1.0f) + d_s_h;

    return h;
}
