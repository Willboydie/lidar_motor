#include "PIDControl.hpp"

PIDControl::PIDControl(float p, float i, float d, float _output_ramp, float _lower_limit, float _upper_limit)
    : P(p)
    , I(i)
    , D(d)
    , output_ramp(_output_ramp)    // output derivative limit [volts/second]
    , upper_limit(_upper_limit)    // max output value from controller
    , lower_limit(_lower_limit)    // output supply limit     [amps]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();
}

// PID controller function
float PIDControl::operator() (float error){
    // calculate the time from the last call
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // Two callers in the same micros() tick would give Ts == 0 and blow up
    // the derivative term. Clamp to 1 µs.
    if (Ts < 1e-6f) Ts = 1e-6f;
    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = P * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    // antiwindup - limit the output
    integral = _constrain(integral, lower_limit, upper_limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D * (error - error_prev) / Ts;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = _constrain(output, lower_limit, upper_limit);

    // if output ramp defined
    if ( output_ramp > 0.0f ) {
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp * Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp * Ts;
    }
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

void PIDControl::reset() {
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
    timestamp_prev = micros();
}

// // Integer overload - converts to float internally for precision, returns rounded int32_t
// int32_t PIDControl::operator() (int32_t error) {
//     float result = (*this)(static_cast<float>(error));
//     return static_cast<int32_t>(result);
// }