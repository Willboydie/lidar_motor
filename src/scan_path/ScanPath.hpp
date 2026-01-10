#include "../utils/quick_defs.h"
#include "sin_lut_q15.hpp"

class ScanPath {
public:
    ScanPath(uint16_t _FOC_loops_per_phi_step, uint16_t _num_vertical_oscillations, float _horizontal_fov, float _vertical_fov, uint16_t _num_phases) 
    : theta_period( _FOC_loops_per_phi_step * _num_vertical_oscillations )
    , phi_period(_FOC_loops_per_phi_step)
    , num_phases(_num_phases)
    , theta_limit(_horizontal_fov)
    , phi_limit(_vertical_fov)
    {         
        if (theta_period == 0) theta_period = 1;
        if (phi_period == 0) phi_period = 1;
        if (num_phases == 0) num_phases = 1;
    }
    
    virtual int16_t lookUpTheta() = 0;
    virtual int16_t lookUpPhi() = 0;

    uint32_t theta_period; // 32bit to prevent overflow above --pure precaution
    uint16_t phi_period;
    uint32_t step_count = 0;  // 32bit so that phi_period cannot be larger which would break update()
    uint16_t num_phases;
    uint16_t theta_phase = 3 * num_phases / 4; // theta = sin(theta_phase)  --- starts at -limit
    uint16_t phi_phase = 0;

    float theta_limit;
    float phi_limit;

    float calcTargetTheta();
    float calcTargetPhi();

    void update();
    float theta = 0.0f;
    float phi = 0.0f;
};


class SinusoidalScanPath : public ScanPath {
public:
    SinusoidalScanPath(uint16_t _FOC_loops_per_phi_step, uint16_t _num_vertical_oscillations, float _horizontal_fov, float _vertical_fov)
    : ScanPath(
        _FOC_loops_per_phi_step, 
        _num_vertical_oscillations, 
        _horizontal_fov, 
        _vertical_fov, 
        SIN_LUT_SIZE*4 // 4 x Quarter Wave
    )
    {}

protected:
    int16_t lookUpPhi() override;
    int16_t lookUpTheta() override;
};