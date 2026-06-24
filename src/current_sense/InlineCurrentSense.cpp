#include "InlineCurrentSense.hpp"

// Clarke transform for two measured phases (ia + ib + ic = 0).
ABCurrent_s InlineCurrentSense::getABCurrents(PhaseCurrent_s current) {
    ABCurrent_s ab;
    ab.alpha = current.a;
    ab.beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    return ab;
}

// Park transform.
DQCurrent_s InlineCurrentSense::getDQCurrents(ABCurrent_s current, float angle_el) {
    float st, ct;
    fast_sincos(angle_el, &st, &ct);
    DQCurrent_s dq;
    dq.d = current.alpha * ct + current.beta * st;
    dq.q = current.beta * ct - current.alpha * st;
    return dq;
}

DQCurrent_s InlineCurrentSense::getFOCCurrents(float angle_el) {
    PhaseCurrent_s current = getPhaseCurrents();
    ABCurrent_s ab = getABCurrents(current);
    return getDQCurrents(ab, angle_el);
}
