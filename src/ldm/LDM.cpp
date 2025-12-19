#include "LDM.hpp"


LDM::LDM(int _scl, int _sda) : SCL(_scl), SDA(_sda) {}

void LDM::enable() {
    if (enabled) return;
    enabled = true;
}

void LDM::disable() {
    if (!enabled) return;
    enabled = false;
}

void LDM::update() {
    if (!enabled) return;
    if (readSensor()) requestDistance();
}