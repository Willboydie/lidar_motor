#pragma once

#include "LDM.hpp"

struct GarminConfig {
 /// config parameters
};


class GarminV3HP : public LDM {
public:
    void init() override;

private:
    void requestDistance() override;
    bool readSensor() override;

    bool pollStatus();
    float getDistance();

    void write();
    void read();

    GarminConfig config;
    void configure();

    void reset();
    void resetReferenceFilter();
};