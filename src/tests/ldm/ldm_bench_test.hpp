#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../../utils/constants.h"
#include "../../ldm/GarminV3HP.hpp"
#include "../../fault_manager/FaultManager.hpp"
#include "../wiring/wiring_common.hpp"

// LDM bench — Garmin V3HP hardware isolation (I2C + MODE pin + ISR).
// Diagnoses FAULT_LDM_TIMEOUT during init/resetReferenceFilter without the motor stack.
//
// Invoked from setup(); main loop() stays empty.

static constexpr uint8_t LDM_I2C_ADDR = 0x62;
static constexpr uint32_t LDM_I2C_HZ_SAFE = 100000;  // Garmin datasheet: 100 or 400 kHz
static constexpr uint32_t LDM_MODE_TIMEOUT_MS = 1000;
static constexpr uint32_t LDM_RATE_WINDOW_MS = 3000;

// Subclass that initialises Wire/MODE without resetReferenceFilter (which waits
// on isDataReady() before attachInterrupt — always times out in production init).
class BenchGarmin : public GarminV3HP {
public:
    using GarminV3HP::GarminV3HP;

    void initBench(uint32_t i2c_hz) {
        Wire.setSDA(SDA);
        Wire.setSCL(SCL);
        Wire.begin();
        Wire.setClock(i2c_hz);
        pinMode(MODE_PIN, INPUT);
        markInitialised();
    }
};

inline void ldm_print_pins() {
    Serial.print("PINS scl=");
    Serial.print(GARMIN_SCL);
    Serial.print(" sda=");
    Serial.print(GARMIN_SDA);
    Serial.print(" mode=");
    Serial.println(GARMIN_MODE_PIN);
}

inline uint8_t ldm_wire_write(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(LDM_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission();
}

inline uint8_t ldm_wire_read(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(LDM_I2C_ADDR);
    Wire.write(reg);
    const uint8_t wr = Wire.endTransmission(false);
    if (wr != 0) {
        return wr;
    }
    const uint8_t got = Wire.requestFrom(LDM_I2C_ADDR, len, (uint8_t)true);
    for (uint8_t i = 0; i < got; ++i) {
        buf[i] = Wire.read();
    }
    return (got == len) ? 0 : 0xFF;
}

inline uint8_t ldm_i2c_probe() {
    Wire.beginTransmission(LDM_I2C_ADDR);
    return Wire.endTransmission();
}

inline int ldm_i2c_scan(uint8_t* found, int max_found) {
    int n = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            if (n < max_found) {
                found[n] = addr;
            }
            ++n;
        }
    }
    return n;
}

// Poll MODE pin for ready (HIGH). Returns elapsed ms, or -1 on timeout.
// Garmin: MODE as status = LOW while busy, HIGH when ready (acqConfigReg bit0=1).
inline int32_t ldm_wait_mode_ready(uint32_t timeout_ms) {
    const uint32_t t0 = millis();
    while (digitalRead(GARMIN_MODE_PIN) == HIGH && (millis() - t0) < 5) {
        // brief wait for falling edge into busy
    }
    while (digitalRead(GARMIN_MODE_PIN) == LOW) {
        if ((millis() - t0) >= timeout_ms) {
            return -1;
        }
    }
    return static_cast<int32_t>(millis() - t0);
}

inline void ldm_print_ldm_faults() {
    Serial.print("FAULTS i2c=");
    Serial.print(faultManager.faulty(FAULT_LDM_I2C));
    Serial.print(" timeout=");
    Serial.print(faultManager.faulty(FAULT_LDM_TIMEOUT));
    Serial.print(" init=");
    Serial.print(faultManager.faulty(FAULT_LDM_INIT));
    Serial.print(" config=");
    Serial.println(faultManager.faulty(FAULT_LDM_CONFIG));
}

inline bool ldm_phase_i2c() {
    Serial.println();
    Serial.println("--- PHASE A: I2C bus ---");

    Wire.setSDA(GARMIN_SDA);
    Wire.setSCL(GARMIN_SCL);
    Wire.begin();
    Wire.setClock(LDM_I2C_HZ_SAFE);

    Serial.print("I2C clock_hz=");
    Serial.println(LDM_I2C_HZ_SAFE);

    uint8_t found[8];
    const int n = ldm_i2c_scan(found, 8);
    Serial.print("I2C scan_count=");
    Serial.print(n);
    Serial.print(" addrs=");
    for (int i = 0; i < n && i < 8; ++i) {
        if (i) {
            Serial.print(',');
        }
        Serial.print("0x");
        Serial.print(found[i], HEX);
    }
    Serial.println();

    const uint8_t probe = ldm_i2c_probe();
    Serial.print("I2C probe addr=0x62 endTransmission=");
    Serial.print(probe);
    Serial.println(probe == 0 ? " ACK" : " NACK/ERR");

    if (probe != 0) {
        Serial.println("RESULT phase_i2c FAIL — no ACK at 0x62 (wiring/power/address)");
        return false;
    }

    uint8_t status = 0xFF;
    const uint8_t st_err = ldm_wire_read(0x01, &status, 1);
    Serial.print("REG status(0x01) err=");
    Serial.print(st_err);
    Serial.print(" value=0x");
    Serial.println(status, HEX);

    const uint8_t acq = 0x08 | 0x01;  // default + MODE-as-status
    const uint8_t wr = ldm_wire_write(0x04, acq);
    uint8_t acq_rb = 0;
    const uint8_t rd = ldm_wire_read(0x04, &acq_rb, 1);
    Serial.print("REG acqConfig write_err=");
    Serial.print(wr);
    Serial.print(" read_err=");
    Serial.print(rd);
    Serial.print(" wrote=0x");
    Serial.print(acq, HEX);
    Serial.print(" readback=0x");
    Serial.println(acq_rb, HEX);

    const bool ok = (wr == 0 && rd == 0 && (acq_rb & 0x01));
    Serial.print("RESULT phase_i2c ");
    Serial.println(ok ? "PASS" : "FAIL");
    return ok;
}

inline bool ldm_phase_mode_poll() {
    Serial.println();
    Serial.println("--- PHASE B: MODE pin polled measurement ---");
    Serial.println("Aim unit at a nearby wall/target (~0.5–3 m).");

    pinMode(GARMIN_MODE_PIN, INPUT);
    const int idle = digitalRead(GARMIN_MODE_PIN);
    Serial.print("MODE idle_level=");
    Serial.println(idle == HIGH ? "HIGH" : "LOW");

    const uint8_t trig = ldm_wire_write(0x00, 0x01);
    Serial.print("TRIG write_err=");
    Serial.println(trig);
    if (trig != 0) {
        Serial.println("RESULT phase_mode_poll FAIL — trigger I2C error");
        return false;
    }

    const int32_t elapsed = ldm_wait_mode_ready(LDM_MODE_TIMEOUT_MS);
    Serial.print("MODE ready_elapsed_ms=");
    Serial.println(elapsed);
    if (elapsed < 0) {
        Serial.print("MODE stuck_level=");
        Serial.println(digitalRead(GARMIN_MODE_PIN) == HIGH ? "HIGH" : "LOW");
        Serial.println("RESULT phase_mode_poll FAIL — MODE never went ready (pin/wiring/sensor)");
        return false;
    }

    uint8_t dist[2] = {0, 0};
    const uint8_t rd = ldm_wire_read(0x0f, dist, 2);
    const uint16_t cm = static_cast<uint16_t>((dist[0] << 8) | dist[1]);
    Serial.print("DIST read_err=");
    Serial.print(rd);
    Serial.print(" cm=");
    Serial.println(cm);

    const bool ok = (rd == 0 && cm > 0 && cm < 4000);
    Serial.print("RESULT phase_mode_poll ");
    Serial.println(ok ? "PASS" : "FAIL");
    return ok;
}

inline bool ldm_phase_isr_rate() {
    Serial.println();
    Serial.println("--- PHASE C: driver ISR path (enable + rate) ---");

    faultManager.clearAllFaults();

    // Default balanced config + MODE-as-status (same as Garmin config 0).
    ldm_wire_write(0x02, 0x80);
    ldm_wire_write(0x04, 0x08 | 0x01);
    ldm_wire_write(0x12, 0x05);
    ldm_wire_write(0x1c, 0x00);

    BenchGarmin bench(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    bench.initBench(LDM_I2C_HZ_SAFE);
    bench.enable();
    if (!bench.isEnabled()) {
        Serial.println("RESULT phase_isr FAIL — enable() rejected");
        ldm_print_ldm_faults();
        return false;
    }

    const uint32_t t0 = millis();
    uint32_t count = 0;
    uint32_t sum_cm = 0;
    uint16_t min_cm = 0xFFFF;
    uint16_t max_cm = 0;

    while ((millis() - t0) < LDM_RATE_WINDOW_MS) {
        if (bench.isDataReady()) {
            bench.readDistance();
            const uint16_t cm = bench.getLatestDistance();
            sum_cm += cm;
            if (cm < min_cm) {
                min_cm = cm;
            }
            if (cm > max_cm) {
                max_cm = cm;
            }
            ++count;
            bench.triggerMeasurement();
        }
    }
    bench.disable();

    const float hz = 1000.0f * static_cast<float>(count) / static_cast<float>(LDM_RATE_WINDOW_MS);
    const float mean_cm = count ? (static_cast<float>(sum_cm) / static_cast<float>(count)) : 0.0f;

    Serial.print("ISR window_ms=");
    Serial.print(LDM_RATE_WINDOW_MS);
    Serial.print(" samples=");
    Serial.print(count);
    Serial.print(" rate_hz=");
    Serial.print(hz, 1);
    Serial.print(" mean_cm=");
    Serial.print(mean_cm, 1);
    Serial.print(" min_cm=");
    Serial.print(min_cm == 0xFFFF ? 0 : min_cm);
    Serial.print(" max_cm=");
    Serial.println(max_cm);
    ldm_print_ldm_faults();

    // >5 Hz proves MODE interrupt wiring; production is typically 300–400 Hz.
    const bool ok = count >= 15;
    Serial.print("RESULT phase_isr ");
    Serial.println(ok ? "PASS" : "FAIL");
    return ok;
}

inline bool ldm_phase_init_path() {
    Serial.println();
    Serial.println("--- PHASE D: GarminV3HP::init() (includes resetReferenceFilter) ---");
    Serial.println("NOTE: resetReferenceFilter waits on isDataReady() before");
    Serial.println("      attachInterrupt — TIMEOUT here can be software even if A–C passed.");

    faultManager.clearAllFaults();
    GarminV3HP ldm(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    ldm.init();

    // init() calls markInitialised() before resetReferenceFilter, so isInitialised
    // can be true even when the ref-filter wait timed out.
    const bool timed_out = faultManager.faulty(FAULT_LDM_TIMEOUT) != 0;
    const bool i2c_fail = faultManager.faulty(FAULT_LDM_I2C) != 0;

    Serial.print("INIT initialised=");
    Serial.print(ldm.isInitialised() ? 1 : 0);
    Serial.print(" timed_out=");
    Serial.print(timed_out ? 1 : 0);
    Serial.print(" i2c_fail=");
    Serial.println(i2c_fail ? 1 : 0);
    ldm_print_ldm_faults();

    const bool ok = ldm.isInitialised() && !timed_out && !i2c_fail;
    Serial.print("RESULT phase_init ");
    Serial.println(ok ? "PASS" : "FAIL");
    return ok;
}

inline void ldm_bench_test() {
    wiring_serial_begin();

    Serial.print("TEST ldm_bench target=");
    Serial.println(wiring_target_name());
    Serial.println("LDM — Garmin V3HP hardware isolation (I2C / MODE poll / ISR / init)");
    ldm_print_pins();

    const bool a = ldm_phase_i2c();
    bool b = false;
    bool c = false;
    bool d = false;

    if (a) {
        b = ldm_phase_mode_poll();
    } else {
        Serial.println("SKIP phase_mode_poll (I2C failed)");
    }

    if (a && b) {
        c = ldm_phase_isr_rate();
        d = ldm_phase_init_path();
    } else {
        Serial.println("SKIP phase_isr / phase_init (earlier phase failed)");
    }

    Serial.println();
    Serial.print("RESULT ldm_bench ");
    if (a && b && c) {
        Serial.print("PASS");
        if (!d) {
            Serial.print(" (init TIMEOUT is software — ISR not armed during resetReferenceFilter)");
        }
    } else {
        Serial.print("FAIL");
    }
    Serial.print(" i2c=");
    Serial.print(a ? 1 : 0);
    Serial.print(" mode_poll=");
    Serial.print(b ? 1 : 0);
    Serial.print(" isr=");
    Serial.print(c ? 1 : 0);
    Serial.print(" init=");
    Serial.println(d ? 1 : 0);

    Serial.println("LDM bench complete — paste this serial log back into chat.");
    while (true) {
        delay(1000);
    }
}
