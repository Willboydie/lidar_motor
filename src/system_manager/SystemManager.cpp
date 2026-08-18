#include "SystemManager.hpp"

SystemManager::SystemManager(
    FOCController& _focControllerX,
    FOCController& _focControllerY,
    Sensor& _sensorX,
    Sensor& _sensorY,
    InlineCurrentSense& _currentSenseX,
    InlineCurrentSense& _currentSenseY,
    LDM& _ldm,
    PWMDriver& _pwmDriverX,
    PWMDriver& _pwmDriverY,
    FaultManager& _faultManager,
    StateMachine& _stateMachine,
    ScanEngine& _scanEngine
)
: focControllerX(_focControllerX)
, focControllerY(_focControllerY)
, sensorX(_sensorX)
, sensorY(_sensorY)
, currentSenseX(_currentSenseX)
, currentSenseY(_currentSenseY)
, ldm(_ldm)
, pwmDriverX(_pwmDriverX)
, pwmDriverY(_pwmDriverY)
, faultManager(_faultManager)
, stateMachine(_stateMachine)
, scanEngine(_scanEngine)
{}

void SystemManager::init() {
    Debug::msg("SYSTEM MANAGER INIT CALLED");

    if (!sensorX.init()) {
        faultManager.addFault(FAULT_ENC_X_INIT);
        Debug::msg("SENSOR X INIT FAILED");
        return;
    }
    Debug::msg("SENSOR X INITIALISED");
    if (!sensorY.init()) {
        faultManager.addFault(FAULT_ENC_Y_INIT);
        Debug::msg("SENSOR Y INIT FAILED");
        return;
    }
    Debug::msg("SENSOR Y INITIALISED");
    ldm.init();
    if (!ldm.isInitialised()) {
        faultManager.addFault(FAULT_LDM_INIT);
        Debug::msg("LDM INIT FAILED");
        return;
    Debug::msg("LDM INITIALISED");
    }

    if (!pwmDriverX.init()) {
        faultManager.addFault(FAULT_PWM_X_INIT);
        Debug::msg("PWM DRIVER X INIT FAILED");
        return;
    }
    Debug::msg("PWM DRIVER X INITIALISED");
    if (!pwmDriverY.init()) {
        faultManager.addFault(FAULT_PWM_Y_INIT);
        Debug::msg("PWM DRIVER Y INIT FAILED");
        return;
    }
    Debug::msg("PWM DRIVER Y INITIALISED");

    if (!focControllerX.init()) {
        faultManager.addFault(FAULT_FOC_X_INIT);
        Debug::msg("FOC X INIT FAILED");
        return;
    }
    Debug::msg("FOC X INITIALISED");
    if (!focControllerY.init()) {
        faultManager.addFault(FAULT_FOC_Y_INIT);
        Debug::msg("FOC Y INIT FAILED");
        return;
    }
    Debug::msg("FOC Y INITIALISED");
    initialised = true;
    Debug::msg("SYSTEM MANAGER INITIALISED");
}

void SystemManager::enable() {
    if (enabled) return;
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        Debug::msg("SYSTEM NOT INITIALISED - ENABLE FAILED");
        return;
    }
    focControllerX.enable();
    focControllerY.enable();
    sensorX.enable();
    sensorY.enable();
    ldm.enable();
    scanEngine.enable();
    stateMachine.enabled = true;
    enabled = true;
}

void SystemManager::disable() {
    if (!enabled) return;
    focControllerX.disable();
    focControllerY.disable();
    sensorX.disable();
    sensorY.disable();
    ldm.disable();
    scanEngine.disable();
    stateMachine.enabled = false;
    enabled = false;
}

void SystemManager::restart() {
    sleep();
    wake();
}

// MCU low-power entry/exit — compile-time polymorphic.
//   STM32G4: Sleep mode (CPU clock off; peripherals keep running and can wake
//            via interrupt). SysTick is suspended across sleep so it cannot
//            immediately re-wake the core; wake() re-arms it.
//   Teensy 4.1: shallow WFI (SLEEPDEEP clear). Wake sources must already be
//            armed; clocks are intact on return so wake() is a no-op restore.
// Both paths quiesce actuators first so the motors are not left driven while
// the CPU is halted. wake() does not re-enable — the host must call enable().
void SystemManager::sleep() {
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        return;
    }
    if (enabled) {
        disable();
    }
#if defined(STM32G4xx)
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
#elif defined(ARDUINO_TEENSY41)
    SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
    __asm__ volatile ("wfi");
#else
#error "SystemManager::sleep: unsupported target (need STM32G4xx or ARDUINO_TEENSY41)"
#endif
}

void SystemManager::wake() {
#if defined(STM32G4xx)
    HAL_ResumeTick();
#elif defined(ARDUINO_TEENSY41)
    // WFI returns with clocks/peripherals intact — nothing to restore.
#else
#error "SystemManager::wake: unsupported target (need STM32G4xx or ARDUINO_TEENSY41)"
#endif
}

void SystemManager::enterBootloader() {
    if (enabled) {
        disable();
    }
#if defined(STM32G4xx)
    // STM32 system memory bootloader (USB DFU / USART). Address is family-fixed;
    // Nucleo-G474RE maps system memory at 0x1FFF0000.
    constexpr uint32_t kSystemMemory = 0x1FFF0000u;
    const uint32_t sp = *reinterpret_cast<volatile uint32_t*>(kSystemMemory);
    const uint32_t rv = *reinterpret_cast<volatile uint32_t*>(kSystemMemory + 4u);
    auto boot = reinterpret_cast<void (*)(void)>(rv);
    __disable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __set_MSP(sp);
    boot();
#elif defined(ARDUINO_TEENSY41)
    _reboot_Teensyduino_();
#else
#error "SystemManager::enterBootloader: unsupported target (need STM32G4xx or ARDUINO_TEENSY41)"
#endif
}