#include "PWMDriverTeensy41.hpp"

// if compiling for a Teensy 4.1
#if defined(__arm__) && defined(CORE_TEENSY) && ( defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41) )

#pragma message("")
#pragma message("SimpleFOC: compiling for Teensy 4.1")
#pragma message("")



// function setting the pwm duty cycle to the hardware
void PWMDriverTeensy41::writeCenterAligned3PWM(float duty_cycleA, float duty_cycleB, float duty_cycleC, void* params){
  Teensy41DriverParams* p = (Teensy41DriverParams*)params;
  write_pwm_on_pin (p->flextimers[0], p->submodules[0], p->channels[0], duty_cycleA);
  write_pwm_on_pin (p->flextimers[1], p->submodules[1], p->channels[1], duty_cycleB);
  write_pwm_on_pin (p->flextimers[2], p->submodules[2], p->channels[2], duty_cycleC);
}




void PWMDriverTeensy41::write_pwm_on_pin(IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, float duty) {
  uint16_t mask = 1 << submodule;
  uint32_t half_cycle = p->SM[submodule].VAL1;
  int mid_pwm = int((half_cycle)/2.0f);
  int cval = int(mid_pwm*(duty*2-1)) + mid_pwm;

  //printf("flexpwmWrite, p=%08lX, sm=%d, ch=%c, cval=%ld\n",
  //(uint32_t)p, submodule, channel == 0 ? 'X' : (channel == 1 ? 'A' : 'B'), cval);
  p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
  switch (channel) {
    case 0: // X
    p->SM[submodule].VAL0 = half_cycle - cval;
    p->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
    //printf(" write channel X\n");
    break;
    case 1: // A
    p->SM[submodule].VAL2 = -cval;
    p->SM[submodule].VAL3 = cval;
    p->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
    //printf(" write channel A\n");
    break;
    case 2: // B
    p->SM[submodule].VAL4 = -cval;
    p->SM[submodule].VAL5 = cval;
    p->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
    //printf(" write channel B\n");
  }
  p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}




// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// in generic case dont do anything
void* PWMDriverTeensy41::configureCenterAligned3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {

  if ( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, PWM_FREQUENCY_MAX); // constrain to 50kHz max

  IMXRT_FLEXPWM_t *flexpwmA,*flexpwmB,*flexpwmC;
  int submoduleA,submoduleB,submoduleC;
  flexpwmA = get_flexpwm(pinA);
  submoduleA = get_submodule(pinA);
  flexpwmB = get_flexpwm(pinB);
  submoduleB = get_submodule(pinB);
  flexpwmC = get_flexpwm(pinC);
  submoduleC = get_submodule(pinC);  
  int channelA = get_channel(pinA);
  int channelB = get_channel(pinB);
  int channelC = get_channel(pinC);


  // if pins belong to the flextimers and they only use submodules A and B
  // we can configure the center-aligned pwm
  if ((flexpwmA != nullptr) && (flexpwmB != nullptr) && (flexpwmC != nullptr) && (channelA > 0) && (channelB > 0) && (channelC > 0)) {
    Serial.println("TEENSY-DRV: All pins on Flexpwm A or B channels - Configuring center-aligned pwm!");

    // Configure FlexPWM units
    setup_pwm_timer_submodule (flexpwmA, submoduleA, true, pwm_frequency) ;  // others externally synced
    setup_pwm_timer_submodule (flexpwmB, submoduleB, true, pwm_frequency) ;  // others externally synced
    setup_pwm_timer_submodule (flexpwmC, submoduleC, false, pwm_frequency) ; // this is the master, internally synced
    delayMicroseconds (100) ;

    
    // Serial.printf("TEENSY-CS: Syncing to Master FlexPWM: %d, Submodule: %d\n", flexpwm_to_index(flexpwmC), submoduleC);
    // Serial.printf("TEENSY-CS: Slave timers FlexPWM: %d, Submodule: %d and FlexPWM: %d, Submodule: %d\n", flexpwm_to_index(flexpwmA), submoduleA, flexpwm_to_index(flexpwmB), submoduleB);

    // // turn on XBAR1 clock for all but stop mode
    xbar_init() ;

    // // Connect trigger to synchronize all timers 
    xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmA, submoduleA)) ;
    xbar_connect (flexpwm_submodule_to_trig(flexpwmC, submoduleC), flexpwm_submodule_to_ext_sync(flexpwmB, submoduleB)) ;
  
    Teensy41DriverParams* params = new Teensy41DriverParams {
      .pins = { pinA, pinB, pinC },
      .pwm_frequency = pwm_frequency,
      .flextimers = { flexpwmA, flexpwmB, flexpwmC},
      .submodules = { submoduleA, submoduleB, submoduleC},
      .channels = {channelA, channelB, channelC}
    };

    startup_pwm_pair (flexpwmA, submoduleA, channelA) ;
    startup_pwm_pair (flexpwmB, submoduleB, channelB) ;
    startup_pwm_pair (flexpwmC, submoduleC, channelC) ;

    // // config the pins 2/3/6/9/8/7 as their FLEXPWM alternates.
    *portConfigRegister(pinA) = pwm_pin_info[pinA].muxval ;
    *portConfigRegister(pinB) = pwm_pin_info[pinB].muxval ;
    *portConfigRegister(pinC) = pwm_pin_info[pinC].muxval ;

    return params;
  } else {
    Serial.println("TEENSY-DRV: Not all pins on Flexpwm A and B channels - cannot configure center-aligned pwm!");
    return nullptr;
  }  
}



// function which finds the flexpwm instance for a pin
// if it does not belong to the flexpwm timer it returns a null-pointer
IMXRT_FLEXPWM_t* PWMDriverTeensy41::get_flexpwm(uint8_t pin){
 
  const struct pwm_pin_info_struct *info;
  info = pwm_pin_info + pin;
  if (pin >= CORE_NUM_DIGITAL || info->type == 2) {
    Serial.printf("TEENSY-DRV: ERR: Pin: %d not Flextimer pin!\n", pin);
    return nullptr;
  }
  // FlexPWM pin
  IMXRT_FLEXPWM_t *flexpwm;
  switch ((info->module >> 4) & 3) {
    case 0: flexpwm = &IMXRT_FLEXPWM1; break;
    case 1: flexpwm = &IMXRT_FLEXPWM2; break;
    case 2: flexpwm = &IMXRT_FLEXPWM3; break;
    default: flexpwm = &IMXRT_FLEXPWM4;
  }
  if (flexpwm != nullptr){
    // Serial.printf("TEENSY-DRV: Pin: %d on Flextimer %d.\n", pin, ((info->module >> 4) & 3) + 1);
    return flexpwm;
  } 
  return nullptr;
}



// function which finds the timer submodule for a pin
// if it does not belong to the submodule it returns a -1
int PWMDriverTeensy41::get_submodule(uint8_t pin){
 
  const struct pwm_pin_info_struct *info;
  if (pin >= CORE_NUM_DIGITAL){
    Serial.printf("TEENSY-DRV: ERR: Pin: %d not Flextimer pin!\n", pin);
    return -1;
  } 
  
  info = pwm_pin_info + pin;
  int sm1 = info->module&0x3;

  if (sm1 >= 0 && sm1 < 4) {
    // Serial.printf("TEENSY-DRV: Pin %d on submodule %d.\n", pin, sm1);
    return sm1;
  }  else  {
    Serial.printf("TEENSY-DRV: ERR: Pin: %d not in submodule!\n", pin);
    return -1; 
  }
}



// function which finds the channel for flexpwm timer pin
// 0 - X
// 1 - A
// 2 - B
int PWMDriverTeensy41::get_channel(uint8_t pin){
  const struct pwm_pin_info_struct *info;
  info = pwm_pin_info + pin;
  if (pin >= CORE_NUM_DIGITAL || info->type == 2){
    Serial.printf("TEENSY-DRV: ERR: Pin: %d not Flextimer pin!\n", pin);
    return -1;
  }
  // Serial.printf("TEENSY-DRV: Pin: %d on channel %s.\n", pin, info->channel==0 ? "X" : info->channel==1 ? "A" : "B");
  return info->channel;
}



// Helper to set up a FlexPWM submodule.
// can configure sync, prescale 
// sets the desired frequency of the PWM 
// sets the center-aligned pwm
void PWMDriverTeensy41::setup_pwm_timer_submodule (IMXRT_FLEXPWM_t * flexpwm, int submodule, bool ext_sync,  const long frequency) {
  int submodule_mask = 1 << submodule ;
  flexpwm->MCTRL &= ~ FLEXPWM_MCTRL_RUN (submodule_mask) ;  // stop it if its already running
  flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK (submodule_mask) ;  // clear load OK
  
  // calculate the counter and prescaler for the desired pwm frequency
  uint32_t newdiv = (uint32_t)((float)F_BUS_ACTUAL / frequency + 0.5f);
  uint32_t prescale = 0;
  //printf(" div=%lu\n", newdiv);
  while (newdiv > 65535 && prescale < 7) {
    newdiv = newdiv >> 1;
    prescale = prescale + 1;
  }
  if (newdiv > 65535) {
    newdiv = 65535;
  } else if (newdiv < 2) {
    newdiv = 2;
  }

  // the halfcycle of the PWM
  int half_cycle = int(newdiv/2.0f);

  // if the timer should be externally synced with the master timer
  int sel = ext_sync ? 3 : 0;

  // setup the timer
  // https://github.com/PaulStoffregen/cores/blob/master/teensy4/imxrt.h
  flexpwm->SM[submodule].CTRL2 =  FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN | 
                                  FLEXPWM_SMCTRL2_FRCEN | FLEXPWM_SMCTRL2_INIT_SEL(sel) | FLEXPWM_SMCTRL2_FORCE_SEL(6);
  flexpwm->SM[submodule].CTRL  = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_HALF | FLEXPWM_SMCTRL_PRSC(prescale) ;
  // https://github.com/PaulStoffregen/cores/blob/70ba01accd728abe75ebfc8dcd8b3d3a8f3e3f25/teensy4/imxrt.h#L4948
  flexpwm->SM[submodule].OCTRL = 0; //FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB;//channel_to_invert==2 ? 0 : FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB ;
  if (!ext_sync) flexpwm->SM[submodule].TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN (0b000010)  ; // sync trig out on VAL1 match if master timer
  flexpwm->SM[submodule].DTCNT0 = 0 ;  
  flexpwm->SM[submodule].DTCNT1 = 0 ;  
  flexpwm->SM[submodule].INIT = -half_cycle;      // count from -HALFCYCLE to +HALFCYCLE
  flexpwm->SM[submodule].VAL0 = 0;
  flexpwm->SM[submodule].VAL1 = half_cycle;
  flexpwm->SM[submodule].VAL2 = 0  ;
  flexpwm->SM[submodule].VAL3 = 0  ;
  flexpwm->SM[submodule].VAL2 = 0  ;
  flexpwm->SM[submodule].VAL3 = 0  ;

  flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK (submodule_mask) ;  // loading reenabled
  flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN (submodule_mask) ;   // start it running
}



// function finding the flexpwm instance given the submodule
int PWMDriverTeensy41::flexpwm_to_index(IMXRT_FLEXPWM_t* flexpwm){
  if(flexpwm == &IMXRT_FLEXPWM1) return 1;
  if(flexpwm == &IMXRT_FLEXPWM2) return 2;
  if(flexpwm == &IMXRT_FLEXPWM3) return 3;
  if(flexpwm == &IMXRT_FLEXPWM4) return 4;
  return -1;
}



void PWMDriverTeensy41::xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
}



// The i.MXRT1062 uses one config register per two XBAR outputs, so a helper
// function to make code more readable. 
void PWMDriverTeensy41::xbar_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}



// function finding the TRIG event given the flexpwm timer and the submodule
// returning -1 if the submodule is not valid or no trigger is available
// allowing flexpwm1-4 and submodule 0-3
//
// the flags are defined in the imxrt.h file
// https://github.com/PaulStoffregen/cores/blob/dd6aa8419ee173a0a6593eab669fbff54ed85f48/teensy4/imxrt.h#L9662-L9693
int PWMDriverTeensy41::flexpwm_submodule_to_trig(IMXRT_FLEXPWM_t* flexpwm, int submodule){
  if(submodule <0 && submodule > 3) return -1;
  if(flexpwm == &IMXRT_FLEXPWM1){
      return XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0 + submodule; 
  }else if(flexpwm == &IMXRT_FLEXPWM2){
      return XBARA1_IN_FLEXPWM2_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM3){
      return XBARA1_IN_FLEXPWM3_PWM1_OUT_TRIG0 + submodule;
  }else if(flexpwm == &IMXRT_FLEXPWM4){
      return XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0 + submodule;
  }
  return -1;
}



// function finding the EXT_SYNC event given the flexpwm timer and the submodule
// returning -1 if the submodule is not valid or no trigger is available
// allowing flexpwm1-4 and submodule 0-3
//
// the flags are defined in the imxrt.h file
// https://github.com/PaulStoffregen/cores/blob/dd6aa8419ee173a0a6593eab669fbff54ed85f48/teensy4/imxrt.h#L9757
int PWMDriverTeensy41::flexpwm_submodule_to_ext_sync(IMXRT_FLEXPWM_t* flexpwm, int submodule){
  if (submodule < 0 && submodule > 3) return -1;
  if (flexpwm == &IMXRT_FLEXPWM1){
      return XBARA1_OUT_FLEXPWM1_PWM0_EXT_SYNC + submodule; 
  } else if (flexpwm == &IMXRT_FLEXPWM2) {
      return XBARA1_OUT_FLEXPWM2_PWM0_EXT_SYNC + submodule;
  } else if (flexpwm == &IMXRT_FLEXPWM3) {
      return XBARA1_OUT_FLEXPWM3_EXT_SYNC0 + submodule; // TODO verify why they are not called PWM0_EXT_SYNC but EXT_SYNC0
  } else if (flexpwm == &IMXRT_FLEXPWM4) {
      return XBARA1_OUT_FLEXPWM4_EXT_SYNC0 + submodule; // TODO verify why they are not called PWM0_EXT_SYNC but EXT_SYNC0
  }
  return -1;
}



// staring the PWM on A and B channels of the submodule
void PWMDriverTeensy41::startup_pwm_pair (IMXRT_FLEXPWM_t * flexpwm, int submodule, int channel)
{
  int submodule_mask = 1 << submodule ;

  if(channel==1)  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN (submodule_mask); // enable A output
  else if(channel==2)  flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN (submodule_mask); // enable B output
}


void PWMDriverTeensy41::stopPwmHardware(void* params) {
  Teensy41DriverParams* p = (Teensy41DriverParams*)params;
  // 1. Stop PWM outputs (VERY important)
  for (int i = 0; i < 3; i++) {
      IMXRT_FLEXPWM_t* flexpwm = p->flextimers[i];
      int sub = p->submodules[i];
      int ch  = p->channels[i];

      // Disable outputs for this channel on the shared OUTEN register
      int submodule_mask = 1 << sub;
      switch (ch) {
        case 0:
          flexpwm->OUTEN &= ~FLEXPWM_OUTEN_PWMX_EN(submodule_mask);
          break;
        case 1:
          flexpwm->OUTEN &= ~FLEXPWM_OUTEN_PWMA_EN(submodule_mask);
          break;
        case 2:
          flexpwm->OUTEN &= ~FLEXPWM_OUTEN_PWMB_EN(submodule_mask);
          break;
        default:
          break;
      }

      // Stop the submodule counter
      flexpwm->MCTRL &= ~ FLEXPWM_MCTRL_RUN (submodule_mask) ;
  }
}

#endif