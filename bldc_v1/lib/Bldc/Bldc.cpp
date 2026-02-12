// FOC Driver controlled by Teensy 4.1 (IMX RT1060)

#include "Bldc.h"

static Bldc* instance = nullptr;
static uint8_t sd_aux = 0;

BLDC_Logger* logger = new BLDC_Logger; 

void PWM2_CompletedCallback(){
    if (IMXRT_FLEXPWM2.SM[0].STS & 0x1) {  // HF (Half Period Flag)
        IMXRT_FLEXPWM2.SM[0].STS = 0x1;    // Clear HF flag

        // Start both ADCs in parallel
        if(instance->analogChannelADC1 == Bldc::AnalogSensorChannel::throttle){
            instance->analogChannelADC1 = Bldc::AnalogSensorChannel::VBatSense;
            triggerADC_VBat();
        }else{
            instance->analogChannelADC1 = Bldc::AnalogSensorChannel::throttle;
            triggerADC_Throttle();
        }
        // Currents (ADC2)
        if(instance->currentChannelADC2 == Bldc::CurrentSensorChannel::A){
            instance->currentChannelADC2 = Bldc::CurrentSensorChannel::B;
            triggerADC_CurrentB();
        }
        else if(instance->currentChannelADC2 == Bldc::CurrentSensorChannel::B){
            instance->currentChannelADC2 = Bldc::CurrentSensorChannel::C;
            triggerADC_CurrentC();
        }else{
            instance->currentChannelADC2 = Bldc::CurrentSensorChannel::A;
            triggerADC_CurrentA();
        }
    }
    
    if (IMXRT_FLEXPWM2.SM[0].STS & 0x2) {
        instance->newCycle = true;
        IMXRT_FLEXPWM2.SM[0].STS = 0x3;   // Clear FF flag
        IMXRT_FLEXPWM2.SM[0].INTEN = 0x3; // Re-enable half-period and full period interrupt (FF)
    }
}

void ADC1_CompletedConversionCallback(){
    if (ADC1_HS & ADC_HS_COCO0) {
        uint32_t result = ADC1_R0 & 0x0FFF; // Read the ADC result
        switch(instance->analogChannelADC1){
            case Bldc::AnalogSensorChannel::throttle:
            {
                instance->newThrottleVal = true;
                instance->throttleRawVal = result; // Update throttleVal
                logger->data.raw_throttle = result;
                break;
            }
            case Bldc::AnalogSensorChannel::VBatSense:{
                instance->VBatSenseRaw = result;
                logger->data.VBat = result;
                toggleLed();
                break;
            }
        }
        ADC1_HS &= ~ADC_HS_COCO0;
    }
}

void ADC2_CompletedConversionCallback(){
    if (ADC2_HS & ADC_HS_COCO0) {
        uint32_t result = ADC2_R0 & 0x0FFF;
        switch(instance->currentChannelADC2) {
            case Bldc::CurrentSensorChannel::A:
            {
                instance->newCurrentA = true;
                instance->currentA = result;
                logger->data.currentA = result;
                break;
            }
            case Bldc::CurrentSensorChannel::B:
            {
                instance->newCurrentB = true;
                instance->currentB = result;
                logger->data.currentB = result;
                break;
            }
            case Bldc::CurrentSensorChannel::C:
            {
                instance->newCurrentC = true;
                instance->currentC = result;
                logger->data.currentC = result;
                break;
            } 
            default:
            {
                break;
            }
        }
        ADC2_HS &= ~ADC_HS_COCO0;
    }
}

void GPIO_ChangeEdgeCallback(){
    unsigned long now = micros();
    float dt = (now - instance->lastHallChangeTime) / 1000000.0f;
    instance->lastHallChangeTime = now;
    const float MIN_DT = 0.00001f; // 1 us minimum dt
    if(dt < MIN_DT) {
        dt = MIN_DT;
    }
    #ifdef RPM
        float measuredRpm = 60.0f / (TICKS_PER_REV * dt);
    #else
        float measuredRpm = 1.0f / (TICKS_PER_REV * dt);
    #endif    
    const float MAX_RPM = 10000.0f;
    // If the new measured value is more than, say, 5 times the current rpm (and current rpm isn’t near zero), ignore it.
    if (instance->rpm > 0.001f && measuredRpm > instance->rpm * 30.0f) {
        measuredRpm = instance->rpm;
    }

    if(measuredRpm > MAX_RPM) {
        measuredRpm = instance->rpm;
    }
    
    
    instance->rpmBuffer[instance->rpmBufferIndex] = measuredRpm;
    instance->rpmBufferIndex = (instance->rpmBufferIndex + 1) % FILTER_SIZE;
    if(instance->rpmBufferCount < FILTER_SIZE) {
        instance->rpmBufferCount++;
    }
    
    float sum = 0.0f;
    for (int i = 0; i < instance->rpmBufferCount; i++) {
        sum += instance->rpmBuffer[i];
    }
    float filteredRpm = sum / instance->rpmBufferCount;
    
    instance->rpm = filteredRpm;

    uint8_t newHall;
    instance->readHalls(newHall);
    
    // If the hall state has changed, update the instance's hall state and flag.
    if(newHall != instance->hallState) {
        instance->hallState = newHall;
        instance->newHallUpdate = true;
    }

    #ifdef MEASURE_TICKS_PER_REV
        instance->testRev++;
        Serial.println(instance->testRev);
    #endif
}


Bldc::Bldc() : controlType(ControlType::Trap), hallState(0), throttleRawVal(0), throttleNormVal(0), newCycle(false) {
    instance = this; // Set the static instance pointer
    // Initialize phase configurations
    phaseA = {0, 0, kGateAH, Phase::Mode::X, 1};
    phaseB = {0, 0, kGateBH, Phase::Mode::X, 2};
    phaseC = {0, 0, kGateCH, Phase::Mode::X, 3};
    int rpmBufferIndex = 0;
    int rpmBufferCount = 0;
    rpm = 0;
    #ifdef MEASURE_TICKS_PER_REV
        testRev = 0;
    #endif
}

Bldc::~Bldc() {}

void Bldc::driverInit() {
    // Initialize Hall sensor pins
    pinMode(kHallAPin, INPUT);
    pinMode(kHallBPin, INPUT);
    pinMode(kHallCPin, INPUT);

    attachInterrupt(digitalPinToInterrupt(kHallAPin), GPIO_ChangeEdgeCallback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kHallBPin), GPIO_ChangeEdgeCallback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kHallCPin), GPIO_ChangeEdgeCallback, CHANGE);
   
    // Initialize debug pins
    pinMode(kBuiltInLedPin, OUTPUT);
    pinMode(kIrqFlagPin, OUTPUT);

    // Configure hardware modules
    configurePWM();
    configureADC();

#ifdef SERIAL_DEBUG
    Serial.begin(115200);
    Serial.println("Driver Init");
#endif

    // Communication with telemetry
    Serial1.begin(115200);

    // Initialize SD logging 
    logger->init();

    // Trigger ADC conversion
    triggerADC_Throttle();
    triggerADC_CurrentA();
}

void Bldc::run() {
    setPhaseDuty(0,0,0);
}

void Bldc::configurePWM() {
    // Check Ref Manual pg 3074 //
    IMXRT_FLEXPWM2.SM[0].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[0].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[0].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[0].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[0].DTCNT1 = 70;

    IMXRT_FLEXPWM2.SM[2].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[2].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[2].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[2].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[2].DTCNT1 = 70;

    IMXRT_FLEXPWM2.SM[3].CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP;
    IMXRT_FLEXPWM2.SM[3].CTRL = FLEXPWM_SMCTRL_HALF;
    IMXRT_FLEXPWM2.SM[3].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    IMXRT_FLEXPWM2.SM[3].DTCNT0 = 70;
    IMXRT_FLEXPWM2.SM[3].DTCNT1 = 70;
    
    // Enable interrupt for HALF (0x1) or FULL (0x2) CYCLE
    IMXRT_FLEXPWM2.SM[0].STS = 0x3;
    IMXRT_FLEXPWM2.SM[0].INTEN = 0x3;
    
    // Fault interrupt enable
    IMXRT_FLEXPWM2.FCTRL0 |= FLEXPWM_FCTRL0_FLVL(4);

    // PWM4 configured to be the master 
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b00);
    IMXRT_FLEXPWM2.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b10);
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_INIT_SEL(0b10);

    // Force select settings
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(0);
    IMXRT_FLEXPWM2.SM[2].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);
    IMXRT_FLEXPWM2.SM[3].CTRL2 |= FLEXPWM_SMCTRL2_FORCE_SEL(1);

    // PWM2 SM0 forces other submodules initialization for sync
    IMXRT_FLEXPWM2.SM[0].CTRL2 |= FLEXPWM_SMCTRL2_FORCE;

    // Load ready flags
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN((1<<0) | (1<<2) | (1<<3));
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_CLDOK((1<<0) | (1<<2) | (1<<3));
    IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_LDOK((1<<0) | (1<<2) | (1<<3));

    // Enable Hardware trigger for ADC conversions when VAL1 is reached (PWM_OUT_TRIG1)
    IMXRT_FLEXPWM2.SM[0].TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(0); 
    // Set PWM Interrupt (configured to mid cycle)
    attachInterruptVector(IRQ_FLEXPWM2_0, PWM2_CompletedCallback);
    NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_0);

    // Set PWM frequencies 
    #define M(a, b) ((((a) - 1) << 4) | (b))
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, kPwmFrequency);
    setPwmFrequency(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, kPwmFrequency);

    // Write zeros to everything
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, 0, phaseA.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, 0, phaseA.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, 0, phaseB.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, 0, phaseB.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, 0, phaseC.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, 0, phaseC.mode);

    // Configure signal propagation to pads
    #define portConfigRegister(pin)  ((digital_pin_to_info_PGM[(pin)].mux))
    *(portConfigRegister(kGateAH)) =  1;  // GATE AH
    *(portConfigRegister(kGateAL)) =  1;  // GATE AL
    *(portConfigRegister(kGateBH)) =  2;  // GATE BH
    *(portConfigRegister(kGateBL)) =  2;  // GATE BL
    *(portConfigRegister(kGateCH)) =  6;  // GATE CH
    *(portConfigRegister(kGateCL)) =  6;  // GATE CL
    
    //Enable outputs in A and B 
    IMXRT_FLEXPWM2.OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << 3) | FLEXPWM_OUTEN_PWMB_EN(1 << 3) 
                         | FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2)
                         | FLEXPWM_OUTEN_PWMA_EN(1) | FLEXPWM_OUTEN_PWMB_EN(1);

    delay(1);
}

void Bldc::configureADC() {
    uint32_t mode = ADC_CFG_ADICLK(0b00) | ADC_CFG_MODE(0b10) | ADC_CFG_ADLSMP | ADC_CFG_ADIV(0b00) | ADC_CFG_ADSTS(0b11) | ADC_CFG_AVGS(0b10) | ADC_CFG_OVWREN;
    uint32_t avg = ((ADC_GC_AVGE) & (~ADC_GC_ADCO)) | ADC_GC_CAL;

    // Configure ADC1
    ADC1_CFG = mode;
    ADC1_GC = avg;
    while (ADC1_GC & ADC_GC_CAL) {
        yield();  // Wait until calibration is complete
    }

    mode &= ~ADC_CFG_ADLSMP;
    avg &= ~ADC_GC_AVGE;

    // Configure ADC2
    ADC2_CFG = mode;
    ADC2_GC = avg;
    while (ADC2_GC & ADC_GC_CAL) {
        yield();  // Wait until calibration is complete
    }

    // Register ADC interrupt
    attachInterruptVector(IRQ_ADC1, ADC1_CompletedConversionCallback);
    NVIC_ENABLE_IRQ(IRQ_ADC1);

    attachInterruptVector(IRQ_ADC2, ADC2_CompletedConversionCallback);
    NVIC_ENABLE_IRQ(IRQ_ADC2);

}

void Bldc::readHalls(uint8_t &hall) {
    uint8_t hallCountsC = 0, hallCountsB = 0, hallCountsA = 0;
    
    for (uint8_t i = 0; i < kHallOverSample; i++) {
        hallCountsC += digitalRead(kHallCPin);
        hallCountsB += digitalRead(kHallBPin);
        hallCountsA += digitalRead(kHallAPin);
    }
    
    hall = ((hallCountsC > kHallOverSample / 2) << 2) |
           ((hallCountsB > kHallOverSample / 2) << 1) |
           ((hallCountsA > kHallOverSample / 2) << 0);
}


void Bldc::nextStep(uint8_t &currentHallState) {
    /** Override this method */
    // #ifdef SERIAL_DEBUG
    //     Serial.println("Override nextStep method");
    // #endif
}

void Bldc::setGatePWM(Phase phase){
  if (phase.phaseID == 1)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 0) & 3, 2, phase.pwmVal, phase.mode);
    // #ifdef SERIAL_DEBUG
    //     Serial.print("\tA> ");
    //     Serial.print(phase.pwmVal);
    // #endif
  }
  if (phase.phaseID == 2)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 2) & 3, 2, phase.pwmVal, phase.mode);
    // #ifdef SERIAL_DEBUG
    //     Serial.print(" B> ");
    //     Serial.print(phase.pwmVal);
    // #endif
  }
  if (phase.phaseID == 3)
  {
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 1, phase.pwmVal, phase.mode);
    writePwmValue(&IMXRT_FLEXPWM2, M(2, 3) & 3, 2, phase.pwmVal, phase.mode);
    // #ifdef SERIAL_DEBUG
    //     Serial.print(" C> ");
    //     Serial.println(phase.pwmVal);
    // #endif
  }
}

void Bldc::setPhaseDuty(int16_t phaseApwm, int16_t phaseBpwm, int16_t phaseCpwm){
    if (phaseApwm > 1){ 
        phaseA.pwmVal = 4096 - phaseApwm;
        phaseA.mode = Phase::Mode::Complementary;
        setGatePWM(phaseA); 
    } else if (phaseApwm == 0){ 
        phaseA.pwmVal = -1;
        phaseA.mode = Phase::Mode::X;
        setGatePWM(phaseA); 
    } else { 
        phaseA.pwmVal = -1;
        phaseA.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseA); 
    }
    
    if (phaseBpwm > 1){ 
        phaseB.pwmVal =  4096 - phaseBpwm;
        phaseB.mode = Phase::Mode::Complementary;
        setGatePWM(phaseB); 
    } else if (phaseBpwm == 0){ 
        phaseB.pwmVal = -1;
        phaseB.mode = Phase::Mode::X;
        setGatePWM(phaseB); 
    } else { 
        phaseB.pwmVal = -1;
        phaseB.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseB); 
    }

    if (phaseCpwm > 1){ 
        phaseC.pwmVal = 4096 - phaseCpwm;
        phaseC.mode = Phase::Mode::Complementary;
        setGatePWM(phaseC); 
    } else if (phaseCpwm == 0){ 
        phaseC.pwmVal = 0;
        phaseC.mode = Phase::Mode::X;
        setGatePWM(phaseC); 
    } else { 
        phaseC.pwmVal = -1;
        phaseC.mode = Phase::Mode::ZeroComplement;
        setGatePWM(phaseC); 
    }
}

void Bldc::normThrottle(uint16_t &throttle) {
    uint16_t throttleRaw = analogRead(kThrottlePin);
    throttleRaw = constrain(throttleRaw, kThrottleLow, kThrottleHigh);
    throttle = map(throttleRaw, kThrottleLow, kThrottleHigh, 0, kThrottleResolution);
}

void Bldc::readCurrents(int16_t &currentA, int16_t &currentB, int16_t &currentC) {
    currentA = analogRead(kCurrentSenseA);
    currentB = analogRead(kCurrentSenseB);
    currentC = analogRead(kCurrentSenseC);
}

void Bldc::setPwmFrequency(IMXRT_FLEXPWM_t *p, uint8_t submodule, uint8_t channel, float frequency) {
    uint16_t mask = 1 << submodule;
    uint32_t newdiv = static_cast<uint32_t>((F_BUS_ACTUAL / frequency) + 0.5f);
    uint8_t prescale = 0;

    while (newdiv > 65535 && prescale < 7) {
        newdiv >>= 1;
        prescale++;
    }

    newdiv = constrain(newdiv, 2, 65535);

    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    p->SM[submodule].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);
    p->SM[submodule].INIT = -(newdiv / 2);
    p->SM[submodule].VAL1 = (newdiv / 2);
    p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}

void Bldc::writePwmValue(IMXRT_FLEXPWM_t *p, uint8_t submodule, uint8_t channel, int16_t value, Phase::Mode mode) {
    if (mode == Phase::Mode::Complementary || mode == Phase::Mode::ZeroComplement){
        IMXRT_FLEXPWM2.SM[submodule & 0xF].OCTRL &= ~FLEXPWM_SMOCTRL_POLB; 
    } else{
        IMXRT_FLEXPWM2.SM[submodule & 0xF].OCTRL |= FLEXPWM_SMOCTRL_POLB; 
    }
    
    uint16_t mask = 1 << submodule;
    uint32_t modulo = p->SM[submodule].VAL1 * 2;
    uint32_t cval = ((uint32_t)value * (modulo + 1)) >> 12;

    p->MCTRL |= FLEXPWM_MCTRL_CLDOK(mask);
    switch (channel) {
        case 0:  // X
            p->SM[submodule].VAL0 = modulo - cval;
            p->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(mask);
            break;
        case 1:  // A
            if (mode == Phase::Mode::Complementary) {
                p->SM[submodule].VAL2 = -(cval / 2);
                p->SM[submodule].VAL3 = (cval / 2)+1;
            } else if (mode == Phase::Mode::ZeroComplement) {
                p->SM[submodule].VAL2 = p->SM[submodule].INIT;
                p->SM[submodule].VAL3 = p->SM[submodule].VAL1;
            } else {
                p->SM[submodule].VAL2 = p->SM[submodule].INIT;
                p->SM[submodule].VAL3 = p->SM[submodule].INIT;
            }
                p->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(mask);
            
            break;
        case 2:  // B
            if (mode == Phase::Mode::Complementary) {
                p->SM[submodule].VAL4 = -(cval / 2);
                p->SM[submodule].VAL5 = (cval / 2)+1;
            } else if (mode == Phase::Mode::ZeroComplement) {
                p->SM[submodule].VAL4 = p->SM[submodule].INIT;
                p->SM[submodule].VAL5 = p->SM[submodule].VAL1;
            } else {
                p->SM[submodule].VAL4 = p->SM[submodule].INIT;
                p->SM[submodule].VAL5 = p->SM[submodule].INIT;
            }
                p->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(mask);
            break;
    }
    p->MCTRL |= FLEXPWM_MCTRL_LDOK(mask);
}

void Bldc::validateRpm(){
    unsigned long now = micros();
    if (now - lastHallChangeTime > RPM_TIMEOUT_US) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            rpmBuffer[i] = 0.0f;
        }
        rpmBufferCount = 0;
        rpmBufferIndex = 0;
        rpm = 0.0f;
    }
}


float Bldc::hallToAngle(uint8_t hall) {
    // Map hall sensor reading (in CBA order) to rotor angle (in radians)
    switch(hall) {
        case 1:  // 001
            return 0.0f;
        case 3:  // 011
            return M_PI / 3.0f; // 60° in radians
        case 2:  // 010
            return 2.0f * M_PI / 3.0f; // 120°
        case 6:  // 110
            return M_PI; // 180°
        case 4:  // 100
            return 4.0f * M_PI / 3.0f; // 240°
        case 5:  // 101
            return 5.0f * M_PI / 3.0f; // 300°
        default:
            return 0.0f;
    }
}

float Bldc::estimatePosition() {   
    unsigned long now = micros();
    // Compute time delta in seconds since the last position update.
    float dt = (now - lastPosUpdateTime) / 1000000.0f;
    lastPosUpdateTime = now; 
    if(newHallUpdate) {
         rotorPos = hallToAngle(hallState);
         newHallUpdate = false;
    } else {
         float velocity_rad_sec = rpm;
         rotorPos += velocity_rad_sec * dt;
    }
    
    // Wrap rotorPos between 0 and 2π.
    rotorPos = fmod(rotorPos, 2.0f * M_PI);
    if (rotorPos < 0) {
         rotorPos += 2.0f * M_PI;
    }
    
    return rotorPos;
}

double Bldc::computePID(PIDController_t &pid, double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;
    pid.integral += error * dt;
    double derivative = (error - pid.prevError) / dt;
    double output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prevError = error;
    return output;
}

// Trigger ADC conversion for Throttle on ADC1 channel 1
void triggerADC_Throttle() {
    uint8_t ch = ADC_HC_ADCH(1) | ADC_HC_AIEN; // Set channel and enable interrupt
    ADC1_HC0 = ch;
}

// Trigger ADC conversion for battery voltage on ADC1 channel 10
void triggerADC_VBat() {
    uint8_t ch = ADC_HC_ADCH(10) | ADC_HC_AIEN; // Set channel and enable interrupt
    ADC1_HC0 = ch;
}

void triggerADC_CurrentA() {
    uint8_t ch = 0x03 | (1 << 7);  
    ADC2_HC0 = ch;
}

void triggerADC_CurrentB() {
    uint8_t ch = 0x04 | (1 << 7);  
    ADC2_HC0 = ch;
}

// Trigger ADC conversion for Current Sensor C on ADC2 channel (HC2)
void triggerADC_CurrentC() {
    uint8_t ch = 0x01 | (1 << 7);  
    ADC2_HC0 = ch;
}


void toggleLed(){
  digitalWrite(kBuiltInLedPin, !digitalRead(kBuiltInLedPin));
}   

void toggleFlag(){
  digitalWrite(kIrqFlagPin, !digitalRead(kIrqFlagPin));
}

