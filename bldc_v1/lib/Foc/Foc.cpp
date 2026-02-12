#include "Foc.h"


Foc::Foc() {
    /*Velocity*/
    pid_vel.Kp = 1100.0; 
    pid_vel.Ki = 110.0; 
    pid_vel.Kd = 0.0;
    
    pid_vel.prevError = 0.0; 
    pid_vel.integral = 0.0;

    /*Phases values*/
    phases.pwmA = 0;
    phases.pwmB = 0;
    phases.pwmC = 0;
    
    /*Control id currents*/
    pid_d.Kp = 10.0; 
    pid_d.Ki = 5.0; 
    pid_d.Kd = 0.0;
    
    pid_d.prevError = 0.0; 
    pid_d.integral = 0.0;
    
    /*Control iq currents*/
    pid_q.Kp = 10.0; 
    pid_q.Ki = 5.0; 
    pid_q.Kd = 0.0;
    
    pid_q.prevError = 0.0; 
    pid_q.integral = 0.0;

    /*dt*/
    lastRunTime = micros();
}

Foc::~Foc() {}


Foc::ClarkeTransform_t Foc::clarkeTransform(float a, float b, float c) {
    ClarkeTransform_t ct;
    ct.alpha = (2.0f/3.0f) * (a - 0.5f*b - 0.5f*c);
    ct.beta  = (2.0f/3.0f) * ((sqrt(3.0f)/2.0f)*b - (sqrt(3.0f)/2.0f)*c);
    return ct;
}

Foc::ParkTransform_t Foc::parkTransform(float alph, float bet, float theta) {
    ParkTransform_t pt;
    SinCosCalc_t result = trigCalc(theta, 16);
    pt.d = alph*result.cos + bet*result.sin;
    pt.q = -alph*result.sin + bet*result.cos;
    return pt;
}

Foc::InversePark_t Foc::inverseParkTransform(float d, float q, float theta) {
    InversePark_t ip;
    SinCosCalc_t result = trigCalc(theta, 16);
    ip.iAlpha = d*result.cos - q*result.sin; 
    ip.iBeta =  d*result.sin + q*result.cos;
    return ip;
}

Foc::InverseClarke_t Foc::inverseClarkeTransform(float alpha, float beta) {
    InverseClarke_t ic;
    ic.fdbackA = alpha;
    ic.fdbackB = -0.5f * alpha + (sqrt(3.0f)/2.0f)*beta;
    ic.fdbackC = -0.5f * alpha - (sqrt(3.0f)/2.0f)*beta;
    return ic;
}

Foc::SinCosCalc_t Foc::trigCalc(double angle, int n){
    SinCosCalc_t scc;
    double coscalc, sincalc;
    cordic(angle, n, &coscalc, &sincalc);
    scc.cos = coscalc;
    scc.sin = sincalc; 
    return scc;
}

void Foc::cordic(double angle, int n, double *cosval, double *sinval){
    int i, ix, sigma;
    double kn, x, y, atn, t, theta = 0.0, pow2 = 1.0;
    int newsgn = (int)floor(angle / (2.0 * M_PI)) % 2 == 1 ? 1 : -1;
    

    if (angle < -M_PI/2.0 || angle > M_PI/2.0){
        if(angle < 0){
            cordic(angle+M_PI, n, &x, &y);
            
        } else {
            cordic(angle-M_PI, n, &x, &y);
        }
        *cosval = x * newsgn;
        *sinval = y * newsgn;
        return;
    }
    ix = n - 1;
    if(ix > 23) ix = 23;
    kn = kvalues[ix];
    x = 1;
    y =0 ;
    
    for (i = 0; i < n; ++i) {
        atn = angles[i];
        sigma = (theta < angle) ? 1 : -1;
        theta += sigma * atn;
        t = x;
        x -= sigma * y * pow2;
        y += sigma * t * pow2;
        pow2 /= 2.0;
    }
    *cosval = x * kn;
    *sinval = y * kn;    
}

float Foc::getRotorAngle() {
    unsigned long timeNow = millis();
    float seconds = timeNow / 1000.0;
    float frequency = 2.61;
    float angle = fmod(2.0 * M_PI * frequency * seconds, 2.0 * M_PI);
    return angle;
}

float Foc::measureVelocity(float dt) {
    float currentAngle = getRotorAngle();
    float deltaAngle = currentAngle - lastRotorAngle;
    if (deltaAngle > M_PI) {
        deltaAngle -= 2.0 * M_PI;
    } else if (deltaAngle < -M_PI) {
        deltaAngle += 2.0 * M_PI;
    }
    lastRotorAngle = currentAngle;
    return deltaAngle / dt;
}

void Foc::run() {
    if (newThrottleVal) {
        uint16_t throttle;
        normThrottle(throttle);
        throttleNormVal = throttle;
        newThrottleVal = false;
    }
    
    if (newCycle) {
        //setPhaseDuty(phases.pwmA, phases.pwmB, phases.pwmC);        
        validateRpm();
        Serial.println(rpm, 6);
        newCycle = false;
    }
    if (newCurrentC)
    {
        unsigned long now = micros();
        float dt = (now - lastRunTime) / 1000000.0f;
        lastRunTime = now;

        // Velocity sht
        const float MAX_VELOCITY = 25;
        float desiredVelocity = ((float)throttleNormVal / 4096.0) * MAX_VELOCITY;
        
        // Update by irq
        int16_t curA = currentA;
        int16_t curB = currentB; 
        int16_t curC = currentC;

        float a = (float)curA;
        float b = (float)curB;
        float c = (float)curC;
        
        /*Estimate theta*/
        estimatePosition();
        float theta = rotorPos;    

        ClarkeTransform_t ct = clarkeTransform(a, b, c);        
        ParkTransform_t pt = parkTransform(ct.alpha, ct.beta, theta);
        
        double setpoint_d = 0.0;
        double setpoint_q = 0;

        double iq_ref = computePID(pid_vel, desiredVelocity, rpm, dt);
        
        double v_d = computePID(pid_d, setpoint_d, pt.d, dt);
        double v_q = computePID(pid_q, setpoint_q, pt.q, dt);
        
        InversePark_t ip = inverseParkTransform(v_d, v_q, theta);
        InverseClarke_t ic = inverseClarkeTransform(ip.iAlpha, ip.iBeta);
        
        // Map the commanded voltages to PWM duty cycles.
        const float Vmax = 24.0;
        int16_t dutyA = (int16_t)(constrain((ic.fdbackA + Vmax) / (2.0 * Vmax) * 4096.0, 0, 4096));
        int16_t dutyB = (int16_t)(constrain((ic.fdbackB + Vmax) / (2.0 * Vmax) * 4096.0, 0, 4096));
        int16_t dutyC = (int16_t)(constrain((ic.fdbackC + Vmax) / (2.0 * Vmax) * 4096.0, 0, 4096));

        phases.pwmA = dutyA;
        phases.pwmB = dutyB;        
        phases.pwmC = dutyC;        

        newCurrentC = false;
    }
    toggleLed();
}
