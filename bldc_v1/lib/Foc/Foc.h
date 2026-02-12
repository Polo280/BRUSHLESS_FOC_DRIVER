#ifndef FOC_H
#define FOC_H

#include "Bldc.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

const double angles[] = {
    0.78539816339745, 0.46364760900081, 0.24497866312686, 0.12435499454676,
    0.06241880999596, 0.03123983343027, 0.01562372862048, 0.00781234106010,
    0.00390623013197, 0.00195312251648, 0.00097656218956, 0.00048828121119,
    0.00024414062015, 0.00012207031189, 0.00006103515617, 0.00003051757812,
    0.00001525878906, 0.00000762939453, 0.00000381469727, 0.00000190734863,
    0.00000095367432, 0.00000047683716, 0.00000023841858, 0.00000011920929,
    0.00000005960464, 0.00000002980232, 0.00000001490116, 0.00000000745058
};

const double kvalues[] = {
    0.70710678118655, 0.63245553203368, 0.61357199107790, 0.60883391251775,
    0.60764825625617, 0.60735177014130, 0.60727764409353, 0.60725911229889,
    0.60725447933256, 0.60725332108988, 0.60725303152913, 0.60725295913894,
    0.60725294104140, 0.60725293651701, 0.60725293538591, 0.60725293510314,
    0.60725293503245, 0.60725293501477, 0.60725293501035, 0.60725293500925,
    0.60725293500897, 0.60725293500890, 0.60725293500889, 0.60725293500888
};

class Foc : public Bldc {
public:

    typedef struct
    {
        uint32_t pwmA;
        uint32_t pwmB;
        uint32_t pwmC;
    } Phases_t;

    typedef struct {
        float sin;
        float cos;
    } SinCosCalc_t;

    typedef struct {
        float alpha;
        float beta;
    } ClarkeTransform_t;

    typedef struct {
        float d;  
        float q; 
    } ParkTransform_t;

    typedef struct {
        float iAlpha;
        float iBeta;
    } InversePark_t;

    typedef struct {
        double fdbackA;
        double fdbackB;
        double fdbackC;
    } InverseClarke_t;

    Foc();
    ~Foc();
    void run() override;
    
private:
    Phases_t phases;
    PIDController_t pid_d;
    PIDController_t pid_q;
    
    float lastRotorAngle;
    unsigned long lastRunTime;
    
    ClarkeTransform_t clarkeTransform(float a, float b, float c);
    ParkTransform_t parkTransform(float alph, float bet, float theta);
    InversePark_t inverseParkTransform(float d, float q, float theta);
    InverseClarke_t inverseClarkeTransform(float alpha, float beta);
    
    SinCosCalc_t trigCalc(double angle, int n);

    void cordic(double angle, int n, double *cosval, double *sinval);
    float getRotorAngle();
    float measureVelocity(float dt);
};

#endif // FOC_H
