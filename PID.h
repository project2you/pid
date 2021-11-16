#ifndef _PID_H_
#define _PID_H_
#include <Arduino.h>
#define MANUAL_MODE 0
#define AUTO_MODE 1

class PID
{
public:
    PID(float Kc, float tauI, float tauD, float interval);
    void setInputLimits(float inMin, float inMax);
    void setOutputLimits(float outMin, float outMax);
    void setTunings(float Kc, float tauI, float tauD);
    void reset(void);
    void setMode(int mode);
    void setInterval(float interval);
    void setSetPoint(float sp);
    void setProcessValue(float pv);
    void setBias(float bias);
    float compute();

    //  private:
    bool usingFeedForward;
    bool inAuto;

    //Actual tuning parameters used in PID calculation.
    float Kc_;
    float tauR_;
    float tauD_;

    //Raw tuning parameters.
    float pParam_;
    float iParam_;
    float dParam_;

    //The point we want to reach.
    float setPoint_;

    //The thing we measure.
    float processVariable_;
    float prevProcessVariable_;

    //The output that affects the process variable.
    float controllerOutput_;
    float prevControllerOutput_;

    //We work in % for calculations so these will scale from
    //real world values to 0-100% and back again.
    float inMin_;
    float inMax_;
    float inSpan_;
    float outMin_;
    float outMax_;
    float outSpan_;

    //The accumulated error, i.e. integral.
    float accError_;
    //The controller output bias.
    float bias_;

    //The interval between samples.
    float tSample_;

    //Controller output as a real world value.
    volatile float realOutput_;
};

PID::PID(float Kc, float tauI, float tauD, float interval)
{
    usingFeedForward = false;
    inAuto = false;
    setInputLimits(0, 3.3);
    setOutputLimits(0, 100.0);

    tSample_ = interval;
    setTunings(Kc, tauI, tauD);

    setPoint_ = 0.0;
    processVariable_ = 0.0;
    prevProcessVariable_ = 0.0;
    controllerOutput_ = 0.0;
    prevControllerOutput_ = 0.0;

    accError_ = 0.0;
    bias_ = 0.0;
    realOutput_ = 0.0;
}

void PID::setInputLimits(float inMin, float inMax)
{
    prevProcessVariable_ *= (inMax - inMin) / inSpan_;
    accError_ *= (inMax - inMin) / inSpan_;
    prevProcessVariable_ = constrain(prevProcessVariable_, 0, 1);

    inMin_ = inMin;
    inMax_ = inMax;
    inSpan_ = inMax - inMin;
}

void PID::setOutputLimits(float outMin, float outMax)
{
    prevControllerOutput_ *= (outMax - outMin) / outSpan_;
    prevControllerOutput_ = constrain(prevControllerOutput_, 0, 1);

    outMin_ = outMin;
    outMax_ = outMax;
    outSpan_ = outMax - outMin;
}

void PID::setTunings(float Kc, float tauI, float tauD)
{

    if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0)
    {
        return;
    }

    pParam_ = Kc;
    iParam_ = tauI;
    dParam_ = tauD;

    float tempTauR;

    if (tauI == 0.0)
    {
        tempTauR = 0.0;
    }
    else
    {
        tempTauR = (1.0 / tauI) * tSample_;
    }

    if (inAuto)
    {
        if (tempTauR == 0.0)
        {
            accError_ = 0.0;
        }
        else
        {
            accError_ *= (Kc_ * tauR_) / (Kc * tempTauR);
        }
    }

    Kc_ = Kc;
    tauR_ = tempTauR;
    tauD_ = tauD / tSample_;
}

void PID::reset(void)
{
    float scaledBias = 0.0;

    if (usingFeedForward)
    {
        scaledBias = (bias_ - outMin_) / outSpan_;
    }
    else
    {
        scaledBias = (realOutput_ - outMin_) / outSpan_;
    }

    prevControllerOutput_ = scaledBias;
    prevProcessVariable_ = (processVariable_ - inMin_) / inSpan_;

    accError_ = 0;
}

void PID::setMode(int mode)
{
    if (mode != 0 && !inAuto)
    {
        reset();
    }

    inAuto = (mode != 0);
}
void PID::setInterval(float interval)
{
    if (interval > 0)
    {
        tauR_ *= (interval / tSample_);
        accError_ *= (tSample_ / interval);
        tauD_ *= (interval / tSample_);
        tSample_ = interval;
    }
}

void PID::setSetPoint(float sp)
{
    setPoint_ = sp;
}

void PID::setProcessValue(float pv)
{
    processVariable_ = pv;
}

void PID::setBias(float bias)
{
    bias_ = bias;
    usingFeedForward = 1;
}

float PID::compute()
{

    float scaledPV = constrain((processVariable_ - inMin_) / inSpan_, 0.0, 1.0);
    float scaledSP = constrain((setPoint_ - inMin_) / inSpan_, 0.0, 1.0);

    float error = scaledSP - scaledPV;

    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= 0 && error < 0))
    {
        accError_ += error;
    }

    //Compute the current slope of the input signal.
    float dMeas = (scaledPV - prevProcessVariable_) / tSample_;

    float scaledBias = 0.0;

    if (usingFeedForward)
    {
        scaledBias = (bias_ - outMin_) / outSpan_;
    }

    controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
    controllerOutput_ = constrain(controllerOutput_, 0.0, 1.0);

    prevControllerOutput_ = controllerOutput_;
    prevProcessVariable_ = scaledPV;

    return ((controllerOutput_ * outSpan_) + outMin_);
}

#endif
