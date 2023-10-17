/*
PID source: https://www.arxterra.com/lecture-6-pid-controllers/
Antiwindup source: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-reset-windup/

Proportional, Integral, Derivative, and Anti-Windup.
Error based.
Precomputes values based off constant and known cycle frequency.
*/ 

#include "joyride_control_lib/pid.h"

PID::PID(double cycleFreq) {
        this->cycleTime = 1 / cycleFreq;
}

void PID::setTuning(double kp, double ki, double kd) {
    this->kpMod = kp;
    this->kiMod = ki * this->cycleTime;
    this->kdMod = kd / this->cycleTime;
}

void PID::setAntiWindup(double minOutput, double maxOutput, double iTermMin, double iTermMax) {
    this->antiWindupSet = true;

    this->outMin = minOutput;
    this->outMax = maxOutput;
    this->iTermMin = iTermMin;
    this->iTermMax = iTermMax;
}

// Computes output given error.
double PID::loop(double error) {

    this->pTerm = this->kpMod * error;
    
    this->iTerm += this->kiMod * error;

    this->dTerm = this->kdMod * (error - this->lastError);
    this->lastError = error;

    // If not using windup protection
    if (!this->antiWindupSet) {
        this->out = this->pTerm + this->iTerm - this->dTerm;
        return this->out;
    }

    // Windup protection -> limit iTerm
    if      (this->iTerm > this->iTermMax) this->iTerm = this->iTermMax;
    else if (this->iTerm < this->iTermMin) this->iTerm = this->iTermMin;

    this->out = this->pTerm + this->iTerm - this->dTerm;

    // Windup protection -> limit output
    if      (this->out > this->outMax) this->out = this->outMax;
    else if (this->out < this->outMin) this->out = this->outMin;

    return  this->out;
}
