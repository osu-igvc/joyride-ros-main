#ifndef PID_H
#define PID_H

/// Standard PID (proportional, integral, derivative) controller. 
/// Derivative component is filtered using an exponential moving average filter.
class PID {
  public:
    PID(double cycleFreq);
    void setTuning(double kp, double ki, double kd);
    void setAntiWindup(double minOutput, double maxOutput, double iTermMin, double iTermMax); 
    double loop(double error);

  private:
    double kpMod = 0.;
    double kdMod = 0.;
    double kiMod = 0.;
    double cycleTime;

    double pTerm = 0.;
    double iTerm = 0.;
    double dTerm = 0.;

    double lastError = 0.;

    bool antiWindupSet = false;
    double outMin;
    double outMax;
    double iTermMin;
    double iTermMax;
    
    double out = 0.;
    };

#endif