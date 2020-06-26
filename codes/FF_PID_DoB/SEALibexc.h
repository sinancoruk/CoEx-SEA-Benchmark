/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Controller : FF+PID+DOB Controller
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 19.11.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibgen.h"

tustinBlock CFFPn;
tustinBlock ButterworthPn;
tustinBlock Butterworth;


void getExceptionalParameters(void);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);

double fDOB(double Tm, double Qd);
double fCFF(double Qdref);
double fPID(double Qdref, double Qd, double QdDotref, double QdDot);


/* PID controller struct */
typedef struct PIDController_{
    double KpGain;
    double KiGain;
    double KdGain;
    double IntegralTerm;
    double error[2];
    double errorDot;
    double IntegratorNum;
}PIDController;
PIDController PID;
