/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Cascaded PID Controller
*
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 21.07.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibgen.h"

void getExceptionalParameters(void);
double FricCBlock(double inPart);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);
double PIDBlock(double ref, double outPlant);
double PIBlock(double ref, double outPlant);
double PIDDerivativeBlock(double inPart, double DerivationFreq);
double TDLowPassFilterBlock(double inPart);

/* PID controller struct */
typedef struct PIDController_{
    double KpGain;
    double KiGain;
    double KdGain;
    double IntegralTerm;
    double error[2];
}PIDController;
PIDController PID;
PIDController PIx;
