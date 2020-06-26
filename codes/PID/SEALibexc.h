/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Controller : PID Controller
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 19.11.2018
*    Library version: v1.0
*    Test Status: Tested at 19.11.2018
***************************************************************************************/

#include "SEALibgen.h"

void getExceptionalParameters(void);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);
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
