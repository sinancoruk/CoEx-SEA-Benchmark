/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Cascaded PID + DoB Controller 
*
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 11.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibgen.h"

void getExceptionalParameters(void);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);
double fPID(double Tdref, double Td, double TdDotref, double TdDot);
double fPI(double QmDotRef, double QmDot);
double fDOB(double QmDot, double Tm);
double fQDLPF(double Qd);
double TorsionLPFBlock(double inPart);


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
PIDController PIX;

/* Implementing transfer function in a form of tustin blocks (DOB,CFF,PID,LPF,HPF) */
typedef struct tustinBlockX_{
    double num[6];
    double denum[6];
    double output[6];
    double input[6];
		double freq;
}tustinBlockX;
tustinBlockX DOB; // LPF approx diff
tustinBlockX TorsionLPF;
