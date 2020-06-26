/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Includes:
*    1- Higher order disturbance observer (Sariyildiz DF)
*
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 16.11.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibgen.h"

tustinBlock TorsionLPF;

void getExceptionalParameters(void);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);
double chirpSDot(double w1, double w2, double A, double M, double time);
double chirpSDotDot(double w1, double w2, double A, double M, double time);
void DOBUpdate(double Tm);
double DFUpdate(double TdRef, double TdRefDot, double TdRefDotDot);
double Der1Block(double inPart);
double Der2Block(double inPart);
double fPID1(double Qdref, double Qd);
double fPID2(double Qdref, double Qd);
double fPID1Der(double inPart);
double fPID2Der(double inPart);
double TorsionLPFBlock(double inPart);


/* DOB struct */
typedef struct DisturbanceObserver_{
	double IntegratorNum;
	double est_z1[4];
	double est_z2[4];
	double est_z3[4];
	double est_dz1[4];
	double est_dz2[4];
	double est_dz3[4];
	double est_dz1_pre[4];
	double est_dz2_pre[4];
	double est_dz3_pre[4];
	double est_Tdis[4];
	double est_dTdis[4];
	double est_ddTdis[4];
	double LA[3];
	double LB[3];
	double Tempx[4];
	double states[4];
	double A[4][4];
	double B[4];
	double Tdist[6];
}DisturbanceObserver;
DisturbanceObserver DOB;

/* DF struct */
typedef struct DFStruct_{
	double IntegratorNum;
	double ydfo;
	double ydfo_d;
	double ydfo_dd;
	double ydfo_d_Pre;
	double ydfo_dd_Pre;
	double ydfo_ddd;
	double ydfo_dddd;
	double xRef[4];
	double uRef;
	double Kc[4];
}DFStruct;
DFStruct DF;

typedef struct tustinBlockExc_{
    double num[6];
    double denum[6];
    double output[6];
    double input[6];
    double freq;
}tustinBlockExc;
tustinBlockExc Der1; // LPF approx diff
tustinBlockExc Der2; // LPF approx diff

/* PID controller struct */
typedef struct PIDController_{
    double KpGain;
    double KiGain;
    double KdGain;
    double IntegralTerm;
    double error[2];
    double errorDot;
    double IntegratorNum;
		double num[6];
		double denum[6];
		double output[6];
		double input[6];
		double freq;
}PIDController;
PIDController PID1;
PIDController PID2;
