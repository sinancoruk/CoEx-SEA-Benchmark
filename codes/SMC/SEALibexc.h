/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library for Sliding Mode Controller (SEALibexc)
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 23.07.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibgen.h"

void getExceptionalParameters(void);
double FricCBlock(double inPart);
double chirp(double w1, double w2, double A, double M, double time);
double chirpS(double w1, double w2, double A, double M, double time);
double chirpSDot(double w1, double w2, double A, double M, double time);
double chirpSDotDot(double w1, double w2, double A, double M, double time);
double DOBBlock(double motorVelocity, double motorTorque);
double SMCCompute(double TdRef, double TdRefDot, double TdRefDotDot, double C, double P);

/* SMC controller struct */
typedef struct SMCController_{
  double QmRef;
  double QmDotRef;
  double QmDotDotRef;
  double error;
  double errorDot;
  double sigma;
  double d;
}SMCController;
SMCController SMC;
