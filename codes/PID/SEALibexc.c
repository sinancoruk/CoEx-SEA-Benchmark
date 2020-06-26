/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Controller : PID Controller
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 19.11.2018
*    Library version: v1.0
*    Test Status: Tested at 19.11.2018
***************************************************************************************/
#include "SEALibexc.h"

void getExceptionalParameters(void){

    experiment.refAmplitude = 0.2;
    experiment.refStepTime = 2;
    experiment.refFreq = 5;
    experiment.endTime = 10;
    experiment.MotorEmergency = 0;

    PID.KpGain = 100;
    PID.KiGain = 100;
    PID.KdGain = 14;

    PID.IntegralTerm = 0;
    PID.error[0] = 0;
    PID.error[1] = 0;
    PID.errorDot = 0;
    PID.IntegratorNum = 0.0005*0.5;

}

double chirp(double w1, double w2, double A, double M, double time){
  /* This funtion implements Chirp signal */
  double res;
  res=A*cos(w1*time+(w2-w1)*time*time/(2*M));
  return res;
}

double chirpS(double w1, double w2, double A, double M, double time){
  /* This funtion implements Chirp signal */
  double res;
  res=A*sin(w1*time+(w2-w1)*time*time/(2*M));
  return res;
}

double fPID(double Qdref, double Qd, double QdDotref, double QdDot){

  PID.error[0] = Qdref-Qd;
  PID.errorDot = QdDotref-QdDot;
  PID.IntegralTerm = PID.IntegralTerm + PID.IntegratorNum*(PID.error[0]+PID.error[1]);

  return PID.KpGain*PID.error[0] + PID.KiGain*PID.IntegralTerm + PID.KdGain*PID.errorDot;

}