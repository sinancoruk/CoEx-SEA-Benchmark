/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Controller : FF+PID+DOB Controller
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 19.11.2018
*    Library version: v1.0
***************************************************************************************/


#include "SEALibexc.h"

void getExceptionalParameters(void){

    experiment.refAmplitude = 0.2;
    experiment.refStepTime = 2;
    experiment.refFreq = 5;
    experiment.endTime = 10;
    experiment.MotorEmergency = 0;

    Butterworth.num[0] = 0.092716782379214032761;
    Butterworth.num[1] = 0.18543356475842806552;
    Butterworth.num[2] = 0.092716782379214032761;
    Butterworth.denum[0] = -0.97352621498174729542;
    Butterworth.denum[1] = 0.34439334449860353748;
    ButterworthPn.num[0] = 41485.044806603553297;
    ButterworthPn.num[1] = -123585.94285484103602;
    ButterworthPn.num[2] = 123474.70241795042239;
    ButterworthPn.num[3] = -41371.816954745903786;
    ButterworthPn.denum[0] = -1.973490501333776681;
    ButterworthPn.denum[1] = 1.3178847913078186593;
    ButterworthPn.denum[2] = -0.3443810449559347453;
    CFFPn.num[0] = 8246.0595708487271622;
    CFFPn.num[1] = -24565.407887381788896;
    CFFPn.num[2] = 24543.296418693167652;
    CFFPn.num[3] = -8223.5530599880039517;
    CFFPn.denum[0] = -2.4569432936974968662;
    CFFPn.denum[1] = 1.9876239302714444612;
    CFFPn.denum[2] = -0.53067800383704599732;


    Butterworth.output[0] = 0;
    Butterworth.input[0] = 0;
    Butterworth.output[1] = 0;
    Butterworth.input[1] = 0;
    Butterworth.output[2] = 0;
    Butterworth.input[2] = 0;

    ButterworthPn.output[0] = 0;
    ButterworthPn.input[0] = 0;
    ButterworthPn.output[1] = 0;
    ButterworthPn.input[1] = 0;
    ButterworthPn.output[2] = 0;
    ButterworthPn.input[2] = 0;
    ButterworthPn.output[3] = 0;
    ButterworthPn.input[3] = 0;
    ButterworthPn.output[4] = 0;
    ButterworthPn.input[4] = 0;

    CFFPn.output[0] = 0;
    CFFPn.input[0] = 0;
    CFFPn.output[1] = 0;
    CFFPn.input[1] = 0;
    CFFPn.output[2] = 0;
    CFFPn.input[2] = 0;
    CFFPn.output[3] = 0;
    CFFPn.input[3] = 0;
    CFFPn.output[4] = 0;
    CFFPn.input[4] = 0;


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

double fCFF(double Qdref){

  /* Cff block */
  double fCFFOut;

  /* Diff equation */
  CFFPn.input[0] = Qdref;
  fCFFOut = CFFPn.num[0] * CFFPn.input[0] + CFFPn.num[1] *  CFFPn.input[1]  + CFFPn.num[2] *  CFFPn.input[2] + CFFPn.num[3] * CFFPn.input[3];
  fCFFOut = fCFFOut - CFFPn.denum[0] * CFFPn.output[0] - CFFPn.denum[1] * CFFPn.output[1] - CFFPn.denum[2] * CFFPn.output[2];

  /* Update states */
  CFFPn.input[3] = CFFPn.input[2];
  CFFPn.input[2] = CFFPn.input[1];
  CFFPn.input[1] =  CFFPn.input[0];
  CFFPn.output[2] = CFFPn.output[1];
  CFFPn.output[1] = CFFPn.output[0];
  CFFPn.output[0] = fCFFOut;

  return fCFFOut;

}

double fDOB(double Tm, double Qd){

  /* Cff block */
  double fDOBOut1;

  /* Diff equation */
  ButterworthPn.input[0] = Qd;
  fDOBOut1 = ButterworthPn.num[0] * ButterworthPn.input[0] + ButterworthPn.num[1] *  ButterworthPn.input[1]  + ButterworthPn.num[2] *  ButterworthPn.input[2] + ButterworthPn.num[3] * ButterworthPn.input[3];
  fDOBOut1 = fDOBOut1 - ButterworthPn.denum[0] * ButterworthPn.output[0] - ButterworthPn.denum[1] * ButterworthPn.output[1] - ButterworthPn.denum[2] * ButterworthPn.output[2];

  /* Update states */
  ButterworthPn.input[3] = ButterworthPn.input[2];
  ButterworthPn.input[2] = ButterworthPn.input[1];
  ButterworthPn.input[1] =  ButterworthPn.input[0];
  ButterworthPn.output[2] = ButterworthPn.output[1];
  ButterworthPn.output[1] = ButterworthPn.output[0];
  ButterworthPn.output[0] = fDOBOut1;


  double fDOBOut2;

  /* Diff equation */
  Butterworth.input[0] = Tm;
  fDOBOut2 = Butterworth.num[0] * Butterworth.input[0] + Butterworth.num[1] *  Butterworth.input[1]  + Butterworth.num[2] *  Butterworth.input[2];
  fDOBOut2 = fDOBOut2 - Butterworth.denum[0] * Butterworth.output[0] - Butterworth.denum[1] * Butterworth.output[1];

  /* Update states */
  Butterworth.input[2] = Butterworth.input[1];
  Butterworth.input[1] =  Butterworth.input[0];
  Butterworth.output[1] = Butterworth.output[0];
  Butterworth.output[0] = fDOBOut2;

  return (fDOBOut1-fDOBOut2);

}
