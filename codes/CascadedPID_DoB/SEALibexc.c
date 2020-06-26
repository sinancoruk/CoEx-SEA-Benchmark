/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Cascaded PID + DoB Controller 
*
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 11.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibexc.h"

void getExceptionalParameters(void){

    experiment.refAmplitude = 0.2;
    experiment.refStepTime = 2;
    experiment.refFreq = 5;
    experiment.endTime = 200;
    experiment.MotorEmergency = 0;

    DOB.freq = 600;
    DOB.num[0] = 1/(1 + 2/(DOB.freq*0.0005));
    DOB.num[1] = DOB.num[0];
    DOB.denum[0] = (1 - 2/(DOB.freq*0.0005)) * DOB.num[0];
    DOB.output[0] = 0;
    DOB.input[0] = 0;
    DOB.output[1] = 0;
    DOB.input[1] = 0;

    PID.KpGain = 0;
    PID.KiGain = 0;
    PID.KdGain = 0;

    PID.IntegralTerm = 0;
    PID.error[0] = 0;
    PID.error[1] = 0;
    PID.errorDot = 0;
    PID.IntegratorNum = 0.0005*0.5;

    PIX.KpGain = 0;
    PIX.KiGain = 0;
    PIX.KdGain = 0;

    PIX.IntegralTerm = 0;
    PIX.error[0] = 0;
    PIX.error[1] = 0;
    PIX.errorDot = 0;
    PIX.IntegratorNum = 0.0005*0.5;

    TorsionLPF.freq = 600;
    TorsionLPF.num[0] = 1/(1 + 2/(TorsionLPF.freq*0.0005));
    TorsionLPF.num[1] = TorsionLPF.num[0];
    TorsionLPF.denum[0] = (1 - 2/(TorsionLPF.freq*0.0005)) * TorsionLPF.num[0];
    TorsionLPF.output[0] = 0;
    TorsionLPF.input[0] = 0;
    TorsionLPF.output[1] = 0;
    TorsionLPF.input[1] = 0;

}

double TorsionLPFBlock(double inPart){

  double TorsionLPFOut;

  /* Diff equation */
  TorsionLPF.input[0] = inPart;
  TorsionLPFOut = TorsionLPF.num[0] * TorsionLPF.input[0] + TorsionLPF.num[1] *  TorsionLPF.input[1] - TorsionLPF.denum[0]*TorsionLPF.output[0];

  /* Update states */
  TorsionLPF.input[1] =  TorsionLPF.input[0];
  TorsionLPF.output[0] = TorsionLPFOut;

  return TorsionLPFOut;
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

double fPID(double Tdref, double Td, double TdDotref, double TdDot){

  PID.error[0] = Tdref-Td;
  PID.errorDot = TdDotref-TdDot;
  PID.IntegralTerm = PID.IntegralTerm + PID.IntegratorNum*(PID.error[0]+PID.error[1]);
  PID.error[1] = PID.error[0];
  return PID.KpGain*(Tdref-Td) + PID.KiGain*PID.IntegralTerm + PID.KdGain*PID.errorDot;

}

double fPI(double QmDotRef, double QmDot){

  PIX.error[0] = QmDotRef-QmDot;
  PIX.IntegralTerm = PIX.IntegralTerm + PIX.IntegratorNum*(PIX.error[0]+PIX.error[1]);
  PIX.error[1] = PIX.error[0];
  return PIX.KpGain*PIX.error[0] + PIX.KiGain*PIX.IntegralTerm;

}

double fDOB(double QmDot, double Tm){

  /* LPF */
  double DOBOut;

  /* Diff equation */
  DOB.input[0] = Tm + QmDot*0.0278*DOB.freq;
  DOBOut = DOB.num[0] * DOB.input[0] + DOB.num[1] *  DOB.input[1] - DOB.denum[0]*DOB.output[0];

  /* Update states */
  DOB.input[1] =  DOB.input[0];
  DOB.output[0] = DOBOut;

  return QmDot*0.0278*DOB.freq - DOBOut;

}
