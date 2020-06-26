/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library (SEALibexc)
*    Cascaded PID Controller
*
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 21.07.2018
*    Library version: v1.0
***************************************************************************************/

#include "SEALibexc.h"

tustinBlock PIDDerivative;  // For PID1 Derivative

void getExceptionalParameters(void){

    experiment.refAmplitude = 0.2;
    experiment.refStepTime = 2;
    experiment.refFreq = 5;
    experiment.endTime = 200;
    experiment.MotorEmergency = 0;

    PID.KpGain = 16;
    PID.KiGain = 7;
    PID.KdGain = 0;

    PIx.KpGain = 0.045;
    PIx.KiGain = 0.012;
    PIx.KdGain = 0;

    TDLowPassFilter.num[0] = 1/(1 + 2/(1600*0.0005));
    TDLowPassFilter.num[1] = TDLowPassFilter.num[0];
    TDLowPassFilter.denum[0] = (1 - 2/(1600*0.0005)) * TDLowPassFilter.num[0];
    TDLowPassFilter.output[0] = 0;
    TDLowPassFilter.input[0] = 0;
    TDLowPassFilter.output[1] = 0;
    TDLowPassFilter.input[1] = 0;

    PIDDerivative.num[0]= 1/(1+2/(1600 * 0.0005));
    PIDDerivative.num[1]= PIDDerivative.num[0];
    PIDDerivative.denum[0]= (1+2/(1600 * 0.0005))*PIDDerivative.num[0];

    PIDDerivative.output[0] = 0;
    PIDDerivative.input[0] = 0;
    PIDDerivative.output[1] = 0;
    PIDDerivative.input[1] = 0;
    

    PID.IntegralTerm = 0;
    PID.error[0] = 0;
    PID.error[1] = 0;

    PIx.IntegralTerm = 0;
    PIx.error[0] = 0;
    PIx.error[1] = 0;

}


double FricCBlock(double inPart){
  /* This function compensates stiction, input velocity rad/s output Torque Nm */
  double FricCBlockOut;
  if(inPart > 0.000000001){
    FricCBlockOut = 0.07695;
  }
  else if(inPart < -0.000000001){
    FricCBlockOut = -0.06981;
  }
  else{
    FricCBlockOut = 0;
  }

  return FricCBlockOut;
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

double PIDBlock(double ref, double outPlant){
  double PIDOut;
  /* PID part begins */
   PID.error[0] = ref - outPlant;
   PID.IntegralTerm = PID.IntegralTerm + PID.KiGain * 0.0005 * PID.error[0];
   PIDOut = PID.KpGain * PID.error[0] + PID.IntegralTerm + PID.KdGain*PIDDerivativeBlock(PID.error[0],100);

   /*Update states*/
   PID.error[1] = PID.error[0];
   return PIDOut;
  /* PID part ends */
}

double PIBlock(double ref, double outPlant){
  double PIxOut;
  /* PI part begins */
   PIx.error[0] = ref - outPlant;
   PIx.IntegralTerm = PIx.IntegralTerm + PIx.KiGain * 0.0005 * PIx.error[0];
   PIxOut = PIx.KpGain * PIx.error[0] + PIx.IntegralTerm;

   /*Update states*/
   PIx.error[1] = PIx.error[0];
   return PIxOut;
  /* PI part ends */
}

double PIDDerivativeBlock(double inPart, double DerivationFreq){

  /* Derivative block with LPF */
  double PIDDerivativeOut;

  /* Diff equation */
  PIDDerivative.input[0] = inPart * DerivationFreq;
  PIDDerivativeOut = PIDDerivative.num[0] * PIDDerivative.input[0] + PIDDerivative.num[1] *  PIDDerivative.input[1] - PIDDerivative.denum[0]*PIDDerivative.output[0];

  /* Update states */
  PIDDerivative.input[1] =  PIDDerivative.input[0];
  PIDDerivative.output[0] = PIDDerivativeOut;

  return PIDDerivative.input[0] - PIDDerivativeOut;

}


double TDLowPassFilterBlock(double inPart){

  /* LPF */
  double TDLowPassFilterOut;

  /* Diff equation */
  TDLowPassFilter.input[0] = inPart;
  TDLowPassFilterOut = TDLowPassFilter.num[0] * TDLowPassFilter.input[0] + TDLowPassFilter.num[1] *  TDLowPassFilter.input[1] - TDLowPassFilter.denum[0]*TDLowPassFilter.output[0];

  /* Update states */
  TDLowPassFilter.input[1] =  TDLowPassFilter.input[0];
  TDLowPassFilter.output[0] = TDLowPassFilterOut;

  return TDLowPassFilterOut;
}
