/***************************************************************************************
*    Title: Series Elastic Actuator Exceptional Library for Sliding Mode Controller (SEALibexc)
*    Dependencies: BCM2835 from airspayce.com/mikem/bcm2835/
*    Author: Talha Kansizoglu
*    Date: 23.07.2018
*    Library version: v1.0
***************************************************************************************/
#include "SEALibexc.h"

tustinBlock DOBLPF;  // For motor side disturbance observer

void getExceptionalParameters(void){

  experiment.samplingTime = 0.0005;
  experiment.refAmplitude = 0;
  experiment.refStepTime = 2;
  experiment.refFreq = 0;
  experiment.endTime = 200;
  experiment.MotorEmergency = 0;

  DOBLPF.LPFfreq = 1500;
  DOBLPF.num[0] = 1/(1 + 2/(DOBLPF.LPFfreq*experiment.samplingTime));
  DOBLPF.num[1] = MotorPosDerivativeLPF.num[0];
  DOBLPF.denum[0] = (1 - 2/(DOBLPF.LPFfreq*experiment.samplingTime)) * DOBLPF.num[0];
  DOBLPF.output[0] = 0;
  DOBLPF.input[0] = 0;
  DOBLPF.output[1] = 0;
  DOBLPF.input[1] = 0;
}


double chirpSDot(double w1, double w2, double A, double M, double time){
  /* This funtion implements Chirp signal */
  double res;
  res=A*(w1+(w2-w1)*time/(M))*cos(w1*time+(w2-w1)*time*time/(2*M));
  return res;
}

double chirpSDotDot(double w1, double w2, double A, double M, double time){
  /* This funtion implements Chirp signal */
  double res;
  res=-A*(w1+(w2-w1)*time/(M))*(w1+(w2-w1)*time/(M))*sin(w1*time+(w2-w1)*time*time/(2*M));
  return res;
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


double sign(double sigma){
  double ret;
  if(sigma>0.01){
    ret = 1;
  }
  else if(sigma<-0.01){
    ret = -1;
  }
  else{
    ret = 0;
  }
return ret;
}


double DOBBlock(double motorVelocity, double motorTorque){

	// Motor side disturbance observer
  double DOBLPFOut;

  /* Diff equation */
  DOBLPF.input[0] = DOBLPF.LPFfreq*SEAnominal.Jmn*motorVelocity + motorTorque;
  DOBLPFOut = DOBLPF.num[0] * DOBLPF.input[0] + DOBLPF.num[1] *  DOBLPF.input[1] - DOBLPF.denum[0]*DOBLPF.output[0];

  /* Update states */
  DOBLPF.input[1] =  DOBLPF.input[0];
  DOBLPF.output[0] = DOBLPFOut;

  return (DOBLPF.input[0]-motorTorque-DOBLPFOut);
}


double SMCCompute(double QdRef, double QdRefDot, double QdRefDotDot, double C, double P){

  // Sliding Mode Controller with QD Feedback
  double eps = 0.1;
  SMC.error = QdRef-SEA.torsion;
  SMC.errorDot = QdRefDot-SEA.torsionVel;
  SMC.sigma = SMC.errorDot + C * SMC.error;

  double W1 = -(SEAnominal.Kn/(SEAnominal.Jmn*SEAnominal.Nm*SEAnominal.Nm) + SEAnominal.Kn/SEAnominal.Jln);
  double W2 = (SEAnominal.Jln*SEAnominal.Bmn-SEAnominal.Jmn*SEAnominal.Bln)/(SEAnominal.Jmn*SEAnominal.Jln*SEAnominal.Nm);

  return  P*(SMC.sigma/(fabs(SMC.sigma) + eps)) + SEAnominal.Jmn*(QdRefDotDot - W1*SEA.torsion + W2* SEA.motorVel/SEAnominal.Nm + (SEAnominal.Bln/SEAnominal.Jln)*SEA.torsionVel + C*(SMC.errorDot));
}
