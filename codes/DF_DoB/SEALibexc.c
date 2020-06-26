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

#include "SEALibexc.h"

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

void getExceptionalParameters(void){


  experiment.refAmplitude = 0.2;
  experiment.refStepTime = 2;
  experiment.refFreq = 5;
  experiment.endTime = 10;
  experiment.MotorEmergency = 0;

  double ProportionalGain = 1100 * 0.001;
  double DerivativeGain =   22 * 0.001;
  PID1.freq = 150;
  PID2.freq = 150;

  TorsionLPF.freq = 1600;
  TorsionLPF.num[0] = 1/(1 + 2/(TorsionLPF.freq*0.0005));
  TorsionLPF.num[1] = TorsionLPF.num[0];
  TorsionLPF.denum[0] = (1 - 2/(TorsionLPF.freq*0.0005)) * TorsionLPF.num[0];
  TorsionLPF.output[0] = 0;
  TorsionLPF.input[0] = 0;
  TorsionLPF.output[1] = 0;
  TorsionLPF.input[1] = 0;

  PID1.KpGain = -ProportionalGain;
  PID1.KiGain = 0;
  PID1.KdGain = DerivativeGain;
  PID1.IntegralTerm = 0;
  PID1.error[0] = 0;
  PID1.error[1] = 0;
  PID1.errorDot = 0;
  PID1.IntegratorNum = 0.0005*0.5;
  PID1.num[0] = 1/(1 + 2/(PID1.freq*0.0005));
  PID1.num[1] = PID1.num[0];
  PID1.denum[0] = (1 - 2/(PID1.freq*0.0005)) * PID1.num[0];
  PID1.output[0] = 0;
  PID1.input[0] = 0;
  PID1.output[1] = 0;
  PID1.input[1] = 0;
  PID2.KpGain = ProportionalGain*100;
  PID2.KiGain = 0;
  PID2.KdGain = -DerivativeGain*100;
  PID2.IntegralTerm = 0;
  PID2.error[0] = 0;
  PID2.error[1] = 0;
  PID2.errorDot = 0;
  PID2.IntegratorNum = 0.0005*0.5;
  PID2.num[0] = 1/(1 + 2/(PID2.freq*0.0005));
  PID2.num[1] = PID2.num[0];
  PID2.denum[0] = (1 - 2/(PID2.freq*0.0005)) * PID2.num[0];
  PID2.output[0] = 0;
  PID2.input[0] = 0;
  PID2.output[1] = 0;
  PID2.input[1] = 0;


  Der1.freq = 1800;
  Der1.num[0] = 1/(1 + 2/(Der1.freq*0.0005));
  Der1.num[1] = Der1.num[0];
  Der1.denum[0] = (1 - 2/(Der1.freq*0.0005)) * Der1.num[0];
  Der1.output[0] = 0;
  Der1.input[0] = 0;
  Der1.output[1] = 0;
  Der1.input[1] = 0;

  Der2.freq = 1800;
  Der2.num[0] = 1/(1 + 2/(Der2.freq*0.0005));
  Der2.num[1] = Der2.num[0];
  Der2.denum[0] = (1 - 2/(Der2.freq*0.0005)) * Der2.num[0];
  Der2.output[0] = 0;
  Der2.input[0] = 0;
  Der2.output[1] = 0;
  Der2.input[1] = 0;

  DF.IntegratorNum = 0.0005*0.5;
  DF.ydfo = 0;
  DF.ydfo_d = 0;
  DF.ydfo_dd = 0;
  DF.ydfo_d_Pre = 0;
  DF.ydfo_dd_Pre = 0;
  DF.ydfo_ddd = 0;
  DF.ydfo_dddd = 0;
  DF.uRef = 0;
  DF.xRef[0] = 0;
  DF.xRef[1] = 0;
  DF.xRef[2] = 0;
  DF.xRef[3] = 0;

  double Xc = 0.3;
  DF.Kc[0] = -20*Xc;
  DF.Kc[1] = -0.001*Xc;
  DF.Kc[2] = 2000*Xc;
  DF.Kc[3] = 0.005*Xc;

  // DOB INITIALISE
  DOB.A[0][0] = 0;
  DOB.A[0][1] = 1;
  DOB.A[0][2] = 0;
  DOB.A[0][3] = 0;

  DOB.A[1][0] = -1798.5611510791368345962837338448;
  DOB.A[1][1] = -5.3956834532374100719424460431655;
  DOB.A[1][2] = 179856.11510791367618367075920105;
  DOB.A[1][3] = 0;

  DOB.A[2][0] = 0;
  DOB.A[2][1] = 0;
  DOB.A[2][2] = 0;
  DOB.A[2][3] = 1;

  DOB.A[3][0] = 714.28571428571428571428571428571;
  DOB.A[3][1] = 0;
  DOB.A[3][2] = -71428.571428571420256048440933228;
  DOB.A[3][3] = -0.071428571428571428571428571428571;

  DOB.B[0] = 0;
  DOB.B[1] = 3597.1223021582736691925674676895;


  DOB.IntegratorNum = 0.0005*0.5;
  double DobFreq = 200;
  DOB.LB[0] = 3*DobFreq;
  DOB.LB[1] = 3*DobFreq*DobFreq;
  DOB.LB[2] = DobFreq*DobFreq*DobFreq;
  DOB.LA[0] = DOB.LB[0];
  DOB.LA[1] = DOB.LB[1];
  DOB.LA[2] = DOB.LB[2];
  for (int i = 0; i < 4; i++)
  {
    DOB.est_z1[i] = 0;
    DOB.est_z2[i] = 0;
    DOB.est_z3[i] = 0;
    DOB.est_dz1[i] = 0;
    DOB.est_dz2[i] = 0;
    DOB.est_dz3[i] = 0;
    DOB.est_dz1_pre[i] = 0;
    DOB.est_dz2_pre[i] = 0;
    DOB.est_dz3_pre[i] = 0;
    DOB.est_Tdis[i] = 0;
    DOB.est_dTdis[i] = 0;
    DOB.est_ddTdis[i] = 0;
    DOB.Tdist[i] = 0;
    DOB.Tempx[i] = 0;
    DOB.states[i] = 0;
  }

   DOB.Tdist[4] = 0;
   DOB.Tdist[5] = 0;

}

double fPID1(double Qdref, double Qd){

  PID1.error[0] = Qdref-Qd;
  PID1.errorDot = fPID1Der(Qd);
  PID1.IntegralTerm = PID1.IntegralTerm + PID1.IntegratorNum*(PID1.error[0]+PID1.error[1]);

  return PID1.KpGain*PID1.error[0] + PID1.KiGain*PID1.IntegralTerm + PID1.KdGain*PID1.errorDot;

}

double fPID1Der(double inPart){

  double PID1DerOut;

  /* Diff equation */
  PID1.input[0] = inPart*PID1.freq;
  PID1DerOut = PID1.num[0] * PID1.input[0] + PID1.num[1] *  PID1.input[1] - PID1.denum[0]*PID1.output[0];

  /* Update states */
  PID1.input[1] =  PID1.input[0];
  PID1.output[0] = PID1DerOut;

  return inPart*PID1.freq-PID1DerOut;

}

double fPID2(double Qdref, double Qd){

  PID2.error[0] = Qdref-Qd;
  PID2.errorDot = fPID2Der(Qd);
  PID2.IntegralTerm = PID2.IntegralTerm + PID2.IntegratorNum*(PID2.error[0]+PID2.error[1]);

  return PID2.KpGain*PID2.error[0] + PID2.KiGain*PID2.IntegralTerm + PID2.KdGain*PID2.errorDot;

}

double fPID2Der(double inPart){

  double PID2DerOut;

  /* Diff equation */
  PID2.input[0] = inPart*PID2.freq;
  PID2DerOut = PID2.num[0] * PID2.input[0] + PID2.num[1] *  PID2.input[1] - PID2.denum[0]*PID2.output[0];

  /* Update states */
  PID2.input[1] =  PID2.input[0];
  PID2.output[0] = PID2DerOut;

  return inPart*PID2.freq-PID2DerOut;

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


double Der1Block(double inPart){

  double Der1Out;

  /* Diff equation */
  Der1.input[0] = inPart*Der1.freq;
  Der1Out = Der1.num[0] * Der1.input[0] + Der1.num[1] *  Der1.input[1] - Der1.denum[0]*Der1.output[0];

  /* Update states */
  Der1.input[1] =  Der1.input[0];
  Der1.output[0] = Der1Out;

  return inPart*Der1.freq-Der1Out;

}

double Der2Block(double inPart){

  double Der2Out;

  /* Diff equation */
  Der2.input[0] = inPart*Der2.freq;
  Der2Out = Der2.num[0] * Der2.input[0] + Der2.num[1] *  Der2.input[1] - Der2.denum[0]*Der2.output[0];

  /* Update states */
  Der2.input[1] =  Der2.input[0];
  Der2.output[0] = Der2Out;

  return inPart*Der2.freq-Der2Out;
}

double DFUpdate(double TdRef, double TdRefDot, double TdRefDotDot){

  double Jln = 0.07;
  double Bln = 0.005;
  double Jmn = 0.000278;
  double Bmn = 0.0015;
  double Kn = 5000;
  double Nm = 100;
  double Kn_1 = 1/Kn;
  double Nm_1 = 1/Nm;
  double Nm_2 = Nm_1*Nm_1;
  double Kenv  = 1;
  double Benv =  0.1;

  DF.ydfo_dd =  (Nm/Jln)*(Kn_1*TdRef-(Kn_1*Nm)*DOB.Tdist[1] -((Bln+Benv)/Nm)*DF.ydfo_d -(Kenv/(Nm*Nm))*DF.ydfo);
  DF.ydfo_ddd =  (Nm/Jln)*(Kn_1*TdRefDot-(Kn_1*Nm)*DOB.Tdist[3] -((Bln+Benv)/Nm)*DF.ydfo_dd -(Kenv/(Nm*Nm))*DF.ydfo_d);
  DF.ydfo_dddd =  (Nm/Jln)*(Kn_1*TdRefDotDot-(Kn_1*Nm)*DOB.Tdist[5] -((Bln+Benv)/Nm)*DF.ydfo_ddd -(Kenv/(Nm*Nm))*DF.ydfo_dd);

  DF.xRef[0] = (Jln*DF.ydfo_dd) + (Bln+Benv)*DF.ydfo_d + ((Kn+Kenv)/Nm)*DF.ydfo + (Nm*Nm*DOB.Tdist[1]/Kn);
  DF.xRef[1] = (Jln*DF.ydfo_ddd) + (Bln+Benv)*DF.ydfo_dd + ((Kn+Kenv)/Nm)*DF.ydfo_d + (Nm*Nm*DOB.Tdist[3]/Kn);
  DF.xRef[2] = Kn*Nm_2*DF.ydfo;
  DF.xRef[3] = Kn*Nm_2*DF.ydfo_d;

  DF.uRef = (Jmn*Jln)*DF.ydfo_dddd + (Jmn*(Bln+Benv) + Bmn*Jln)*DF.ydfo_ddd +
      (Jln*Kn*Nm_2 + Jmn*(Kn+Kenv)*Nm_1+(Bln+Benv)*Bmn)*DF.ydfo_dd + ((Kn*Nm_2)*(Bln+Benv)+Nm_1*(Kn+Kenv)*Bmn)*DF.ydfo_d +
      DOB.Tdist[0]/Nm + DOB.Tdist[1] + Bmn*Nm*Nm*Kn_1*DOB.Tdist[3] + Jmn*Nm*Nm*Kn_1*DOB.Tdist[5];

  for (int i = 0; i < 4; i++)
  {
    DF.ydfo_d = DF.ydfo_d + DF.IntegratorNum*(DF.ydfo_dd + DF.ydfo_dd_Pre);
    DF.ydfo = DF.ydfo + DF.IntegratorNum*(DF.ydfo_d + DF.ydfo_d_Pre);
    DF.ydfo_dd_Pre = DF.ydfo_dd;
    DF.ydfo_d_Pre = DF.ydfo_d;
  }

  //return DF.uRef - (DF.Kc[0]*(DF.xRef[0] - DOB.states[0]) + DF.Kc[1]*(DF.xRef[1] - DOB.states[1]) + DF.Kc[2]*(DF.xRef[2] - DOB.states[2]) + DF.Kc[3]*(DF.xRef[3] - DOB.states[3]));
  return DF.uRef - ( fPID1(DF.xRef[0], DOB.states[0]) + fPID2(DF.xRef[2], DOB.states[2]) );

}

void DOBUpdate(double Tm){

    for (int i = 0; i < 4; i++)
    {
          for (int j = 0; j < 4; j++)
          {
              DOB.Tempx[i] += DOB.A[i][j]*DOB.states[j];
          }
          DOB.Tempx[i] = DOB.Tempx[i] + DOB.B[i]*Tm + DOB.states[i]*DOB.LA[0];
    }


    for (int i = 0; i < 4; i++)
    {
      DOB.est_dz1[i] = -DOB.LA[0]*DOB.est_z1[i] + DOB.est_z2[i] + DOB.LA[0]*(DOB.Tempx[i]) - DOB.LA[1]*DOB.states[i];
      DOB.est_dz2[i] = -DOB.LA[1]*DOB.est_z1[i] + DOB.est_z3[i] + DOB.LA[1]*(DOB.Tempx[i]) - DOB.LA[2]*DOB.states[i];
      DOB.est_dz3[i] = -DOB.LA[2]*DOB.est_z1[i] + DOB.LA[2]*(DOB.Tempx[i]);
    }

  for (int i = 0; i < 4; i++)
  {
    DOB.est_z1[i] = DOB.est_z1[i] + DOB.IntegratorNum*(DOB.est_dz1[i]+DOB.est_dz1_pre[i]);
    DOB.est_z2[i] = DOB.est_z2[i] + DOB.IntegratorNum*(DOB.est_dz2[i]+DOB.est_dz2_pre[i]);
    DOB.est_z3[i] = DOB.est_z3[i] + DOB.IntegratorNum*(DOB.est_dz3[i]+DOB.est_dz3_pre[i]);
    DOB.est_dz1_pre[i] = DOB.est_dz1[i];
    DOB.est_dz2_pre[i] = DOB.est_dz2[i];
    DOB.est_dz3_pre[i] = DOB.est_dz3[i];
   }

     for (int i = 0; i < 4; i++)
  {
    DOB.est_Tdis[i]   = DOB.est_z1[i] - DOB.LB[0]*DOB.states[i];
    DOB.est_dTdis[i]  = DOB.est_z2[i] - DOB.LB[1]*DOB.states[i];
    DOB.est_ddTdis[i] = DOB.est_z3[i] - DOB.LB[2]*DOB.states[i];
  }

  DOB.Tdist[0] = Jm*100*DOB.est_Tdis[1]; //d1
  DOB.Tdist[1] = Jl*0.01*DOB.est_Tdis[3];  //d2
  DOB.Tdist[2] = Jm*100*DOB.est_dTdis[1];  //d1 dot
  DOB.Tdist[3] = Jl*0.01*DOB.est_dTdis[3];  // d2 dot
  DOB.Tdist[4] = Jm*100*DOB.est_ddTdis[1]; // d1 dot dot
  DOB.Tdist[5] = Jl*0.01*DOB.est_ddTdis[3]; // d2 dot dot

  for (int j = 0; j < 4; j++){
    DOB.Tempx[j] = 0;
  }

}
