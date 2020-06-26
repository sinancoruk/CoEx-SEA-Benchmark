/***************************************************************************************
*    Title: FF+PID+DOB Controller
*    Log File: (General.log)
*    Library Version : SEALib v6.0
*    Author: Talha Kansizoglu
*    Date: 19.11.2018
*    Code version: v1.0
***************************************************************************************/

#include "SEALibexc.h"

/*REALTIME CONFIGURATION DO NOT EDIT*/
#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

/* For minor page faults*/
void stack_prefault(void) {
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}

int main(int argc, char* argv[]){
  /* TIME ADJUSTMENTS */
  struct timespec t, t_reference;
  struct sched_param param;
  double t_currenttime = 0;
  double t_referencetime = 0;
  double time= 0;
  double Reference = 0;
  double ReferenceDot = 0;
  double DOBOut = 0;
  double CFFOut = 0;
  double PIDOut = 0;
  int a = 0;

  /* OPEN A LOG FILE FOR DATA LOGGING */
  FILE *fptr;
  fptr = fopen("General.log", "w");

  /* INITIALISE CONFIG */
  getGeneralParameters();        // Experiment parameters
  getExceptionalParameters();    // Controller parameters
  setConfig();

  SEA.inputSignal = 0;
  DACWrite();

  bsp_DelayUS(50);
  readEncodersInit();

  /* DECLARE THIS IS A REALTIME TASK */
  param.sched_priority = MY_PRIORITY;
  if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  /* LOCK MEMORY */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* PREFAULT OUR STACK */
  stack_prefault();

  /* GET REFERENCE TIME */
  clock_gettime(CLOCK_MONOTONIC ,&t);
  clock_gettime(CLOCK_MONOTONIC ,&t_reference);
  t_referencetime = t_reference.tv_sec * 1000000 + t_reference.tv_nsec / 1000; /*In miliseconds*/
  time = ((t_currenttime-t_referencetime)/1000000-1); /*normalize time*/

  /* Wait one second*/
  t.tv_sec++;
  int LAST_TIME = t.tv_sec+experiment.endTime;

  while(t.tv_sec < LAST_TIME ) {

    /* wait until next shot */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    clock_gettime(CLOCK_MONOTONIC ,&t);
    t_currenttime = t.tv_sec * 1000000 + t.tv_nsec / 1000; /*In miliseconds*/

     /* REFERENCE GENERATION PART*/

    /* Step input*/
    if((time)<1){
 		   Reference = 0;
 		 }
 		 else{
       Reference = 5*0.0002;
       ReferenceDot = 0.0;
 		 }

  /* Stair input
  if((time)>0  && (time)<1){
    Reference = 0*0.0002;
  }

  if((time)>1  && (time)<3){
    Reference = 10*0.0002;
  }

  if((time)>3  && (time)<5){
    Reference = -10*0.0002;
  }

  if((time)>5  && (time)<7){
    Reference = 10*0.0002;
  }
  if((time)>7  && (time)<9){
    Reference = -10*0.0002;
  }
  if((time)>9){
    Reference = 0*0.0002;
  }*/

  /* Sinusoidal input
  if((time)<1){
     Reference = 0;
  }
  else{
		 Reference = 2*0.0002*sin(6*(time-1));
     ReferenceDot = 6*2*0.0002*cos(6*(time-1));
  }*/

  // Reference = ((chirpS(0.2,150,10*0.0002,experiment.endTime,(time))));

  //Reference = 0;

    /* READ ENCODERS HERE */
    readEncoders();

    PIDOut = fPID(Reference, SEA.torsion, ReferenceDot, SEA.torsionVel);
    DOBOut = fDOB(SEA.previnputSignal, SEA.torsion);
    CFFOut = fCFF(Reference);
    SEA.inputSignal = PIDOut + CFFOut - DOBOut;


    saturateMID();


    /* SEND MOTOR COMMANDS HERE */

    /* Wait 2 cycles before cotrolling */
   if((time)<0.01)
      SEA.inputSignal = 0;


    SEA.previnputSignal = SEA.inputSignal;

    DACWrite();

    /* LOG THE EXPERIMENT DATA */
    fprintf(fptr,"%f %e %e %e %e %e %e\n", (t_currenttime-t_referencetime)/1000000.-1, SEA.torsion, SEA.motorPos, SEA.torsionVel, SEA.motorVel, Reference, DOBOut);

    t.tv_nsec += 500000;
    while (t.tv_nsec >= NSEC_PER_SEC) {
        t.tv_nsec -= NSEC_PER_SEC;
        t.tv_sec++;
    }

  }

  SEA.inputSignal = 0;
  DACWrite();
  fclose(fptr);
  bcm2835_spi_end();
  bcm2835_close();
}
