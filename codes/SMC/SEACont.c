/***************************************************************************************
*    Title: Sliding Mode Controller
*    Log File: (General.log)
*    Library Version : SEALib v6.0
*    Author: Talha Kansizoglu
*    Date: 23.07.2018
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
  double t_currenttime = 0.0;
  double t_referencetime = 0.0;
  double time= 0;
  double Reference = 0.0;
  double ReferenceDot = 0.0;
  double ReferenceDotDot = 0.0;
  double DOBOut = 0.0;
  double SMCOut = 0.0;
  double RawInput = 0.0;

  double w1 = 0.2;
  double w2 = 100.0;

  /* OPEN A LOG FILE FOR DATA LOGGING */
  FILE *fptr;
  fptr = fopen("General.log", "w");

  /* INITIALISE CONFIG */
  getExceptionalParameters();    // Controller parameters
  getGeneralParameters();        // Experiment parameters
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

    /* Step input
    if((time)<1){
      Reference = 0.0;
    }
    else{
      Reference = 0.002;
    }*/

    /* Sinuoidal input
    if((time)<1){
      Reference = 0;
    }
    else{
      experiment.refFreq = 12;
      experiment.refAmplitude = 0.001;
      Reference = experiment.refAmplitude*sin(experiment.refFreq*(time-1));
      ReferenceDot = experiment.refAmplitude*experiment.refFreq*cos(experiment.refFreq*(time-1));
      ReferenceDotDot = -experiment.refAmplitude*experiment.refFreq*experiment.refFreq*sin(experiment.refFreq*(time-1));
      }*/

     Reference = ((chirpS(w1,w2,0.002,experiment.endTime,(time))));
     ReferenceDot = ((chirpSDot(w1,w2,0.002,experiment.endTime,(time))));
     ReferenceDotDot = ((chirpSDotDot(w1,w2,0.002,experiment.endTime,(time))));

    /* Stair input
    if((time)>0  && (time)<1){
    Reference = 0;
    }
    if((time)>1  && (time)<4){
    Reference = 0.0004;
    }
    if((time)>4  && (time)<7){
    Reference = 0.0008;
    }
    if((time)>7  && (time)<11){
    Reference = 0.0016;
    }
    if((time)>11  && (time)<14){
    Reference = 0.0024;
    }*/

    /* READ ENCODERS HERE */
    readEncoders();

    /* CONTROL ALGORITHM HERE */
    DOBOut = DOBBlock(SEA.motorVel, SEA.previnputSignal);
    SMCOut = SMCCompute(Reference, ReferenceDot, ReferenceDotDot, 1000, 0.55);  // C  P
    RawInput = SMCOut - DOBOut;
    SEA.inputSignal = RawInput;
    saturateMID();

    SEA.previnputSignal = SEA.inputSignal;

    /* SEND MOTOR COMMANDS HERE */

    /* Wait for 0.001 seconds for LPF and Derivatives */
   if((time)<0.001){
      SEA.inputSignal = 0.0;
      //SEA.torsionInit = SEA.torsionInit + SEA.torsionFiltered;
    }

    DACWrite();


    /* LOG THE EXPERIMENT DATA HERE*/
    fprintf(fptr,"%f %e %e %e %e %e %e\n", (t_currenttime-t_referencetime)/1000000.-1, SEA.torsion, SEA.motorPos, SEA.torsionVel, SMC.sigma, Reference, SEA.inputSignal);


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
