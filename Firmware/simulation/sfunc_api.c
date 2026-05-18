/* foc_wrapper: C MEX S-Function interface */
#define S_FUNCTION_NAME foc_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "tmwtypes.h"

extern void sim_sfunc_init(SimStruct *S);
extern void sim_sfunc_terminate(SimStruct *S);
extern void sim_sfunc_core(SimStruct *S);
extern void sim_sfunc_output(SimStruct *S);

extern int_T simGetInPortNums(void);
extern int_T simGetInPortNumWidth(int port);
extern int_T simGetOutPortNums(void);
extern int_T simGetOutPortNumWidth(int port);

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, simGetInPortNums())) return;
    for (int port = 0; port < simGetInPortNums(); port++) {
        ssSetInputPortWidth(S, port, simGetInPortNumWidth(port));
        ssSetInputPortDirectFeedThrough(S, port, 1);
        ssSetInputPortDataType(S, port, SS_DOUBLE);
    }

    if (!ssSetNumOutputPorts(S, simGetOutPortNums())) return;
    for (int port = 0; port < simGetOutPortNums(); port++) {
        ssSetOutputPortWidth(S, port, simGetOutPortNumWidth(port));
        ssSetOutputPortDataType(S, port, SS_DOUBLE);
    }
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    sim_sfunc_init(S);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    sim_sfunc_core(S);
    sim_sfunc_output(S);
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
    sim_sfunc_terminate(S);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
