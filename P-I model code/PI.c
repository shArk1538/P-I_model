#define S_FUNCTION_NAME  PI
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#include "math.h"
#include "windows.h"
#include "stdlib.h"
#include "string.h"

double p(double r)  //密度函数p(r)
{
   return exp(-0.07*(r-2)*(r-2));
}
  double Er[10000];
  double Eri[10000]; //定义全局变量
     
//=======================================================================//

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ==========================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0); //连续状态数量
    ssSetNumDiscStates(S, 0); //离散状态数量

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    
    ssSetNumDWork(S, 2);
    ssSetDWorkWidth(S, 0, 1);  //第一个工作区，储存vi[0]
    ssSetDWorkWidth(S, 1, 1);  //第二个工作区，未使用

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the operating point save/restore compliance to be same as a 
     * built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    ssSetOptions(S, 0);
    //此处写全局变量的初始化程序
     double Er[10000]={0}; 
     double Eri[10000]={0};
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.001);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
      real_T *vi = (real_T *) ssGetDWork(S,0);
      vi[0] = 0.0;
  }
#endif /* MDL_INITIALIZE_CONDITIONS */
  
  
#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {  
      double p(double r); //声明函数 
  }

#endif /*  MDL_START */

  
/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T i;
    const real_T  *v = (const real_T*) ssGetInputPortSignal(S,0);
    real_T        *u = ssGetOutputPortSignal(S,0);
    real_T        *vi = (real_T *)ssGetDWork(S,0);
    
    double Integral = 0.0;
    
    for(i=0;i<9999;i++)  //黎曼公式计算定积分
      {
         Er[i] = min(0.001*i,max(-0.001*i,v[0]-vi[0]+Eri[i]));  //将10000个stop算子以数组形式储存，每个微元对应计算一次
         Integral = Integral + 0.001 * p(0.001*i) * Er[i];  //积分
      }
    
         u[0] = Integral; //输出
     
         vi[0] = v[0]; //向量形式储存上一时刻的电压  
         
    for(i=0;i<9999;i++)
      { 
         Eri[i] = Er[i]; //数组形式储存上一时刻的stop算子
      }        
}


#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
      
  }
#endif /* MDL_UPDATE */

  
#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
     
  }
#endif /* MDL_DERIVATIVES */

  
/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
 static void mdlTerminate(SimStruct *S)
  {
     memset(Er,0,sizeof Er);
     memset(Eri,0,sizeof Eri); //清空数组
  }

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif