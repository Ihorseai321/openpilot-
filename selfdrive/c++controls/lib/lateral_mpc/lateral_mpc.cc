#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "lateral_mpc.h"
#include <stdio.h>

#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define DEG_TO_RAD 3.1415926535898 / 180

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void lateral_init_weights(float pathCost, float laneCost, float headingCost, float steerRateCost){
  int i;
  const int STEP_MULTIPLIER = 3;

  for (i = 0; i < N; i++) {
    int f = 1;
    if (i > 4){
      f = STEP_MULTIPLIER;
    }
    // Setup diagonal entries
    acadoVariables.W[NY*NY*i + (NY+1)*0] = pathCost * f;
    acadoVariables.W[NY*NY*i + (NY+1)*1] = laneCost * f;
    acadoVariables.W[NY*NY*i + (NY+1)*2] = laneCost * f;
    acadoVariables.W[NY*NY*i + (NY+1)*3] = headingCost * f;
    acadoVariables.W[NY*NY*i + (NY+1)*4] = steerRateCost * f;
  }
  acadoVariables.WN[(NYN+1)*0] = pathCost * STEP_MULTIPLIER;
  acadoVariables.WN[(NYN+1)*1] = laneCost * STEP_MULTIPLIER;
  acadoVariables.WN[(NYN+1)*2] = laneCost * STEP_MULTIPLIER;
  acadoVariables.WN[(NYN+1)*3] = headingCost * STEP_MULTIPLIER;
}

void lateral_init(float pathCost, float laneCost, float headingCost, float steerRateCost){
  acado_initializeSolver();
  int i;

  /* Initialize the states and controls. */
  for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.1;

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
  for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

  /* MPC: initialize the current state feedback. */
  for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;

  lateral_init_weights(pathCost, laneCost, headingCost, steerRateCost);
}

int lateral_run_mpc(lateral_state_t * x0, lateral_log_t * solution,
             float l_poly[4], float r_poly[4], float d_poly[4],
             float l_prob, float r_prob, float curvature_factor, float v_ref, float lane_width){

  int i;

  for (i = 0; i <= NOD * N; i+= NOD){
    acadoVariables.od[i] = curvature_factor;
    acadoVariables.od[i+1] = v_ref;

    acadoVariables.od[i+2] = l_poly[0];
    acadoVariables.od[i+3] = l_poly[1];
    acadoVariables.od[i+4] = l_poly[2];
    acadoVariables.od[i+5] = l_poly[3];

    acadoVariables.od[i+6] = r_poly[0];
    acadoVariables.od[i+7] = r_poly[1];
    acadoVariables.od[i+8] = r_poly[2];
    acadoVariables.od[i+9] = r_poly[3];

    acadoVariables.od[i+10] = d_poly[0];
    acadoVariables.od[i+11] = d_poly[1];
    acadoVariables.od[i+12] = d_poly[2];
    acadoVariables.od[i+13] = d_poly[3];


    acadoVariables.od[i+14] = l_prob;
    acadoVariables.od[i+15] = r_prob;
    acadoVariables.od[i+16] = lane_width;

  }

  acadoVariables.x0[0] = x0->x;
  acadoVariables.x0[1] = x0->y;
  acadoVariables.x0[2] = x0->psi;
  acadoVariables.x0[3] = x0->delta;


  acado_preparationStep();
  acado_feedbackStep();

  /* printf("lat its: %d\n", acado_getNWSR());  // n iterations
  printf("Objective: %.6f\n", acado_getObjective());  // solution cost */

  for (i = 0; i <= N; i++){
    solution->x[i] = acadoVariables.x[i*NX];
    solution->y[i] = acadoVariables.x[i*NX+1];
    solution->psi[i] = acadoVariables.x[i*NX+2];
    solution->delta[i] = acadoVariables.x[i*NX+3];
    if (i < N){
      solution->rate[i] = acadoVariables.u[i];
    }
  }
  solution->cost = acado_getObjective();

  // Dont shift states here. Current solution is closer to next timestep than if
  // we use the old solution as a starting point
  //acado_shiftStates(2, 0, 0);
  //acado_shiftControls( 0 );

  return acado_getNWSR();
}
