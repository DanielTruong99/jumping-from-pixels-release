#include "NeuralMPC_interface.h"
#include "../convexMPC/common_types.h"
#include "NeuralSolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define N_NUM_LEGS 4

neural_mpc_problem_setup n_problem_config;
neural_mpc_update_data_t n_update;

pthread_mutex_t neural_problem_cfg_mt;
pthread_mutex_t neural_mpc_update_mt;
pthread_t neural_mpc_solve_thread;

u8 n_first_run = 1;

void neural_initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&neural_problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&neural_mpc_update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");
}

void neural_setup_problem(float* dt, int horizon, double mu, double f_max)
{
  if(n_first_run) {
    n_first_run = false;
    neural_initialize_mpc();
  }

  //pthread_mutex_lock(&problem_cfg_mt);

  n_problem_config.horizon = horizon;
  n_problem_config.f_max = f_max;
  n_problem_config.mu = mu;
  n_problem_config.dt = dt;

  //pthread_mutex_unlock(&problem_cfg_mt);
  neural_resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
inline void neural_mpf_to_flt(flt* dst, mfp* src, s32 n_items) {
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

inline void neural_mint_to_u8(u8* dst, mint* src, s32 n_items) {
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int neural_has_solved = 0;

//safely copies problem data and starts the solver
std::tuple<float, float> neural_update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait)
{
  neural_mpf_to_flt(n_update.p,p,3);
  neural_mpf_to_flt(n_update.v,v,3);
  neural_mpf_to_flt(n_update.q,q,4);
  neural_mpf_to_flt(n_update.w,w,3);
  neural_mpf_to_flt(n_update.r,r,12);
  n_update.yaw = yaw;
  neural_mpf_to_flt(n_update.weights,weights,12);
  //this is safe, the solver isn't running, and n_update_problem_data and setup_problem
  //are called from the same thread
  neural_mpf_to_flt(n_update.traj,state_trajectory,12*n_problem_config.horizon);
  n_update.alpha = alpha;
  neural_mint_to_u8(n_update.gait,gait,4*n_problem_config.horizon);

  std::tuple<float, float> solve_times = neural_solve_mpc(&n_update, &n_problem_config);
  neural_has_solved = 1;
  return solve_times;
}

std::tuple<float, float> neural_update_problem_data_floats(float* p, float* v, float* q, float* w,
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, int* gait)
{
  //Timer t3;
  n_update.alpha = alpha;
  n_update.yaw = yaw;
  neural_mint_to_u8(n_update.gait,gait,4*n_problem_config.horizon);
  memcpy((void*)n_update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)n_update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)n_update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)n_update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)n_update.r,(void*)r,sizeof(float)*12);
  memcpy((void*)n_update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)n_update.traj,(void*)state_trajectory, sizeof(float) * 12 * n_problem_config.horizon);
  //printf("presolve %f ms\n", t3.getMs());
  std::tuple<float, float> solve_times = neural_solve_mpc(&n_update, &n_problem_config);
  neural_has_solved = 1;
  //printf("postsolve %f ms\n", t3.getMs());
  return solve_times;
}

double neural_get_solution(int index)
{
  if(!neural_has_solved) return 0.f;
  mfp* qs = neural_get_q_soln();
  return qs[index];
}

double neural_get_obj_val()
{
  if(!neural_has_solved) return 0.f;
  double objVal = neural_get_q_obj_val();
  return objVal;
}