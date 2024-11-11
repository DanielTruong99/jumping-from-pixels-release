#ifndef _Neural_mpc_interface
#define _Neural_mpc_interface

#define N_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <tuple>

struct neural_mpc_problem_setup
{
  float* dt;
  float mu;
  float f_max;
  int horizon;
};

struct neural_mpc_update_data_t
{
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float yaw;
  float weights[12];
  float traj[12*N_MAX_GAIT_SEGMENTS];
  float alpha;
  unsigned char gait[N_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  float x_drag;
};

EXTERNC void neural_setup_problem(float* dt, int horizon, double mu, double f_max);
EXTERNC std::tuple<float, float> neural_update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double neural_get_solution(int index);
EXTERNC double neural_get_obj_val();
EXTERNC std::tuple<float, float> neural_update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

void update_x_drag(float x_drag);
#endif
