#ifndef neural_solver_mpc
#define neural_solver_mpc


#include <eigen3/Eigen/Dense>
#include "../convexMPC/common_types.h"
#include "NeuralMPC_interface.h"
#include <iostream>
#include <stdio.h>
#include <tuple>

using Eigen::Matrix;
using Eigen::Quaternionf;
using Eigen::Quaterniond;


template <class T>
T neural_t_min(T a, T b) {
    if(a<b) return a;
    return b;
}

template <class T>
T neural_sq(T a) {
    return a*a;
}


std::tuple<float, float> neural_solve_mpc(neural_mpc_update_data_t* update, neural_mpc_problem_setup* setup);
void neural_quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy);
void neural_ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B);
void neural_resize_qp_mats(s16 horizon);
void neural_c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt* dt,s16 horizon);
mfp* neural_get_q_soln();
double neural_get_q_obj_val();
#endif
