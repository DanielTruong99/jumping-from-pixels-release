#include "NeuralSolverMPC.h"
#include "../convexMPC/common_types.h"
#include "NeuralMPC_interface.h"
#include "NeuralRobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>

#define N_BIG_NUMBER 5e10
//#define DEBUG_SOLVE_TIME
//big enough to act like infinity, small enough to avoid numerical weirdness.

NeuralRobotState n_rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

Matrix<fpt,Dynamic,13> nA_qp;
Matrix<fpt,Dynamic,Dynamic> nB_qp;
Matrix<fpt,13,12> nBdt;
Matrix<fpt,13,13> nAdt;
Matrix<fpt,25,25> nABc,nABcdt,n_expmm;
Matrix<fpt,Dynamic,Dynamic> nS;
Matrix<fpt,Dynamic,1> nX_d;
Matrix<fpt,Dynamic,1> nU_b;
Matrix<fpt,Dynamic,Dynamic> n_fmat;

Matrix<fpt, Dynamic, Dynamic> nB_qp_trans_S;
Matrix<fpt,Dynamic,Dynamic> n_qH;
Matrix<fpt,Dynamic,1> n_qg;

Matrix<fpt,Dynamic,Dynamic> n_eye_12h;

qpOASES::real_t* nH_qpoases;
qpOASES::real_t* ng_qpoases;
qpOASES::real_t* nA_qpoases;
qpOASES::real_t* nlb_qpoases;
qpOASES::real_t* nub_qpoases;
qpOASES::real_t* nq_soln;

qpOASES::real_t* nH_red;
qpOASES::real_t* ng_red;
qpOASES::real_t* nA_red;
qpOASES::real_t* nlb_red;
qpOASES::real_t* nub_red;
qpOASES::real_t* nq_red;
u8 n_real_allocated = 0;

double objVal;


char n_var_elim[2000];
char n_con_elim[2000];

mfp* neural_get_q_soln() {
  return nq_soln;
}

double neural_get_q_obj_val() {
  return objVal;
}

s8 n_near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 n_near_one(fpt a)
{
  return n_near_zero(a-1);
}
void n_matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}


void neural_c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt* dt,s16 horizon)
{
  nABc.setZero();
  nABc.block(0,0,13,13) = Ac;
  nABc.block(0,13,13,12) = Bc;
  nABc = dt[0]*nABc;
  n_expmm = nABc.exp();
  nAdt = n_expmm.block(0,0,13,13);
  nBdt = n_expmm.block(0,13,13,12);
  if(horizon > 19) {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt,13,13> powerMats[20];
  powerMats[0].setIdentity();
  for(int i = 1; i < horizon+1; i++) {
    //nABcdt = dt[i-1] * nABc;
    //n_expmm = nABcdt.exp();
    //nAdt = n_expmm.block(0,0,13,13);
    powerMats[i] = nAdt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    nA_qp.block(13*r,0,13,13) = powerMats[r+1];
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        //nABcdt = dt[r] * nABc;
        //n_expmm = nABcdt.exp();
        //nBdt = n_expmm.block(0,13,13,12);
        s16 a_num = r-c;
        nB_qp.block(13*r,12*c,13,12) = powerMats[a_num] * nBdt;
      }
    }
  }

}

void neural_resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon*horizon;

  nA_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  nB_qp.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  nS.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  nX_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  nU_b.resize(20*horizon, Eigen::NoChange);
  mcount += 20*horizon;

  n_fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  n_qH.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  n_qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  n_eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  nA_qp.setZero();
  nB_qp.setZero();
  nS.setZero();
  nX_d.setZero();
  nU_b.setZero();
  n_fmat.setZero();
  n_qH.setZero();
  n_eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(n_real_allocated)
  {

    free(nH_qpoases);
    free(ng_qpoases);
    free(nA_qpoases);
    free(nlb_qpoases);
    free(nub_qpoases);
    free(nq_soln);
    free(nH_red);
    free(ng_red);
    free(nA_red);
    free(nlb_red);
    free(nub_red);
    free(nq_red);
  }

  nH_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  ng_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  nA_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  nlb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  nub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  nq_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  nH_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  ng_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  nA_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  nlb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  nub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  nq_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  n_real_allocated = 1;

  //printf("malloc'd %d floating point numbers.\n",mcount);



#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
     r(2), 0.f, -r(0),
     -r(1), r(0), 0.f;
  return I_inv * cm;
}
//continuous time state space matrices.
void neural_ct_ss_mats(Matrix<fpt,3,3> nI_world, fpt m, Matrix<fpt,3,4> r_feet, 
    Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B, float x_drag)
{
  A.setZero();
  A(3,9) = 1.f;
  A(9,9) = x_drag;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_yaw.transpose();

  B.setZero();
  Matrix<fpt,3,3> I_inv = nI_world.inverse();

  for(s16 b = 0; b < 4; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
}


void neural_quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  //from my MATLAB implementation

  //edge case!
  fpt as = neural_t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),neural_sq(q.w()) + neural_sq(q.x()) - neural_sq(q.y()) - neural_sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),neural_sq(q.w()) - neural_sq(q.x()) - neural_sq(q.y()) + neural_sq(q.z()));

}

Matrix<fpt,13,1> n_x_0;
Matrix<fpt,3,3> nI_world;
Matrix<fpt,13,13> nA_ct;
Matrix<fpt,13,12> nB_ct_r;


std::tuple<float, float> neural_solve_mpc(neural_mpc_update_data_t* update, neural_mpc_problem_setup* setup)
{

#ifdef DEBUG_SOLVE_TIME
  //Timer t3;
#endif

  n_rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);

  //printf("cpa %f ms\n", t3.getMs());

  //roll pitch yaw
  Matrix<fpt,3,1> rpy;
  neural_quat_to_rpy(n_rs.q,rpy);

  //initial state (13 state representation)
  n_x_0 << rpy(2), rpy(1), rpy(0), n_rs.p , n_rs.w, n_rs.v, -9.8f;
  nI_world = n_rs.R_yaw * n_rs.I_body * n_rs.R_yaw.transpose(); //original
  neural_ct_ss_mats(nI_world,n_rs.m,n_rs.r_feet,n_rs.R_yaw,nA_ct,nB_ct_r, update->x_drag);

#ifdef DEBUG_SOLVE_TIME
  //printf("cpb %f ms\n", t3.getMs());
#endif

  //QP matrices
  neural_c2qp(nA_ct,nB_ct_r,setup->dt,setup->horizon);

#ifdef DEBUG_SOLVE_TIME
  //printf("cpc %f ms\n", t3.getMs());
#endif

  //weights
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  nS.diagonal() = full_weight.replicate(setup->horizon,1);

#ifdef DEBUG_SOLVE_TIME
  //printf("cpd %f ms\n", t3.getMs());
#endif

  nB_qp_trans_S = nB_qp.transpose()*nS;

#ifdef DEBUG_SOLVE_TIME
  //printf("cp1 %f ms\n", t3.getMs());
#endif

  //trajectory
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++)
      nX_d(13*i+j,0) = update->traj[12*i+j];
  }
  //cout<<"XD:\n"<<vX_d<<endl;

  //note - I'm not doing the shifting here.
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 4; j++)
    {
      nU_b(5*k + 0) = N_BIG_NUMBER;
      nU_b(5*k + 1) = N_BIG_NUMBER;
      nU_b(5*k + 2) = N_BIG_NUMBER;
      nU_b(5*k + 3) = N_BIG_NUMBER;
      nU_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;
      k++;
    }
  }

  fpt mu = 1.f/setup->mu;
  Matrix<fpt,5,3> f_block;

  f_block <<  mu, 0,  1.f,
          -mu, 0,  1.f,
          0,  mu, 1.f,
          0, -mu, 1.f,
          0,   0, 1.f;

  for(s16 i = 0; i < setup->horizon*4; i++)
  {
    n_fmat.block(i*5,i*3,5,3) = f_block;
  }

#ifdef DEBUG_SOLVE_TIME
  //printf("cp2 %f ms\n", t3.getMs());
#endif

  //nx0, nXd
  /*
  std::cout << "n_x_0\n";
  for(int i = 0; i < 12; i++){
    std::cout << n_x_0[i] << " ";
  }
  std::cout << "\n\n";

  std::cout << "nX_d\n";
  for(int i = 0; i < setup->horizon; i++){
  for(int j = 0; j < 12; j++){
    std::cout << nX_d(i*13+j, 0) << " ";
  }
  std::cout << "\n";
  }
  std::cout << "\n";

  std::cout << "X_d:\n" << nX_d << endl;

  std::cout << "A_qp:\n" << nA_qp << endl;
  */

  //construct_timer;

  // these are some of the slowest operations

  n_qH = 2*(nB_qp_trans_S*nB_qp + update->alpha*n_eye_12h);
  n_qg = 2*nB_qp_trans_S*(nA_qp*n_x_0 - nX_d);
  
  float construct_time = 0;//construct_timer.getMs();
  

  n_matrix_to_real(nH_qpoases,n_qH,setup->horizon*12, setup->horizon*12);
  n_matrix_to_real(ng_qpoases,n_qg,setup->horizon*12, 1);
  n_matrix_to_real(nA_qpoases,n_fmat,setup->horizon*20, setup->horizon*12);
  n_matrix_to_real(nub_qpoases,nU_b,setup->horizon*20, 1);

  for(s16 i = 0; i < 20*setup->horizon; i++)
    nlb_qpoases[i] = 0.0f;

  s16 num_constraints = 20*setup->horizon;
  s16 num_variables = 12*setup->horizon;


  qpOASES::int_t nWSR = 100;

  ///******

  int new_vars = num_variables;
  int new_cons = num_constraints;

#ifdef DEBUG_SOLVE_TIME
  std::cout << "initially, (new_vars, new_cons) = (" << new_vars << ", " << new_cons << ")\n";
#endif
  for(int i =0; i < num_constraints; i++)
    n_con_elim[i] = 0;

  for(int i = 0; i < num_variables; i++)
    n_var_elim[i] = 0;


  for(int i = 0; i < num_constraints; i++)
  {
    //std::cout << "elimination conditions: (n_near_zero(nlb_qpoases[i]), n_near_zero(nub_qpoases[i])) = (" << n_near_zero(nlb_qpoases[i]) << ", " << n_near_zero(nub_qpoases[i])  << ", " << nlb_qpoases[i] << ", " << nub_qpoases[i] << ")\n";
    if(! (n_near_zero(nlb_qpoases[i]) && n_near_zero(nub_qpoases[i]))) continue;
    double* c_row = &nA_qpoases[i*num_variables];
    for(int j = 0; j < num_variables; j++)
    {
      //std::cout << "elimination conditions: n_near_one(c_row[j]) = " << n_near_one(c_row[j]) << "\n";
      if(n_near_one(c_row[j]))
      {
        new_vars -= 3;
        new_cons -= 5;
        int cs = (j*5)/3 -3;
        n_var_elim[j-2] = 1;
        n_var_elim[j-1] = 1;
        n_var_elim[j  ] = 1;
        n_con_elim[cs] = 1;
        n_con_elim[cs+1] = 1;
        n_con_elim[cs+2] = 1;
        n_con_elim[cs+3] = 1;
        n_con_elim[cs+4] = 1;
      }
    }
  }
#ifdef DEBUG_SOLVE_TIME
  std::cout << "after elimination, (new_vars, new_cons) = (" << new_vars << ", " << new_cons << ")\n";
#endif

#ifdef DEBUG_SOLVE_TIME
  //printf("cp3 %f ms\n", t3.getMs());
#endif

  //if(new_vars != num_variables)
  if(1==1)
  {
    int var_ind[new_vars];
    int n_con_ind[new_cons];
    int nc = 0;
    for(int i = 0; i < num_variables; i++)
    {
      if(!n_var_elim[i])
      {
        if(!(nc<new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[nc] = i;
        nc++;
      }
    }
    nc = 0;
    for(int i = 0; i < num_constraints; i++)
    {
      if(!n_con_elim[i])
      {
        if(!(nc<new_cons))
        {
          printf("BAD ERROR 1\n");
        }
        n_con_ind[nc] = i;
        nc++;
      }
    }
    for(int i = 0; i < new_vars; i++)
    {
      int olda = var_ind[i];
      ng_red[i] = ng_qpoases[olda];
      for(int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
        nH_red[i*new_vars + j] = nH_qpoases[olda*num_variables + oldb];
      }
    }

    for (int con = 0; con < new_cons; con++)
    {
      for(int st = 0; st < new_vars; st++)
      {
        float cnAl = nA_qpoases[(num_variables*n_con_ind[con]) + var_ind[st] ];
        nA_red[con*new_vars + st] = cnAl;
      }
    }
    for(int i = 0; i < new_cons; i++)
    {
      int old = n_con_ind[i];
      nub_red[i] = nub_qpoases[old];
      nlb_red[i] = nlb_qpoases[old];
    }

#ifdef DEBUG_SOLVE_TIME
    //printf("cp4 %f ms\n", t3.getMs());
#endif

    //solve_timer;

    qpOASES::QProblem problem_red (new_vars, new_cons);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);
    //int_t nWSR = 50000;

    /*
    for(int i = 0; i < 12; i++){
      for(int j = 0; j < 12; j++){
        std::cout << nH_red[i*new_vars+j] << " ";
      }
      std::cout << "\n";
    }
    std::cout << "\n";

    for(int i = 0; i < new_vars; i++){
        std::cout << ng_red[i] << " ";
    }
    std::cout << "\n";

    for(int i = 0; i < 12; i++){
      for(int j = 0; j < 12; j++){
        std::cout << nA_red[i*new_vars+j] << " ";
      }
      std::cout << "\n";
    }
    std::cout << "\n";

    for(int i = 0; i < new_cons; i++){
      std::cout << nlb_red[i] << " ";
    }
    std::cout << "\n";

    for(int i = 0; i < new_cons; i++){
        std::cout << nub_red[i] << " ";
    }
    std::cout << "\n";

    std::cout << nWSR << "\n";
    */

    int rnAl = problem_red.init(nH_red, ng_red, nA_red, NULL, NULL, nlb_red, nub_red, nWSR);

#ifdef DEBUG_SOLVE_TIME
    //problem_red.printProperties();
    //printf("cp5 %f ms\n", t3.getMs());
#endif

    (void)rnAl;
    int rnAl2 = problem_red.getPrimalSolution(nq_red);
    objVal = problem_red.getObjVal();
    if(rnAl2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    float solve_time = 0;//solve_timer.getMs();

#ifdef DEBUG_SOLVE_TIME
    //printf("cp6 %f ms\n", t3.getMs());


    printf("[NeuralSolverMPC] solve time: %.3f ms, size %d, %d, horizon %d\n", solve_time, new_vars, new_cons, setup->horizon);
#endif

    nc = 0;
    for(int i = 0; i < num_variables; i++)
    {
      if(n_var_elim[i])
      {
        nq_soln[i] = 0.0f;
      }
      else
      {
        nq_soln[i] = nq_red[nc];
        nc++;
      }
    }
    return std::make_tuple(solve_time, construct_time);
  }

}
