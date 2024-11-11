#ifndef CHEETAH_SOFTWARE_NEURAL_MPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_NEURAL_MPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include "cppTypes.h"
// #include "rl_action_lcmt.hpp"
// #include "wbc_params_lcmt.hpp"
// #include "rl_obs_t.hpp"
//#include <torch/torch.h>
//#include <torch/script.h>
#include <thread>
#include <lcm-cpp.hpp>

using Eigen::Array4f;
using Eigen::Array4i;


class NeuralGait
{
public:
  NeuralGait();
  ~NeuralGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int getIterationsBetweenMPC();
  int getIterationsToNextUpdate(int mpc_steps);
  Vec3<float> getVel();
  Vec3<float> getVelRpy();
  Vec3<float> getPos();
  Vec3<float> getFuturePos(float offset_time);
  Vec3<float> getFutureRpy(float offset_time);
  Vec3<float> getRpy();
  void initializeMPCTable(Vec4<int> offsets, Vec4<int> durations, int iters, Vec3<float> vel, Vec3<float> vel_rpy, Vec3<float> initial_pose);
  void setNextGait(int offset, int horizon, DMat<int> contact_table, int refIteration);
  void setNextVel(int offset, int horizon, DMat<float> vel_table, DMat<float> vel_rpy_table, int refIteration);
  void setNextIterationsBetweenMPC(int offset, int horizon, DMat<int> iteration_table, int refIteration);
  void advanceGait();
  void recomputeContacts();
  void set_dt(float dt);
  void reset();
  float* mpc_vel();
  float* mpc_vel_rpy();
  float* mpc_pose();
  float* mpc_rpy();
  int* mpc_gait();
  int* mpc_iters();
  void setIterations(int iterationsPerMPC, int currentIteration);
  void decrementIteration();
  void incrementIteration();
  void set_nIterations(int nIterations);
  void print_mpc_table();
  void print_iters_table();
  void print_vel_table();
  void print_vel_rpy_table();
  void print_pose_table();
  void print_rpy_table();
  DMat<float> get_cartesian_trajectory();
  void set_cartesian_trajectory(DMat<float> cartesian_trajectory);
  Vec4<int> _stance;
  Vec4<int> _swing;
  Vec4<int> swing_duration;
  Vec4<int> contact_duration;
  Vec4<int> contact_idx;
  Vec4<int> liftoff_idx;
  Vec4<int> next_contact_idx;
  Vec4<int> next_liftoff_idx;
  Vec4<int> contact_iteration;
  Vec4<int> liftoff_iteration;
  Vec4<int> next_contact_iteration;
  Vec4<int> next_liftoff_iteration;
  Vec4<int> swing_start;
  Vec4<int> contact_start;
  Array4f swing_progress;
  Array4f contact_progress;
  Vec3<float>  _vel;
  Vec3<float>  _vel_rpy;
  Vec3<float>  _pose;
  Vec3<float>  _rpy;
  int* _mpc_table;
  float* _vel_table;
  float* _vel_rpy_table;
  float* _pose_table;
  float* _rpy_table;
  int* _iterationsWBCtable;
  int _currentIteration;
  int _iterationsPerMPC;
  int iteration_prev_update = 0;
  int _iteration;
  bool recompute_swingduration_every_step;


private:
  int _nMPC_segments;
  Array4i _offsets_prev; // offset in mpc segments
  Array4i _durations_prev; // duration of step in mpc segments
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4i _offsets_next; // offset in mpc segments
  Array4i _durations_next; // duration of step in mpc segments
  Array4f _offsetsPrevFloat; // offsets in phase (0 to 1)
  Array4f _durationsPrevFloat; // durations in phase (0 to 1)
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  Array4f _offsetsNextFloat; // offsets in phase (0 to 1)
  Array4f _durationsNextFloat; // durations in phase (0 to 1)
  int _nIterations;
  bool _use_gait_smoothing;
  float _phase;
  int _mpc_table_length;
  float _dt;
};


class NeuralMPCLocomotion {
public:
  NeuralMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
  ~NeuralMPCLocomotion();
  void initialize();

  template<typename T>
  void run(ControlFSMData<T>& data, const DMat<float> & height_map);

  template<typename T>
  void runParamsFixed(ControlFSMData<T>& data, 
      const Vec3<T> & vel_cmd, const Vec3<T> & vel_rpy_cmd, const Vec2<T> (& fp_rel_cmd)[4], const Vec4<float> & fh_rel_cmd,
      const float footswing_height, const int iterationsBetweenMPC_act);


  template<typename T>
  void runParamsFixedWrapper(ControlFSMData<T>& data,
    const Vec3<T> & vel_cmd, const Vec3<T> & vel_rpy_cmd, const Vec2<T> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, const Vec4<int> & offsets_cmd,
    const Vec4<int> & durations_cmd, const float footswing_height_cmd, const int iterationsBetweenMPC_cmd, const DMat<float> & height_map, const bool use_gait_smoothing, 
    const bool use_vel_smoothing, const bool use_gait_cycling);

  
  template<typename T>
  void runParamsFixedNewStyleWrapper(ControlFSMData<T>& data,
    const Vec3<T> & vel_cmd, const Vec3<T> & vel_rpy_cmd, const Vec2<T> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, 
    const float footswing_height_cmd, const int iterationsBetweenMPC_act, const DMat<int> & mpc_table_update, const DMat<float> & vel_table_update, const DMat<float> & vel_rpy_table_update,  const DMat<int> & iterations_table_update, 
    const int planningHorizon, const int adaptationHorizon, const int adaptationSteps);
  
  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;
  
  int iterationCounter = 0;
  int iterationPrevQuery = 0;
  int iterationsBetweenMPC = 14;
  int iterationsBetweenMPC_cmd;
  int iterationsSinceCmd;
  int timeLastMPC = 0;

  float trajAll[12*36];
  void solveDenseMPC(int *mpcTable, int* itersList, ControlFSMData<float> &data);
  float objVal;

  Vec3<float> pFoot[4];

  NeuralGait custom;
  NeuralGait* gait;

  float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
  int horizonLength;

  bool _updatedCommand;
  bool _OFFLINE_MPC;

  std::tuple<float, float> solve_times;
  float total_update_time;
  float total_step_time;

  int _adaptationHorizon = 10;
  int _adaptationSteps = 10;

  FootSwingTrajectory<float> footSwingTrajectories[4];
  

private:
  void _UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,
      const DMat<float> & height_map);
  void _IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected, 
      const DMat<int> & idx_map);
  float getFootHeight(Vec3<float>& foot, const Vec3<float>& body_pos);
    //const DMat<float> & height_map);

  void _SetupCommand(ControlFSMData<float> & data);

  //void handleWBCActionLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const wbc_params_lcmt* msg);
  //void handleActionLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const rl_action_lcmt* msg);

  Vec3<float> _fin_foot_loc[4];
  float grid_size = 1.0/30.0; //0.015;

  Vec3<float> v_des_world;
  Vec3<float> rpy_des;
  Vec3<float> v_rpy_des;

  float _body_height = 0.31;
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
  float dt;
  float dtMPC;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  NeuralGait trotting, bounding, pronking, galloping, standing, trotRunning, cyclic;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstUpdate = true;
  int iterationFirstUpdate = 0;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float swingTimeSoFar[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  bool _smooth_gait;
  bool _smooth_vel;
  bool _cycle_gait;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;

  MIT_UserParameters* _parameters = nullptr;

  Vec3<float> vel_act;
  Vec3<float> vel_act_prev;
  Vec3<float> vel_act_next;
  Vec3<float> vel_rpy_act;
  Vec2<float> fp_rel_act[4];
  Vec4<float>  fh_rel_act;
  Vec4<int> offsets_act_prev;
  Vec4<int> offsets_act;
  Vec4<int> offsets_act_next;
  Vec4<int> durations_act_prev;
  Vec4<int> durations_act;
  Vec4<int> durations_act_next;
  float footswing_height_act;

  bool _policyRecieved;
  bool _terminalFlag;
  int lastBroadcastIteration = -1;

  bool nmpc_jump_ctrl = true;
  bool nmpc_block_invalid = true;
  bool nmpc_adaptive_foot_placements = true;
  bool nmpc_use_vel_control = true;
  bool zero_yaw = false;

  lcm::LCM _neuralLCM;

  DMat<int> local_iterations_table;//(100, 1);
  DMat<int> local_mpc_table;//(400, 1);
  DMat<float> local_vel_table;//(300, 1);
  DMat<float> local_vel_rpy_table;//(300, 1);
  
  
  void neuralLCMThread() { 
      while (!_terminalFlag) {
        std::cout << "handling\n";
        _neuralLCM.handle();
      }
  }
  
  std::thread _neuralLCMThread;



};


#endif //CHEETAH_SOFTWARE_NEURAL_MPCLOCOMOTION_H
