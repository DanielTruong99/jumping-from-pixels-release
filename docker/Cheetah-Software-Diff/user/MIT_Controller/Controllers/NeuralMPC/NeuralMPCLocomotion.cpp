//line 1772

#include <iostream>
#include <eigen3/Eigen/Core>
#include <unistd.h>
#include <chrono>

#include "NeuralMPCLocomotion.h"
#include "NeuralMPC_interface.h"

///////////////
// GAIT, SMOOTHED
///////////////
NeuralGait::NeuralGait() 
{
  int nMPC_segments = 10;
  _nIterations = nMPC_segments;
  _mpc_table_length = 50;
  _mpc_table = new int[_mpc_table_length*4];
  _vel_table = new float[_mpc_table_length*3];
  _vel_rpy_table = new float[_mpc_table_length*3];
  _pose_table = new float[_mpc_table_length*3];
  _rpy_table = new float[_mpc_table_length*3];
  _iterationsWBCtable = new int[_mpc_table_length];
  _iterationsPerMPC = 14; 
  _dt = 0.002; //dummy
  recompute_swingduration_every_step = false;
  initializeMPCTable({0, 0, 0, 0}, {10, 10, 10, 10}, 14, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.29});
  //std::cout << "NeuralGait::NeuralGait(): INITIALIZED \n";
}


NeuralGait::~NeuralGait() {
  delete[] _mpc_table;
  delete[] _vel_table;
  delete[] _pose_table;
  delete[] _vel_rpy_table;
  delete[] _rpy_table;
  delete[] _iterationsWBCtable;
}

void NeuralGait::reset(){
  int nMPC_segments = 10;
  _nIterations = nMPC_segments;
  _mpc_table_length = 50;
  //_mpc_table = new int[_mpc_table_length*4];
  //_vel_table = new float[_mpc_table_length*3];
  //_vel_rpy_table = new float[_mpc_table_length*3];
  //_pose_table = new float[_mpc_table_length*3];
  //_rpy_table = new float[_mpc_table_length*3];
  //_iterationsWBCtable = new int[_mpc_table_length];
  _iterationsPerMPC = 14; 
  _dt = 0.002; //dummy
  //recompute_swingduration_every_step = false;
  initializeMPCTable({0, 0, 0, 0}, {10, 10, 10, 10}, 14, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.29});
  //std::cout << "NeuralGait::NeuralGait(): INITIALIZED \n";
  iteration_prev_update = 0;
}

DMat<float> NeuralGait::get_cartesian_trajectory(){
  DMat<float> cartesian_trajectory(_nIterations * 16, 1);
  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      cartesian_trajectory(i*16+j, 0) = _rpy_table[((_iteration + i)  % _mpc_table_length)*3+j];
      cartesian_trajectory(i*16+3+j, 0) = _pose_table[((_iteration + i) % _mpc_table_length)*3+j];
      cartesian_trajectory(i*16+6+j, 0) = _vel_rpy_table[((_iteration + i) % _mpc_table_length)*3+j];
      cartesian_trajectory(i*16+9+j, 0) = _vel_table[((_iteration + i)  % _mpc_table_length)*3+j];
    }
    for(int j = 0; j < 4; j++){
      cartesian_trajectory(i*16+12+j, 0) = _mpc_table[((_iteration + i) % _mpc_table_length)*4+j];
    }
  }
  return cartesian_trajectory;
}

void NeuralGait::set_cartesian_trajectory(DMat<float> cartesian_trajectory){

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      _pose_table[((_iteration+i) % _mpc_table_length)*3+j] = cartesian_trajectory(i*16+j, 0);
      _vel_table[((_iteration+i) % _mpc_table_length)*3+j] = cartesian_trajectory(i*16+3+j, 0);
      _rpy_table[((_iteration+i) % _mpc_table_length)*3+j] = cartesian_trajectory(i*16+6+j, 0);
      _vel_rpy_table[((_iteration+i)% _mpc_table_length)*3+j] = cartesian_trajectory(i*16+9+j, 0);
    }
    for(int j = 0; j < 4; j++){
      _mpc_table[((_iteration+i) % _mpc_table_length)*4+j] = cartesian_trajectory(i*16+12+j, 0);
    }
  }
}

void NeuralGait::set_dt(float dt){
  _dt = dt;
}

void NeuralGait::set_nIterations(int nIterations){
  _nIterations = nIterations;
  //std::cout << "[NeuralMPCLocomotion] SET ITERATIONS" << nIterations << "\n";
}

void NeuralGait::initializeMPCTable(Vec4<int> offsets, Vec4<int> durations, int iters, Vec3<float> vel, Vec3<float> vel_rpy, Vec3<float> initial_pose) {
  //std::cout << "[NeuralGait] initialized with " << offsets << ", " << durations << "\n";
  int defaultCycle = 10;
  for(int i = 0; i < defaultCycle; i++)
  {
    //int iter = (i + _iteration + 1) % defaultIterations;
    //Array4i progress = iter - _offsets_next;
    for(int j = 0; j < 4; j++)
    {
      //if(progress[j] < 0) progress[j] += _nIterations;
      if((offsets[j] + durations[j] > i and i >= offsets[j]) or 
          (offsets[j] + durations[j] > defaultCycle and i < offsets[j] + durations[j] - defaultCycle)){
        for(int k = 0; k < _mpc_table_length/defaultCycle; k++){
          _mpc_table[(i+defaultCycle*k)*4 + j] = 1;
        }
      }
      else{
        for(int k = 0; k < _mpc_table_length/defaultCycle; k++){
          _mpc_table[(i+defaultCycle*k)*4 + j] = 0;
        }
      }
    }
  }

  for(int i = 0; i < 4; i++){
    contact_idx[i] = 0;
    liftoff_idx[i] = 0;
    next_contact_idx[i] = offsets[i];
    next_liftoff_idx[i] = offsets[i] + durations[i];
    swing_duration[i] = (defaultCycle - durations[i]) * iters;
    swing_start[i] = offsets[i] + durations[i];
    contact_duration[i] = (durations[i]) * iters;
    contact_start[i] = offsets[i];
    swing_progress[i] = 0;
    contact_progress[i] = 0;
    contact_iteration[i] = 0;
    liftoff_iteration[i] = 0;
    next_contact_iteration[i] = offsets[i] * iters;
    next_liftoff_iteration[i] = (offsets[i] + durations[i]) * iters;
  }
  //print_mpc_table();

  _pose_table[0] = initial_pose[0];
  _pose_table[1] = initial_pose[1];
  _pose_table[2] = 0.29;
  _rpy_table[0] = 0;
  _rpy_table[1] = 0;
  _rpy_table[2] = 0;

  for(int i = 0; i < _mpc_table_length; i++){
    _iterationsWBCtable[i] = iters;
    for(int j = 0; j < 3; j++){
      _vel_table[i*3+j] = vel[j];
      _vel_rpy_table[i*3+j] = vel_rpy[j];
      if(i > 0){
        _pose_table[i*3+j] = _pose_table[(i-1)*3+j] + vel[j] * iters * _dt;
        _rpy_table[i*3+j] = _rpy_table[(i-1)*3+j] + vel_rpy[j] * iters * _dt;
      }
    }
  }
  for(int i = 0; i < 3; i++){
    _vel[i] = vel[i];
    _pose[i] = _pose_table[i];
    _vel_rpy[i] = vel_rpy[i];
    _rpy[i] = _rpy_table[i];
  }

  iteration_prev_update = 0;
  _currentIteration = 0;
  _iteration = 0;
}


void NeuralGait::setNextGait(int offset, int horizon, DMat<int> contact_table, int refIteration) {
  //std::cout << "refIteration, offset, horizon" << refIteration << " " << offset << " " << horizon << "\n";

  for(int i = 0; i < horizon; i++){
    for(int j = 0; j < 4; j++){
      _mpc_table[(refIteration * 4 + offset*4 + i*4 + j) % (_mpc_table_length * 4)] = contact_table(i*4 + j, 0);
    }
  }
  //recomputeContacts();
}

void NeuralGait::setNextVel(int offset, int horizon, DMat<float> vel_table, DMat<float> vel_rpy_table, int refIteration) {
  //print_pose_table();
  //std::cout << "setNextVel( offset = " << offset << " horizon = " << horizon << " refIteration = " << refIteration << ")\n";
  for(int i = 0; i < horizon; i++){
    for(int j = 0; j < 3; j++){
      _vel_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] = vel_table(i*3 + j, 0);
      _vel_rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] = vel_rpy_table(i*3 + j, 0);
    }
  }
  for(int i = 1; i < _mpc_table_length - horizon - offset; i++){
    for(int j = 0; j < 3; j++){
      //std::cout << vel_table((i-1)*3 + j, 0) << " " << _iterationsWBCtable[(_iteration + offset + i - 1) % (_mpc_table_length)] << " " << _dt << " " << _pose_table[(_iteration * 3 + offset*3 + (i-1)*3 + j) % (_mpc_table_length * 3)] << "\n";
      _pose_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] = _vel_table[(refIteration * 3 + offset*3 + (i-1)*3 + j) % (_mpc_table_length * 3)] * _iterationsWBCtable[(refIteration + offset + i - 1) % (_mpc_table_length)] * _dt + _pose_table[(refIteration * 3 + offset*3 + (i-1)*3 + j) % (_mpc_table_length * 3)];
      _rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] = _vel_rpy_table[(refIteration * 3 + offset*3 + (i-1)*3 + j) % (_mpc_table_length * 3)] * _iterationsWBCtable[(refIteration + offset + i - 1) % (_mpc_table_length)] * _dt + _rpy_table[(refIteration * 3 + offset*3 + (i-1)*3 + j) % (_mpc_table_length * 3)];
      if(_rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] > M_PI){
        _rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] -= 2*M_PI;
      }
      if(_rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] < -M_PI){
        _rpy_table[(refIteration * 3 + offset*3 + i*3 + j) % (_mpc_table_length * 3)] += 2*M_PI;
      }
    }
  }
  //print_pose_table();
  //advanceGait();
}

void NeuralGait::setNextIterationsBetweenMPC(int offset, int horizon, DMat<int> iteration_table, int refIteration) {
 //std::cout << "_iteration: " << _iteration << "\n";
  for(int i = 0; i < horizon; i++){
    _iterationsWBCtable[(refIteration + offset + i) % (_mpc_table_length)] = iteration_table(i, 0);
  }
  //advanceGait();
}

void NeuralGait::advanceGait(){
  // current iteration is _iteration
  //std::cout << "currentIteration, iteration_prev_update, iterationsWBCTable[_iteration]" << _currentIteration << " " << iteration_prev_update <<  " " << _iterationsWBCtable[_iteration] << "\n";
  
  if(_currentIteration - iteration_prev_update >= _iterationsWBCtable[_iteration] or _currentIteration == 0){ // need mpc update
    //std::cout << "iteration" << _iteration << "\n";
    _iterationsPerMPC = _iterationsWBCtable[(_iteration % _mpc_table_length)];
    
    //std::cout << "iterationsPerMPC" << _iterationsPerMPC << "\n";
    //std::cout << "recompute pose, iteration " << _iteration << "\n";
    for(int i = 0; i < 3; i++){
      _vel[i] = _vel_table[((_iteration) % _mpc_table_length)*3+i];
      _vel_rpy[i] = _vel_rpy_table[((_iteration) % _mpc_table_length)*3+i];
      _pose[i] = _pose_table[((_iteration) % _mpc_table_length)*3+i];
      _rpy[i] = _rpy_table[((_iteration) % _mpc_table_length)*3+i];
     }

    for(int i = 0; i < 4; i++){
      if(_mpc_table[(_iteration % _mpc_table_length)*4 + i] == 0){ // foot i in in swing
        //std::cout << "swing at: " << _iteration % _mpc_table_length << "\n";
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 1){ //we just broke contact
          //std::cout << "stance at: " << ((_mpc_table_length + _iteration-1) % _mpc_table_length) << "\n";
          liftoff_idx[i] = _iteration;
          liftoff_iteration[i] = _currentIteration;
        }
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 1 or recompute_swingduration_every_step){ //we just broke contact
          next_contact_iteration[i] = _currentIteration;
          for(int j = 1; j < _mpc_table_length; j++){ // find time to next contact
            next_contact_iteration[i] = next_contact_iteration[i] + _iterationsWBCtable[(_iteration+j-1)%_mpc_table_length];
            if(_mpc_table[((_iteration+j)%_mpc_table_length)*4+i] == 1){
              //std::cout << "stance at: " << (_iteration+j) % _mpc_table_length << "\n";
              next_contact_idx[i] = ((_iteration+j)%_mpc_table_length);
              break;
            }
          }
          next_liftoff_iteration[i] = next_contact_iteration[i];
          for(int j = next_contact_idx[i]+1; j < (_mpc_table_length + next_contact_idx[i]); j++){ // then, find time to next liftoff
            next_liftoff_iteration[i] = next_liftoff_iteration[i] + _iterationsWBCtable[(j-1)%_mpc_table_length];
            if(_mpc_table[(j%_mpc_table_length)*4+i] == 0 or j == (next_contact_idx[i] + 10) % _mpc_table_length){ 
              next_liftoff_idx[i] = j %_mpc_table_length;
              break;
            }
          }
          /*
          swing_duration[i] = ((_mpc_table_length + next_contact_idx[i] - liftoff_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
          _swing[i] = swing_duration[i];
          _stance[i] = ((_mpc_table_length + next_liftoff_idx[i] - next_contact_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
          swing_progress[i] = (float)((_mpc_table_length * _iterationsPerMPC + _currentIteration - liftoff_idx[i] * _iterationsPerMPC) % (_mpc_table_length  * _iterationsPerMPC)) / (swing_duration[i]);
          contact_progress[i] = 0.;
          */
          //std::cout << "currentIteration, liftoff_iteration, next_contact_iteration" << _currentIteration << " " << liftoff_iteration[i] << " " << next_contact_iteration[i] << "\n";
          }
          swing_duration[i] = next_contact_iteration[i] - liftoff_iteration[i];
          _swing[i] = swing_duration[i];
          _stance[i] = next_liftoff_iteration[i] - next_contact_iteration[i];
          swing_progress[i] = (float)(_currentIteration - liftoff_iteration[i]) / swing_duration[i];
          //std::cout << "CP " << contact_progress << "\n";
          if(contact_progress[i] >= 1.0 or contact_progress[i] == 0){
            contact_progress[i] = 0;
          } else{
            contact_progress[i] = 1.0;
          }
          //std::cout << "CP " << contact_progress << "\n";
          //std::cout << "swing, contact progress" << swing_progress[i] << " " << contact_progress[i] << "\n";
        
      }
      else{ // foot i is in contact
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 0){ //we just made contact
          contact_idx[i] = _iteration;
          contact_iteration[i] = _currentIteration;
        }
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 0 or recompute_swingduration_every_step){
          next_liftoff_iteration[i] = _currentIteration;
          for(int j = 1; j < _mpc_table_length; j++){ // find time to next liftoff
            next_liftoff_iteration[i] = next_liftoff_iteration[i] + _iterationsWBCtable[(_iteration+j-1)%_mpc_table_length];
            if(_mpc_table[((_iteration+j)%_mpc_table_length)*4+i] == 0 or j == 10){
              next_liftoff_idx[i] = ((_iteration+j)%_mpc_table_length);
              break;
            }
          }
          next_contact_iteration[i] = next_liftoff_iteration[i];
          for(int j = next_liftoff_idx[i]+1; j < _mpc_table_length+next_liftoff_idx[i]; j++){ // then, find time to next contact
            next_contact_iteration[i] = next_contact_iteration[i] + _iterationsWBCtable[(j-1)%_mpc_table_length];
            if(_mpc_table[(j%_mpc_table_length)*4+i] == 1){
              next_contact_idx[i] = j % _mpc_table_length;
              break;
            }
        }

        /*
        contact_duration[i] = ((_mpc_table_length + next_liftoff_idx[i] - contact_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
        _swing[i] = ((_mpc_table_length + next_contact_idx[i] - next_liftoff_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
        _stance[i] = contact_duration[i];
        contact_progress[i] = (float)((_mpc_table_length * _iterationsPerMPC + _currentIteration - contact_idx[i] * _iterationsPerMPC) % (_mpc_table_length  * _iterationsPerMPC)) / (contact_duration[i]); // for SwingState
        swing_progress[i] = 0.; // TODO: will take more math to make the contact progress correct given true iterations between mpc...
        */
        }
        contact_duration[i] = next_liftoff_iteration[i] - contact_iteration[i];
        _swing[i] = next_contact_iteration[i] - next_liftoff_iteration[i];
        _stance[i] = contact_duration[i];
        contact_progress[i] = (float)(_currentIteration - contact_iteration[i]) / contact_duration[i];
        if(swing_progress[i] >= 1.0 or swing_progress[i] == 0){
          swing_progress[i] = 0;
        } else{
          swing_progress[i] = 1.0;
        }
        //std::cout << "swing, contact progress, contact duration" << swing_progress[i] << " " << contact_progress[i] << " " << contact_duration[i] << "\n";
        
      }
    }
    _iteration = (_iteration + 1) % _mpc_table_length;
    //std::cout << "incremented iteration" << _iteration << "\n";
    iteration_prev_update = _currentIteration;

    // update previous pose for next cycle
    for(int i = 0; i < 3; i++){
      //std::cout << _pose_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i]  << " " <<_vel_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] << " " <<_iterationsWBCtable[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] << " " <<_dt;
      _pose_table[(_iteration % _mpc_table_length)*3 + i] = _pose_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] + _vel_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] * _iterationsWBCtable[(_mpc_table_length + _iteration-1) % _mpc_table_length+ i] * _dt;
      _rpy_table[(_iteration % _mpc_table_length)*3 + i] = _rpy_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] + _vel_rpy_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*3 + i] * _iterationsWBCtable[(_mpc_table_length + _iteration-1) % _mpc_table_length+ i] * _dt;
      if(_rpy_table[(_iteration % _mpc_table_length)*3 + i] > M_PI){
        _rpy_table[(_iteration % _mpc_table_length)*3 + i] -= 2*M_PI;
      }
      if(_rpy_table[(_iteration % _mpc_table_length)*3 + i] < -M_PI){
        _rpy_table[(_iteration % _mpc_table_length)*3 + i] += 2*M_PI;
      }
    }

  }
  else{
    for(int i = 0; i < 4; i++){
      //std::cout << "currentIteration, liftoff_iteration[i]: " << _currentIteration << " " << liftoff_iteration[i] << "\n";
      //std::cout << "((_iteration-1) % _mpc_table_length)*4 + i, _mpc_table[*]" << ((_iteration-1 + _mpc_table_length) % _mpc_table_length)*4 + i << " " << _mpc_table[((_iteration-1) % _mpc_table_length)*4 + i] << "\n";
      if(_mpc_table[((_iteration-1 + _mpc_table_length) % _mpc_table_length)*4 + i] == 0){ // foot i in in swing
          swing_progress[i] = (float)(_currentIteration - liftoff_iteration[i]) / swing_duration[i];
          //std::cout << "CP " << contact_progress << "\n";
          if(contact_progress[i] >= 1.0 or contact_progress[i] == 0){
            contact_progress[i] = 0;
          } else{
            contact_progress[i] = 1.0;
          }
          //std::cout << "CP " << contact_progress << "\n";
      }
      else{
        contact_progress[i] = (float)(_currentIteration - contact_iteration[i]) / contact_duration[i];

        if(swing_progress[i] >= 1.0 or swing_progress[i] == 0){
            swing_progress[i] = 0;
        } else{
          swing_progress[i] = 1.0;
        }
      }
      //std::cout << "b swing, contact progress" << swing_progress[i] << " " << contact_progress[i] << "\n";
  }
  }

  // std::cout << "recompute pose, iteration " << _iteration << " phase " << _phase << "\n";
  // for(int i = 0; i < 3; i++){
  //   _vel[i] = _vel_table[((_iteration-1) % _mpc_table_length)*3+i];
  //   _vel_rpy[i] = _vel_rpy_table[((_iteration-1) % _mpc_table_length)*3+i];
  //   _pose[i] = _pose_table[((_iteration-1) % _mpc_table_length)*3+i] * (1 - _phase) + _pose_table[((_iteration) % _mpc_table_length)*3+i] * _phase;
  //   _rpy[i] = _rpy_table[((_iteration-1) % _mpc_table_length)*3+i] * (1 - _phase) + _rpy_table[((_iteration) % _mpc_table_length)*3+i] * _phase;
  //  }
 

}

void NeuralGait::recomputeContacts(){
  
    _iterationsPerMPC = _iterationsWBCtable[(_iteration % _mpc_table_length)];
    for(int i = 0; i < 3; i++){
      _vel[i] = _vel_table[(_iteration % _mpc_table_length)*3+i];
      _vel_rpy[i] = _vel_rpy_table[(_iteration % _mpc_table_length)*3+i];
      _pose[i] = _pose_table[(_iteration % _mpc_table_length)*3+i];
      _rpy[i] = _rpy_table[(_iteration % _mpc_table_length)*3+i];
    }
    //std::cout << "iterationsPerMPC" << _iterationsPerMPC << "\n";

    for(int i = 0; i < 4; i++){
      if(_mpc_table[(_iteration % _mpc_table_length)*4 + i] == 0){ // foot i in in swing
        //std::cout << "swing at: " << _iteration % _mpc_table_length << "\n";
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 1){ //we just broke contact
          //std::cout << "stance at: " << ((_mpc_table_length + _iteration-1) % _mpc_table_length) << "\n";
          liftoff_idx[i] = _iteration;
          liftoff_iteration[i] = _currentIteration;
        }
        
          next_contact_iteration[i] = _currentIteration;
          for(int j = 1; j < _mpc_table_length; j++){ // find time to next contact
            next_contact_iteration[i] = next_contact_iteration[i] + _iterationsWBCtable[(_iteration+j-1)%_mpc_table_length];
            if(_mpc_table[((_iteration+j)%_mpc_table_length)*4+i] == 1){
              //std::cout << "stance at: " << (_iteration+j) % _mpc_table_length << "\n";
              next_contact_idx[i] = ((_iteration+j)%_mpc_table_length);
              break;
            }
          }
          next_liftoff_iteration[i] = next_contact_iteration[i];
          for(int j = next_contact_idx[i]+1; j < (_mpc_table_length + next_contact_idx[i]); j++){ // then, find time to next liftoff
            next_liftoff_iteration[i] = next_liftoff_iteration[i] + _iterationsWBCtable[(j-1)%_mpc_table_length];
            if(_mpc_table[(j%_mpc_table_length)*4+i] == 0 or j == (10 + next_contact_idx[i]) % _mpc_table_length){ 
              next_liftoff_idx[i] = j %_mpc_table_length;
              break;
            }
          }
          /*
          swing_duration[i] = ((_mpc_table_length + next_contact_idx[i] - liftoff_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
          _swing[i] = swing_duration[i];
          _stance[i] = ((_mpc_table_length + next_liftoff_idx[i] - next_contact_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
          swing_progress[i] = (float)((_mpc_table_length * _iterationsPerMPC + _currentIteration - liftoff_idx[i] * _iterationsPerMPC) % (_mpc_table_length  * _iterationsPerMPC)) / (swing_duration[i]);
          contact_progress[i] = 0.;
          */
          //std::cout << "currentIteration, liftoff_iteration, next_contact_iteration" << _currentIteration << " " << liftoff_iteration[i] << " " << next_contact_iteration[i] << "\n";
          
          swing_duration[i] = next_contact_iteration[i] - liftoff_iteration[i];
          _swing[i] = swing_duration[i];
          _stance[i] = next_liftoff_iteration[i] - next_contact_iteration[i];
          swing_progress[i] = (float)(_currentIteration - liftoff_iteration[i]) / swing_duration[i];
          contact_progress[i] = 0;
          //std::cout << "swing, contact progress" << swing_progress[i] << " " << contact_progress[i] << "\n";
        
      }
      else{ // foot i is in contact
        if(_mpc_table[((_mpc_table_length + _iteration-1) % _mpc_table_length)*4 + i] == 0){ //we just made contact
          contact_idx[i] = _iteration;
          contact_iteration[i] = _currentIteration;
        }

          next_liftoff_iteration[i] = _currentIteration;
          for(int j = 1; j < _mpc_table_length; j++){ // find time to next liftoff
            next_liftoff_iteration[i] = next_liftoff_iteration[i] + _iterationsWBCtable[(_iteration+j-1)%_mpc_table_length];
            if(_mpc_table[((_iteration+j)%_mpc_table_length)*4+i] == 0 or j == 10){
              next_liftoff_idx[i] = ((_iteration+j)%_mpc_table_length);
              break;
            }
          }
          next_contact_iteration[i] = next_liftoff_iteration[i];
          for(int j = next_liftoff_idx[i]+1; j < _mpc_table_length+next_liftoff_idx[i]; j++){ // then, find time to next contact
            next_contact_iteration[i] = next_contact_iteration[i] + _iterationsWBCtable[(j-1)%_mpc_table_length];
            if(_mpc_table[(j%_mpc_table_length)*4+i] == 1){
              next_contact_idx[i] = j % _mpc_table_length;
              break;
            }
        }

        /*
        contact_duration[i] = ((_mpc_table_length + next_liftoff_idx[i] - contact_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
        _swing[i] = ((_mpc_table_length + next_contact_idx[i] - next_liftoff_idx[i]) % _mpc_table_length) * _iterationsPerMPC;
        _stance[i] = contact_duration[i];
        contact_progress[i] = (float)((_mpc_table_length * _iterationsPerMPC + _currentIteration - contact_idx[i] * _iterationsPerMPC) % (_mpc_table_length  * _iterationsPerMPC)) / (contact_duration[i]); // for SwingState
        swing_progress[i] = 0.; // TODO: will take more math to make the contact progress correct given true iterations between mpc...
        */
        
        contact_duration[i] = next_liftoff_iteration[i] - contact_iteration[i];
        _swing[i] = next_contact_iteration[i] - next_liftoff_iteration[i];
        _stance[i] = contact_duration[i];
        contact_progress[i] = (float)(_currentIteration - contact_iteration[i]) / contact_duration[i];
        swing_progress[i] = 0;
        //std::cout << "swing, contact progress" << swing_progress[i] << " " << contact_progress[i] << "\n";
        
      }
    }
  
}

void NeuralGait::print_mpc_table(){
  std::cout << "NeuralGait::print_mpc_table() enter printer! " << _mpc_table_length << "\n";

  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _mpc_table[i*4] << " " << _mpc_table[i*4+1] << " " << _mpc_table[i*4+2] << " " << _mpc_table[i*4+3] << "\n";
        }
}

void NeuralGait::print_iters_table(){
  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _iterationsWBCtable[i] << " " ;
        }
        std::cout << "\n";
}

void NeuralGait::print_vel_table(){
  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _vel_table[i*3] << " " << _vel_table[i*3+1] << " " << _vel_table[i*3+2] << "\n";
        }
}

void NeuralGait::print_vel_rpy_table(){
  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _vel_rpy_table[i*3] << " " << _vel_rpy_table[i*3+1] << " " << _vel_rpy_table[i*3+2] << "\n";
        }
}

void NeuralGait::print_rpy_table(){
  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _rpy_table[i*3] << " " << _rpy_table[i*3+1] << " " << _rpy_table[i*3+2] << "\n";
        }
}

void NeuralGait::print_pose_table(){
  for(int i=0; i<_mpc_table_length; i++){
          std::cout << _pose_table[i*3] << " " << _pose_table[i*3+1] << " " << _pose_table[i*3+2] << "\n";
        }
}

Vec4<float> NeuralGait::getContactState() {
  
  return contact_progress.matrix();
}

Vec4<float> NeuralGait::getSwingState() {
  
  
  return swing_progress.matrix();
}

int NeuralGait::getIterationsBetweenMPC(){
  return _iterationsPerMPC;
}

int NeuralGait::getIterationsToNextUpdate(int mpc_steps){
  int its = 0;
  for(int i = 0; i < mpc_steps; i++){
    its += _iterationsWBCtable[((i+_iteration) % _mpc_table_length)];
  }
  return its;
}

Vec3<float> NeuralGait::getVel() {
  return _vel;
}

Vec3<float> NeuralGait::getVelRpy() {
  return _vel_rpy;
}

Vec3<float> NeuralGait::getPos() {
  return _pose;
}

Vec3<float> NeuralGait::getFuturePos(float offset_time) {
  // assume constant iterationsBetweenMPC for now
  int offset_steps = (int)(offset_time / _dt);
  int futureStep = _currentIteration + offset_steps;
  int futureIteration = (futureStep - iteration_prev_update) / _iterationsPerMPC + _iteration;
  float futurePhase = (float)((futureStep - iteration_prev_update) % _iterationsPerMPC) / (float)(_iterationsPerMPC);
  Vec3<float> future_pose;
  future_pose[0] = _pose_table[(futureIteration % _mpc_table_length)*3] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3] * futurePhase;
  future_pose[1] = _pose_table[(futureIteration % _mpc_table_length)*3+1] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3+1] * futurePhase;
  future_pose[2] = _pose_table[(futureIteration % _mpc_table_length)*3+2] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3+2] * futurePhase;
  return future_pose;
}

Vec3<float> NeuralGait::getFutureRpy(float offset_time) {
  // assume constant iterationsBetweenMPC for now
  int offset_steps = (int)(offset_time / _dt);
  int futureStep = _currentIteration + offset_steps;
  int futureIteration = (futureStep - iteration_prev_update) / _iterationsPerMPC + _iteration;
  float futurePhase = (float)((futureStep - iteration_prev_update) % _iterationsPerMPC) / (float)(_iterationsPerMPC);
  Vec3<float> future_rpy;
  future_rpy[0] = _rpy_table[(futureIteration % _mpc_table_length)*3] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3] * futurePhase;
  future_rpy[1] = _rpy_table[(futureIteration % _mpc_table_length)*3+1] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3+1] * futurePhase;
  future_rpy[2] = _rpy_table[(futureIteration % _mpc_table_length)*3+2] * (1 - futurePhase) + _pose_table[((futureIteration+1) % _mpc_table_length)*3+2] * futurePhase;
  return future_rpy;
}

/*
Vec3<float> NeuralGait::getRaibertTarget(int foot_idx) {
  // assume constant iterationsBetweenMPC for now
  

}
*/

Vec3<float> NeuralGait::getRpy() {
  return _rpy;
}

float* NeuralGait::mpc_vel() {
  float* local_vel_table = new float[_nIterations * 3];

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      local_vel_table[i*3+j] = _vel_table[((i+_iteration) % _mpc_table_length)*3+j];
    }
  }


  return local_vel_table;
}

float* NeuralGait::mpc_vel_rpy() {
  float* local_vel_rpy_table = new float[_nIterations * 3];

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      local_vel_rpy_table[i*3+j] = _vel_rpy_table[((i+_iteration) % _mpc_table_length)*3+j];
    }
  }


  return local_vel_rpy_table;
}

float* NeuralGait::mpc_pose() {
  //std::cout << "nIterations " << _nIterations << "\n";
  float* local_pose_table = new float[_nIterations * 3];

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      local_pose_table[i*3+j] = _pose_table[((i+_iteration) % _mpc_table_length)*3+j];
    }
  }


  return local_pose_table;
}

float* NeuralGait::mpc_rpy() {
  //std::cout << "nIterations " << _nIterations << "\n";
  float* local_rpy_table = new float[_nIterations * 3];

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 3; j++){
      local_rpy_table[i*3+j] = _rpy_table[((i+_iteration) % _mpc_table_length)*3+j];
      if(local_rpy_table[j] - local_rpy_table[i*3+j] > M_PI){ // UNRESOLVED BUG CAUSES FAILURE AT YAW=PI
        local_rpy_table[i*3+j] = local_rpy_table[i*3+j] + 2*M_PI;
      }
      else if(local_rpy_table[j] - local_rpy_table[i*3+j] < -M_PI){
        local_rpy_table[i*3+j] = local_rpy_table[i*3+j] - 2*M_PI;
      }
    }
  }


  return local_rpy_table;
}

int* NeuralGait::mpc_gait() {


  int* local_mpc_table = new int[_nIterations * 4];

  for(int i = 0; i < _nIterations; i++){
    for(int j = 0; j < 4; j++){
      local_mpc_table[i*4+j] = _mpc_table[((i+_iteration) % _mpc_table_length)*4+j];
    }
  }


  return local_mpc_table;

}

int* NeuralGait::mpc_iters() {

  int* local_iteration_table = new int[_nIterations];

  for(int i = 0; i < _nIterations; i++){
    local_iteration_table[i] = _iterationsWBCtable[(i+_iteration) % _mpc_table_length];
  }


  return local_iteration_table;

}

void NeuralGait::setIterations(int iterationsPerMPC, int currentIteration) {
  /*
  if(currentIteration - iteration_prev_update >= _iterationsPerMPC){
    iteration_prev_update = currentIteration;
    _iteration = (_iteration + 1) % _mpc_table_length;
  }*/
  //_iteration = (currentIteration / iterationsPerMPC) % _mpc_table_length;
  _phase = (float)(currentIteration % (iterationsPerMPC)) / (float) (iterationsPerMPC);
  _currentIteration = currentIteration;
  _iterationsPerMPC = iterationsPerMPC;
  advanceGait();
  //std::cout << "iteration " << _iteration << " phase " << _phase << "\n";
}



////////////////////
// Controller
////////////////////

NeuralMPCLocomotion::NeuralMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  _neuralLCM(getLcmUrl(255))
{
  _parameters = parameters;
  //dtMPC = dt * iterationsBetweenMPC;
  //printf("[Neural MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
  //    dt, iterationsBetweenMPC, dtMPC);
  float dtMPClist[horizonLength];
  for(int i=0; i<horizonLength; i++){
    dtMPClist[i] = dt * iterationsBetweenMPC;
  }
  neural_setup_problem(dtMPClist,horizonLength,0.4,120);
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  _terminalFlag = false;
  // if(parameters->nmpc_use_lcm and parameters->use_lcm_comm){
  //   _neuralLCM.subscribe("rl_gait_action", &NeuralMPCLocomotion::handleActionLCM, this);
  //   _neuralLCM.subscribe("wbc_command", &NeuralMPCLocomotion::handleWBCActionLCM, this);
  //   if(parameters->nmpc_lcm_nonblocking){
  //     _neuralLCMThread = std::thread(&NeuralMPCLocomotion::neuralLCMThread, this);
  //   }
  // }

  //initSparseMPC();
  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
  rpy_des.setZero();
  v_rpy_des.setZero();

  iterationsBetweenMPC_cmd = iterationsBetweenMPC;
  // initialization
  for(int i=0; i<3; i++){ vel_act[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_prev[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_next[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = 0.0;}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = 0.0;} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = 0.0;}
  footswing_height_act = 0.06;
  offsets_act_prev[0] = 0; offsets_act_prev[1] = 0; offsets_act_prev[2] = 0; offsets_act_prev[3] = 0;
  for(int i=0; i<4; i++){ durations_act_prev[i] = 10; }
  offsets_act[0] = 0; offsets_act[1] = 0; offsets_act[2] = 0; offsets_act[3] = 0;
  for(int i=0; i<4; i++){ durations_act[i] = 10; }
  offsets_act_next[0] = 0; offsets_act_next[1] = 0; offsets_act_next[2] = 0; offsets_act_next[3] = 0;
  for(int i=0; i<4; i++){ durations_act_next[i] = 10; }

  _smooth_gait = true;
  _smooth_vel = false;

  _updatedCommand = false;
  _OFFLINE_MPC = false;

  custom.recompute_swingduration_every_step = false; // ADJUST!! parameters->recompute_swingduration_every_step;

  custom.set_dt(dt);
  custom.set_nIterations(horizonLength);
  custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, {0., 0., 0.29});

  local_iterations_table.resize(100, 1);
  local_mpc_table.resize(400, 1);
  local_vel_table.resize(300, 1);
  local_vel_rpy_table.resize(300, 1);

  for(int l = 0; l < 100; l++){
      local_iterations_table(l, 0) = 0;
      for(int m = 0; m < 4; m++){
        local_mpc_table(l*4+m, 0) = 0;
      }
      for(int m = 0; m < 3; m++){
        local_vel_table(l*3+m, 0) = 0;
        local_vel_rpy_table(l*3+m, 0) = 0;
      }
     }
  firstUpdate = true;

}

NeuralMPCLocomotion::~NeuralMPCLocomotion(){
  _terminalFlag = true;
  // if(_parameters->nmpc_lcm_nonblocking){
  //   _neuralLCMThread.join();
  // }
}

void NeuralMPCLocomotion::initialize(){
  //std::cout << "RESET NMPC\n";

  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  rpy_des.setZero();
  v_rpy_des.setZero();

  _policyRecieved = 0;
  _OFFLINE_MPC = false;
  // initialization
  for(int i=0; i<3; i++){ vel_act[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_prev[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_act_next[i] = 0.0;}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = 0.0;}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = 0.0;} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = 0.0;}
  footswing_height_act = 0.06;
  offsets_act_prev[0] = 0; offsets_act_prev[1] = 0; offsets_act_prev[2] = 0; offsets_act_prev[3] = 0;
  for(int i=0; i<4; i++){ durations_act_prev[i] = 10; }
  offsets_act[0] = 0; offsets_act[1] = 0; offsets_act[2] = 0; offsets_act[3] = 0;
  for(int i=0; i<4; i++){ durations_act[i] = 10; }
  offsets_act_next[0] = 0; offsets_act_next[1] = 0; offsets_act_next[2] = 0; offsets_act_next[3] = 0;
  for(int i=0; i<4; i++){ durations_act_next[i] = 10; }

  _smooth_gait = true;
  _smooth_vel = false;
  _updatedCommand = false;
   
  iterationsBetweenMPC_cmd = iterationsBetweenMPC;

  //_iteration = 0;
  iterationCounter = 0;
  timeLastMPC = 0;
  iterationsSinceCmd = 0;
  custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, {0., 0., 0.29});
  //custom.advanceGait();


  //add below

  //delete &custom;
  custom.reset();
  //custom.recompute_swingduration_every_step = parameters->recompute_swingduration_every_step;

  custom.set_dt(dt);
  custom.set_nIterations(horizonLength);
  custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, {0., 0., 0.29});


  // iterationCounter = 0;
  // iterationPrevQuery = 0;
  // iterationsBetweenMPC = 14;
  // //iterationsBetweenMPC_cmd;
  // //iterationsSinceCmd;
  // timeLastMPC = 0;
  // _body_height = 0.31;
  // //dt;
  // //dtMPC;
  // //f_ff[4];
  // //swingTimes;

  // firstRun = true;
  // firstUpdate = true;
  // iterationFirstUpdate = 0;
  // //firstSwing[4];
  // //swingTimeRemaining[4];
  // //swingTimeSoFar[4];
  // //stand_traj[6];
  // x_comp_integral = 0;
  // lastBroadcastIteration = -1;

  //custom = &custom_ptr

  // float dtMPClist[horizonLength];
  // for(int i=0; i<horizonLength; i++){
  //   dtMPClist[i] = dt * iterationsBetweenMPC;
  // }
  // neural_setup_problem(dtMPClist,horizonLength,0.4,120);
  // rpy_comp[0] = 0;
  // rpy_comp[1] = 0;
  // rpy_comp[2] = 0;
  // rpy_int[0] = 0;
  // rpy_int[1] = 0;
  // rpy_int[2] = 0;

  // for(int i = 0; i < 4; i++)
  //   firstSwing[i] = true;

  // _terminalFlag = false;
  

  // //initSparseMPC();
  // pBody_des.setZero();
  // vBody_des.setZero();
  // aBody_des.setZero();
  // rpy_des.setZero();
  // v_rpy_des.setZero();

  // iterationsBetweenMPC_cmd = iterationsBetweenMPC;
  // // initialization
  // for(int i=0; i<3; i++){ vel_act[i] = 0.0;}
  // for(int i=0; i<3; i++){ vel_act_prev[i] = 0.0;}
  // for(int i=0; i<3; i++){ vel_act_next[i] = 0.0;}
  // for(int i=0; i<3; i++){ vel_rpy_act[i] = 0.0;}
  // for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = 0.0;} 
  // for(int i=0; i<4; i++){ fh_rel_act[i] = 0.0;}
  // footswing_height_act = 0.06;
  // offsets_act_prev[0] = 0; offsets_act_prev[1] = 0; offsets_act_prev[2] = 0; offsets_act_prev[3] = 0;
  // for(int i=0; i<4; i++){ durations_act_prev[i] = 10; }
  // offsets_act[0] = 0; offsets_act[1] = 0; offsets_act[2] = 0; offsets_act[3] = 0;
  // for(int i=0; i<4; i++){ durations_act[i] = 10; }
  // offsets_act_next[0] = 0; offsets_act_next[1] = 0; offsets_act_next[2] = 0; offsets_act_next[3] = 0;
  // for(int i=0; i<4; i++){ durations_act_next[i] = 10; }

  // _smooth_gait = true;
  // _smooth_vel = false;

  // _updatedCommand = false;
  // _OFFLINE_MPC = false;

  // custom.set_dt(dt);
  // custom.set_nIterations(horizonLength);
  // custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, {0., 0., 0.29});

  // local_iterations_table.resize(100, 1);
  // local_mpc_table.resize(400, 1);
  // local_vel_table.resize(300, 1);
  // local_vel_rpy_table.resize(300, 1);

  // for(int l = 0; l < 100; l++){
  //     local_iterations_table(l, 0) = 0;
  //     for(int m = 0; m < 4; m++){
  //       local_mpc_table(l*4+m, 0) = 0;
  //     }
  //     for(int m = 0; m < 3; m++){
  //       local_vel_table(l*3+m, 0) = 0;
  //       local_vel_rpy_table(l*3+m, 0) = 0;
  //     }
  //    }
  // firstUpdate = true;
  
}
void NeuralMPCLocomotion::_UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,
    const DMat<float> & height_map){

    // Vec3<float> local_pf = foot - body_pos;
    (void)body_pos;
    (void)height_map;

    // int row_idx_half = height_map.rows()/2;
    // int col_idx_half = height_map.cols()/2;

    // int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    // int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;

    // int x_idx_selected = x_idx;
    // int y_idx_selected = y_idx;

    //std::cout << local_pf[0] << " " << local_pf[1];
    //std::cout << " " << x_idx << " " << y_idx << "\n";

    foot[2] = 0;
    

}

float NeuralMPCLocomotion::getFootHeight(Vec3<float>& foot, const Vec3<float>& body_pos){
        //const DMat<float> & height_map){
    /*
    //std::cout << "body_pos " << body_pos << "\n";
    //std::cout << "foot " << foot << "\n";
    Vec3<float> local_pf = foot - body_pos;
    //std::cout << "local_pf " << local_pf << "\n";

    int row_idx_half = height_map.rows()/2;
    int col_idx_half = height_map.rows()/2;

    int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;

    //std::cout << "x_idx, y_idx " << x_idx << " " << y_idx << "\n";
    */
    (void)foot;
    (void)body_pos;
    return 0;//height_map(x_idx, y_idx);

}

void NeuralMPCLocomotion::_IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected, 
    const DMat<int> & idx_map){

  if(idx_map(x_idx, y_idx) == 0){ // (0,0)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx) == 0){ // (1, 0)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx+1) == 0){ // (1, 1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx, y_idx+1) == 0){ // (0, 1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx+1) == 0){ // (-1, 1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx) == 0){ // (-1, 0)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx-1, y_idx-1) == 0){ // (-1, -1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx, y_idx-1) == 0){ // (0, -1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+1, y_idx-1) == 0){ // (1, -1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx-1) == 0){ // (2, -1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx) == 0){ // (2, 0)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+2, y_idx+1) == 0){ // (2, 1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx+2, y_idx+2) == 0){ // (2, 2)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+2;

  }else{
    printf("no proper step location (%d, %d)\n", x_idx, y_idx);
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;
  }
}

void NeuralMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
    /*if( data._quadruped->_robotType == RobotType::MINI_CHEETAH ||
        data._quadruped->_robotType == RobotType::MINI_CHEETAH_VISION ){
      _body_height = 0.29;
    }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
      _body_height = 0.45;
    }else{
      assert(false);
    }


    for(int i=0; i<3; i++){ vel_act[i] = data._desiredStateCommand->data.action[i];}
    for(int i=0; i<3; i++){ vel_rpy_act[i] = data._desiredStateCommand->data.action[i+3];}
    for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = data._desiredStateCommand->data.action[i+6];} 
    for(int i=0; i<4; i++){ fh_rel_act[i] = data._desiredStateCommand->data.action[i+14];}
    footswing_height_act = data._desiredStateCommand->data.action[18];
    for(int i=0; i<4; i++){ offsets_act[i] = data._desiredStateCommand->data.action[i+19]; }
    for(int i=0; i<4; i++){ durations_act[i] = data._desiredStateCommand->data.action[i+23]; }
 
    //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
    int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
    float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
    iterationCounter = _phase * data._desiredStateCommand->data.action[27] * horizonLength + _iteration * data._desiredStateCommand->data.action[27];
    std::cout << "iteration " << _iteration << " phase " << _phase << "\n";

    iterationsBetweenMPC_cmd = data._desiredStateCommand->data.action[27];
    */
  (void)data;
  _body_height = 0.29;

}
      

template<>
void NeuralMPCLocomotion::runParamsFixed(ControlFSMData<float>& data, 
    const Vec3<float> & vel_cmd, const Vec3<float> & vel_rpy_cmd, const Vec2<float> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, const float footswing_height, const int iterationsBetweenMPC_act) {
    
  
  //std::cout << "NeuralMPCLocomotion::run" << "\n";
  
  (void)fp_rel_cmd;
  (void)vel_cmd;
  (void)vel_rpy_cmd;
  //(void)offsets_cmd;
  //(void)durations_cmd;
  //(void)use_gait_cycling;

  _SetupCommand(data);

  auto& seResult = data._stateEstimator->getResult();

  // if(data.userParameters->use_rc ){
  //   data.userParameters->cmpc_gait = data._desiredStateCommand->rcCommand->variable[0];
  // }
  
  gaitNumber = data.userParameters->cmpc_gait;

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
    world_position_desired[2] = 0.29;

  }
  /*
  // pick gait
  NeuralGait* gait = &trotting;
  if(gaitNumber == 1)         gait = &bounding;
  else if(gaitNumber == 2)    gait = &pronking;
  else if(gaitNumber == 3)    gait = &galloping;
  else if(gaitNumber == 4)    gait = &standing;
  else if(gaitNumber == 5)    gait = &trotRunning;
  current_gait = gaitNumber;
  */
  iterationsBetweenMPC = iterationsBetweenMPC_act;
  dtMPC = dt * iterationsBetweenMPC;
  //printf("[Neural MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
  //    dt, iterationsBetweenMPC, dtMPC);
  //neural_setup_problem(dtMPC, horizonLength, 0.4, 120);
  
  

  // Can modify
  gait = &custom; // set custom gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  //gait->setNextGait(10, 10, )
  //int* mpcTable = gait->mpc_gait();
  //updateMPCIfNeeded(mpcTable, data);

  // integrate position setpoint
  v_des_world[0] = (gait->getVel())[0];//vel_cmd[0];
  v_des_world[1] = (gait->getVel())[1];//vel_cmd[1];
  v_des_world[2] = (gait->getVel())[2];//vel_cmd[2];
  if(nmpc_use_vel_control){
    rpy_des[0] = 0;//seResult.rpy[0];
    rpy_des[1] = 0;//seResult.rpy[1];
    if(zero_yaw){
      rpy_des[2] = 0;
    }
    else{
      rpy_des[2] = seResult.rpy[2];// + dt * vel_rpy_cmd[2];
    }
  } else{
    rpy_des[0] = (gait->getRpy())[0];//0;//seResult.rpy[0];
    rpy_des[1] = (gait->getRpy())[1];//0;//seResult.rpy[1];
    rpy_des[2] = (gait->getRpy())[2];//seResult.rpy[2];// + dt * vel_rpy_cmd[2];
  }
  v_rpy_des[0] = (gait->getVelRpy())[0];
  v_rpy_des[1] = (gait->getVelRpy())[1]; // Pitch not yet accounted for in MPC
  v_rpy_des[2] = (gait->getVelRpy())[2];
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");
  
  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2) {  //avoid dividing by zero 
    rpy_int[1] += dt*(rpy_des[1] - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1) {
    rpy_int[0] += dt*(rpy_des[0] - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
    //std::cout << "pFoot[" << i << "]: " << pFoot[i] << "\n";
  }

  if(gait != &standing) {
    if(nmpc_use_vel_control){
      //world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
      world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], v_des_world[2]);
    } else{
      world_position_desired = gait->getPos();
      //std::cout << "POS CONTROL";
    }
  }

  // some first time initialization
  if(firstRun)
  {
    if(nmpc_use_vel_control){
      world_position_desired[0] = seResult.position[0];
      world_position_desired[1] = seResult.position[1];
      //world_position_desired[2] = seResult.rpy[2];//seResult.position[2];
      // if(data.userParameters->fix_body_height){
      //   world_position_desired[2] = _body_height; //seResult.position[2];
      // }
      // else{
      world_position_desired[2] = seResult.position[2];
      //}
      rpy_des[0] = seResult.rpy[0];
      rpy_des[1] = seResult.rpy[1];
      rpy_des[2] = seResult.rpy[2];
    } else{
      world_position_desired = gait->getPos();
      rpy_des = gait->getRpy();
    }

    for(int i = 0; i < 4; i++)
    {
      //std::cout << "INITIALIZING FST \n";
      footSwingTrajectories[i].setHeight(footswing_height);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
    //std::cout << "FIRST RUN!";
    custom._iteration = 0;
  }
  
  // foot placement
  swingTimes[0] = dt * gait->_swing[0]; // swing_time_cmd;
  swingTimes[1] = dt * gait->_swing[1]; // swing_time_cmd;
  swingTimes[2] = dt * gait->_swing[2]; // swing_time_cmd;
  swingTimes[3] = dt * gait->_swing[3]; // swing_time_cmd;

  //std::cout << "swingTimes: " << swingTimes[0] << " " << swingTimes[1] << " " << swingTimes[2] << " " << swingTimes[3] << "\n";

  float side_sign[4] = {-1, 1, -1, 1};

  for(int i = 0; i < 4; i++) {

    if(firstSwing[i]) {
      swingTimeSoFar[i] = 0;

    } else {
      swingTimeSoFar[i] += dt;
    }

    swingTimeRemaining[i] = swingTimes[i] - swingTimeSoFar[i];

    // Swing Height
    //footSwingTrajectories[i].setHeight(_fin_foot_loc[i][2] + footswing_height);
    //footSwingTrajectories[foot].setHeight(footswing_height + seResult.position[2] - 0.29);

    // Foot under the hip
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    
    // Account for desired yaw rate
    

    Vec3<float> Pf;
    // Account for body velocity
    //if(data.userParameters->nmpc_use_vel_control){
      Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, 
          -v_rpy_des[2] * gait->_stance[i] * dt / 2) * pRobotFrame;

      Vec3<float> des_vel = seResult.rBody * v_des_world;
      Pf = seResult.position +
        seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);
    //} else{
    //  Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, 
    //      -v_rpy_des[2] * gait->_stance[i] * dt / 2) * pRobotFrame;
    //  Pf = gait->getFuturePos(swingTimeRemaining[i] + .5 * gait->_stance[i] * dt) + seResult.rBody.transpose() * pYawCorrected;// + gait->getRpy();
    //}

    // Heuristics for foot placement
    float p_rel_max = 0.5f;//0.3f;
    float pfx_rel;
    float pfy_rel;
    //if(data.userParameters->nmpc_use_vel_control){
      pfx_rel = seResult.vWorld[0] * .5 * gait->_stance[i] * dt +
        .03f*(seResult.vWorld[0]-v_des_world[0]) +
        (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*v_rpy_des[2]);

      pfy_rel = seResult.vWorld[1] * .5 * gait->_stance[i] * dt +
        .03f*(seResult.vWorld[1]-v_des_world[1]) +
        (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*v_rpy_des[2]);

      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

      //std::cout << "Pf, pfx_rel, pfy_rel: " << Pf << " " << pfx_rel << " " << pfy_rel << "\n";

      Pf[0] +=  pfx_rel;
      Pf[1] +=  pfy_rel;
    //} 
    
    Pf[2] = 0.; // should it be less than 0? In other repo it was

#ifdef DRAW_DEBUG_FOOTHOLD // Nominal foothold location
        footPlaceSafe = true;
        Vec3<float> Pf_nom = Pf;
        //_UpdateFoothold(Pf_nom, seResult.position, height_map);
        auto* nominalSphere = data.visualizationData->addSphere();
        nominalSphere->position = Pf_nom;
        nominalSphere->radius = 0.02;
        nominalSphere->color = {0.9, 0.55, 0.05, 0.7}; // orange = nominal
#endif

    Pf[0] = Pf[0] + fp_rel_cmd[i][0];
    Pf[1] = Pf[1] + fp_rel_cmd[i][1];
    //std::cout << fp_rel_cmd[i][0] << ", " << fp_rel_cmd[i][1] << " == " << Pf[0] << ", " << Pf[1] << "\n";

    //_UpdateFoothold(Pf, seResult.position, height_map);
    Pf[2] = Pf[2] + fh_rel_cmd[i];

    _fin_foot_loc[i] = Pf;
    //Pf[2] -= 0.003;
    //printf("%d, %d) foot: %f, %f, %f \n", x_idx, y_idx, local_pf[0], local_pf[1], Pf[2]);
    //std::cout << "str, ttnu, ht " << swingTimeRemaining[i] << " " << gait->getIterationsToNextUpdate(_adaptationHorizon) * dt << " " << seResult.position[2] << "\n";
    //Vec3<float> offset(0, side_sign[i] * .065, 0);
    float dFootHip = (seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + offset) + seResult.position - pFoot[i]).norm();

    //std::cout << firstSwing[i] << " or (" << swingTimeRemaining[i] << " > " << gait->getIterationsToNextUpdate(_adaptationHorizon) * dt << " and " << data.userParameters->nmpc_jump_ctrl << ") or (" << dFootHip << " > 0.38 and " << data.userParameters->nmpc_block_invalid << ") or " << data.userParameters->nmpc_adaptive_foot_placements << "\n";
    //if(firstSwing[i] or ((swingTimeRemaining[i] > gait->getIterationsToNextUpdate(_adaptationHorizon) * dt and data.userParameters->nmpc_jump_ctrl) or (dFootHip > 0.38 and data.userParameters->nmpc_block_invalid)) or data.userParameters->nmpc_adaptive_foot_placements) {
    if(firstSwing[i] or ((swingTimeRemaining[i] > gait->getIterationsToNextUpdate(_adaptationHorizon) * dt and nmpc_jump_ctrl) or (dFootHip > 0.38 and nmpc_block_invalid)) or nmpc_adaptive_foot_placements) {
      //std::cout << "adjust final position target\n";
      //std::cout << "FLIGHT MODE TRIGGER\n";
      //std::cout << "str " << swingTimeRemaining[i] << " i " << i << " dFootHip " << dFootHip << "\n";
      footSwingTrajectories[i].setFinalPosition(Pf);
      //std::cout << "Final position " << i << " " << Pf << "\n";
    }


#ifdef DRAW_DEBUG_FOOTHOLD // Corrected foothold location
        auto* correctedSphere = data.visualizationData->addSphere();
        correctedSphere->position = Pf;
        correctedSphere->radius = 0.02;
        if (footPlaceSafe)
            correctedSphere->color = {0.1, 0.9, 0.1, 0.7}; // green = safe
        else
            correctedSphere->color = {0.9, 0.1, 0.1, 0.7}; // red = unsafe
#endif
  }

  // load LCM leg swing gains
  Kp << 700, 0, 0,
    0, 700, 0,
    0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 11, 0, 0,
    0, 11, 0,
    0, 0, 11;
  Kd_stance = Kd;
  
  // gait update
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  iterationCounter++;

  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();

  int* mpcTable = gait->mpc_gait();
  
  updateMPCIfNeeded(mpcTable, data);
  

  Vec4<float> se_contactState(0,0,0,0);
  Vec4<float> se_footHeight(0,0,0,0);

  // Body height
  float floor_height = 0;
  int stance_leg_cnt = 0;

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];//;contact_cmd[foot]
    float swingState = swingStates[foot];

    Vec3<float> offset(0, side_sign[foot] * .065, 0);
      float dFootHip = (seResult.rBody.transpose() * (data._quadruped->getHipLocation(foot) + offset) + seResult.position - pFoot[foot]).norm();

    if(swingState > 0) // foot is in swing
    {
      //std::cout << "swingState" << swingState << "\n";
      //std::cout << "str, inu " <<  swingTimeRemaining[foot]  << " " <<  gait->getIterationsToNextUpdate(_adaptationHorizon) * dt << "\n";
      if(firstSwing[foot] or ((swingTimeRemaining[foot] > gait->getIterationsToNextUpdate(_adaptationHorizon) * dt and nmpc_jump_ctrl) or (dFootHip > 0.38 and nmpc_block_invalid))){
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        //std::cout << "Initial position " << foot << " " << pFoot[foot] << "\n";
        //std::cout << "footswing height " << footswing_height << " pFoot[foot][2] " << pFoot[foot][2] << "\n";
        if(firstSwing[foot])
        {
          firstSwing[foot] = false;
          footSwingTrajectories[foot].setHeight(footswing_height);
        }
        else{
          footSwingTrajectories[foot].setHeight(0);
        }
      }
      

#ifdef DRAW_DEBUG_SWINGS
            auto* debugPath = data.visualizationData->addPath();
            if(debugPath) {
                debugPath->num_points = 100;
                debugPath->color = {0.2,1,0.2,0.5};
                float step = (1.f - swingState) / 100.f;
                for(int i = 0; i < 100; i++) {
                    footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
                    debugPath->position[i] = footSwingTrajectories[foot].getPosition();
                }
            }

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
            auto* actualSphere = data.visualizationData->addSphere();
            actualSphere->position = pFoot[foot];
            actualSphere->radius = 0.02;
            actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      //footSwingTrajectories[foot].setHeight(_fin_foot_loc[foot][2] + footswing_height);
      
      //footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      //std::cout << "swingState " << swingState << "\n";
      //std::cout << "STR " << swingTimeRemaining[foot] << " " << swingTimes[foot] << " " << 1.0 - swingTimeRemaining[foot] / std::min(double(swingTimes[foot]), 0.29)  << "\n";

      if(gait->getIterationsToNextUpdate(_adaptationHorizon) * dt < double(swingTimes[foot])){ // swing duration longer than adaptation horizon
        //std::cout << "compute EXC\n";
        //std::cout << "foot " << foot << " frac " << 1.0 - ((gait->getIterationsToNextUpdate(_adaptationHorizon) * dt)) / swingTimes[foot] <<  " it " << gait->getIterationsToNextUpdate(_adaptationHorizon) * dt <<" st " << swingTimes[foot] << "\n";
        //footSwingTrajectories[foot].computeSwingTrajectoryBezier(0.5 - 0.5 * swingTimeRemaining[foot] / ((gait->getIterationsToNextUpdate(_adaptationHorizon) * dt)), swingTimes[foot]);
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(1.0 -  ((gait->getIterationsToNextUpdate(_adaptationHorizon) * dt)) / swingTimes[foot], gait->getIterationsToNextUpdate(_adaptationHorizon) * dt);

      }
      else{
        //std::cout << "compute classic\n";
        //std::cout << "swingTimeRemaining" << swingTimeRemaining[foot] << "\n";
        //std::cout << "(b) foot " << foot << " frac " << 1.0 - swingTimeRemaining[foot] / swingTimes[foot] << " st " << swingTimes[foot] << "\n";
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(1.0 - swingTimeRemaining[foot] / swingTimes[foot], swingTimes[foot]);
      }
      

      Vec3<float> pDesFootWorld;
      Vec3<float> vDesFootWorld;
      Vec3<float> aDesFootWorld;

      //std::cout << "swing, foot " << foot << ": " << gait->_swing[foot] << "\n";
      //std::cout << "swingTimeRemaining: " << foot << ' ' << swingTimeRemaining[foot] << "\n";
      //std::cout << "getIterationsToNextUpdate" << gait->getIterationsToNextUpdate(_adaptationHorizon) * dt << "\n";
      //Vec3<float> offset(0, side_sign[foot] * .065, 0);
      //float dFootHip = (seResult.rBody.transpose() * (data._quadruped->getHipLocation(foot) + offset) + seResult.position - pFoot[foot]).norm();
      //std::cout << "dFootHip" << dFootHip << "\n";
      
      if((swingTimeRemaining[foot] > gait->getIterationsToNextUpdate(_adaptationHorizon) * dt or dFootHip > 0.38) and nmpc_jump_ctrl){
        //std::cout << "FLIGHT??\n";
        // Foot under the hip
        //Vec3<float> offset(0, side_sign[foot] * .065, 0);
        Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(foot) + offset);
        pRobotFrame[2] = -0.26;
        pDesFootWorld = seResult.rBody.transpose() * pRobotFrame + seResult.position;
        //std::cout << "robot pose" << seResult.position << "\n";
        //std::cout << "pDesFootWorld" <<  pDesFootWorld << "\n";
        //std::cout << "data.userParameters->nmpc_jump_ctrl " << data.userParameters->nmpc_jump_ctrl << "\n";
        //std::cout << "pFootWorld" << pFoot[foot] << "\n";

        //pDesFootWorld[2] = seResult.position[2] - 0.28;
        //pDesFootWorld = seResult.rBody * pDesFootWorld;
        vDesFootWorld = v_des_world;
        //vDesFootWorld[2] = seResult.vWorld[2];
        aDesFootWorld = seResult.aWorld;
        //vDesFootWorld = seResult.vWorld;
        //std::cout << "vDesFootWorld" <<  vDesFootWorld << "\n";
        //firstSwing[std::cout << "pDesFootWorld" <<  pDesFootWorld << "\n";foot] = true;
        //swingTimes[foot] -= dt;
        //swingTimeRemaining[foot] -= dt;
        //std::cout << "jumper default foot pos\n";
      }
      else{
        //footSwingTrajectories[foot].computeSwingTrajectoryBezier(0, 1);
        pDesFootWorld = footSwingTrajectories[foot].getPosition();
        vDesFootWorld = footSwingTrajectories[foot].getVelocity();
        aDesFootWorld = footSwingTrajectories[foot].getAcceleration();
        //std::cout << "footswing trajectory foot pos\n";
      }
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);


      // Update for WBC
      //std::cout << "update pFoot\n";
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = aDesFootWorld;

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;

        //singularity barrier
        data._legController->commands[foot].tauFeedForward[2] = 
          50*(data._legController->datas[foot].q(2)<.1)*data._legController->datas[foot].q(2);
      }
      se_footHeight[foot] = pDesFootWorld[2];
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      stance_leg_cnt++;

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(0, 1);
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //std::cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }

      // Update for WBC
      //std::cout << "update pFoot\n";
      //pFoot_des[foot] = pDesFootWorld;
      //vFoot_des[foot] = vDesFootWorld;
      //aFoot_des[foot] = aDesFootWorld;


      floor_height += pDesFootWorld[2];
      se_contactState[foot] = contactState;

      se_footHeight[foot] = getFootHeight(pFoot[foot], seResult.position);//, height_map);
    }
  }

  // Get floor height
  floor_height /= stance_leg_cnt;
  //Vec3<float> avgStanceFootPos = data._stateEstimator->getAverageStanceFootPosWorld();

  // Set contact/foot height info
  data._stateEstimator->setContactPhase(se_contactState);
  //data._stateEstimator->setFootHeights(se_footHeight);

  // Body Height
  const float HEIGHT_ABOVE_GROUND = 0.29;
  //_body_height = avgStanceFootPos[2] + HEIGHT_ABOVE_GROUND;
  // TEST
  //_body_height = floor_height + HEIGHT_ABOVE_GROUND;
  _body_height = HEIGHT_ABOVE_GROUND;

  // Update For WBC
  //std::cout << "posetarget " <<  world_position_desired[0] << "\n";

  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  //pBody_des[2] = _body_height; 
  pBody_des[2] = world_position_desired[2];

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  //vBody_des[2] = 0.;
  vBody_des[2] = v_des_world[2];

  pBody_RPY_des[0] = rpy_des[0];
  pBody_RPY_des[1] = rpy_des[1]; 
  pBody_RPY_des[2] = rpy_des[2];

  vBody_Ori_des[0] = v_rpy_des[0];
  vBody_Ori_des[1] = v_rpy_des[1];
  vBody_Ori_des[2] = v_rpy_des[2];

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update

  iterationsSinceCmd += 1;

  /*
  for(int i = 0; i < 4; i++){
    std::cout << pFoot_des[i] << " ";
  }
  std::cout << ": pFoot_des[0]\n";
  */
}


template<>
void NeuralMPCLocomotion::runParamsFixedWrapper(ControlFSMData<float>& data,
    const Vec3<float> & vel_cmd, const Vec3<float> & vel_rpy_cmd, const Vec2<float> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, const Vec4<int> & offsets_cmd,
    const Vec4<int> & durations_cmd, const float footswing_height_cmd, const int iterationsBetweenMPC_act, const DMat<float> & height_map, const bool use_gait_smoothing, const bool use_vel_smoothing, const bool use_gait_cycling){

  (void)height_map;
  (void)use_vel_smoothing;
  (void)use_gait_smoothing;

  // smooth offsets, durations transition
  int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;

  //std::cout << "_updatedCommand, _iteration, iterationCounter" << _updatedCommand << " " << _iteration << " " << iterationCounter << "\n";
  if(_updatedCommand or (_iteration == 0 and iterationCounter % (iterationsBetweenMPC * 10) == 0)){ // updated command!
     iterationsSinceCmd = 0;
     //for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
     for(int i=0; i<4; i++){ offsets_act_prev[i] = offsets_act[i];}
     for(int i=0; i<4; i++){ offsets_act[i] = offsets_act_next[i];}
     for(int i=0; i<4; i++){ durations_act_prev[i] = durations_act[i];}
     for(int i=0; i<4; i++){ durations_act[i] = durations_act_next[i];}



      // build local mpc table from next commanded gait
      //std::cout << durations_cmd << " " << offsets_cmd << "\n";
      //DMat<int> local_mpc_table(40, 1);
      for(int i = 0; i < 10; i++){
        for(int j = 0; j < 4; j++){
          
          if((offsets_cmd[j] <= i and i < offsets_cmd[j] + durations_cmd[j])){
            local_mpc_table(i*4+j, 0) = 1;
          }
          else{
            local_mpc_table(i*4+j, 0) = 0;
          }
        }
      }
      
      custom.setNextGait(10, 10, local_mpc_table, custom._iteration+1);
      custom.setIterations(iterationsBetweenMPC, iterationCounter);

      _updatedCommand = false;
  }
  //for(int i = 0; i < 4; i++){
  //  if((offsets_act[i] + durations_act[i]) % horizonLength == _iteration){ //foot lifting off
  //    offsets_act[i] = offsets_act_next[i]; // update offsets, durations on liftoff
  
  
  for(int i=0; i<3; i++){ vel_act[i] = vel_cmd[i];}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = vel_rpy_cmd[i];}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = fp_rel_cmd[i%4][i/4];} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = fh_rel_cmd[i];}
  footswing_height_act = footswing_height_cmd;
  for(int i=0; i<4; i++){ offsets_act_next[i] = offsets_cmd[i]; }
  for(int i=0; i<4; i++){ durations_act_next[i] = durations_cmd[i]; }
  
  // limit duration commands to not cycle over
  if(not use_gait_cycling){
    for(int i = 0; i < 4; i++){
      if(offsets_act_next[i] + durations_act_next[i] > 10){ // wrapping
        durations_act_next[i] = 10 - offsets_act_next[i];
      }
    }
  }
   
  //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
  
  float _phase = (float)(iterationCounter % (iterationsBetweenMPC * horizonLength)) / (float) (iterationsBetweenMPC * horizonLength);
  if(iterationsBetweenMPC_act != iterationsBetweenMPC){
    iterationCounter = _phase * iterationsBetweenMPC_act * horizonLength + _iteration * iterationsBetweenMPC_act;
    iterationsBetweenMPC = iterationsBetweenMPC_act;
  }

   
  
  runParamsFixed(data, vel_act, vel_rpy_act, fp_rel_act, fh_rel_act, footswing_height_act, iterationsBetweenMPC_act);
}



template<>
void NeuralMPCLocomotion::runParamsFixedNewStyleWrapper(ControlFSMData<float>& data,
    const Vec3<float> & vel_cmd, const Vec3<float> & vel_rpy_cmd, const Vec2<float> (& fp_rel_cmd)[4], const Vec4<float>  & fh_rel_cmd, 
    const float footswing_height_cmd, const int iterationsBetweenMPC_act, const DMat<int> & mpc_table_update, const DMat<float> & vel_table_update, const DMat<float> & vel_rpy_table_update, const DMat<int> & iterations_table_update, 
    const int planningHorizon, const int adaptationHorizon, const int adaptationSteps){

  //Timer t4;

  horizonLength = planningHorizon;
  custom.set_nIterations(horizonLength);
  _adaptationHorizon = adaptationHorizon;
  _adaptationSteps = adaptationSteps;

  // smooth offsets, durations transition
  //int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
 
  //std::cout << "_updatedCommand, iterationsSinceCmd, _adaptationSteps, iterationsBetweenMPC" << _updatedCommand << " " << iterationsSinceCmd << " " << _adaptationSteps << " " << iterationsBetweenMPC << "\n";
  //std::cout << "[NeuralMPCLocomotion] iterationsBetweenMPC" << iterationsBetweenMPC << "\n";
  if(_updatedCommand or iterationsSinceCmd >= _adaptationSteps * iterationsBetweenMPC){//gait->getIterationsBetweenMPC()){ // updated command!
     iterationsSinceCmd = 0;
     //for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
     // build local mpc table from next commanded gait
     //std::cout << "update type 1";
      
      custom.setNextIterationsBetweenMPC(adaptationHorizon, adaptationSteps, iterations_table_update, custom._iteration+1);
      custom.setNextVel(adaptationHorizon, adaptationSteps, vel_table_update, vel_rpy_table_update, custom._iteration+1);
      custom.setNextGait(adaptationHorizon, adaptationSteps, mpc_table_update, custom._iteration+1);
      //custom.setIterations(iterationsBetweenMPC, iterationCounter);
      //std::cout << "UPDATED MPC TABLE";
      //gait->print_mpc_table();
      _updatedCommand = false;
  }
  //for(int i = 0; i < 4; i++){
  //  if((offsets_act[i] + durations_act[i]) % horizonLength == _iteration){ //foot lifting off
  //    offsets_act[i] = offsets_act_next[i]; // update offsets, durations on liftoff
  
  
  for(int i=0; i<3; i++){ vel_act[i] = vel_cmd[i];}
  for(int i=0; i<3; i++){ vel_rpy_act[i] = vel_rpy_cmd[i];}
  for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = fp_rel_cmd[i%4][i/4];} 
  for(int i=0; i<4; i++){ fh_rel_act[i] = fh_rel_cmd[i];}
  footswing_height_act = footswing_height_cmd;
  
  
  //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
  
  //float _phase = (float)(iterationCounter % (iterationsBetweenMPC * horizonLength)) / (float) (iterationsBetweenMPC * horizonLength);
  //if(iterationsBetweenMPC_act != iterationsBetweenMPC){
  //  iterationCounter = _phase * iterationsBetweenMPC_act * horizonLength + _iteration * iterationsBetweenMPC_act;
  //  iterationsBetweenMPC = iterationsBetweenMPC_act;
  //}
  // ADD GOOD WAY TO DO THIS
  
  runParamsFixed(data, vel_act, vel_rpy_act, fp_rel_act, fh_rel_act, footswing_height_act, iterationsBetweenMPC_act);

  //total_step_time = t4.getMs();
}




template<>
void NeuralMPCLocomotion::run(ControlFSMData<float>& data, const DMat<float> & height_map){
  (void)height_map;
  /*
  //Timer t3;
  if(_OFFLINE_MPC){
    std::cout << "receive WBC parameters from offboard!";
    if(not data.userParameters->nmpc_lcm_nonblocking){
      _neuralLCM.handle();
    }
    auto& seResult = data._stateEstimator->getResult();
    //publishState()
    return;
  }
  //auto& seResult = data._stateEstimator->getResult();

  //std::cout << " run iter counts: " << iterationCounter << " adaptationSteps * iterationsBetweenMPC " << _adaptationSteps * iterationsBetweenMPC << "\n";
  //std::cout << " iteration " << custom._iteration << "\n";
  
  //if((iterationCounter % (iterationsBetweenMPC * _adaptationSteps)) == 0){ // not the right condition
  
  if(custom._iteration % _adaptationSteps == 0 and lastBroadcastIteration != custom._iteration){
  //int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
  //if(iterationsSinceCmd >= _adaptationSteps * iterationsBetweenMPC){
  //std::cout << " run iter counts: " << iterationsSinceCmd << " " << _adaptationSteps * iterationsBetweenMPC << "\n";
  //auto& seResult = data._stateEstimator->getResult();
  //auto& limbData = data._stateEstimator->getLimbData();
  iterationsSinceCmd = 0;

  lastBroadcastIteration = custom._iteration;
  */
  
  // if(data.userParameters->nmpc_use_lcm and data.userParameters->use_lcm_comm){

  //   if(firstRun){
  //     std::cout << "firstRun LCM!";
  //     auto& seResult = data._stateEstimator->getResult();
  //     custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, seResult.position);
  //     //custom.advanceGait();
  //   }
  
  //   auto& seResult = data._stateEstimator->getResult();
  //   // Build 58-dimensional robot state
  //   rl_obs_t rl_obs;

  //   // body height
  //   rl_obs.body_ht = seResult.position[2];
  //   for(int i=0; i<3; i++){ rl_obs.robot_world_pos[i] = seResult.position[i]; } // not used in policy, for ground truth logging
  //   // body orientation
  //   for(int i=0; i<3; i++){ rl_obs.rpy[i] = seResult.rpy[i]; }
  //   // joint angles
  //   for(int i=0; i<4; i++){
  //    for(int j=0; j<3; j++){ 
  //      rl_obs.q[i*3+j] = data._stateEstimator->getLimbData(i)->q[j]; 
  //     }
  //   }
  //   // body (linear and angular) velocities
  //   for(int i=0; i<3; i++){ rl_obs.vBody[i] =  seResult.vBody[i]; }
  //   for(int i=0; i<3; i++){ rl_obs.omegaWorld[i] = seResult.omegaWorld[i]; }
  //   // previous command
  //   //
  //   //obs_accessor[34:38] = 
  //   //obs_accessor[38:42] = 
  //   //obs_accessor[42:45] = 
  //   //obs_accessor[45] = 
  //   // joint velocities
  //   for(int i=0; i<4; i++){
  //  for(int j=0; j<3; j++){
  //    rl_obs.qd[i*3+j] = data._stateEstimator->getLimbData(i)->qd[j]; 
  //  }
  //   }
  //   // Append heightmap to observation
  //   for(int i=0; i<height_map.cols(); i++){
  //   for(int j=0; j<height_map.rows(); j++){
  //     rl_obs.height_map[i][j] = height_map(i, j);
  //         }
  //   }
  //   rl_obs.mpc_progress = (iterationCounter / iterationsBetweenMPC) % 10; 

  //   //std::cout << rl_obs.mpc_progress;
    
  //   // store prev vel (for smooth transition)
  //   for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
  //   for(int i=0; i<4; i++){ offsets_act_prev[i] = offsets_act[i];}
  //   for(int i=0; i<4; i++){ offsets_act[i] = offsets_act_next[i];}
  //   for(int i=0; i<4; i++){ durations_act_prev[i] = durations_act[i];}
  //   for(int i=0; i<4; i++){ durations_act[i] = durations_act_next[i];}

  //   rl_obs.id = custom._iteration;

  //   auto now = std::chrono::high_resolution_clock::now();
  //   auto now_ns = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  //   rl_obs.timestamp_us = (int64_t)now_ns;
  

  //   // send obs to python via LCM
  //   // request action from python

  //   _policyRecieved = 0;
  //   std::cout << "Sending policy request... with ID " << rl_obs.id << "\n";
  //   _neuralLCM.publish("rl_gait_obs", &rl_obs);

  //   //wait for response
  //   //while( _policyRecieved < 1){
  //   //  usleep(100);
  //   //  printf("Waiting for a response from python...");
  //     //_neuralLCM.publish("rl_gait_obs", &rl_obs);
  //   //}
  //   iterationPrevQuery = custom._iteration+1;
  //   //std::cout << "vel_act: " << vel_act << "vel_rpy_act" << vel_rpy_act << "\n";
  //   if(not data.userParameters->nmpc_lcm_nonblocking){
  //     _neuralLCM.handle();
  //   }

  // } else{
    // for(int i=0; i<3; i++){ vel_act[i] = data._desiredStateCommand->data.action[i];}
    // for(int i=0; i<3; i++){ vel_rpy_act[i] = data._desiredStateCommand->data.action[i+3];}
    // for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = data._desiredStateCommand->data.action[i+6];} 
    // for(int i=0; i<4; i++){ fh_rel_act[i] = data._desiredStateCommand->data.action[i+14];}
    // footswing_height_act = data._desiredStateCommand->data.action[18];
    // for(int i=0; i<4; i++){ offsets_act[i] = (int)data._desiredStateCommand->data.action[i+19]; }
    // for(int i=0; i<4; i++){ durations_act[i] = (int)data._desiredStateCommand->data.action[i+23]; }

    // int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;

    // if(_iteration == 0 or iterationCounter % (iterationsBetweenMPC * 10) == 0){ // updated command!
    //  _updatedCommand = true;
    // }
    // //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
    // //int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
    // //float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
    // //iterationCounter = _phase * data._desiredStateCommand->data.action[27] * horizonLength + _iteration * data._desiredStateCommand->data.action[27];

    // //iterationsBetweenMPC_cmd = (int)data._desiredStateCommand->data.action[27];
    
    // //std::cout << "UPDATE \n";

    //}

  // }
  //std::cout << "vel_act: " << vel_act << "vel_rpy_act" << vel_rpy_act << "fp_rel_act " << fp_rel_act << "fh_rel_act" << fh_rel_act << "footswing_height_act" <<  footswing_height_act << "iterationsBetweenMPC_cmd" << iterationsBetweenMPC_cmd << "\n";



  // smooth velocity transition
  
  //float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
  
  //if(_smooth_vel){
  //   for(int i=0; i<3; i++){ vel_act[i] = (vel_act_next[i] - vel_act_prev[i]) * _phase + vel_act_prev[i];}
  // } else{
  //   for(int i=0; i<3; i++){ vel_act[i] = vel_act_next[i];}
  // }
  // smooth offsets, durations transition
  //int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
  
  // limit duration commands to not cycle over
  if(not _cycle_gait){
    for(int i = 0; i < 4; i++){
      if(offsets_act_next[i] + durations_act_next[i] > 10){ // wrapping
        durations_act_next[i] = 10 - offsets_act_next[i];
      }
    }
  }
    
  //vel_act, vel_rpy_act, fp_rel_act, fh_rel_act, footswing_height_act, iterationsBetweenMPC_act
  runParamsFixed(data, vel_act, vel_rpy_act, fp_rel_act, fh_rel_act, footswing_height_act, iterationsBetweenMPC_cmd);
  //total_update_time = t3.getMs();
  //printf("Total NMPC update time %f ms\n", t3.getMs());
}

// void NeuralMPCLocomotion::handleActionLCM(const lcm::ReceiveBuffer *rbuf,
//     const std::string &chan,
//     const rl_action_lcmt *msg) {
//   (void)rbuf;
//   (void)chan;

//   if(_OFFLINE_MPC){
//     return;
//   }

//   printf("Recieved action\n");
//   std::cout << "ok\n"; 
//   std::cout << vel_act << "\n"; 
//   std::cout << msg->vel_act << "ok\n"; 
//   /*
//   for(int i=0; i<3; i++){ vel_act_next[i] = msg->vel_act[i];}
//   std::cout << "ok\n"; 
//   for(int i=0; i<3; i++){ vel_rpy_act[i] = msg->vel_rpy_act[i];}
//   for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = msg->fp_rel_act[i];} 
//   for(int i=0; i<4; i++){ fh_rel_act[i] = msg->fh_rel_act[i];}
//   std::cout << "ok\n"; 
//   footswing_height_act = msg->footswing_height_act;
//   for(int i=0; i<4; i++){ offsets_act_prev[i] = offsets_act[i];}
//   for(int i=0; i<4; i++){ offsets_act[i] = offsets_act_next[i];}
//   for(int i=0; i<4; i++){ offsets_act_next[i] = msg->offsets_act[i]; }
//   for(int i=0; i<4; i++){ durations_act_prev[i] = durations_act[i];}
//   for(int i=0; i<4; i++){ durations_act[i] = durations_act_next[i];}
//   for(int i=0; i<4; i++){ durations_act_next[i] = msg->durations_act[i]; }

//   _smooth_gait = msg->smooth_gait;
//   _smooth_vel = msg->smooth_vel;
//   _cycle_gait = msg->cycle_gait;
   
//   std::cout << "ok\n";
//   std::cout << iterationCounter << " " << iterationsBetweenMPC_cmd << " " << horizonLength << " \n";
//   //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
//   int _iteration = (iterationCounter / iterationsBetweenMPC_cmd) % horizonLength;
//   float _phase = (float)(iterationCounter % (iterationsBetweenMPC_cmd * horizonLength)) / (float) (iterationsBetweenMPC_cmd * horizonLength);
//   iterationCounter = _phase * msg->iterationsBetweenMPC_act * horizonLength + _iteration * msg->iterationsBetweenMPC_act;
//   iterationsBetweenMPC_cmd = msg->iterationsBetweenMPC_act;
//   //std::cout << "iteration " << _iteration << " phase " << _phase << "\n";
//   */
//   //std::cout << "adaptationHorizon " << msg->adaptationHorizon << " adaptationSteps " << msg->adaptationSteps << " planningHorizon " << msg->planningHorizon << "\n";



//   horizonLength = msg->planningHorizon;
//   custom.set_nIterations(horizonLength);

//   int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
//   _adaptationHorizon = msg->adaptationHorizon;
//   _adaptationSteps = msg->adaptationSteps;

//   // smooth offsets, durations transition
//   //int _iteration = (iterationCounter / iterationsBetweenMPC) % horizonLength;
//   iterationsBetweenMPC_cmd = msg->iterationsBetweenMPC_act;

//   if(firstUpdate){
//       std::cout << "firstUpdate!";
//       iterationFirstUpdate = custom._iteration;
//       //custom.initializeMPCTable(offsets_act_next, durations_act_next, iterationsBetweenMPC_cmd, vel_act, vel_rpy_act, custom.getPos());
//       //iterationCounter;
//       //custom.advanceGait();
//       firstUpdate = false;
      
//   }
//   else{
//     gait->print_mpc_table();
//   }
 
//   //std::cout << "[NeuralMPCLocomotion] iterationsBetweenMPC" << iterationsBetweenMPC << "\n";
//   //std::cout << "[NeuralMPCLocomotion] adaptationSteps" << _adaptationSteps << "\n";
//   //if(_updatedCommand or iterationsSinceCmd >= _adaptationSteps * iterationsBetweenMPC){//gait->getIterationsBetweenMPC()){ // updated command!
//    //std::cout << "UPDATING MPC TABLE\n";
//    //iterationsSinceCmd = 0;
//    //for(int i=0; i<3; i++){ vel_act_prev[i] = vel_act_next[i];}
//    //copy local table commands into DMat form
//    for(int l = 0; l < 100; l++){
//     //std::cout << l << "\n";
//     local_iterations_table(l, 0) = msg->iterations_table_update[l];
//     for(int m = 0; m < 4; m++){
//       local_mpc_table(l*4+m, 0) = msg->mpc_table_update[l*4+m];
//     }
//     for(int m = 0; m < 3; m++){
//       local_vel_table(l*3+m, 0) = msg->vel_table_update[l*3+m];
//       local_vel_rpy_table(l*3+m, 0) = msg->vel_rpy_table_update[l*3+m];
//     }
//    }
//    // build local mpc table from next commanded gait
//     //custom.setIterations(iterationsBetweenMPC, iterationCounter);
   
//     //std::cout << "update type 2";
//     int iterationToUpdate = msg->id + iterationFirstUpdate + 1;// * msg->adaptationSteps;
//     //std::cout << "itu " << iterationToUpdate << " ipq " << iterationPrevQuery << " iteration " << custom._iteration << " equivalent lag " << (custom._iteration - iterationToUpdate) *  dt * iterationsBetweenMPC << "\n";

//     custom.setNextIterationsBetweenMPC(msg->adaptationHorizon, msg->adaptationSteps, local_iterations_table, iterationToUpdate);
//     custom.setNextVel(msg->adaptationHorizon, msg->adaptationSteps, local_vel_table, local_vel_rpy_table, iterationToUpdate);
//     custom.setNextGait(msg->adaptationHorizon, msg->adaptationSteps, local_mpc_table, iterationToUpdate);
//     //custom.setIterations(iterationsBetweenMPC, iterationCounter);
   
//     //
//     //_updatedCommand = false;
  
//   //for(int i = 0; i < 4; i++){
//   //  if((offsets_act[i] + durations_act[i]) % horizonLength == _iteration){ //foot lifting off
//   //    offsets_act[i] = offsets_act_next[i]; // update offsets, durations on liftoff
  
  
//   for(int i=0; i<3; i++){ vel_act[i] = msg->vel_act[i];}
//   for(int i=0; i<3; i++){ vel_rpy_act[i] = msg->vel_rpy_act[i];}
//   for(int i=0; i<8; i++){ fp_rel_act[i%4][i/4] = msg->fp_rel_act[i];} 
//   for(int i=0; i<4; i++){ fh_rel_act[i] = msg->fh_rel_act[i];}
//   footswing_height_act = msg->footswing_height_act;
  
  
//   //iterationCounter += -(iterationCounter %  msg->iterationsBetweenMPC_act) + (iterationCounter % iterationsBetweenMPC_cmd); 
  
//   float _phase = (float)(iterationCounter % (iterationsBetweenMPC * horizonLength)) / (float) (iterationsBetweenMPC * horizonLength);
  

//   printf("Successfully read action\n");

//   _policyRecieved = 1;
//   _updatedCommand = true;

// }

// void NeuralMPCLocomotion::handleWBCActionLCM(const lcm::ReceiveBuffer *rbuf,
//     const std::string &chan,
//     const wbc_params_lcmt *msg) {
//   (void)rbuf;
//   (void)chan;

//   std::cout << "Received WBC Action!\n";

//   // Update For WBC

//   pBody_des[0] = msg->pBody_des[0];
//   pBody_des[1] = msg->pBody_des[1];
//   //pBody_des[2] = _body_height; 
//   pBody_des[2] = msg->pBody_des[2];

//   vBody_des[0] = msg->vBody_des[0];
//   vBody_des[1] = msg->vBody_des[1];
//   //vBody_des[2] = 0.;
//   vBody_des[2] = msg->vBody_des[2];

//   pBody_RPY_des[0] = msg->pBody_RPY_des[0];
//   pBody_RPY_des[1] = msg->pBody_RPY_des[1];
//   pBody_RPY_des[2] = msg->pBody_RPY_des[2];

//   vBody_Ori_des[0] = msg->vBody_Ori_des[0];
//   vBody_Ori_des[1] = msg->vBody_Ori_des[1];
//   vBody_Ori_des[2] = msg->vBody_Ori_des[2];

//   for(int foot = 0; foot < 4; foot++){
//     for(int i = 0; i < 3; i++){
//       pFoot_des[foot][i] = msg->pFoot_des[foot*3+i];
//       vFoot_des[foot][i] = msg->vFoot_des[foot*3+i];
//       aFoot_des[foot][i] = msg->aFoot_des[foot*3+i];
//       Fr_des[foot][i] = msg->Fr_des[foot*3+i];
//     }
//     contact_state[foot] = msg->contact_state[foot];
//   }

  

  

//   // 
//   _OFFLINE_MPC = true;

// }


void NeuralMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) {
  //iterationsBetweenMPC = 30;
  //std::cout <<"[NeuralMPCLocomotion] iterationCounter, timeLastMPC: " << iterationCounter << " " << timeLastMPC << "\n"; 
  //std::cout << "HORIZON LENGTH " << horizonLength << "\n";
  if(iterationCounter - timeLastMPC >= gait->getIterationsBetweenMPC() or iterationCounter == 0)
  {
    //Timer t3;

    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();
    float* vel_table = gait->mpc_vel();
    float* vel_rpy_table = gait->mpc_vel_rpy();
    float* pose_table = gait->mpc_pose();
    float* rpy_table = gait->mpc_rpy();

    int* itersList = gait->mpc_iters();
    float dtMPClist[horizonLength];
    //std::cout << "dtMPCList ";
    for(int i=0; i<horizonLength; i++){
      //dtMPClist[i] = dt * iterationsBetweenMPC;
      dtMPClist[i] = dt * itersList[i];
      //std::cout << dtMPClist[i] << " ";
    }
    //std::cout << "\n";



    if(current_gait == 4)    {
      float trajInitial[12] = {(float)rpy_des[0], // Roll
                               (float)rpy_des[1], // Pitch
                               (float)stand_traj[5],
                               (float)stand_traj[0],
                               (float)stand_traj[1],
                               (float)_body_height,
                               0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    } else {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      //float xStart = p[0];
      //float yStart = p[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;



      if(nmpc_use_vel_control){
              float trajInitial[12] = {(float)rpy_comp[0],  // 0
                                   (float)rpy_comp[1],    // 1
                                   (float)rpy_des[2],    // 2
                                   xStart,                                   // 3
                                   yStart,                                   // 4
                                   pose_table[2],      // 5
                                   vel_rpy_table[0],                                        // 6
                                   vel_rpy_table[1],                                        // 7
                                   vel_rpy_table[2],  // 8
                                   vel_table[0],                           // 9
                                   vel_table[1],                           // 10
                                   vel_table[2]};                                       // 11

            for(int i = 0; i < horizonLength; i++) {
              for(int j = 0; j < 12; j++)  trajAll[12*i+j] = trajInitial[j];
              if(i == 0) // start at current position  TODO consider not doing this
              {
                //trajAll[3] = hw_i->state_estimator->se_pBody[0];
                //trajAll[4] = hw_i->state_estimator->se_pBody[1];
                trajAll[2] = seResult.rpy[2];
                //std::cout << "seResult yaw" << trajAll[2] << "\n";
              } else {
                
                trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPClist[i] * vel_table[0+i*3];
                trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPClist[i] * vel_table[1+i*3];
                trajAll[12*i + 5] = trajAll[12 * (i - 1) + 5] + dtMPClist[i] * vel_table[2+i*3];
                trajAll[12*i + 0] = trajAll[12 * (i - 1) + 0] + dtMPClist[i] * vel_rpy_table[0+i*3];
                trajAll[12*i + 1] = trajAll[12 * (i - 1) + 1] + dtMPClist[i] * vel_rpy_table[1+i*3];
                trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPClist[i] * vel_rpy_table[2+i*3];
                
                trajAll[12*i+6] = vel_rpy_table[0+i*3];
                trajAll[12*i+7] = vel_rpy_table[1+i*3];
                trajAll[12*i+8] = vel_rpy_table[2+i*3];
                trajAll[12*i+9] = vel_table[0+i*3];
                trajAll[12*i+10] = vel_table[1+i*3];
                trajAll[12*i+11] = vel_table[2+i*3];
              }
            }
      } else{

            //std::cout << "pose table\n";
            //gait->print_pose_table();

            float trajInitial[12] = {rpy_table[0],  // 0
                                     rpy_table[1],    // 1
                                     rpy_table[2],    // 2
                                     pose_table[0],                                   // 3
                                     pose_table[1],                                   // 4
                                     pose_table[2],      // 5
                                     vel_rpy_table[0],                                        // 6
                                     vel_rpy_table[1],                                        // 7
                                     vel_rpy_table[2],  // 8
                                     vel_table[0],                           // 9
                                     vel_table[1],                           // 10
                                     vel_table[2]};                                       // 11

            for(int i = 1; i < horizonLength; i++) {
              for(int j = 0; j < 12; j++)  trajAll[12*i+j] = trajInitial[j];
              //if(i == 0) // start at current position  TODO consider not doing this
              //{
                //trajAll[3] = hw_i->state_estimator->se_pBody[0];
                //trajAll[4] = hw_i->state_estimator->se_pBody[1];
                //trajAll[2] = seResult.rpy[2];
              //} else {
                
                //trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * vel_table[0+i*3];
                //trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * vel_table[1+i*3];
                //trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * v_rpy_des[2];
                
                trajAll[12*i+0] = rpy_table[0+i*3];
                trajAll[12*i+1] = rpy_table[1+i*3];
                trajAll[12*i+2] = rpy_table[2+i*3];
                trajAll[12*i+3] = pose_table[0+i*3];
                trajAll[12*i+4] = pose_table[1+i*3];
                trajAll[12*i+5] = pose_table[2+i*3];
                trajAll[12*i+6] = vel_rpy_table[0+i*3];
                trajAll[12*i+7] = vel_rpy_table[1+i*3];
                trajAll[12*i+8] = vel_rpy_table[2+i*3];
                trajAll[12*i+9] = vel_table[0+i*3];
                trajAll[12*i+10] = vel_table[1+i*3];
                trajAll[12*i+11] = vel_table[2+i*3];
              //}
            }
      }

    }

    //gait->print_mpc_table();
    //gait->print_vel_table();
    //gait->print_iters_table();

    solveDenseMPC(mpcTable, itersList, data);
    timeLastMPC = iterationCounter;

    //total_update_time = t3.getMs();
    //printf("Total MPC update time %f ms\n", t3.getMs());
  }
}

void NeuralMPCLocomotion::solveDenseMPC(int *mpcTable, int* itersList, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();


  float r[12];
  for(int i = 0; i < 12; i++) r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }
  
  //int* iterslist = gait->mpc_iters();
  float dtMPClist[horizonLength];
  for(int i=0; i<horizonLength; i++){
    //dtMPClist[i] = dt * iterationsBetweenMPC;
    dtMPClist[i] = dt * itersList[i];
  }
  
  // debugging
  /*
  std::cout << " MPC inputs " << p[0] << " " << p[1] << " " << p[2] << "\n" << v[0] << " " << v[1] << " " << v[2] << "\n" << w[0] << " " << w[1] << " " << w[2] << "\n" << q[0] << " " << q[1] << " " << q[2] << "\n"  << yaw <<  "\n ";
  
  for(int i = 0; i < horizonLength; i++){
    for(int j = 0; j < 12; j++){
      std::cout << trajAll[i*12+j] << " ";
    }
    std::cout << "\n";
  }
  for(int i = 0; i < 12; i++){std::cout << pFoot[i%4][i/4] << " ";}
  std::cout << "\n";
  for(int i = 0; i < horizonLength; i++){std::cout << dtMPClist[i] << " ";}
  std::cout << "\n";
  */
  
  //dtMPC = dt * iterationsBetweenMPC;
  neural_setup_problem(dtMPClist,horizonLength,0.4,120);
  update_x_drag(x_comp_integral);
  //if() // compensatory drag turned off
  solve_times = neural_update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = neural_get_solution(leg*3 + axis);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
  objVal = neural_get_obj_val();

}
