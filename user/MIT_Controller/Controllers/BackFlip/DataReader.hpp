#ifndef BACKFLIP_DATA_READER_H
#define BACKFLIP_DATA_READER_H
#include <cstdlib>

#include <cppTypes.h>
#include <FSM_States/FSM_State.h>

// Column offsets into one row of a control plan. Joint-indexed blocks are
// leg-major -- [ab/ad, hip, knee] for each of the four legs -- matching
// FBModelState::q and LegController, which is what DataProcessor writes.
//
// NOTE: the legacy MATLAB plans in config/ (front_jump_*.dat, mc_flip.dat,
// backflip.dat) predate this layout and use 20 columns per row with a
// different joint ordering. load_control_plan() now rejects them rather than
// replaying them as garbage; they need regenerating.
enum plan_offsets {
  q0_offset = 0,    // base: x, y, z, roll, pitch, yaw | then 12 joint positions
  qd0_offset = 18,  // base twist: linear xyz, angular xyz | then 12 joint vels
  tau_offset = 36,  // 12 feedforward joint torques
  force_offset = 48 // 4 contact forces, xyz each
};

typedef Eigen::Matrix<float, 7, 1> Vector7f;

class DataReader {
 public:
  static const int plan_cols = 60;

  DataReader(const RobotType &, FSM_StateName stateNameIn);
  ~DataReader() { free(plan_buffer); }
  void load_control_plan(const char *filename);
  void unload_control_plan();
  float *get_initial_configuration();
  float *get_plan_at_time(int timestep);
  int plan_timesteps = -1;

 private:
  RobotType _type;
  float *plan_buffer = nullptr;
  bool plan_loaded = false;
};

#endif  // BACKFLIP_DATA_READER_H
