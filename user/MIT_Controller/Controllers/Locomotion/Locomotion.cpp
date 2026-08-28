#include "Locomotion.hpp"

#include <algorithm>
#include <cmath>

template <typename T>
Locomotion<T>::Locomotion(DataReader* data_reader, float _dt)
    : DataReadCtrl<T>(data_reader, _dt) {}

template <typename T>
Locomotion<T>::~Locomotion() {}

template <typename T>
void Locomotion<T>::OneStep(float _curr_time, bool b_preparation,
                            LegControllerCommand<T>* command) {
  DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;
  DataCtrl::_b_Preparation = b_preparation;

  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];
      command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx];
      command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];
      command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];
      command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];
    }
  }
}

template <typename T>
void Locomotion<T>::_update_joint_command() {
  constexpr int pre_mode_duration = 10;

  DataCtrl::_des_jpos.setZero();
  DataCtrl::_des_jvel.setZero();
  DataCtrl::_jtorque.setZero();

  DataCtrl::_Kp_joint = {10.0, 10.0, 10.0};
  DataCtrl::_Kd_joint = {1.0, 1.0, 1.0};

  // Move to the initial configuration before replaying the plan.
  float tau_mult;
  if ((DataCtrl::pre_mode_count < pre_mode_duration) ||
      DataCtrl::_b_Preparation) {
    if (DataCtrl::pre_mode_count == 0) {
      printf("[Locomotion] replaying plan: %d timesteps\n",
             DataCtrl::_data_reader->plan_timesteps);
    }
    DataCtrl::pre_mode_count += DataCtrl::_key_pt_step;
    DataCtrl::current_iteration = 0;
    tau_mult = 0.f;
  } else {
    tau_mult = 1.2f;
  }

  if (DataCtrl::current_iteration >
      DataCtrl::_data_reader->plan_timesteps - 1) {
    DataCtrl::current_iteration = DataCtrl::_data_reader->plan_timesteps - 1;
  }

  const float* current_step =
      DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration);
  if (current_step == nullptr) {
    // No plan loaded. Commands stay zeroed rather than dereferencing null.
    return;
  }

  // The plan and _des_jpos use the same leg-major layout: [ab/ad, hip, knee]
  // for each of the four legs, matching FBModelState::q and LegController.
  for (int i = 0; i < 12; ++i) {
    DataCtrl::_des_jpos[i] = current_step[q0_offset + 6 + i];
    DataCtrl::_des_jvel[i] = current_step[qd0_offset + 6 + i];
    DataCtrl::_jtorque[i] = tau_mult * current_step[tau_offset + i] / 2.0f;
  }

  // Keep the hips from swinging far enough back to strike the body.
  for (int leg = 0; leg < 4; ++leg) {
    const int hip = 3 * leg + 1;
    if (DataCtrl::_des_jpos[hip] < -M_PI / 2.2) {
      DataCtrl::_des_jpos[hip] = -M_PI / 2.2;
      DataCtrl::_des_jvel[hip] = 0.;
      DataCtrl::_jtorque[hip] = 0.;
    }
  }

  // DataReadCtrl::_key_pt_step is ceil(dt * 1000), which assumes a 1 kHz plan.
  // Plans written by DataProcessor are sampled at DataProcessor::kDt, so step
  // through them at the ratio of the two rates instead.
  const int plan_step = std::max(
      1, static_cast<int>(std::lround(DataCtrl::dt / DataProcessor::kDt)));
  DataCtrl::current_iteration += plan_step;
}

template class Locomotion<double>;
template class Locomotion<float>;
