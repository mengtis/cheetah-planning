#include "DataReader.hpp"
#include <Configuration.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

DataReader::DataReader(const RobotType& type, FSM_StateName stateNameIn) : _type(type) {
  if (_type == RobotType::MINI_CHEETAH) {
    
    if (stateNameIn == FSM_StateName::BACKFLIP) {
      //load_control_plan(THIS_COM "user/WBC_Controller/WBC_States/BackFlip/data/mc_flip.dat");
      load_control_plan(THIS_COM "config/mc_flip.dat");
      printf("[Backflip DataReader] Setup for mini cheetah\n");
    }
    else if (stateNameIn == FSM_StateName::FRONTJUMP || stateNameIn == FSM_StateName::DIRECTCOLLOCATION) {
      //load_control_plan(THIS_COM "user/MIT_Controller/Controllers/FrontJump/front_jump_data.dat"); // front_jump_data.dat for succesfull test 1 file
      // load_control_plan(THIS_COM "config/front_jump_pitchup_v2.dat");
      load_control_plan(THIS_COM "config/Tryout.dat");
      printf("[Front Jump DataReader] Setup for mini cheetah\n");
    }
  } else {
    printf("[Backflip DataReader] Setup for cheetah 3\n");
    load_control_plan(THIS_COM "user/WBC_Controller/WBC_States/BackFlip/data/backflip.dat");
  }
  printf("[Backflip DataReader] Constructed.\n");
}

void DataReader::load_control_plan(const char* filename) {
  printf("[Backflip DataReader] Loading control plan %s...\n", filename);
  FILE* f = fopen(filename, "rb");
  if (!f) {
    printf("[Backflip DataReader] Error loading control plan!\n");
    return;
  }
  fseek(f, 0, SEEK_END);
  uint64_t file_size = ftell(f);
  fseek(f, 0, SEEK_SET);

  printf("[Backflip DataReader] Allocating %ld bytes for control plan\n",
         file_size);

  // A plan is a headerless row-major float32 array, plan_cols per timestep.
  // Validate that before trusting the contents: a file written in a different
  // layout (or as text) otherwise loads silently and replays as garbage.
  const uint64_t row_bytes = sizeof(float) * plan_cols;
  if (file_size == 0 || file_size % row_bytes) {
    printf(
        "[DataReader] Error: %s is %llu bytes, not a multiple of %llu "
        "(%d cols x %zu bytes). Refusing to load.\n",
        filename, (unsigned long long)file_size, (unsigned long long)row_bytes,
        plan_cols, sizeof(float));
    fclose(f);
    return;
  }

  plan_buffer = (float*)malloc(file_size);

  if (!plan_buffer) {
    printf("[DataReader] malloc failed!\n");
    fclose(f);
    return;
  }

  uint64_t read_success = fread(plan_buffer, file_size, 1, f);
  fclose(f);

  if (!read_success) {
    printf("[DataReader] Error: fread failed.\n");
    free(plan_buffer);
    plan_buffer = nullptr;
    return;
  }

  plan_loaded = true;
  plan_timesteps = file_size / row_bytes;
  printf("[Backflip DataReader] Done loading plan for %d timesteps\n",
         plan_timesteps);
}

float* DataReader::get_initial_configuration() {
  if (!plan_loaded) {
    printf(
        "[Backflip DataReader] Error: get_initial_configuration called without "
        "a plan!\n");
    return nullptr;
  }

  return plan_buffer + 3;
}

float* DataReader::get_plan_at_time(int timestep) {
  if (!plan_loaded) {
    printf(
        "[Backflip DataReader] Error: get_plan_at_time called without a "
        "plan!\n");
    return nullptr;
  }

  if (timestep < 0 || timestep >= plan_timesteps) {
    printf(
        "[Backflip DataReader] Error: get_plan_at_time called for timestep %d\n"
        "\tmust be between 0 and %d\n",
        timestep, plan_timesteps - 1);
    timestep = plan_timesteps - 1;
    // return nullptr; // TODO: this should estop the robot, can't really
    // recover from this!
  }

  // if(timestep < 0) { return plan_buffer + 3; }
  // if(timestep >= plan_timesteps){ timestep = plan_timesteps-1; }

  return plan_buffer + plan_cols * timestep;
}

void DataReader::unload_control_plan() {
  free(plan_buffer);
  plan_buffer = nullptr;
  plan_timesteps = -1;
  plan_loaded = false;
  printf("[Backflip DataReader] Unloaded plan.\n");
}
