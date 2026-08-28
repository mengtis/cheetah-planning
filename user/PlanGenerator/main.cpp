/*!
 * Offline tool: convert a TOWR trajectory JSON into the binary .dat control
 * plan that DataReader loads.
 *
 *   ./user/PlanGenerator/plan_generator [input.json] [output.dat]
 *
 * Defaults are resolved against the source tree via THIS_COM, so the tool
 * works from any build directory and on any machine.
 */
#include <cstdio>
#include <string>

#include <Configuration.h>

#include "Controllers/Locomotion/DataProcessor.hpp"

int main(int argc, char** argv) {
  const std::string json_path =
      (argc > 1) ? argv[1] : THIS_COM "user/MIT_Controller/ros_output.json";
  const std::string dat_path =
      (argc > 2) ? argv[2] : THIS_COM "config/Tryout.dat";

  DataProcessor processor;

  if (!processor.Load(json_path)) return 1;
  if (!processor.Generate()) return 1;
  if (!processor.WriteDat(dat_path)) return 1;

  printf("[plan_generator] done\n");
  return 0;
}
