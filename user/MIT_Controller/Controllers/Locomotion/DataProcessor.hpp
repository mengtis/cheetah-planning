#ifndef LOCOMOTION_DATA_PROCESSOR_HPP
#define LOCOMOTION_DATA_PROCESSOR_HPP

#include <string>

#include <json/value.h>

#include <cppTypes.h>
#include <Dynamics/FloatingBaseModel.h>

/*!
 * Converts the trajectory JSON emitted by TOWR into the binary control plan
 * consumed by DataReader.
 *
 * The output format is dictated by DataReader: a row-major array of float32,
 * kPlanCols values per timestep, laid out according to `enum plan_offsets`
 * in BackFlip/DataReader.hpp:
 *
 *   [ 0.. 5]  base pose      (x, y, z, roll, pitch, yaw)
 *   [ 6..17]  joint position (12)
 *   [18..23]  base twist     (vx, vy, vz, wx, wy, wz)
 *   [24..35]  joint velocity (12)
 *   [36..47]  joint torque   (12)
 *   [48..59]  contact force  (4 feet x 3)
 */
class DataProcessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Must stay in sync with DataReader::plan_cols.
  static constexpr int kPlanCols = 60;
  //! Sample spacing of the trajectory JSON, in seconds.
  static constexpr double kDt = 0.002;
  //! Timestamp of the first sample; keys are formatted "%.6f" of kT0 + kDt*i.
  static constexpr double kT0 = 1e-6;

  DataProcessor();

  //! Parse the trajectory JSON. Returns false instead of exiting on failure.
  bool Load(const std::string& json_path);
  //! Run inverse dynamics over every sample and build the plan matrix.
  bool Generate();
  //! Write the plan as row-major float32, the layout DataReader expects.
  bool WriteDat(const std::string& dat_path) const;

  const Eigen::MatrixXd& plan() const { return _plan; }
  int timesteps() const { return _timesteps; }

 private:
  static std::string TimeKey(int index);
  void FillRow(int index, const Json::Value& sample);

  FloatingBaseModel<double> _model;
  FBModelState<double> _x;
  Json::Value _opt;
  int _timesteps = 0;

  Eigen::MatrixXd _q;    //!< _timesteps x 12 joint positions
  Eigen::MatrixXd _qd;   //!< _timesteps x 12 joint velocities
  Eigen::MatrixXd _qdd;  //!< _timesteps x 12 joint accelerations
  Eigen::MatrixXd _plan; //!< _timesteps x kPlanCols

  Vec12<double> _jointTorques;
  Vec12<double> _footForces;
};

#endif  // LOCOMOTION_DATA_PROCESSOR_HPP
