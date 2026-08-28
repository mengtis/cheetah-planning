#include "DataProcessor.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

#include <json/reader.h>

#include <Dynamics/MiniCheetah.h>
#include <Math/orientation_tools.h>

namespace {

//! Read a fixed-length numeric JSON array into an Eigen vector.
template <typename Derived>
bool ReadArray(const Json::Value& node, Eigen::MatrixBase<Derived>& out) {
  if (!node.isArray() || static_cast<int>(node.size()) < out.size()) {
    return false;
  }
  for (int i = 0; i < out.size(); ++i) {
    out[i] = node[i].asDouble();
  }
  return true;
}

//! Read the flattened 3x3 rotation matrix stored under "Quat".
bool ReadRotation(const Json::Value& node, Mat3<double>& out) {
  if (!node.isArray() || node.size() < 9) return false;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out(i, j) = node[i * 3 + j].asDouble();
    }
  }
  return true;
}

}  // namespace

DataProcessor::DataProcessor() {
  // The model is trajectory-independent, so build it once here rather than
  // once per timestep.
  _model = buildMiniCheetah<double>().buildModel();
  _jointTorques.setZero();
  _footForces.setZero();
}

std::string DataProcessor::TimeKey(int index) {
  // Keys are the sample timestamp printed with 6 decimals, e.g. "0.002001".
  // Computed from the integer index in double precision: accumulating a float
  // drifts past the 1e-6 key resolution well before the end of a trajectory.
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6f", kT0 + kDt * index);
  return std::string(buf);
}

bool DataProcessor::Load(const std::string& json_path) {
  std::ifstream file_in(json_path);
  if (!file_in.is_open()) {
    std::cerr << "[DataProcessor] cannot open " << json_path << std::endl;
    return false;
  }

  Json::CharReaderBuilder builder;
  std::string errs;
  if (!Json::parseFromStream(builder, file_in, &_opt, &errs)) {
    std::cerr << "[DataProcessor] failed to parse " << json_path << ": " << errs
              << std::endl;
    return false;
  }

  if (!_opt.isMember("data")) {
    std::cerr << "[DataProcessor] no \"data\" object in " << json_path
              << std::endl;
    return false;
  }

  // Trust the file for the horizon rather than a hard-coded count: walk the
  // uniform time grid until a key is missing.
  const Json::Value& data = _opt["data"];
  _timesteps = 0;
  while (data.isMember(TimeKey(_timesteps))) {
    ++_timesteps;
  }

  if (_timesteps == 0) {
    std::cerr << "[DataProcessor] no samples on the expected time grid"
              << std::endl;
    return false;
  }

  printf("[DataProcessor] loaded %s: %d timesteps (%.3f s)\n",
         json_path.c_str(), _timesteps, kDt * (_timesteps - 1));
  return true;
}

bool DataProcessor::Generate() {
  if (_timesteps == 0) {
    std::cerr << "[DataProcessor] Generate() called before a successful Load()"
              << std::endl;
    return false;
  }

  const Json::Value& data = _opt["data"];

  // Pass 1: joint positions. The JSON has no joint velocity or acceleration
  // field, so those come from finite differences of this in pass 2.
  _q.setZero(_timesteps, 12);
  for (int i = 0; i < _timesteps; ++i) {
    Eigen::VectorXd row(12);
    if (!ReadArray(data[TimeKey(i)]["Joint position"], row)) {
      std::cerr << "[DataProcessor] bad \"Joint position\" at t="
                << TimeKey(i) << std::endl;
      return false;
    }
    _q.row(i) = row.transpose();
  }

  // Pass 2: central differences, one-sided at the endpoints.
  _qd.setZero(_timesteps, 12);
  _qdd.setZero(_timesteps, 12);
  if (_timesteps >= 2) {
    for (int i = 0; i < _timesteps; ++i) {
      if (i == 0) {
        _qd.row(i) = (_q.row(1) - _q.row(0)) / kDt;
      } else if (i == _timesteps - 1) {
        _qd.row(i) = (_q.row(i) - _q.row(i - 1)) / kDt;
      } else {
        _qd.row(i) = (_q.row(i + 1) - _q.row(i - 1)) / (2.0 * kDt);
      }
    }
    for (int i = 0; i < _timesteps; ++i) {
      if (i == 0) {
        _qdd.row(i) = (_qd.row(1) - _qd.row(0)) / kDt;
      } else if (i == _timesteps - 1) {
        _qdd.row(i) = (_qd.row(i) - _qd.row(i - 1)) / kDt;
      } else {
        _qdd.row(i) = (_qd.row(i + 1) - _qd.row(i - 1)) / (2.0 * kDt);
      }
    }
  }

  // Pass 3: inverse dynamics per sample.
  _plan.setZero(_timesteps, kPlanCols);

  Mat3<double> rotation;
  Vec3<double> linear_pos, linear_vel, euler_vel, linear_acc, euler_acc, foot;

  for (int i = 0; i < _timesteps; ++i) {
    const Json::Value& sample = data[TimeKey(i)];

    if (!ReadRotation(sample["Quat"], rotation) ||
        !ReadArray(sample["Base linear position"], linear_pos) ||
        !ReadArray(sample["Base linear velocity"], linear_vel) ||
        !ReadArray(sample["Base euler velocity"], euler_vel) ||
        !ReadArray(sample["Base linear acceleration"], linear_acc) ||
        !ReadArray(sample["Base euler acceleration"], euler_acc)) {
      std::cerr << "[DataProcessor] malformed sample at t=" << TimeKey(i)
                << std::endl;
      return false;
    }

    _x.bodyOrientation = ori::rotationMatrixToQuaternion(rotation);
    _x.bodyPosition = linear_pos;
    // SVec is [angular; linear] -- see spatialToLinearAcceleration().
    _x.bodyVelocity.head(3) = euler_vel;
    _x.bodyVelocity.tail(3) = linear_vel;
    _x.q = _q.row(i).transpose();
    _x.qd = _qd.row(i).transpose();

    // State must be pushed to the model before anything reads back from it.
    _model.setState(_x);

    FBModelStateDerivative<double> dx;
    dx.dBodyVelocity.head(3) = euler_acc;
    dx.dBodyVelocity.tail(3) = linear_acc;
    dx.qdd = _qdd.row(i).transpose();

    // inverseDynamics() calls setDState() internally.
    const DVec<double> genForce = _model.inverseDynamics(dx);
    _jointTorques = genForce.tail(12);

    const Json::Value& forces = sample["Contact forces"];
    for (int leg = 0; leg < 4; ++leg) {
      const Json::Value& node = forces[std::to_string(leg)];
      if (!ReadArray(node, foot)) {
        std::cerr << "[DataProcessor] bad \"Contact forces\" at t="
                  << TimeKey(i) << std::endl;
        return false;
      }
      _footForces.segment(3 * leg, 3) = foot;
    }

    FillRow(i, sample);
  }

  return true;
}

void DataProcessor::FillRow(int index, const Json::Value& sample) {
  // Base pose: the reader expects roll/pitch/yaw here, not quaternion
  // components, so take the euler angles straight from the trajectory.
  Vec3<double> euler_pos = Vec3<double>::Zero();
  ReadArray(sample["Base euler position"], euler_pos);

  _plan.block<1, 3>(index, 0) = _x.bodyPosition.transpose();
  _plan.block<1, 3>(index, 3) = euler_pos.transpose();
  _plan.block<1, 12>(index, 6) = _x.q.transpose();
  // Base twist is stored linear-then-angular, the reverse of SVec's order.
  _plan.block<1, 3>(index, 18) = _x.bodyVelocity.tail(3).transpose();
  _plan.block<1, 3>(index, 21) = _x.bodyVelocity.head(3).transpose();
  _plan.block<1, 12>(index, 24) = _x.qd.transpose();
  _plan.block<1, 12>(index, 36) = _jointTorques.transpose();
  _plan.block<1, 12>(index, 48) = _footForces.transpose();
}

bool DataProcessor::WriteDat(const std::string& dat_path) const {
  if (_plan.size() == 0) {
    std::cerr << "[DataProcessor] nothing to write" << std::endl;
    return false;
  }

  // DataReader fread()s this as row-major float32, so convert explicitly
  // rather than dumping Eigen's column-major double storage.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> out =
      _plan.cast<float>();

  FILE* f = fopen(dat_path.c_str(), "wb");
  if (!f) {
    std::cerr << "[DataProcessor] cannot open " << dat_path << " for writing"
              << std::endl;
    return false;
  }

  const size_t count = static_cast<size_t>(out.size());
  const size_t written = fwrite(out.data(), sizeof(float), count, f);
  fclose(f);

  if (written != count) {
    std::cerr << "[DataProcessor] short write to " << dat_path << std::endl;
    return false;
  }

  printf("[DataProcessor] wrote %s: %d timesteps x %d cols (%zu bytes)\n",
         dat_path.c_str(), _timesteps, kPlanCols, written * sizeof(float));
  return true;
}
