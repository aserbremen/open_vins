/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2021 Andreas Serov
 * Copyright (C) 2021 Joachim Clemens
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// OVVU

#ifndef OV_MSCKF_UPDATER_VEHICLE_H
#define OV_MSCKF_UPDATER_VEHICLE_H

#include "core/VioManagerOptions.h"
#include "state/Propagator.h"
#include "types/Type.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <boost/math/distributions/chi_squared.hpp>

namespace ov_msckf {

class UpdaterVehicle {

public:
  UpdaterVehicle(VioManagerOptions &manager_options, std::shared_ptr<Propagator> prop) : _prop(prop) {

    // Set extrinsic calibration between odometry and IMU frame
    _R_ItoO = ov_core::quat_2_Rot(manager_options.pose_OtoI.block(0, 0, 4, 1));
    _p_OinI = manager_options.pose_OtoI.block(4, 0, 3, 1);
    std::cout << "UpdaterVehicle(): R_ItoO" << std::endl
              << _R_ItoO << std::endl
              << "p_OinI" << std::endl
              << _p_OinI.transpose() << std::endl;

    _use_yaw_odom_second_order = manager_options.use_yaw_odom_second_order;
    _use_yaw_jacobi_second_order = manager_options.use_yaw_jacobi_second_order;

    // Speed update noise
    _sigma_speed_x = manager_options.sigma_speed_x;
    _sigma_zero_speed_y = manager_options.sigma_zero_speed_y;
    _sigma_zero_speed_z = manager_options.sigma_zero_speed_z;
    _sigma_speed_x_2 = std::pow(manager_options.sigma_speed_x, 2);
    _sigma_zero_speed_y_2 = std::pow(manager_options.sigma_zero_speed_y, 2);
    _sigma_zero_speed_z_2 = std::pow(manager_options.sigma_zero_speed_z, 2);
    _vehicle_speed_chi2_multiplier = manager_options.vehicle_speed_chi2_multiplier;

    // Steering update noise
    _sigma_steering_angle = manager_options.sigma_steering_angle;
    _sigma_steering_angle_2 = std::pow(manager_options.sigma_steering_angle, 2);
    _vehicle_steering_chi2_multiplier = manager_options.vehicle_steering_chi2_multiplier;

    // Steering update parameters
    _steering_update_min_speed = manager_options.steering_angle_update_min_speed;
    _ackermann_drive_msg_contains_steering_wheel_angle = manager_options.ackermann_drive_msg_contains_steering_wheel_angle;
    _steering_ratio = manager_options.steering_ratio;
    _wheel_base = manager_options.wheel_base;
    _max_steering_angle = manager_options.max_steering_angle;

    // Initialize the chi squared test table with confidence level 0.95
    // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
    for (int i = 1; i < 1000; i++) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
  }

  void feed_ackermann_drive(const ov_core::AckermannDriveData &message) {

    // Append it to our vecto
    ackermann_drive_data.emplace_back(message);

    // Loop through and delete Ackermann drive messages that are older then 10 seconds
    // TODO: we should probably have more elegant logic then this
    // TODO: but this prevents unbounded memory growth and slow prop with high freq imu
    auto it0 = ackermann_drive_data.begin();
    while (it0 != ackermann_drive_data.end()) {
      if (message.timestamp - (*it0).timestamp > 10) {
        it0 = ackermann_drive_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Feed function for wheel speeds data
   * @param message Contains our timestamp and wheel speeds data
   */
  void feed_wheel_speeds(const ov_core::WheelSpeedsData &message) {

    // Append it to our vector
    wheel_speeds_data.emplace_back(message);

    // Loop through and delete wheel speeds messages that are older then 10 seconds
    // TODO: we should probably have more elegant logic then this
    // TODO: but this prevents unbounded memory growth and slow prop with high freq imu
    auto it0 = wheel_speeds_data.begin();
    while (it0 != wheel_speeds_data.end()) {
      if (message.timestamp - (*it0).timestamp > 10) {
        it0 = wheel_speeds_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Apped it to our vector
    imu_data.emplace_back(message);

    // Loop through and delete imu messages that are older than our requested time
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (message.timestamp - (*it0).timestamp > oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Updates speed of vehicle by using speed measurement in forward direction and assuming zero speeds in y and z
   * @param state State of the filter
   */
  void update_speed_vector(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message);

  /**
   * @brief Updates speed of vehicle by using speed measurement in forward direction
   * @param state State of the filter
   */
  void update_speed_x(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message);

  /**
   * @brief Updates the filter using the effective steering wheel angle which is related to speed in x-direction and the turn rate around
   * the z-axis (yaw)
   * @param state State of the filter
   */
  void update_steering(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message);

  /**
   * @brief Updates the filter using a Ackermann (single-track) model in a preintegrated fashion. All Ackermann drive messages between two
   * camera times are used to calculate the vehicle's 2D-odometry and perform an update. See https://ieeexplore.ieee.org/document/9841243
   * @param state State of the filter
   * @param speed_data Speed measurements
   * @param steering_wheel_data Steering wheel measurements
   */
  void update_vehicle_preintegrated_single_track(std::shared_ptr<State> state, double last_cam_timestamp, double last_prop_time_offset);

  /**
   * @brief Updates the filter using a differential drive model in a preintegrated fashon. All wheel speeds messages between two
   * camera times are used to calculate the vehicle's 2D-odometry and perform an update. See https://ieeexplore.ieee.org/document/9841243/
   * @param state State of the filter
   * @param last_cam_timestamp Timestamp of last camera/visual update timestamp
   * @param last_prop_time_offset Time offset between camera and IMU at last camera/visual update timestamp
   */
  void update_vehicle_preintegrated_differential(std::shared_ptr<State> state, double last_cam_timestamp, double last_prop_time_offset);

  /**
   * @brief Returns a vector of steering wheel readings for a preintegrated vehicle update using a single track drive.
   * @param time0 Last camera timestamp in IMU clock (t_cam + t_offset_cam_imu)
   * @param time1 Current camera timestamp in IMU clock (t_cam + t_offset_cam_imu)
   * @param warn Whether warning messages are to be printed
   * @return std::vector<ov_core::WheelSpeedsData>
   */
  std::vector<ov_core::AckermannDriveData> select_ackermann_drive_readings(double time0, double time1, bool warn = true);

  /**
   * @brief Returns a vector of wheel speed readings for a preintegrated vehicle update using a differential drive model.
   * @param time0 Last camera timestamp in IMU clock (t_cam + t_offset_cam_imu)
   * @param time1 Current camera timestamp in IMU clock (t_cam + t_offset_cam_imu)
   * @param warn Whether warning messages are to be printed
   * @return std::vector<ov_core::WheelSpeedsData>
   */
  std::vector<ov_core::WheelSpeedsData> select_wheel_speeds_readings(double time0, double time1, bool warn = true);

  /**
   * @brief Linearly interpolates a number between two readings
   * @param t0 First timestamp
   * @param value0 First value
   * @param t1 Second timestamp
   * @param value1 Second value
   * @param time Actual timestamp of the data to be interpolated at, t0 <= t <= t1
   * @return double
   */
  double interpolate_double(double t0, double value0, double t1, double value1, double time);

  /**
   * @brief Get the ackermann drive data vector
   * @return const std::vector<ov_core::AckermannDriveData>&
   */
  const std::vector<ov_core::AckermannDriveData> &get_ackermann_drive_data() { return ackermann_drive_data; }

  /**
   * @brief Get the wheel speeds data vector
   * @return const std::vector<ov_core::WheelSpeedsData>&
   */
  const std::vector<ov_core::WheelSpeedsData> &get_wheel_speeds_data() { return wheel_speeds_data; }

protected:
  /// Our propagator!
  std::shared_ptr<Propagator> _prop;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  /// Our history of ackermann drive messages (time, speed, steering angle)
  std::vector<ov_core::AckermannDriveData> ackermann_drive_data;

  /// Our history of wheel speeds messages
  std::vector<ov_core::WheelSpeedsData> wheel_speeds_data;

  /// 3D position of the odometry frame in the IMU frame
  Eigen::Vector3d _p_OinI;

  /// Rotation from the IMU frame into the odometry frame
  Eigen::Matrix3d _R_ItoO;

  /// Speed update

  /// Vehicle speed x noise (m/s)
  double _sigma_speed_x = 0.01;

  /// Vehicle speed x variance (m^2/s^2)
  double _sigma_speed_x_2 = std::pow(0.01, 2);

  /// Vehicle zero speed y noise (m/s)
  double _sigma_zero_speed_y = 0.1;

  /// Vehicle zero speed y variance (m^2/s^2)
  double _sigma_zero_speed_y_2 = std::pow(0.1, 2);

  /// Vehicle zero speed y noise (m/s)
  double _sigma_zero_speed_z = 0.05;

  /// Vehicle zero speed y variance (m^2/s^2)
  double _sigma_zero_speed_z_2 = std::pow(0.05, 2);

  /// Vehicle speed chi2 multiplier
  double _vehicle_speed_chi2_multiplier = 1.0;

  /// Steering update

  /// Our vector of IMU data
  std::vector<ov_core::ImuData> imu_data;

  /// Steering angle variance noise (rad)
  double _sigma_steering_angle = 1.0 * M_PI / 180;

  /// Steering angle variance variance (rad)
  double _sigma_steering_angle_2 = std::pow(1.0 * M_PI / 180, 2);

  /// Vehicle steering chi2 multiplier
  double _vehicle_steering_chi2_multiplier = 1.0;

  /// Minimum speed in x-direction below which steering udpates are rejected in (m/s)
  double _steering_update_min_speed = 3.0;

  /// Steering wheel angle to steering angle ratio assumed to be constant ()
  double _steering_ratio = 15.2;

  /// Whether the Ackermann drive message contains steering wheel angle or steering angle
  bool _ackermann_drive_msg_contains_steering_wheel_angle = false;

  /// Wheel base necessary for steering update (m)
  double _wheel_base = 2.791;

  /// Maximum steering wheel angle for which a steering update is performed
  double _max_steering_angle = 100.0;

  /// Track length between both rear wheels, sometimes called baselink length (m)
  double _track_length = 1.568;

  /// Preintegrated updates

  /// Whether to use second order yaw in the calculation of the preintegrated odometry model
  bool _use_yaw_odom_second_order = true;

  /// Whether to use second order yaw in the Jacobian derivation of the preintegrated odometry model
  bool _use_yaw_jacobi_second_order = false;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_VEHICLE_H
