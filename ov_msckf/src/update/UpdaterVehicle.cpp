/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2022 Andreas Serov
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

#include "UpdaterVehicle.h"

using namespace ov_msckf;
using namespace ov_type;
using namespace ov_core;

void UpdaterVehicle::update_speed_vector(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message) {

  // Construct measurement by taking the last speed measurement for speed in x-direction
  // y- and z-speed are assumed to be 0 for ground vehicles, cars
  Eigen::Vector3d velocity_meas = Eigen::Vector3d(ackermann_drive_message.speed, 0, 0);

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->v());

  // Large final matrices used for update
  int h_size = 6;
  int m_size = 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size, m_size);

  // Measurement residual z - ẑ
  res.block(0, 0, 3, 1) = velocity_meas - (_R_ItoO * state->_imu->Rot() * state->_imu->vel());
  PRINT_DEBUG("speed update vector: meas: %.4f %.4f %.4f, state: %.4f %.4f %.4f\n", velocity_meas[0], velocity_meas[1], velocity_meas[2],
              (_R_ItoO * state->_imu->Rot() * state->_imu->vel())[0], (_R_ItoO * state->_imu->Rot() * state->_imu->vel())[1],
              (_R_ItoO * state->_imu->Rot() * state->_imu->vel())[2]);

  // Measurement Jacobian
  // quat in order qx qy qz qw
  auto q_GtoI = state->_imu->quat();
  // First 3 rows of equation (280) [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508) by Joan Solà
  // applied to JPL quaternion
  // The same matrix is given in the left side of the equation (70) [Indirect Kalman Filter for 3D Attitude
  // Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) by Trawny
  Eigen::Matrix3d q_delta_theta = Eigen::Matrix3d::Zero();
  q_delta_theta << q_GtoI(3), -q_GtoI(2), q_GtoI(1), q_GtoI(2), q_GtoI(3), -q_GtoI(0), -q_GtoI(1), q_GtoI(0), q_GtoI(3);
  q_delta_theta *= 0.5;
  H.block(0, 0, 3, 3) = -skew_x(_R_ItoO * state->_imu->Rot() * state->_imu->vel()) * q_delta_theta;
  H.block(0, 3, 3, 3) = _R_ItoO * state->_imu->Rot();

  // Measurement noise
  R(0, 0) = _sigma_speed_x_2;
  R(1, 1) = _sigma_zero_speed_y_2;
  R(2, 2) = _sigma_zero_speed_z_2;

  // Get our Chi2 threshohld
  // double chi2_check = chi_squared_table[res.rows()];

  // // Chi2 distance test
  // Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, Hx_order);
  // Eigen::MatrixXd S = H * P * H.transpose() + R;
  // double chi2 = res.dot(S.llt().solve(res));
  // Check against our thrshold
  // if (chi2 > _vehicle_speed_chi2_multiplier * chi2_check) {
  //   PRINT_DEBUG(RED "UpdaterVehicle::update_speed_vector(): chi2 check failed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_speed_chi2_multiplier * chi2_check);
  // } else {
  //   // Chi2 check succeeded
  //   PRINT_DEBUG(GREEN "UpdaterVehicle::update_speed_vector(): chi2 check passed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_speed_chi2_multiplier * chi2_check);
  // }

  // Finally perform the EKF update
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

void UpdaterVehicle::update_speed_x(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message) {

  // Construct measurement by taking the last speed measurement for speed in x-direction
  double velocity_x = ackermann_drive_message.speed;

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->v());

  // Large final matrices used for update
  int h_size = 6;
  int m_size = 1;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size, m_size);

  // Measurement residual z - ẑ
  res << velocity_x - (_R_ItoO * state->_imu->Rot() * state->_imu->vel())[0];
  PRINT_DEBUG("speed update x: meas: %.4f, state: %.4f\n", velocity_x, (_R_ItoO * state->_imu->Rot() * state->_imu->vel())[0]);

  // Measurement Jacobian
  // quat in order qx qy qz qw
  auto q_GtoI = state->_imu->quat();
  // First 3 rows of equation (280) [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508) by Joan Solà
  // applied to JPL quaternion
  // The same matrix is given in the left side of the equation (70) [Indirect Kalman Filter for 3D Attitude
  // Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) by Trawny
  Eigen::Matrix3d q_delta_theta = Eigen::Matrix3d::Zero();
  q_delta_theta << q_GtoI(3), -q_GtoI(2), q_GtoI(1), q_GtoI(2), q_GtoI(3), -q_GtoI(0), -q_GtoI(1), q_GtoI(0), q_GtoI(3);
  q_delta_theta *= 0.5;
  H.block<1, 3>(0, 0) = (-skew_x(_R_ItoO * state->_imu->Rot() * state->_imu->vel()) * q_delta_theta).row(0);
  H.block<1, 3>(3, 0) = (_R_ItoO * state->_imu->Rot()).row(0);

  // Measurement noise
  R << _sigma_speed_x_2;

  // Get our Chi2 threshohld
  // double chi2_check = chi_squared_table[res.rows()];

  // // Chi2 distance test
  // Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, Hx_order);
  // Eigen::MatrixXd S = H * P * H.transpose() + R;
  // double chi2 = res.dot(S.llt().solve(res));
  // Check against our thrshold
  // if (chi2 > _vehicle_speed_chi2_multiplier * chi2_check) {
  //   PRINT_DEBUG(RED "UpdaterVehicle::update_speed_vector(): chi2 check failed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_speed_chi2_multiplier * chi2_check);
  // } else {
  //   // Chi2 check succeeded
  //   PRINT_DEBUG(GREEN "UpdaterVehicle::update_speed_vector(): chi2 check passed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_speed_chi2_multiplier * chi2_check);
  // }

  // Finally perform the EKF update
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

void UpdaterVehicle::update_steering(std::shared_ptr<State> state, const ov_core::AckermannDriveData &ackermann_drive_message) {

  // Get current speed in x-direction of the body frame
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  double speed_odometry_frame_x = (_R_ItoO * R_GtoI * state->_imu->vel())[0];

  // At small velocities, the steering angle is not suited for computing the turn rate
  if (fabs(speed_odometry_frame_x) < _steering_update_min_speed)
    return;

  // Calculate effective steering angle
  double steering_angle = ackermann_drive_message.steering_angle;
  if (_ackermann_drive_msg_contains_steering_wheel_angle) {
    steering_angle /= _steering_ratio;
  }

  // Return if the current steering wheel angle exceeds the max threshold
  if (fabs(steering_angle) * 180 / M_PI > _max_steering_angle) {
    PRINT_DEBUG(RED "No steering update: steering angle %.2f steering wheel angle %.2f max %.2f\n" RESET, steering_angle * 180 / M_PI,
                ackermann_drive_message.steering_angle * 180 / M_PI, _max_steering_angle);
    return;
  }

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->v());
  Hx_order.push_back(state->_imu->bg());

  // Large final matrices used for update
  int h_size = 9;
  int m_size = 1;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size, m_size);

  // We interpolate or find the closest omega_z value available in our imu data
  double omegaz = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < imu_data.size() - 1; i++) {
    if (imu_data[i].timestamp < ackermann_drive_message.timestamp && imu_data[i + 1].timestamp > ackermann_drive_message.timestamp) {
      omegaz = interpolate_double(imu_data[i].timestamp, imu_data[i].wm[2], imu_data[i + 1].timestamp, imu_data[i + 1].wm[2],
                                  ackermann_drive_message.timestamp);
    }
  }
  if (omegaz == std::numeric_limits<double>::infinity()) {
    PRINT_WARNING(YELLOW "Could not find imu data to interpolate omegaz for ackermann drive ts %.9f.\
        Setting to closest available imu msg wm value.\n" RESET,
                  ackermann_drive_message.timestamp);
    double delta_time_min = std::numeric_limits<double>::infinity();
    for (const auto &imu_msg : imu_data) {
      if (fabs(imu_msg.timestamp - ackermann_drive_message.timestamp) < delta_time_min) {
        delta_time_min = fabs(imu_msg.timestamp - ackermann_drive_message.timestamp);
        omegaz = imu_msg.wm[2];
      }
    }
  }
  // Precompute some values
  double omegaz_minus_bgz = omegaz - state->_imu->bias_g()[2];
  double denominator = std::pow(omegaz_minus_bgz, 2) * std::pow(_wheel_base, 2) + std::pow(speed_odometry_frame_x, 2);
  if (denominator < 1e-10) {
    PRINT_WARNING(YELLOW "Steering update Jacobian denominator: %.16f\n" RESET, denominator);
  }

  // Measurement residual z - ẑ
  res(0) = steering_angle - std::atan(omegaz_minus_bgz * _wheel_base / speed_odometry_frame_x);
  if (fabs(res(0)) > 0.5) {
    PRINT_WARNING(YELLOW "Steering update hight residual: %.6f\n" RESET, res(0));
    PRINT_WARNING(YELLOW "omegaz_minus_bgz: %.6f, speed_odometry_frame_x: %.6f\n" RESET, omegaz_minus_bgz, speed_odometry_frame_x);
    PRINT_WARNING(YELLOW "omegaz: %.6f, state->_imu->bias_g()[2]: %.6f\n" RESET, omegaz, state->_imu->bias_g()[2]);
  }

  PRINT_DEBUG("steering update meas: %6.f, state: %.6f, res: %.6f\n", steering_angle,
              std::atan(omegaz_minus_bgz * _wheel_base / speed_odometry_frame_x), res(0));

  // Measurement Jacobian
  // quat in order qx qy qz qw
  Eigen::Vector4d q_GtoI = state->_imu->quat();
  // First 3 rows of equation (280) [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508) by Joan
  // Solà applied to JPL quaternion
  // The same matrix is given in the left side of the equation (70) [Indirect Kalman Filter for 3D Attitude
  // Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) by Trawny
  Eigen::Matrix3d q_delta_theta = Eigen::Matrix3d::Zero();
  q_delta_theta << q_GtoI(3), -q_GtoI(2), q_GtoI(1), q_GtoI(2), q_GtoI(3), -q_GtoI(0), -q_GtoI(1), q_GtoI(0), q_GtoI(3);
  q_delta_theta *= 0.5;

  H.block<1, 3>(0, 0) = omegaz_minus_bgz * _wheel_base / denominator * skew_x((R_GtoI * state->_imu->vel())).row(0) * q_delta_theta;
  H.block<1, 3>(0, 3) = -omegaz_minus_bgz * _wheel_base / denominator * R_GtoI.row(0);
  H.block<1, 3>(0, 6) << 0, 0, -speed_odometry_frame_x * _wheel_base / denominator;

  R << _sigma_steering_angle_2;

  // Get our Chi2 threshohld
  // double chi2_check = chi_squared_table[res.rows()];

  // // Chi2 distance test
  // Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, Hx_order);
  // Eigen::MatrixXd S = H * P * H.transpose() + R;
  // double chi2 = res.dot(S.llt().solve(res));
  // Check against our threshold
  // if (chi2 > _vehicle_steering_chi2_multiplier * chi2_check) {
  //   PRINT_DEBUG(RED "UpdaterVehicle::update_speed_vector(): chi2 check failed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_steering_chi2_multiplier * chi2_check);
  // } else {
  //   // Chi2 check succeeded
  //   PRINT_DEBUG(GREEN "UpdaterVehicle::update_speed_vector(): chi2 check passed chi2 %.9f  multiplied chi2_check %.9f\n" RESET, chi2,
  //               _vehicle_steering_chi2_multiplier * chi2_check);
  // }

  // Finally perform the EKF update
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

void UpdaterVehicle::update_vehicle_preintegrated_single_track(std::shared_ptr<State> state, double last_cam_timestamp,
                                                               double last_prop_time_offset) {

  // Gather speed and steering messages, asssume IMU and CAN are synchronized, so that we still need to account for time offset between IMU
  // and cam
  double time0 = last_cam_timestamp + last_prop_time_offset;
  double time1 = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);
  std::vector<ov_core::AckermannDriveData> ackermann_drive_data_preintegration = select_ackermann_drive_readings(time0, time1);

  // Create our measured preintegrated odometry values, we follow the formulas of Visual-Inertial Wheel Odometry and Online calibration Tech
  // Report. The odometry preintegration function can be written as g = [dyaw dx dy]^T
  double dyaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  // Preintegration covariance matrix which is going to be built accumatively
  Eigen::Matrix3d P_odom = Eigen::Matrix3d::Zero();
  // This sensor noise matrix is going to be transformed into state space
  Eigen::Matrix2d R_odom = Eigen::Matrix2d::Zero();
  R_odom(0, 0) = _sigma_speed_x_2;
  R_odom(1, 1) = _sigma_steering_angle_2;
  for (size_t i = 0; i < ackermann_drive_data_preintegration.size() - 1; i++) {
    // We follow the midpoint paradigm for preintegrating the single-track model
    double dt = ackermann_drive_data_preintegration[i + 1].timestamp - ackermann_drive_data_preintegration[i].timestamp;
    double vx = (ackermann_drive_data_preintegration[i].speed + ackermann_drive_data_preintegration[i + 1].speed) / 2.0;
    double sa = (ackermann_drive_data_preintegration[i].steering_angle + ackermann_drive_data_preintegration[i + 1].steering_angle) / 2.0;
    if (_ackermann_drive_msg_contains_steering_wheel_angle) {
      sa /= _steering_ratio;
    }

    // Precompute values, and set some values (for readability)
    double tan_sa = std::tan(sa);
    double l = _wheel_base;

    // preintegration following Visual inertial odometry with wheel calibration tech report frame definitions
    double dyaw_last = dyaw;
    dyaw -= vx / l * tan_sa * dt;
    if (_use_yaw_odom_second_order) {
      // We have to handle the singularity when the steering angle is zero or almost zero
      if (std::abs(tan_sa) < 1e-6) {
        dx += vx * std::cos(dyaw_last) * dt;
        dy -= vx * std::sin(dyaw_last) * dt;
      } else {
        dx -= l / tan_sa * (std::sin(dyaw) - std::sin(dyaw_last));
        dy -= l / tan_sa * (std::cos(dyaw) - std::cos(dyaw_last));
      }
    } else {
      dx += vx * std::cos(dyaw_last) * dt;
      dy -= vx * std::sin(dyaw_last) * dt;
    }

    // We construct H_odom which is needed to transform R_odom into state space R = H_odom * R_odom * H_odom^T
    double tan_sa_2 = tan_sa * tan_sa;
    double tan_sa_2_p1 = tan_sa_2 + 1;
    double tan_sa_2_p1_2 = tan_sa_2_p1 * tan_sa_2_p1;
    double tan_sa_2_p1_3 = tan_sa_2_p1 * tan_sa_2_p1 * tan_sa_2_p1;
    double dt_2 = dt * dt;
    double vx_2 = vx * vx;
    // Jacobian of g wrt our measurements: H_odom = (\partial g) / (\partial z), z = [vx sa]^T
    Eigen::Matrix<double, 3, 2> H_odom;
    // Jacobian of g wrt preintegrated odometry state: G_odom = (\partial g) / (\partial u), u = [dyaw_last dx_last dy_last]^T
    Eigen::Matrix3d G_odom = Eigen::Matrix3d::Identity();
    if (_use_yaw_jacobi_second_order) {
      // Entries wrt vx
      H_odom(0, 0) = -dt * tan_sa / l;
      H_odom(1, 0) = dt * std::cos(dyaw);
      H_odom(2, 0) = -dt * sin(dyaw);
      // Entries wrt sa
      H_odom(0, 1) = -dt * vx / l * tan_sa_2_p1;

      // We have to handle the singularity when the steering angle is zero or almost zero
      if (std::abs(tan_sa) < 1e-6) {
        H_odom(1, 1) =
            (2 * l * tan_sa_2_p1_2 * (std::sin(dyaw) - std::sin(dyaw_last)) +
             4 * l * tan_sa_2 * tan_sa_2_p1 * (std::sin(dyaw) - std::sin(dyaw_last)) - dt_2 * vx_2 / l * std::sin(dyaw) * tan_sa_2_p1_3 -
             6 * dt * vx * std::cos(dyaw) * tan_sa * tan_sa_2_p1_2) /
                (2 * tan_sa_2_p1_2 + 4 * tan_sa_2 * tan_sa_2_p1) +
            (dt_2 * vx_2 / l * std::sin(dyaw) * tan_sa_2_p1_2 + 2 * dt * vx * std::cos(dyaw) * tan_sa * tan_sa_2_p1) / tan_sa_2_p1;
        H_odom(2, 1) =
            (2 * l * tan_sa_2_p1_2 * (std::cos(dyaw) - std::cos(dyaw_last)) +
             4 * l * tan_sa_2 * tan_sa_2_p1 * (std::cos(dyaw) - std::cos(dyaw_last)) - dt_2 * vx_2 / l * std::cos(dyaw) * tan_sa_2_p1_3 +
             6 * dt * vx * std::sin(dyaw) * tan_sa * tan_sa_2_p1_2) /
                (2 * tan_sa_2_p1_2 + 4 * tan_sa_2 * tan_sa_2_p1) +
            (dt_2 * vx_2 / l * std::cos(dyaw) * tan_sa_2_p1_2 - 2 * dt * vx * std::sin(dyaw) * tan_sa * tan_sa_2_p1) / tan_sa_2_p1;
      } else {
        H_odom(1, 1) =
            (l * tan_sa_2_p1 * (std::sin(dyaw) - std::sin(dyaw_last))) / tan_sa_2 + (dt * vx * std::cos(dyaw) * tan_sa_2_p1) / tan_sa;

        H_odom(2, 1) =
            (l * tan_sa_2_p1 * (std::cos(dyaw) - std::cos(dyaw_last))) / tan_sa_2 - (dt * vx * std::sin(dyaw) * tan_sa_2_p1) / tan_sa;
      }
      // We have to handle the singularity when the steering angle is zero or almost zero
      if (std::abs(tan_sa) < 1e-6) {
        G_odom(1, 0) = -dt * vx * std::sin(dyaw);
        G_odom(2, 0) = -dt * vx * std::cos(dyaw);
      } else {
        G_odom(1, 0) = -(l * (std::cos(dyaw) - std::cos(dyaw_last))) / tan_sa;
        G_odom(2, 0) = (l * (std::sin(dyaw) - std::sin(dyaw_last))) / tan_sa;
      }
    } else {
      // Entries wrt vx
      H_odom(0, 0) = -dt * tan_sa / l;
      H_odom(1, 0) = dt * std::cos(dyaw_last);
      H_odom(2, 0) = -dt * std::sin(dyaw_last);
      // Entries wrt sa
      H_odom(0, 1) = -dt * vx / l * tan_sa_2_p1;

      G_odom(1, 0) = -dt * vx * std::sin(dyaw_last);
      G_odom(2, 0) = -dt * vx * std::cos(dyaw_last);
    }

    // Update the preintegration odometry covariance
    // std::cout << "G_odom" << std::endl << G_odom << std::endl;
    // std::cout << "H_odom" << std::endl << H_odom << std::endl;
    P_odom = G_odom * P_odom * G_odom.transpose() + H_odom * R_odom * H_odom.transpose();
  }

  // Set some values for readability
  auto p_IinG_last = state->_clones_IMU.at(last_cam_timestamp)->pos();
  auto R_GtoI_last = state->_clones_IMU.at(last_cam_timestamp)->Rot();
  auto p_IinG_current = state->_clones_IMU.at(state->_timestamp)->pos();
  auto R_GtoI_current = state->_clones_IMU.at(state->_timestamp)->Rot();

  // Calculate the measurement function
  // see eq (84) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  Eigen::Vector3d log_drot_state = ov_core::log_so3(_R_ItoO * R_GtoI_current * R_GtoI_last.transpose() * _R_ItoO.transpose());

  // see eq (85) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  Eigen::Vector3d dtrans_state =
      _R_ItoO * R_GtoI_last * (p_IinG_current - p_IinG_last + R_GtoI_current.transpose() * _p_OinI - R_GtoI_last.transpose() * _p_OinI);

  // Construct the Jacobians
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_clones_IMU.at(last_cam_timestamp)); // Jacobian w.r.t. the IMU state at last camera time
  Hx_order.push_back(state->_clones_IMU.at(state->_timestamp));  // Jacobian w.r.t. the IMU state at current camera time

  // Large final matrices used for update
  int h_size = 12;
  int m_size = 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::Vector3d res;
  Eigen::MatrixXd R = P_odom;

  // Construct our residual
  // "prediction - measurement" for orienational part, see eq (84) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  res[0] = log_drot_state.z() - dyaw;
  res[1] = dx - dtrans_state[0];
  res[2] = dy - dtrans_state[1];

  PRINT_DEBUG(BLUE "meas x %.9f odom x %.9f  meas - state = %.9f\n" RESET, dx, dtrans_state[0], dx - dtrans_state[0]);
  PRINT_DEBUG(BLUE "meas y %.9f odom y %.9f  meas - state = %.9f\n" RESET, dy, dtrans_state[1], dy - dtrans_state[1]);
  PRINT_DEBUG(BLUE "odom yaw %.6f state yaw %.6f residual %.6f\n" RESET, dyaw, log_drot_state.z(), log_drot_state.z() - dyaw);

  // Jacobian w.r.t. the IMU state at last camera time, see eq (95) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  // Take only z-row
  H.block(0, 0, 1, 3) = -(_R_ItoO * R_GtoI_current * R_GtoI_last.transpose()).row(2);
  // Take only x- and y-row
  H.block(1, 0, 2, 3) =
      (_R_ItoO * ov_core::skew_x(R_GtoI_last * (p_IinG_current + R_GtoI_current.transpose() * _p_OinI - p_IinG_last))).block(0, 0, 2, 3);
  // Take only x- and y-row
  H.block(1, 3, 2, 3) = -(_R_ItoO * R_GtoI_last).block(0, 0, 2, 3);

  // Jacobian w.r.t. the IMU state at current camera time, see eq (94) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  // Take only z-row
  H.block(0, 6, 1, 3) = _R_ItoO.row(2);
  // Take only x- and y-row
  H.block(1, 6, 2, 3) = -(_R_ItoO * R_GtoI_last * R_GtoI_current.transpose() * ov_core::skew_x(_p_OinI)).block(0, 0, 2, 3);
  // Take only x- and y-row
  H.block(1, 9, 2, 3) = (_R_ItoO * R_GtoI_last).block(0, 0, 2, 3);

  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

void UpdaterVehicle::update_vehicle_preintegrated_differential(std::shared_ptr<State> state, double last_cam_timestamp,
                                                               double last_prop_time_offset) {

  // Gather wheel speeds messages, asssume IMU and CAN are synchronized, so we still need to account for time offset between IMU and cam
  double time0 = last_cam_timestamp + last_prop_time_offset;
  double time1 = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);

  std::vector<ov_core::WheelSpeedsData> wheel_speeds_data_preintegration = select_wheel_speeds_readings(time0, time1);

  // Create our measured preintegrated odometry values, we follow the formulas of Visual-Inertial Wheel Odometry and Online calibration Tech
  // Report. The odometry preintegration function can be written as g = [dyaw dx dy]^T
  double dyaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  // Preintegration covariance matrix which is going to be built accumatively
  Eigen::Matrix3d P_odom = Eigen::Matrix3d::Zero();
  // This sensor noise matrix is going to be transformed into state space
  Eigen::Matrix2d R_odom = Eigen::Matrix2d::Zero();
  R_odom(0, 0) = _sigma_speed_x_2;
  R_odom(1, 1) = _sigma_speed_x_2;
  for (size_t i = 0; i < wheel_speeds_data_preintegration.size() - 1; i++) {
    // We follow the midpoint paradigm for preintegrating the single-track model
    double dt = wheel_speeds_data_preintegration[i + 1].timestamp - wheel_speeds_data_preintegration[i].timestamp;
    double v_rear_left =
        (wheel_speeds_data_preintegration[i + 1].wheel_rear_left + wheel_speeds_data_preintegration[i].wheel_rear_left) / 2.0;
    double v_rear_right =
        (wheel_speeds_data_preintegration[i + 1].wheel_rear_right + wheel_speeds_data_preintegration[i].wheel_rear_right) / 2.0;
    double vx = (v_rear_left + v_rear_right) / 2.0;
    double omega = (v_rear_right - v_rear_left) / _track_length;

    // Precompute values, and set some values (for readability)
    double dyaw_last = dyaw;
    dyaw -= omega * dt;
    if (_use_yaw_odom_second_order) {
      // We have to handle the singularity when wheel speeds induced turn rate is zero
      if (std::abs(omega) < 1e-6) {
        PRINT_DEBUG("update_preintegrated_differential(): omega %.6f almost zero \n" RESET, std::abs(omega));
        dx += vx * std::cos(dyaw) * dt;
        dy -= vx * std::sin(dyaw) * dt;
      } else {
        dx -= vx * (std::sin(dyaw) - std::sin(dyaw_last)) / omega;
        dy -= vx * (std::cos(dyaw) - std::cos(dyaw_last)) / omega;
      }
    } else {
      dx += vx * std::cos(dyaw_last) * dt;
      dy -= vx * std::sin(dyaw_last) * dt;
    }

    // Jacobian of g wrt our measurements: H_odom = (\partial g) / (\partial z), z = [wheel_speed_rear_left wheel_speed_rear_right]^T
    Eigen::Matrix<double, 3, 2> H_odom;
    // Jacobian of g wrt preintegrated odometry state: G_odom = (\partial g) / (\partial u), u = [dyaw_last dx_last dy_last]^T
    Eigen::Matrix3d G_odom = Eigen::Matrix3d::Identity();
    if (_use_yaw_jacobi_second_order) {
      // Entries wrt wheel_speed_rear_left
      H_odom(0, 0) = dt / _track_length;
      H_odom(1, 0) = dt * std::cos(dyaw_last) / 2.0;
      H_odom(2, 0) = -dt * std::sin(dyaw_last) / 2.0;
      // Entries wrt wheel_speed_rear_right
      H_odom(0, 1) = -dt / _track_length;
      H_odom(1, 1) = dt * std::cos(dyaw_last) / 2.0;
      H_odom(2, 1) = -dt * std::sin(dyaw_last) / 2.0;

      // Entries wrt dyaw
      if (std::abs(omega) < 1e-6) {
        G_odom(1, 0) = -vx * std::sin(dyaw) * dt;
        G_odom(2, 0) = -vx * std::cos(dyaw) * dt;
      } else {
        G_odom(1, 0) = -vx * (std::cos(dyaw) - std::cos(dyaw_last)) / omega;
        G_odom(2, 0) = vx * (std::sin(dyaw) - std::sin(dyaw_last)) / omega;
      }
    } else {
      // Entries wrt wheel_speed_rear_left
      H_odom(0, 0) = dt / _track_length;
      H_odom(1, 0) = dt * std::cos(dyaw_last) / 2.0;
      H_odom(2, 0) = -dt * std::sin(dyaw_last) / 2.0;
      // Entries wrt wheel_speed_rear_right
      H_odom(0, 1) = -dt / _track_length;
      H_odom(1, 1) = dt * std::cos(dyaw_last) / 2.0;
      H_odom(2, 1) = -dt * std::sin(dyaw_last) / 2.0;

      // Entries wrt to dyaw_last
      G_odom(1, 0) = -vx * std::sin(dyaw_last) * dt;
      G_odom(2, 0) = -vx * std::cos(dyaw_last) * dt;
    }

    // Update the preintegration odometry covariance
    // std::cout << "G_odom" << std::endl << G_odom << std::endl;
    // std::cout << "H_odom" << std::endl << H_odom << std::endl;
    P_odom = G_odom * P_odom * G_odom.transpose() + H_odom * R_odom * H_odom.transpose();
  }

  // Set some values for readability
  auto p_IinG_last = state->_clones_IMU.at(last_cam_timestamp)->pos();
  auto R_GtoI_last = state->_clones_IMU.at(last_cam_timestamp)->Rot();
  auto p_IinG_current = state->_clones_IMU.at(state->_timestamp)->pos();
  auto R_GtoI_current = state->_clones_IMU.at(state->_timestamp)->Rot();

  // Calculate the measurement function
  // see eq (84) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  Eigen::Vector3d log_drot_state = ov_core::log_so3(_R_ItoO * R_GtoI_current * R_GtoI_last.transpose() * _R_ItoO.transpose());

  // see eq (85) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  Eigen::Vector3d dtrans_state =
      _R_ItoO * R_GtoI_last * (p_IinG_current - p_IinG_last + R_GtoI_current.transpose() * _p_OinI - R_GtoI_last.transpose() * _p_OinI);

  // Construct the Jacobians
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_clones_IMU.at(last_cam_timestamp)); // Jacobian w.r.t. the IMU state at last camera time
  Hx_order.push_back(state->_clones_IMU.at(state->_timestamp));  // Jacobian w.r.t. the IMU state at current camera time

  // Large final matrices used for update
  int h_size = 12;
  int m_size = 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::Vector3d res;
  Eigen::MatrixXd R = P_odom;

  // Construct our residual
  // "prediction - measurement" for orienational part, see eq (84) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  res[0] = log_drot_state.z() - dyaw;
  res[1] = dx - dtrans_state[0];
  res[2] = dy - dtrans_state[1];

  PRINT_DEBUG(RED "meas trans odom x %.9f state trans odom x %.9f  meas - state = %.9f\n" RESET, dx, dtrans_state[0], dx - dtrans_state[0]);
  PRINT_DEBUG(RED "meas trans odom y %.9f state trans odom y %.9f  meas - state = %.9f\n" RESET, dy, dtrans_state[1], dy - dtrans_state[1]);
  PRINT_DEBUG(RED "odom yaw %.6f state yaw %.6f residual %.6f\n" RESET, dyaw, log_drot_state.z(), log_drot_state.z() - dyaw);

  // Jacobian w.r.t. the IMU state at last camera time, see eq (95) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  // Take only z-row
  H.block(0, 0, 1, 3) = -(_R_ItoO * R_GtoI_current * R_GtoI_last.transpose()).row(2);
  // Take only x- and y-row
  H.block(1, 0, 2, 3) =
      (_R_ItoO * ov_core::skew_x(R_GtoI_last * (p_IinG_current + R_GtoI_current.transpose() * _p_OinI - p_IinG_last))).block(0, 0, 2, 3);
  // Take only x- and y-row
  H.block(1, 3, 2, 3) = -(_R_ItoO * R_GtoI_last).block(0, 0, 2, 3);

  // Jacobian w.r.t. the IMU state at current camera time, see eq (94) of Visual-Inertial Wheel Odometry and Online calibration Tech Report
  // Take only z-row
  H.block(0, 6, 1, 3) = _R_ItoO.row(2);
  // Take only x- and y-row
  H.block(1, 6, 2, 3) = -(_R_ItoO * R_GtoI_last * R_GtoI_current.transpose() * ov_core::skew_x(_p_OinI)).block(0, 0, 2, 3);
  // Take only x- and y-row
  H.block(1, 9, 2, 3) = (_R_ItoO * R_GtoI_last).block(0, 0, 2, 3);

  // std::cout << "H" << std::endl << H << std::endl;
  // std::cout << "P_odom_final" << std::endl << P_odom << std::endl;

  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
}

std::vector<ov_core::AckermannDriveData> UpdaterVehicle::select_ackermann_drive_readings(double time0, double time1, bool warn) {
  // Our vector of ackermann drive  readings
  std::vector<ov_core::AckermannDriveData> ackermann_drive_data_preintegration;

  // Ensure we have some measurements in the first place!
  if (ackermann_drive_data.empty()) {
    if (warn)
      PRINT_ERROR(RED "UpdaterVehicle::select_speed_readings(): No Ackermann drive measurements!!!\n" RESET);
    return ackermann_drive_data_preintegration;
  }

  // Loop through and find all the needed measurements to propagate with
  // Note we split measurements based on the given state time, and the update timestamp
  for (size_t i = 0; i < ackermann_drive_data.size() - 1; i++) {

    // START OF THE INTEGRATION PERIOD
    // If the next timestamp is greater then our current state time
    // And the current is not greater then it yet...
    // Then we should "split" our current IMU measurement
    if (ackermann_drive_data.at(i + 1).timestamp > time0 && ackermann_drive_data.at(i).timestamp < time0) {
      ov_core::AckermannDriveData data;
      data.speed = interpolate_double(ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).speed,
                                      ackermann_drive_data.at(i + 1).timestamp, ackermann_drive_data.at(i + 1).speed, time0);
      data.steering_angle =
          interpolate_double(ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).steering_angle,
                             ackermann_drive_data.at(i + 1).timestamp, ackermann_drive_data.at(i + 1).steering_angle, time0);
      data.timestamp = time0;
      ackermann_drive_data_preintegration.push_back(data);

      continue;
    }

    // MIDDLE OF INTEGRATION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then we should just append the whole measurement time to our propagation vector
    if (ackermann_drive_data.at(i).timestamp >= time0 && ackermann_drive_data.at(i + 1).timestamp <= time1) {
      ackermann_drive_data_preintegration.push_back(ackermann_drive_data.at(i));
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
    // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
    if (ackermann_drive_data.at(i + 1).timestamp > time1) {
      // If we have a very low frequency IMU then, we could have only recorded the first integration (i.e. case 1) and nothing else
      // In this case, both the current IMU measurement and the next is greater than the desired intepolation, thus we should just cut the
      // current at the desired time Else, we have hit CASE2 and this IMU measurement is not past the desired propagation time, thus add the
      // whole IMU reading
      if (ackermann_drive_data.at(i).timestamp > time1 && i == 0) {
        // This case can happen if we don't have any imu data that has occured before the startup time
        // This means that either we have dropped IMU data, or we have not gotten enough.
        // In this case we can't propgate forward in time, so there is not that much we can do.
        break;
      } else if (ackermann_drive_data.at(i).timestamp > time1) {
        ov_core::AckermannDriveData data;
        data.speed = interpolate_double(ackermann_drive_data.at(i - 1).timestamp, ackermann_drive_data.at(i - 1).speed,
                                        ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).speed, time1);
        data.steering_angle = interpolate_double(ackermann_drive_data.at(i - 1).timestamp, ackermann_drive_data.at(i - 1).steering_angle,
                                                 ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).steering_angle, time1);

        data.timestamp = time1;
        ackermann_drive_data_preintegration.push_back(data);
        // printf("speed readings #%d = CASE 3.1 = %.9f => %.9f\n", (int)i,
        //        ackermann_drive_data.at(i).timestamp - ackermann_drive_data_preintegration.at(0).timestamp,
        //        ackermann_drive_data.at(i).timestamp - time0);
      } else {
        ackermann_drive_data_preintegration.push_back(ackermann_drive_data.at(i));
        // printf("speed readings #%d = CASE 3.2 = %.9f => %.9f\n", (int)i,
        //        ackermann_drive_data.at(i).timestamp - ackermann_drive_data_preintegration.at(0).timestamp,
        //        ackermann_drive_data.at(i).timestamp - time0);
      }
      // If the added IMU message doesn't end exactly at the camera time
      // Then we need to add another one that is right at the ending time
      if (ackermann_drive_data_preintegration.at(ackermann_drive_data_preintegration.size() - 1).timestamp != time1) {
        ov_core::AckermannDriveData data;
        data.speed = interpolate_double(ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).speed,
                                        ackermann_drive_data.at(i + 1).timestamp, ackermann_drive_data.at(i + 1).speed, time1);
        data.steering_angle =
            interpolate_double(ackermann_drive_data.at(i).timestamp, ackermann_drive_data.at(i).steering_angle,
                               ackermann_drive_data.at(i + 1).timestamp, ackermann_drive_data.at(i + 1).steering_angle, time1);

        data.timestamp = time1;
        ackermann_drive_data_preintegration.push_back(data);
        // printf("speed readings #%d = CASE 3.3 = %.9f => %.9f\n", (int)i, data.timestamp -
        // ackermann_drive_data_preintegration.at(0).timestamp,
        //        data.timestamp - time0);
      }
      break;
    }
  }

  // Some debug output
  PRINT_DEBUG(MAGENTA "UpdaterVehicle::select_speed_readings(): time0 = %.9f time1 = %.9f\n" RESET, time0, time1);
  for (size_t i = 0; i < ackermann_drive_data_preintegration.size(); i++) {
    PRINT_DEBUG(MAGENTA "UpdaterVehicle::select_speed_readings(): ackermann_drive_data_preintegration #%d t = %.9f\n" RESET, (int)i,
                ackermann_drive_data_preintegration[i].timestamp);
  }

  // Check that we have at least one measurement to propagate with
  if (ackermann_drive_data_preintegration.empty()) {
    if (warn)
      PRINT_WARNING(
          YELLOW "UpdaterVehicle::select_speed_readings(): No speed measurements to propagate do preintegration with (%d of 2)!!!\n" RESET,
          (int)ackermann_drive_data_preintegration.size());
    return ackermann_drive_data_preintegration;
  }

  // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to
  // reach) Then we should just "stretch" the last measurement to be the whole period (case 3 in the above loop)
  // if(time1-imu_data.at(imu_data.size()-1).timestamp > 1e-3) {
  //    printf(YELLOW "UpdaterVehicle::select_speed_readings(): Missing inertial measurements to propagate with (%.6f sec missing).
  //    IMU-CAMERA are likely messed up!!!\n" RESET, (time1-imu_data.at(imu_data.size()-1).timestamp)); return
  //    ackermann_drive_data_preintegration;
  //}

  // Loop through and ensure we do not have an zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < ackermann_drive_data_preintegration.size() - 1; i++) {
    if (std::abs(ackermann_drive_data_preintegration.at(i + 1).timestamp - ackermann_drive_data_preintegration.at(i).timestamp) < 1e-12) {
      if (warn)
        PRINT_WARNING(YELLOW "UpdaterVehicle::select_speed_readings(): Zero DT between IMU reading %d and %d, removing it!\n" RESET, (int)i,
                      (int)(i + 1));
      ackermann_drive_data_preintegration.erase(ackermann_drive_data_preintegration.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (ackermann_drive_data_preintegration.size() < 2) {
    if (warn)
      PRINT_WARNING(
          YELLOW "UpdaterVehicle::select_speed_readings(): No speed measurements to propagate do preintegration with (%d of 2)!!!\n" RESET,
          (int)ackermann_drive_data_preintegration.size());

    return ackermann_drive_data_preintegration;
  }

  // Success :D
  return ackermann_drive_data_preintegration;
}

std::vector<ov_core::WheelSpeedsData> UpdaterVehicle::select_wheel_speeds_readings(double time0, double time1, bool warn) {
  // Our vector wheel speeds readings
  std::vector<ov_core::WheelSpeedsData> wheel_speeds_data_preintegration;

  // Ensure we have some measurements in the first place!
  if (wheel_speeds_data.empty()) {
    if (warn)
      printf(YELLOW "UpdaterVehicle::select_wheel_speeds_readings(): No wheel speeds measurements!!!\n" RESET);
    return wheel_speeds_data_preintegration;
  }

  // Loop through and find all the needed measurements to propagate with
  // Note we split measurements based on the given state time, and the update timestamp
  for (size_t i = 0; i < wheel_speeds_data.size() - 1; i++) {

    // START OF THE INTEGRATION PERIOD
    // If the next timestamp is greater then our current state time
    // And the current is not greater then it yet...
    // Then we should "split" our current IMU measurement
    if (wheel_speeds_data.at(i + 1).timestamp > time0 && wheel_speeds_data.at(i).timestamp < time0) {
      ov_core::WheelSpeedsData data;
      data.wheel_front_left =
          interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_left,
                             wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_front_left, time0);
      data.wheel_front_right =
          interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_right,
                             wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_front_right, time0);
      data.wheel_rear_left = interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_left,
                                                wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_rear_left, time0);
      data.wheel_rear_right =
          interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_right,
                             wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_rear_right, time0);
      data.timestamp = time0;
      wheel_speeds_data_preintegration.push_back(data);

      continue;
    }

    // MIDDLE OF INTEGRATION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then we should just append the whole measurement time to our propagation vector
    if (wheel_speeds_data.at(i).timestamp >= time0 && wheel_speeds_data.at(i + 1).timestamp <= time1) {
      wheel_speeds_data_preintegration.push_back(wheel_speeds_data.at(i));
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
    // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
    if (wheel_speeds_data.at(i + 1).timestamp > time1) {
      // If we have a very low frequency IMU then, we could have only recorded the first integration (i.e. case 1) and nothing else
      // In this case, both the current IMU measurement and the next is greater than the desired intepolation, thus we should just cut the
      // current at the desired time Else, we have hit CASE2 and this IMU measurement is not past the desired propagation time, thus add the
      // whole IMU reading
      if (wheel_speeds_data.at(i).timestamp > time1 && i == 0) {
        // This case can happen if we don't have any imu data that has occured before the startup time
        // This means that either we have dropped IMU data, or we have not gotten enough.
        // In this case we can't propgate forward in time, so there is not that much we can do.
        break;
      } else if (wheel_speeds_data.at(i).timestamp > time1) {
        ov_core::WheelSpeedsData data;
        data.wheel_front_left = interpolate_double(wheel_speeds_data.at(i - 1).timestamp, wheel_speeds_data.at(i - 1).wheel_front_left,
                                                   wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_left, time1);
        data.wheel_front_right = interpolate_double(wheel_speeds_data.at(i - 1).timestamp, wheel_speeds_data.at(i - 1).wheel_front_right,
                                                    wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_right, time1);
        data.wheel_rear_left = interpolate_double(wheel_speeds_data.at(i - 1).timestamp, wheel_speeds_data.at(i - 1).wheel_rear_left,
                                                  wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_left, time1);
        data.wheel_rear_right = interpolate_double(wheel_speeds_data.at(i - 1).timestamp, wheel_speeds_data.at(i - 1).wheel_rear_right,
                                                   wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_right, time1);
        data.timestamp = time1;
        wheel_speeds_data_preintegration.push_back(data);
        // printf("speed readings #%d = CASE 3.1 = %.9f => %.9f\n", (int)i,
        //        wheel_speeds_data.at(i).timestamp - wheel_speeds_data_preintegration.at(0).timestamp, wheel_speeds_data.at(i).timestamp -
        //        time0);
      } else {
        wheel_speeds_data_preintegration.push_back(wheel_speeds_data.at(i));
        // printf("speed readings #%d = CASE 3.2 = %.9f => %.9f\n", (int)i,
        //        wheel_speeds_data.at(i).timestamp - wheel_speeds_data_preintegration.at(0).timestamp, wheel_speeds_data.at(i).timestamp -
        //        time0);
      }
      // If the added IMU message doesn't end exactly at the camera time
      // Then we need to add another one that is right at the ending time
      if (wheel_speeds_data_preintegration.at(wheel_speeds_data_preintegration.size() - 1).timestamp != time1) {
        ov_core::WheelSpeedsData data;
        data.wheel_front_left =
            interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_left,
                               wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_front_left, time1);
        data.wheel_front_right =
            interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_front_right,
                               wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_front_right, time1);
        data.wheel_rear_left =
            interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_left,
                               wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_rear_left, time1);
        data.wheel_rear_right =
            interpolate_double(wheel_speeds_data.at(i).timestamp, wheel_speeds_data.at(i).wheel_rear_right,
                               wheel_speeds_data.at(i + 1).timestamp, wheel_speeds_data.at(i + 1).wheel_rear_right, time1);
        data.timestamp = time1;
        wheel_speeds_data_preintegration.push_back(data);
        // printf("speed readings #%d = CASE 3.3 = %.9f => %.9f\n", (int)i, data.timestamp -
        // wheel_speeds_data_preintegration.at(0).timestamp,
        //        data.timestamp - time0);
      }
      break;
    }
  }

  // Some debug output
  PRINT_DEBUG(MAGENTA "UpdaterVehicle::select_wheel_speeds_readings(): time0 = %.9f time1 = %.9f\n" RESET, time0, time1);
  for (size_t i = 0; i < wheel_speeds_data_preintegration.size(); i++) {
    PRINT_DEBUG(MAGENTA "UpdaterVehicle::select_wheel_speeds_readings(): wheel_speeds_data_preintegration #%d t = %.9f\n" RESET, (int)i,
                wheel_speeds_data_preintegration[i].timestamp);
  }

  // Check that we have at least one measurement to propagate with
  if (wheel_speeds_data_preintegration.empty()) {
    if (warn)
      PRINT_WARNING(
          YELLOW
          "UpdaterVehicle::select_wheel_speeds_readings(): No speed measurements to propagate do preintegration with (%d of 2)!!!\n" RESET,
          (int)wheel_speeds_data_preintegration.size());
    return wheel_speeds_data_preintegration;
  }

  // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to
  // reach) Then we should just "stretch" the last measurement to be the whole period (case 3 in the above loop)
  // if(time1-imu_data.at(imu_data.size()-1).timestamp > 1e-3) {
  //    printf(YELLOW "UpdaterVehicle::select_speed_readings(): Missing inertial measurements to propagate with (%.6f sec missing).
  //    IMU-CAMERA are likely messed up!!!\n" RESET, (time1-imu_data.at(imu_data.size()-1).timestamp)); return
  //    wheel_speeds_data_preintegration;
  //}

  // Loop through and ensure we do not have an zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < wheel_speeds_data_preintegration.size() - 1; i++) {
    if (std::abs(wheel_speeds_data_preintegration.at(i + 1).timestamp - wheel_speeds_data_preintegration.at(i).timestamp) < 1e-12) {
      if (warn)
        PRINT_WARNING(YELLOW "UpdaterVehicle::select_wheel_speeds_readings(): Zero DT between IMU reading %d and %d, removing it!\n" RESET,
                      (int)i, (int)(i + 1));
      wheel_speeds_data_preintegration.erase(wheel_speeds_data_preintegration.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (wheel_speeds_data_preintegration.size() < 2) {
    if (warn)
      PRINT_WARNING(
          YELLOW
          "UpdaterVehicle::select_wheel_speeds_readings(): No speed measurements to propagate do preintegration with (%d of 2)!!!\n" RESET,
          (int)wheel_speeds_data_preintegration.size());

    return wheel_speeds_data_preintegration;
  }

  // Success :D
  return wheel_speeds_data_preintegration;
}

double UpdaterVehicle::interpolate_double(double time0, double value0, double time1, double value1, double time) {
  if (time < time0 || time > time1) {
    PRINT_ERROR(RED "time0 %.9f and time1 %.9f, time to interpolate at %.9f\n" RESET, time0, time1, time);
  }
  double lambda = (time - time1) / (time1 - time0);
  return (1 - lambda) * value0 + lambda * value1;
}
