/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "VioManager.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();
  params.print_and_load_vehicle_updates(); // OVVU

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.use_multi_threading ? -1 : 0);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,";
    // OVVU: Also track timing of vehicle updates
    if (params.vehicle_update_mode != params.VEHICLE_UPDATE_NONE && params.vehicle_update_mode != params.VEHICLE_UPDATE_UNKNOWN) {
      of_statistics << "vehicle updates,";
    }
    of_statistics << "msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  trackDATABASE = std::make_shared<FeatureDatabase>();
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, params.init_options.init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, params.init_options.init_max_features, state->_options.max_aruco_features, params.use_stereo,
        params.histogram_method, params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }

  // Feature initializer for active tracks
  active_tracks_initializer = std::make_shared<FeatureInitializer>(params.featinit_options);

  // OVVU: If we are using vehicle updates, then create the updater
  if (params.vehicle_update_mode != params.VEHICLE_UPDATE_NONE && params.vehicle_update_mode != params.VEHICLE_UPDATE_UNKNOWN) {
    updaterVehicle = std::make_shared<UpdaterVehicle>(params, propagator);
  }
}

void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                             const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM = std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->_cam_intrinsics_cameras, state->_options.max_aruco_features);
    trackFEATS = trackSIM;
    PRINT_WARNING(RED "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
  }

  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  if (is_initialized_vio) {
    trackDATABASE->append_new_measurements(trackSIM->get_feature_database());
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      return;
    }
  }

  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[SIM]: your vio system should already be initialized before simulating features!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->_cam_intrinsics_cameras.at(camid)->w();
    int height = state->_cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

void VioManager::feed_measurement_ackermann_drive(const ov_core::AckermannDriveData &message) {
  if (is_initialized_vio && params.use_ackermann_drive_measurements && updaterVehicle != nullptr) {
    ackermann_drive_queue.push_back(message);
    updaterVehicle->feed_ackermann_drive(message);
  }
}

void VioManager::feed_measurement_wheel_speeds(const ov_core::WheelSpeedsData &message) {
  if (is_initialized_vio && params.use_wheel_speeds_measurements && updaterVehicle != nullptr) {
    updaterVehicle->feed_wheel_speeds(message);
  }
}

void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);
  if (is_initialized_vio) {
    trackDATABASE->append_new_measurements(trackFEATS->get_feature_database());
  }

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
    trackDATABASE->append_new_measurements(trackARUCO->get_feature_database());
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->_timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "image received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (message.timestamp - state->_timestamp));
    return;
  }

  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->_timestamp != message.timestamp) {

    // OVVU: Instead of propagating and cloning at the same time, we split those two steps due to the updating logic of vehicle updates with
    // addtional state propagations between two camera times.

    // OVVU: Last updating times for preintegrated update
    double last_cam_timestamp = state->_timestamp;
    double last_prop_time_offset = propagator->get_last_prop_time_offset();

    // OVVU: perform vehicle updates by propagating the state at sensor frequency of ackermann drive messages
    auto time_tmp_vehicle = boost::posix_time::microsec_clock::local_time();
    time_vehicle_update = 0;
    if (params.vehicle_update_mode == params.VEHICLE_UPDATE_SPEED || params.vehicle_update_mode == params.VEHICLE_UPDATE_STEERING ||
        params.vehicle_update_mode == params.VEHICLE_UPDATE_SPEED_AND_STEERING) {
      while (updaterVehicle != nullptr && !ackermann_drive_queue.empty()) {
        // OVVU: Perform vehicle updates when max clones are reached for stability reasons
        if ((int)state->_clones_IMU.size() == state->_options.max_clone_size &&
            ackermann_drive_queue.front().timestamp < message.timestamp) {
          // OVVU: When the latest ackermann drive time is lower than the IMU time, we can propagate and do the update. Otherwise we only
          // perform the update without propagating because, we would have to extrapolate IMU measurements into the future.
          if (ackermann_drive_queue.front().timestamp < message.timestamp + state->_calib_dt_CAMtoIMU->value()(0)) {
            propagator->propagate(state, ackermann_drive_queue.front().timestamp, false);
            PRINT_DEBUG("Propagating and updating vehicle at ts %.9f\n" RESET, ackermann_drive_queue.front().timestamp);
          } else {
            PRINT_DEBUG("Skipping propagation, but updating vehicle at ts %.9f\n" RESET, ackermann_drive_queue.front().timestamp);
          }
          auto time_tmp_vehicle = boost::posix_time::microsec_clock::local_time();
          // Perform speed udpate
          if (params.vehicle_update_mode == params.VEHICLE_UPDATE_SPEED ||
              params.vehicle_update_mode == params.VEHICLE_UPDATE_SPEED_AND_STEERING) {
            if (params.speed_update_mode == params.SPEED_UPDATE_VECTOR) {
              updaterVehicle->update_speed_vector(state, ackermann_drive_queue.front());
            } else if (params.speed_update_mode == params.SPEED_UPDATE_X) {
              updaterVehicle->update_speed_x(state, ackermann_drive_queue.front());
            }
          }
          // Perform steering udpate
          if (params.vehicle_update_mode == params.VEHICLE_UPDATE_STEERING ||
              params.vehicle_update_mode == params.VEHICLE_UPDATE_SPEED_AND_STEERING) {
            updaterVehicle->update_steering(state, ackermann_drive_queue.front());
          }
          time_vehicle_update += (boost::posix_time::microsec_clock::local_time() - time_tmp_vehicle).total_microseconds() * 1e-6;
        }
        // In any case we pop the latest ackermann drive message
        ackermann_drive_queue.pop_front();
        // If we have an ackermann drive message that is newer than the current camera time, we break at this point and perform the
        // corresponding updates with the next image.
        if (!ackermann_drive_queue.empty()) {
          if (ackermann_drive_queue.front().timestamp > message.timestamp) {
            break;
          }
        }
      }
    }

    // OVVU: After potential vehicle updates propagate the filter to the current camera time
    propagator->propagate(state, message.timestamp, true);
    propagator->clone(state);

    // OVVU: Perform preintegrated vehicle updates at this point, note that we either perform preintegrated updates or vehicle updates at
    // sensor frequency (see above).
    time_tmp_vehicle = boost::posix_time::microsec_clock::local_time();
    // Perform preintegrated vehicle update using the Ackermann drive model after cloning the propagated state. Also only perform the update
    // if max clones are reached for stability reasons.
    if (updaterVehicle != nullptr && params.vehicle_update_mode == params.VEHICLE_UPDATE_PREINTEGRATED_SINGLE_TRACK &&
        (int)state->_clones_IMU.size() >= state->_options.max_clone_size) {
      updaterVehicle->update_vehicle_preintegrated_single_track(state, last_cam_timestamp, last_prop_time_offset);
    }
    // Perform preintegrated vehicle update using differential drive model after cloning the propagated state. Also only perform the update
    // if max clones are reached for stability reasons.
    if (updaterVehicle != nullptr && params.vehicle_update_mode == params.VEHICLE_UPDATE_PREINTEGRATED_DIFFERENTIAL_DRIVE &&
        (int)state->_clones_IMU.size() >= state->_options.max_clone_size) {
      updaterVehicle->update_vehicle_preintegrated_differential_drive(state, last_cam_timestamp, last_prop_time_offset);
    }
    time_vehicle_update = (boost::posix_time::microsec_clock::local_time() - time_tmp_vehicle).total_microseconds() * 1e-6;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can start triangulating things...
  if ((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size, 5)) {
    PRINT_DEBUG("waiting for enough clone states (%d of %d)....\n", (int)state->_clones_IMU.size(),
                std::min(state->_options.max_clone_size, 5));
    return;
  }

  // Return if we where unable to propagate
  if (state->_timestamp != message.timestamp) {
    PRINT_WARNING(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    PRINT_WARNING(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET, message.timestamp - state->_timestamp);
    return;
  }
  has_moved_since_zupt = true;

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================

  // Now, lets get all features that should be used for an update that are lost in the newest frame
  // We explicitly request features that have not been deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;
  feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp, false, true);

  // Don't need to get the oldest features until we reach our max number of clones
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep(), false, true);
    if (trackARUCO != nullptr && message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
    }
  }

  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to use those in the update yet
  // E.g: thus we wait until cam0 process its newest image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(), camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) != feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  // Find tracks that have reached max length, these can be made into SLAM features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      if ((int)cams.second.size() > state->_options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((int)(*it0).second->_featid <= 4 * state->_options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
  if (state->_options.max_slam_features > 0 && message.timestamp - startup_time >= params.dt_slam_delay &&
      (int)state->_features_SLAM.size() < state->_options.max_slam_features + curr_aruco_tags) {
    // Get the total amount to add, then the max amount that we can add given our marginalize feature array
    int amount_to_add = (state->_options.max_slam_features + curr_aruco_tags) - (int)state->_features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
    }
  }

  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
  // Note: we only enforce this if the current camera message is where the feature was seen from
  // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark : state->_features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->_unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(), landmark.second->_unique_camera_id) != message.sensor_ids.end();
    if (feat2 == nullptr && current_unique_cam)
      landmark.second->should_marg = true;
  }

  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());

  //===================================================================================
  // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
  //===================================================================================

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  });

  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
  // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
  if ((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end() - state->_options.max_msckf_in_update);
  updaterMSCKF->update(state, featsup_MSCKF);
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
  std::vector<std::shared_ptr<Feature>> feats_slam_UPDATE_TEMP;
  while (!feats_slam_UPDATE.empty()) {
    // Get sub vector of the features we will update with
    std::vector<std::shared_ptr<Feature>> featsup_TEMP;
    featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(),
                        feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(),
                            feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    // Do the update
    updaterSLAM->update(state, featsup_TEMP);
    feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
  }
  feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
  rT5 = boost::posix_time::microsec_clock::local_time();
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  rT6 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Update our visualization feature set, and clean up the old features
  //===================================================================================

  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {

    // Re-triangulate features
    retriangulate_active_tracks(message);

    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // Save all the MSCKF features used in the update
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }

  // First do anchor change if we are about to lose an anchor pose
  updaterSLAM->change_anchors(state);

  // Cleanup any features older then the marginalization time
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
    trackDATABASE->cleanup_measurements(state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
    }
  }

  // Finally marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  rT7 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Debug info, and stats tracking
  //===================================================================================

  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
  double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;
  double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;
  double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6;
  double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;
  double time_total = (rT7 - rT1).total_microseconds() * 1e-6;

  // OVVU: If we perform vehicle updates we have to subtract the vehicle update time from time_prop because vehicle related updates take
  // place between rT2 and rT3.
  if (updaterVehicle != nullptr) {
    time_prop -= time_vehicle_update;
  }

  // Timing information
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for vehicle related updates\n" RESET, time_vehicle_update);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for MSCKF update (%d feats)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
  if (state->_options.max_slam_features > 0) {
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET, time_slam_update, (int)state->_features_SLAM.size());
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
  }
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for re-tri & marg (%d clones in state)\n" RESET, time_marg, (int)state->_clones_IMU.size());

  std::stringstream ss;
  ss << "[TIME]: " << std::setprecision(4) << time_total << " seconds for total (camera";
  for (const auto &id : message.sensor_ids) {
    ss << " " << id;
  }
  ss << ")" << std::endl;
  PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());

  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << "," << std::fixed << std::setprecision(6) << time_track << ","
                  << time_prop << ",";
    // OVVU: append to file if we are performing vehicle related update
    if (updaterVehicle != nullptr) {
      of_statistics << time_vehicle_update << ",";
    }
    of_statistics << time_msckf << ",";
    if (state->_options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_marg << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // Update our distance traveled
  if (timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
    Eigen::Matrix<double, 3, 1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();
  }
  timelastupdate = message.timestamp;

  // Debug, print our current state
  PRINT_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n", state->_imu->quat()(0),
             state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3), state->_imu->pos()(0), state->_imu->pos()(1),
             state->_imu->pos()(2), distance);
  PRINT_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_INFO("camera-imu timeoffset = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                 calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }
}

bool VioManager::try_to_initialize(const ov_core::CameraData &message) {

  // Directly return if the initialization thread is running
  // Note that we lock on the queue since we could have finished an update
  // And are using this queue to propagate the state forward. We should wait in this case
  if (thread_init_running) {
    std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
    camera_queue_init.push_back(message.timestamp);
    return false;
  }

  // If the thread was a success, then return success!
  if (thread_init_success) {
    return true;
  }

  // Run the initialization in a second thread so it can go as slow as it desires
  thread_init_running = true;
  std::thread thread([&] {
    // Returns from our initializer
    double timestamp;
    Eigen::MatrixXd covariance;
    std::vector<std::shared_ptr<ov_type::Type>> order;
    auto init_rT1 = boost::posix_time::microsec_clock::local_time();

    // Try to initialize the system
    // We will wait for a jerk if we do not have the zero velocity update enabled
    // Otherwise we can initialize right away as the zero velocity will handle the stationary case
    bool wait_for_jerk = (updaterZUPT == nullptr);
    bool success = initializer->initialize(timestamp, covariance, order, state->_imu, wait_for_jerk);

    // If we have initialized successfully we will set the covariance and state elements as needed
    // TODO: set the clones and SLAM features here so we can start updating right away...
    if (success) {

      // Set our covariance (state should already be set in the initializer)
      StateHelper::set_initial_covariance(state, covariance, order);

      // Set the state time
      state->_timestamp = timestamp;
      startup_time = timestamp;

      // Cleanup any features older than the initialization time
      // Also increase the number of features to the desired amount during estimation
      trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
      trackFEATS->set_num_features(params.num_pts);
      if (trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
      }

      // If we are moving then don't do zero velocity update4
      if (state->_imu->vel().norm() > params.zupt_max_velocity) {
        has_moved_since_zupt = true;
      }

      // Else we are good to go, print out our stats
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_INFO(GREEN "[init]: successful initialization in %.4f seconds\n" RESET, (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      PRINT_INFO(GREEN "[init]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET, state->_imu->quat()(0), state->_imu->quat()(1),
                 state->_imu->quat()(2), state->_imu->quat()(3));
      PRINT_INFO(GREEN "[init]: bias gyro = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_g()(0), state->_imu->bias_g()(1),
                 state->_imu->bias_g()(2));
      PRINT_INFO(GREEN "[init]: velocity = %.4f, %.4f, %.4f\n" RESET, state->_imu->vel()(0), state->_imu->vel()(1), state->_imu->vel()(2));
      PRINT_INFO(GREEN "[init]: bias accel = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_a()(0), state->_imu->bias_a()(1),
                 state->_imu->bias_a()(2));
      PRINT_INFO(GREEN "[init]: position = %.4f, %.4f, %.4f\n" RESET, state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));

      // Remove any camera times that are order then the initialized time
      // This can happen if the initialization has taken a while to perform
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      std::vector<double> camera_timestamps_to_init;
      for (size_t i = 0; i < camera_queue_init.size(); i++) {
        if (camera_queue_init.at(i) > timestamp) {
          camera_timestamps_to_init.push_back(camera_queue_init.at(i));
        }
      }

      // Now we have initialized we will propagate the state to the current timestep
      // In general this should be ok as long as the initialization didn't take too long to perform
      // Propagating over multiple seconds will become an issue if the initial biases are bad
      size_t clone_rate = (size_t)((double)camera_timestamps_to_init.size() / (double)params.state_options.max_clone_size) + 1;
      for (size_t i = 0; i < camera_timestamps_to_init.size(); i += clone_rate) {
        propagator->propagate_and_clone(state, camera_timestamps_to_init.at(i));
        StateHelper::marginalize_old_clone(state);
      }
      PRINT_DEBUG(YELLOW "[init]: moved the state forward %.2f seconds\n" RESET, state->_timestamp - timestamp);
      thread_init_success = true;
      camera_queue_init.clear();

    } else {
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_DEBUG(YELLOW "[init]: failed initialization in %.4f seconds\n" RESET, (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      thread_init_success = false;
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      camera_queue_init.clear();
    }

    // Finally, mark that the thread has finished running
    thread_init_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!params.use_multi_threading) {
    thread.join();
  } else {
    thread.detach();
  }
  return false;
}

void VioManager::retriangulate_active_tracks(const ov_core::CameraData &message) {

  // Start timing
  boost::posix_time::ptime retri_rT1, retri_rT2, retri_rT3, retri_rT4, retri_rT5;
  retri_rT1 = boost::posix_time::microsec_clock::local_time();

  // Clear old active track data
  active_tracks_time = state->_timestamp;
  active_image = message.images.at(0).clone();
  active_tracks_posinG.clear();
  active_tracks_uvd.clear();

  // Get all features which are tracked in the current frame
  // NOTE: This database should have all features from all trackers already in it
  // NOTE: it also has the complete history so we shouldn't see jumps from deleting measurements
  std::vector<std::shared_ptr<Feature>> active_features = trackDATABASE->features_containing_older(state->_timestamp);

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  //    Also remove any that we are unable to triangulate (due to not having enough measurements)
  auto it0 = active_features.begin();
  while (it0 != active_features.end()) {

    // Skip if it is a SLAM feature since it already is already going to be added
    if (state->_features_SLAM.find((*it0)->featid) != state->_features_SLAM.end()) {
      it0 = active_features.erase(it0);
      continue;
    }

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough and am not a SLAM feature which doesn't need triangulation
    if (ct_meas < (int)std::max(4.0, std::floor(state->_options.max_clone_size * 2.0 / 5.0))) {
      it0 = active_features.erase(it0);
    } else {
      it0++;
    }
  }
  retri_rT2 = boost::posix_time::microsec_clock::local_time();

  // Return if no features
  if (active_features.empty() && state->_features_SLAM.empty())
    return;

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // Get current camera pose
      Eigen::Matrix3d R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Vector3d p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // Append to our map
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }
  retri_rT3 = boost::posix_time::microsec_clock::local_time();

  // 3. Try to triangulate all features that have measurements
  auto it1 = active_features.begin();
  while (it1 != active_features.end()) {

    // Triangulate the feature and remove if it fails
    bool success_tri = true;
    if (active_tracks_initializer->config().triangulate_1d) {
      success_tri = active_tracks_initializer->single_triangulation_1d(it1->get(), clones_cam);
    } else {
      success_tri = active_tracks_initializer->single_triangulation(it1->get(), clones_cam);
    }

    // Remove the feature if not a success
    if (!success_tri) {
      it1 = active_features.erase(it1);
      continue;
    }
    it1++;
  }
  retri_rT4 = boost::posix_time::microsec_clock::local_time();

  // Return if no features
  if (active_features.empty() && state->_features_SLAM.empty())
    return;

  // Points which we have in the global frame
  for (const auto &feat : active_features) {
    active_tracks_posinG[feat->featid] = feat->p_FinG;
  }
  for (const auto &feat : state->_features_SLAM) {
    Eigen::Vector3d p_FinG = feat.second->get_xyz(false);
    if (LandmarkRepresentation::is_relative_representation(feat.second->_feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(feat.second->_anchor_cam_id != -1);
      // Get calibration for our anchor camera
      Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(feat.second->_anchor_cam_id)->Rot();
      Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(feat.second->_anchor_cam_id)->pos();
      // Anchor pose orientation and position
      Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(feat.second->_anchor_clone_timestamp)->Rot();
      Eigen::Vector3d p_IinG = state->_clones_IMU.at(feat.second->_anchor_clone_timestamp)->pos();
      // Feature in the global frame
      p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (feat.second->get_xyz(false) - p_IinC) + p_IinG;
    }
    active_tracks_posinG[feat.second->_featid] = p_FinG;
  }

  // Calibration of the first camera (cam0)
  std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(0);
  std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(0);
  Eigen::Matrix<double, 3, 3> R_ItoC = calibration->Rot();
  Eigen::Matrix<double, 3, 1> p_IinC = calibration->pos();

  // Get current IMU clone state
  std::shared_ptr<PoseJPL> clone_Ii = state->_clones_IMU.at(active_tracks_time);
  Eigen::Matrix3d R_GtoIi = clone_Ii->Rot();
  Eigen::Vector3d p_IiinG = clone_Ii->pos();

  // 4. Next we can update our variable with the global position
  //    We also will project the features into the current frame
  for (const auto &feat : active_tracks_posinG) {

    // Project the current feature into the current frame of reference
    Eigen::Vector3d p_FinIi = R_GtoIi * (feat.second - p_IiinG);
    Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
    double depth = p_FinCi(2);
    Eigen::Vector2d uv_norm, uv_dist;
    uv_norm << p_FinCi(0) / depth, p_FinCi(1) / depth;
    uv_dist = state->_cam_intrinsics_cameras.at(0)->distort_d(uv_norm);

    // Skip if not valid (i.e. negative depth, or outside of image)
    if (depth < 0.1) {
      continue;
    }

    // Skip if not valid (i.e. negative depth, or outside of image)
    int width = state->_cam_intrinsics_cameras.at(0)->w();
    int height = state->_cam_intrinsics_cameras.at(0)->h();
    if (uv_dist(0) < 0 || (int)uv_dist(0) >= width || uv_dist(1) < 0 || (int)uv_dist(1) >= height) {
      // PRINT_DEBUG("feat %zu -> depth = %.2f | u_d = %.2f | v_d = %.2f\n",(*it2)->featid,depth,uv_dist(0),uv_dist(1));
      continue;
    }

    // Finally construct the uv and depth
    Eigen::Vector3d uvd;
    uvd << uv_dist, depth;
    active_tracks_uvd.insert({feat.first, uvd});
  }
  retri_rT5 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  // PRINT_DEBUG(CYAN "[RETRI-TIME]: %.4f seconds for cleaning\n" RESET, (retri_rT2-retri_rT1).total_microseconds() * 1e-6);
  // PRINT_DEBUG(CYAN "[RETRI-TIME]: %.4f seconds for triangulate setup\n" RESET, (retri_rT3-retri_rT2).total_microseconds() * 1e-6);
  // PRINT_DEBUG(CYAN "[RETRI-TIME]: %.4f seconds for triangulation\n" RESET, (retri_rT4-retri_rT3).total_microseconds() * 1e-6);
  // PRINT_DEBUG(CYAN "[RETRI-TIME]: %.4f seconds for re-projection\n" RESET, (retri_rT5-retri_rT4).total_microseconds() * 1e-6);
  // PRINT_DEBUG(CYAN "[RETRI-TIME]: %.4f seconds total\n" RESET, (retri_rT5-retri_rT1).total_microseconds() * 1e-6);
}
