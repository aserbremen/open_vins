#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
# source /home/serov/code/cpp/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# estimator configurations
modes=(
    "mono"
    # "stereo"
)

# dataset locations
bagpaths=(
    "/data/datasets/conti/2021-04-20_Parking_and_atCity/2021-04-20-10-08-59-693_F-TZ_9900_CamLoc_Roedelheim/roedelheim1.bag"
    "/data/datasets/conti/2021-04-20_Parking_and_atCity/2021-04-20-10-24-03-775_F-TZ_9900_CamLoc_Roedelheim/roedelheim2.bag"
    "/data/datasets/conti/2021-04-20_Parking_and_atCity/2021-04-20-15-00-01-016_F-TZ_9900_CamLoc_Eschborn/eschborn1.bag"
    "/data/datasets/conti/2021-04-20_Parking_and_atCity/2021-04-20-15-10-59-607_F-TZ_9900_CamLoc_Eschborn/eschborn2.bag"
    "/data/datasets/conti/2021-07-07_camera_localization/city1/city1.bag"
    "/data/datasets/conti/2021-07-07_camera_localization/city2/city2.bag"
)

# short bag names
bagnames=(
    "roedelheim1" # 0
    "roedelheim2" # 1
    "eschborn1" # 2
    "eschborn2" # 3
    "city1" # 4
    "city2" # 5
)
# determine the range of algorithms we want to evaluate (inclusive)
DATASET_START=3
DATASET_END=3

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
     "0" # roedelheim1
     "0" # roedelheim2 for xsens 39.718, will otherwise crash
     "0" # eschborn1
     "0" # eschborn2
     "0" # city1
     "0" # city2
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bag_durations=(
     "-1" # roedelheim1
     "-1" # roedelheim2 for xsens 506, will otherwise crash
     "-1" # eschborn1
     "-1" # eschborn2
     "-1" # city1
     "-1" # city2
)

init_window_time=(
    "0.75" # roedelheim1
    "0.75" # roedelheim2
    "0.75" # eschborn1
    "0.75" # eschborn2
    "0.75" # city1
    "0.75" # city2
)

# threshold for variance to detect if the unit has moved yet
imuthreshold=(
    "0.12" # roedelheim1
    "0.12" # roedelheim2
    "0.12" # eschborn1
    "0.12" # eschborn2
    "0.12" # city1
    "0.12" # city2
)

# location to save log files into, also saves 
save_path="/home/serov/code/cpp/catkin_ws_ov/src/open_vins/results/atcity"
rpg_base_path="/home/serov/code/cpp/multi_sensor_odometry/rpg_trajectory_evaluation/results"
platform="refactor" # change to not overwrite existing results
# other params
align_type="posyaw" # posyaw, none, se3
align_num_frames="-1"
dovisualize="true"
do_time="true"
dosave="true"
use_yaw_odom_second_order="true"
use_yaw_jacobian_second_order="false"
calib_cam_timeoffset="false"

# use_sc13_imu="false"
imus=(
    # xsens
    sc13
)
vehicle_update_modes=(
    "VEHICLE_UPDATE_NONE"
    "VEHICLE_UPDATE_SPEED_PROPAGATE"
    # "VEHICLE_UPDATE_STEERING_PROPAGATE"
    "VEHICLE_UPDATE_VEHICLE_PROPAGATE"
    "VEHICLE_UPDATE_PREINTEGRATED_SINGLE_TRACK"
    "VEHICLE_UPDATE_PREINTEGRATED_DIFFERENTIAL"
)
algo_names=(
    "ov" # "VEHICLE_UPDATE_NONE"
    "ov_speed_propagate" # "VEHICLE_UPDATE_SPEED_PROPAGATE"
    # "ov_steering_propagate" # "VEHICLE_UPDATE_STEERING_PROPAGATE"
    "ov_vehicle_propagate" # "VEHICLE_UPDATE_VEHICLE_PROPAGATE"
    "ov_preintegrated_single_track" # "VEHICLE_UPDATE_PREINTEGRATED_SINGLE_TRACK"
    "ov_preintegrated_differential" # "VEHICLE_UPDATE_PREINTEGRATED_DIFFERENTIAL"
)

algo_suffix=""
speed_update_mode="SPEED_UPDATE_VECTOR" # options: SPEED_UPDATE_VECTOR, SPEED_UPDATE_X

#=============================================================
#=============================================================
#=============================================================

##### Set to "false" if you want to run the script, otherwise only the commands are echoed
dry_run="false"

# Loop through all modes
for h in "${!modes[@]}"; do
    # Loop through all datasets
    for i in $(seq $DATASET_START $DATASET_END); do
        # Loop through the algos
        for j in "${!vehicle_update_modes[@]}"; do
            # Monte Carlo runs for this dataset
            # If you want more runs, change the below loop
            for imu_type in "${imus[@]}"; do
                # number of cameras
                if [ "${modes[h]}" == "mono" ]; then 
                    max_cams="1"
                    stereo="false"
                fi
                if [ "${modes[h]}" == "stereo" ]; then
                    max_cams="2"
                    stereo="true"
                fi


                # Start timing
                start_time="$(date -u +%s)"
                algo_name="${algo_names[j]}${algo_suffix}"
                folder_open_vins_est="$save_path/${platform}/${algo_name}/${bagnames[i]}_${imu_type}"
                filename_est="${folder_open_vins_est}/${start_time}_estimate.txt"
                folder_rpg_est="${rpg_base_path}/${platform}/${algo_name}/${platform}_${algo_name}_${bagnames[i]}_${imu_type}"
                filename_rpg_est="${folder_rpg_est}/stamped_traj_estimate.txt"

                # Run our ROS launch file (note we send console output to terminator)
                cmd="roslaunch ov_msckf serial_vehicle_updates.launch "
                cmd="${cmd} max_cameras:=$max_cams use_stereo:="$stereo" bag:="${bagpaths[i]}""
                # if xsens and roedelheim 2 use special bag start time and duration due to corrupted measurement
                if [[ ( "${imu_type}" == "xsens" ) && ( "${bagnames[i]}" == "roedelheim2" ) ]]; then
                    cmd="${cmd} bag_start:=39.718 bag_durr:=506"
                else
                    cmd="${cmd} bag_start:=${bagstarttimes[i]}  bag_durr:=${bag_durations[i]}"
                fi
                cmd="${cmd} init_imu_thresh:="${imuthreshold[i]}" init_window_time:="${init_window_time[i]}""
                cmd="${cmd} dosave:="${dosave}" path_est:="${filename_est}" path_rpg_est:="${filename_rpg_est}""
                cmd="${cmd} vehicle_update_mode:="${vehicle_update_modes[j]}" speed_update_mode:="${speed_update_mode}""
                cmd="${cmd} calib_cam_timeoffset:="${calib_cam_timeoffset}""
                cmd="${cmd} dovisualize:="${dovisualize}"  max_steering_angle:=100.0"

                if [[ "${imu_type}" == "xsens" || "${imu_type}" == "" ]]; then 
                    cmd="$cmd topic_imu:=/imu0"
                elif [ "${imu_type}" == "sc13" ]; then 
                    cmd="$cmd topic_imu:=/imu1"
                fi
                
                if [[ "${algo_names[j]}" ==  "ov_preintegrated_single_track" || "${algo_names[j]}" == "ov_preintegrated_differential" ]]; then
                    cmd="${cmd} use_yaw_odom_second_order:="${use_yaw_odom_second_order}" use_yaw_jacobian_second_order:="${use_yaw_jacobian_second_order}""
                fi

                if [ "${do_time}" == "true" ]; then
                    cmd="${cmd} path_time:=${folder_open_vins_est}/timing.time"
                fi
                
                echo ${cmd} 
                if [ "${dry_run}" == "false" ]; then
                    ${cmd}
                    # Copy groundtruth to rpg folder
                    mkdir -p ${folder_rpg_est}
                    cp "${save_path}/truth/${bagnames[i]}.txt" "${folder_rpg_est}/stamped_groundtruth.txt"
                    # Copy timing file
                    if [ "${do_time}" == "true" ]; then
                        cp "${folder_open_vins_est}/timing.time" "${folder_rpg_est}/timing.time"
                    fi
                    # create eval_cfg.yaml file for rpg trajectory evaluation
                    touch "${folder_rpg_est}/eval_cfg.yaml"
                    echo -e "align_type: ${align_type}\nalign_num_frames: ${align_num_frames}" > "${folder_rpg_est}/eval_cfg.yaml"
                    # save cmd to both result folders for checking exact params in future
                    touch "${folder_rpg_est}/ov_cmd.cmd"
                    touch "${folder_open_vins_est}/ov_cmd.cmd" # dont use .txt as file ending otherwise eval scripts will think its a result file
                    echo -e "${cmd}" > "${folder_rpg_est}/ov_cmd.cmd"
                    echo -e "${cmd}" > "${folder_open_vins_est}/ov_cmd.cmd"
                    
                    # print out the time elapsed
                    end_time="$(date -u +%s)"
                    elapsed="$(($end_time-$start_time))"
                    echo "BASH: ${modes[h]} - ${bagpaths[i]} - run $k took $elapsed seconds"
                fi
            done
        done
    done
done


