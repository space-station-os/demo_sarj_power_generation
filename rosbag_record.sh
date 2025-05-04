
output_base_dir_path="./rosbag2_out"
mkdir -p output_base_dir_path
output_dir_path="${output_base_dir_path}/rosbag2_$(date +%Y-%m-%d_%H-%M-%S)/"

ros2 bag record -o ${output_dir_path} \
/sarj_angle \
/simu_time \
/soc \
/ss_attitude \
/ss_in_sunlight \
/sun_direction_ssbf \
/ss_position_eci \
/ss_velocity_eci \
/ss_acceleration_eci