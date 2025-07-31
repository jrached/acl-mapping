std_msgs/Header header
builtin_interfaces/Time imu_stamp
geometry_msgs/Vector3 gyro
geometry_msgs/Vector3 accel
float64 loop_time # period at which imu cb runs, in seconds