std_msgs/Header header
geometry_msgs/Quaternion q      # desired attitude
geometry_msgs/Vector3 w         # desired angular rates
geometry_msgs/Vector3 f_w       # desired force (expr in world frame)
bool power # true if motors should be able to spin
