Header header

string name             # Name of this diagnostic
float64 gt_distance     # Ground truth total distance travelled since initialization
float64 abs_dist_err    # Absolute distance between the estimated pose and ground truth pose, in meters
float64 abs_rot_err     # Absolute orientation difference between estimated pose and GT pose, in radians

float64 relative_dist_err  # Distance error divided by total distance traveled (No units)

float64 abs_linear_vel_err  # Absolute difference between estimated linear velocity and ground truth, in m/s
float64 abs_rot_vel_err     # Absolute difference between estimated rotational velocity and ground truth, in rad/s
float64 rel_linear_vel_err  # Relative linear velocity error, compared to ground truth
float64 rel_rot_vel_err     # Relative rotational velocity error, compared to ground truth

geometry_msgs/Pose err  # Difference between the estimated and ground-truth pose