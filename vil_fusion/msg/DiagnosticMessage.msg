Header header

string name             # Name of this diagnostic
float64 gt_distance     # Ground truth total distance travelled since initialization
float64 abs_dist_err    # Absolute distance between the estimated pose and ground truth pose, in meters
float64 abs_rot_err     # Absolute orientation difference between estimated pose and GT pose, in radians

float64 relative_dist_err  # Distance error divided by total distance traveled

geometry_msgs/Pose err  # Difference between the estimated and ground-truth pose