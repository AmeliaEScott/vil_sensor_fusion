Header header

string name         # Name of this degeneracy detection function

float64 score_all       # The score calculated for this measurement, including all 6 DoF
float64 derivative_all  # Difference between current score and previous score, for all 6 D0F

float64 score_trans     # Score calculated for this measurement, including only the 3 translational DoF
float64 derivative_trans  # Difference between current and previous score_trans

float64 score_rot       # Score calculated for this measurement, including only the 3 rotational DoF
float64 derivative_rot  # Difference between current score_rot and previous score_rot
