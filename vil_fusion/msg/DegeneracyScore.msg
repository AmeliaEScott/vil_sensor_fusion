Header header

string name         # Name of this degeneracy detection function
float64 score       # The score calculated for this measurement
float64 threshold   # The threshold at which this measurement is considered degenerate.
                    # Note: This can be either an upper threshold or lower threshold. It is only here for diagnostics,
                    # so whoever reads this message should know ahead of time which it is.
bool degenerate     # True if the associated measurement is degenerate, false if it's OK.

