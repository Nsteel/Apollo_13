#ifndef unicycle_motions_h
#define unicycle_motions_h

#include <vector>
#include "Pose2.h"

/// Return a vector of intermediate poses on the unicycle-based motion from $start to $goal
std::vector<Pose2_cont> generate_unicycle_motion(const Pose2_cont& start, const Pose2_cont& goal);

#endif
