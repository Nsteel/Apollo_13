#include <Eigen/Dense>
#include <Eigen/SVD>
#include "unicycle_motions.h"

double NUM_ANGLES = 16;
int NUM_SAMPLES = 10;

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

// @from http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00173.html
static bool pinv(const Eigen::Matrix2d& a, Eigen::Matrix2d& a_pinv)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    if (a.rows() < a.cols())
        return false;

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix2d> svdA(a, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::Vector2d vPseudoInvertedSingular(svdA.matrixV().cols(),1);

    for (int iRow = 0; iRow < vSingular.rows(); iRow++)
    {
        if (fabs(vSingular(iRow)) <= 1e-10) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow, 0) = 0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow, 0) = 1. / vSingular(iRow);
        }
    }

    // A little optimization here
    Eigen::Matrix2d mAdjointU = svdA.matrixU().adjoint().block(0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());

    // Pseudo-Inversion : V * S * U'
    a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;

    return true;
}

double NormalizeAngle(double angle_rad, double angle_min_rad, double angle_max_rad)
{
    if (fabs(angle_rad) > 2.0 * M_PI) { // normalize to [-2*pi, 2*pi] range
        angle_rad -= ((int)(angle_rad / (2.0 * M_PI))) * 2.0 * M_PI;
    }

    while (angle_rad > angle_max_rad) {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < angle_min_rad) {
        angle_rad += 2 * M_PI;
    }

    return angle_rad;
}

double ShortestAngleDist(double a1_rad, double a2_rad)
{
    double a1_norm = NormalizeAngle(a1_rad, 0.0, 2.0 * M_PI);
    double a2_norm = NormalizeAngle(a2_rad, 0.0, 2.0 * M_PI);
    return std::min(fabs(a1_norm - a2_norm), 2.0 * M_PI - fabs(a2_norm - a1_norm));
}

double ShortestAngleDiff(double a1_rad, double a2_rad)
{
    double a1_norm = NormalizeAngle(a1_rad, 0.0, 2.0 * M_PI);
    double a2_norm = NormalizeAngle(a2_rad, 0.0, 2.0 * M_PI);

    double dist = ShortestAngleDist(a1_rad, a2_rad);
    if (ShortestAngleDist(a1_norm + dist, a2_norm) < ShortestAngleDist(a1_norm - dist, a2_norm)) {
        return -dist;
    }
    else {
        return dist;
    }
}

static inline double interp(double from, double to, double alpha)
{
    return (1.0 - alpha) * from + alpha * to;
}

static inline double interp_angle(double from, double to, double alpha)
{
    return from + alpha * ShortestAngleDiff(to, from);
}

std::vector<Pose2_cont> create_interpolated_motion(const Pose2_cont& start, const Pose2_cont& goal)
{
    auto sqrd = [](double d) { return d * d; };
    std::vector<Pose2_cont> motion;

    const double res = 0.01;
    double dist = sqrt(sqrd(goal.x - start.x) +  sqrd(goal.y - start.y));
    const int num_samples = (int)std::ceil(dist / res);

    motion.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        double alpha = (double)i / (num_samples - 1);

        double interm_x = interp(start.x, goal.x, alpha);
        double interm_y = interp(start.y, goal.y, alpha);
        double interm_yaw = interp_angle(start.yaw, goal.yaw, alpha);

        motion.emplace_back(interm_x, interm_y, interm_yaw);
    }

    return motion;
}

double shortest_angle_diff(double af, double ai)
{
    auto cmodf = [](double a, double n) { return fmod(fmod(a, n) + n, n); };
    double a = af - ai;
    a = cmodf(a + M_PI, 2.0 * M_PI) - M_PI;
    return a;
}

std::vector<Pose2_cont>
generate_unicycle_motion(const Pose2_cont& start, const Pose2_cont& goal)
{
    DEBUG_PRINT("--------------------------------------------------------------------------------\n");
    DEBUG_PRINT("generating unicycle motion between %s and %s\n", to_string(start).c_str(), to_string(goal).c_str());
    DEBUG_PRINT("--------------------------------------------------------------------------------\n");

    auto almost_equals = [](double lhs, double rhs, double eps) { return fabs(lhs - rhs) < eps; };
    const double eps = 1e-6;

    // check for straight-line motion or turn-in-place
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double dtheta = shortest_angle_diff(goal.yaw, start.yaw);
    if ((dx == 0.0 && dy == 0.0) || (dtheta == 0.0)) {
        if (almost_equals(atan2(dy, dx), start.yaw, eps) && almost_equals(atan2(dy, dx), goal.yaw, eps)) {
            DEBUG_PRINT("Interpolated Motion\n");
            return create_interpolated_motion(start, goal);
        }
        else {
            DEBUG_PRINT("No unicycle motion for turns-in-place or skidding\n");
            return { };
        }
    }

    DEBUG_PRINT("Arc Motion\n");
    Eigen::Matrix2d R;
    R(0, 0) = cos(start.yaw);
    R(0, 1) = sin(goal.yaw) - sin(start.yaw);
    R(1, 0) = sin(start.yaw);
    R(1, 1) = -(cos(goal.yaw) - cos(start.yaw));

    Eigen::Matrix2d Rpinv;
    if (!pinv(R, Rpinv)) {
        DEBUG_PRINT("Failed to compute Moore-penrose pseudo-inverse");
        return { };
    }

    Eigen::Vector2d S = Rpinv * Eigen::Vector2d(goal.x - start.x, goal.y - start.y);

    double straight_length = S(0);
    double radius = S(1);

    if (fabs(radius) < 1e-6)  {
        DEBUG_PRINT("Unable to turn with radius %0.3f\n", radius);
        return { };
    }

    double w = shortest_angle_diff(goal.yaw, start.yaw) + (straight_length / radius);
    double v = radius * w;
    double tl = straight_length / v;

    if (straight_length < 0) {
        DEBUG_PRINT("Not allowed to go backwards (length = %0.3f)\n", straight_length);
        return { };
    }

    if (v < 0) {
        DEBUG_PRINT("Not allowed to go backwards (velocity = %0.3f)\n", v);
        return { };
    }

    if (tl < 0.0 || tl > 1.0) {
        DEBUG_PRINT("Another dimension! Another dimension! (tl = %0.3f)\n", tl);
        return { };
    }

    DEBUG_PRINT("R = [ %0.3f, %0.3f; %0.3f, %0.3f]\n", R(0, 0), R(0, 1), R(1, 0), R(1, 1));
    DEBUG_PRINT("S = [%0.3f, %0.3f]\n", S(0), S(1));
    DEBUG_PRINT("straight_length = %0.3f\n", straight_length);
    DEBUG_PRINT("radius = %0.3f\n", radius);
    DEBUG_PRINT("w = %0.3f\n", w);
    DEBUG_PRINT("v = %0.3f\n", v);
    DEBUG_PRINT("tl = %0.3f\n", tl);

    const double res = 0.1;
    double arc_length = w * (1.0 - tl);
    double total_length = fabs(straight_length) + fabs(arc_length);

    const int num_samples = (int)std::ceil(total_length / res);
    std::vector<Pose2_cont> interm_poses;
    interm_poses.resize(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        double dt = (double)i / (double)(num_samples - 1);
        if (dt < tl) {
            double x = start.x + v * dt * cos(start.yaw);
            double y = start.y + v * dt * sin(start.yaw);
            double theta = start.yaw;
            interm_poses[i] = { x, y, theta };
        }
        else {
            double x = start.x + straight_length * cos(start.yaw) + radius * sin(w * (dt - tl) + start.yaw) - radius * sin(start.yaw);
            double y = start.y + straight_length * sin(start.yaw) - radius * cos(w * (dt - tl) + start.yaw) + radius * cos(start.yaw);
            double theta = start.yaw + w * (dt - tl);
            interm_poses[i] = { x, y, theta };
        }
    }

    return interm_poses;
}
