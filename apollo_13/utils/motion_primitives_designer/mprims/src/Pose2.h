#ifndef Pose2_h
#define Pose2_h

#include <cmath>
#include <sstream>
#include <string>

template <typename StorageType>
struct Pose2
{
    Pose2(StorageType x, StorageType y, StorageType yaw) : x(x), y(y), yaw(yaw) { }
    Pose2() : x(), y(), yaw() { }
    StorageType x, y, yaw;
};

template <typename StorageType>
std::string to_string(const Pose2<StorageType>& p)
{
    std::stringstream ss;
    ss << "(" << p.x << ", " << p.y << ", " << 180.0 * p.yaw / M_PI << ")";
    return ss.str();
}

typedef Pose2<double> Pose2_cont;
typedef Pose2<int> Pose2_disc;

#endif
