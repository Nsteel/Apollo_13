#ifndef RANGESENSOR_H_
#define RANGESENSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <memory>
#include <simulation/RangeSensorConfig.h>
#include <nav_msgs/MapMetaData.h>
#include <iostream>

namespace rs {
typedef std::shared_ptr<std::vector<float> > rangeArray_ptr;
enum SensorType {laser, us};

double correctYawAngle(const double theta, const double increment);
const int sgn(const int x);
double degToRad(double angle);
double radToDeg(double angle);

class RangeSensor;
}

using rs::RangeSensor;
class RangeSensor {
public:
                RangeSensor(const SensorType& type);
                void setConfig(simulation::RangeSensorConfig& config);
                void setMap(const cv::Mat& map);
                void setMapMetaData(const nav_msgs::MapMetaData& mapInfo);
                const rs::rangeArray_ptr getLaserScan(const cv::Point sensorPos, const double theta) const;
                const double getUSScan(const cv::Point sensorPos, const double theta) const;
private:
                SensorType type;
                simulation::RangeSensorConfig config;
								cv::Mat map;
                nav_msgs::MapMetaData mapInfo;
};

#endif /* RANGESENSOR_H_ */
