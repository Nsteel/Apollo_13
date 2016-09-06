#include "RangeSensor.h"

namespace rs {

double correctYawAngle(const double theta, const double increment) {
        double yaw = 0;
        double angle = theta + increment;
        if (angle > 180.0) {
                yaw = -180.0 - (180.0 - angle);
        } else if (angle < -180.0) {
                yaw = 180.0 - (-180.0 - angle);
        } else {
                yaw = angle;
        }
        return yaw;
}

const int sgn(const int x) {
        return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

double degToRad(double angle){
        return angle/180.0*CV_PI;
}

double radToDeg(double angle){
        return angle/CV_PI*180.0;
}

}

RangeSensor::RangeSensor(const SensorType& type) : type(type){
};

void RangeSensor::setConfig(simulation::RangeSensorConfig& config){
  this->config = config;
}

void RangeSensor::setMap(const cv::Mat& map){
        this->map = map;
}

void RangeSensor::setMapMetaData(const nav_msgs::MapMetaData& mapInfo){
        this->mapInfo = mapInfo;
}

const rs::rangeArray_ptr RangeSensor::getLaserScan(const cv::Point sensorPos, const double theta) const {
        //yaw needs to be shifted by -90deg because robot people put their yaw=0 on the positive x-axis
        //double yaw = rs::correctYawAngle(theta, -90);
        //yaw = theta;
        //get sensor configuration
        double FOV, resolution, range;
        int obstacleColor;
        if(type == laser) {
                FOV = config.laser_field_of_view;
                range = config.laser_max_sensor_distance+0.5;
                resolution = config.laser_angular_resolution;
                obstacleColor = config.laser_obstacle_color;
        }else{
                FOV = config.us_field_of_view;
                range = config.us_max_sensor_distance+0.5;
                resolution = config.us_angular_resolution;
                obstacleColor = config.us_obstacle_color;
        }

        //init loop for iteratiting over n-angles
        int nAngles = FOV / resolution;
        std::vector<float> rangeArray = std::vector<float>(nAngles);
        double angle = rs::correctYawAngle(theta, -FOV / 2);
        double maxDistance = range/mapInfo.resolution;

        for (int a = 0; a < nAngles; a++) {
                cv::Point P2;

                P2.x = (int) round(
                        sensorPos.x
                        + maxDistance * cv::cos((-angle) * CV_PI / 180.0)); // - vor angle entfernt
                P2.y = (int) round(
                        sensorPos.y
                        + maxDistance * cv::sin((-angle) * CV_PI / 180.0)); // - vor angle entfernt

                cv::LineIterator it(map, sensorPos, P2, 8);
                cv::Point currentPos = it.pos();
                for (int i = 0; i < it.count; i++, ++it) {
                        currentPos = it.pos();
                        if (map.at<uchar>(currentPos) == obstacleColor) {
                                break;
                        }
                }
                double scannedRange = cv::norm(sensorPos-currentPos)*mapInfo.resolution;
                rangeArray[a]=scannedRange;
                angle = rs::correctYawAngle(angle, resolution);
        }
        return std::make_shared<std::vector<float> >(rangeArray);
}
const double RangeSensor::getUSScan(const cv::Point sensorPos, const double theta) const{
  std::vector<float> rangeArray = *getLaserScan(sensorPos, theta);
  double minOfRange = rangeArray[0];
  for(auto current : rangeArray) {
                  if(current < minOfRange) {
                                  minOfRange = current;
                  }
  }
  return minOfRange;
}
