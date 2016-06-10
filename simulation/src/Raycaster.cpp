/*
 * Raycaster.cpp
 *
 *  Created on: May 20, 2016
 *      Author: sebastian
 */

#include "Raycaster.h"

namespace rc {

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
								return angle/180.0*rc::PI;
}

double radToDeg(double angle){
								return angle/rc::PI*180.0;
}

}

Raycaster::Raycaster() {
								map = cv::Mat();
								meterPerPixel = 1.0;
								maxDistance = 0.0;
								FOV = 0.0;
								angularResolution = 0.0;
								init();

}
Raycaster::Raycaster(const Raycaster& other) :
								map(other.map), meterPerPixel(other.meterPerPixel), maxDistance(other.maxDistance), FOV(other.FOV), angularResolution(
																other.angularResolution), obstacleColor(other.obstacleColor), origin(
																other.origin), rangeArray(other.rangeArray), color(other.color) {
}

Raycaster::Raycaster(const cv::Mat map, const double meterPerPixel, const double maxDistance,
																					const double FOV, const double angularResolution) :
								map(map), meterPerPixel(meterPerPixel), maxDistance(maxDistance), FOV(FOV), angularResolution(
																angularResolution) {
								init();

}

void Raycaster::init() {
								//set true black as obstacle color
								obstacleColor = cv::Vec3b(0, 0, 0);
								origin = cv::Vec2i(0, 0);
								rangeArray = std::vector<float>();
								color = cv::Vec3b(254, 0, 0);
}

const rc::rangeArray_ptr Raycaster::getRangeInfo(const rc::vec2i_ptr pointOfOrigin, const double theta) {
								origin = *pointOfOrigin;
								double eps = 0.001;
								//yaw needs to be shifted by -90deg because robot people put their yaw=0 on the positive x-axis
								double viewAngle = rc::correctYawAngle(theta, -90);
								std::pair<double, double> minMaxFov = *angleMinMax(viewAngle);
								rangeArray = std::vector<float>((int)FOV/angularResolution);
								int counter = 0;
								for (double angle = minMaxFov.first; !(angle <= minMaxFov.second+eps && angle >= minMaxFov.second-eps); angle =
																					rc::correctYawAngle(angle, angularResolution)) {
																rangeArray[counter] = norm(*bresenham(*calcEndpoint(angle)), origin)*meterPerPixel;
																counter++;
								}
								return std::make_shared<std::vector<float> >(rangeArray);

}

const double Raycaster::getUsRangeInfo(const rc::vec2i_ptr pointOfOrigin, const double theta){
								getRangeInfo(pointOfOrigin, theta);
								double minOfRange = rangeArray[0];
								for(auto current : rangeArray) {
																if(current < minOfRange) {
																								minOfRange = current;
																}
								}
								return minOfRange;
}

const rc::minMaxFov_ptr Raycaster::angleMinMax(const double theta) const {
								double minAngle = rc::correctYawAngle(theta, -FOV / 2);
								double maxAngle = rc::correctYawAngle(theta, FOV / 2);
								std::pair<double,double> out(minAngle, maxAngle);
								return std::make_shared<std::pair<double,double> >(std::make_pair(minAngle, maxAngle));
}

const rc::vec2i_ptr Raycaster::calcEndpoint(const double heading) const {
								double xMax;
								double yMax;
								double m;

								if (heading == 0) { //forward
																xMax = 0;
																yMax = maxDistance;
																m = 0;
								} else if (heading == 180.0) { //backward
																xMax = 0;
																yMax = -maxDistance;
																m = 0;
								} else if (heading == 90.0) { //left
																xMax = -maxDistance;
																yMax = 0;
																m = 0;
								} else if (heading == -90.0) { //right
																xMax = maxDistance;
																yMax = 0;
																m = 0;
								} else if (heading < 0 && heading > -90.0) { // first quadrant
																float modheading = rc::degToRad(heading + 90.0);
																m = tan(modheading);
																xMax = cos(modheading) * maxDistance;
																yMax = m * xMax;
								} else if (heading < -90.0 && heading != 180.0) { // fourth quadrant
																float modheading = rc::degToRad(heading + 90.0);
																m = tan(modheading);
																xMax = cos(modheading) * maxDistance;
																yMax = m * xMax;
								} else if (heading > 0 && heading < 90.0) { // second quadrant
																float modheading = rc::degToRad(heading - 90.0);
																m = tan(modheading);
																xMax = -cos(modheading) * maxDistance;
																yMax = m * xMax;
								} else { // third quadrant -> heading > 90
																float modheading = rc::degToRad(heading - 90.0);
																m = tan(modheading);
																xMax = -cos(modheading) * maxDistance;
																yMax = m * xMax;
								}

								cv::Vec2i endpoint(origin[0] + (int) xMax, origin[1] - (int) yMax);
								return std::make_shared<cv::Vec2i>(endpoint);
}

const rc::vec2i_ptr Raycaster::bresenham(const cv::Vec2i& end) {
								int x, y, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, es, el, err;
								cv::Vec2i trueEnd = end;

								dx = end[0] - origin[0];
								dy = end[1] - origin[1];

								incx = rc::sgn(dx);
								incy = rc::sgn(dy);
								if (dx < 0)
																dx = -dx;
								if (dy < 0)
																dy = -dy;

								if (dx > dy) {
																pdx = incx;
																pdy = 0;
																ddx = incx;
																ddy = incy;
																es = dy;
																el = dx;
								} else {
																pdx = 0;
																pdy = incy;
																ddx = incx;
																ddy = incy;
																es = dx;
																el = dy;
								}

								x = origin[0];
								y = origin[1];
								err = el / 2;

								for (t = 0; t < el; ++t) {
																err -= es;
																if (err < 0) {
																								err += el;
																								x += ddx;
																								y += ddy;
																} else {
																								x += pdx;
																								y += pdy;
																}
																if (y >= 0 && x >= 0 && y < map.rows && x < map.cols) {
																								if (map.at<cv::Vec3b>(y, x) != obstacleColor) {
																																trueEnd[0]=x;
																																trueEnd[1]=y;
																								} else {
																																return std::make_shared<cv::Vec2i>(trueEnd);
																								}
																}else{
																								return std::make_shared<cv::Vec2i>(trueEnd);
																}

								}
								return std::make_shared<cv::Vec2i>(trueEnd);

}
