/*
 * FloatingEdge.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#include "Contour.h"


Contour::Contour(const edge& vectorOfPoints, const cv::Mat& map) : vectorOfPoints(vectorOfPoints), map(map) {
								calcLength();

}
Contour::Contour(const Contour& other) : vectorOfPoints(other.vectorOfPoints), map(other.map), centroid(other.centroid), midEdge(other.midEdge), yaw(other.yaw), length(other.length){
}

void Contour::initContour(){
								calcCentroid();
								calcMidEdge();
								calcDirectionToUnknown();
}
const edge& Contour::getContour(){
								return vectorOfPoints;
}

const cv::Point& Contour::getCentroid() const {
								return centroid;
}

const double Contour::getYaw() const {
								return yaw;
}

const cv::Point& Contour::getMid() const {
								return midEdge;
}

const double Contour::getLength() const {
								return length;
}

void Contour::calcCentroid(){
								cv::Point result;
								double x_mean = 0;
								double y_mean = 0;
								for(auto current : vectorOfPoints) {
																x_mean += (double)current.x;
																y_mean += (double)current.y;
								}
								x_mean /= vectorOfPoints.size();
								y_mean /= vectorOfPoints.size();
								result.x= (int)x_mean;
								result.y= (int)y_mean;
								centroid = result;
}

void Contour::calcMidEdge(){
								edge hull;
								cv::convexHull( vectorOfPoints, hull);
								midEdge = hull[hull.size()/2];

}

void Contour::calcLength(){
								length = cv::arcLength(vectorOfPoints, false);
}

void Contour::calcDirectionToUnknown(){
								//select region of interest
								cv::Point2f center;
								float radius;
								cv::minEnclosingCircle(vectorOfPoints, center, radius);
								edge circlePoints;
								int max_x = (center.x+radius<map.cols) ? center.x+radius : map.cols-1;
								int min_x = (center.x-radius>=0) ? center.x-radius : 0;
								int max_y = (center.y+radius<map.rows) ? center.y+radius : map.rows-1;
								int min_y = (center.y-radius>=0) ? center.y-radius : 0;

								circlePoints.push_back(cv::Point(min_x, center.y));
								circlePoints.push_back(cv::Point(center.x, max_y));
								circlePoints.push_back(cv::Point(center.x, min_y));
								circlePoints.push_back(cv::Point(max_x, center.y));

								cv::Rect roi = cv::boundingRect(circlePoints);

								//crop region of interest
								cv::Mat contourMap = map(roi).clone();
								//since map is in gray scale, this step is not needed
								//cv::cvtColor(contourMap, contourMap, CV_RGB2GRAY);
								//reduce noise
								cv::blur(contourMap, contourMap, cv::Size(5,5));

								//calc gray scale gradient
								cv::Mat grad_x, grad_y, grad;
								int scale = 1;
								int delta = 0;
								int ddepth = CV_64F;

								cv::Sobel( contourMap, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT ); //dx
								cv::Sobel( contourMap, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT ); //dy

								//find phase angle
								cv::Mat orientation = cv::Mat::zeros(grad_x.rows, grad_y.cols, CV_64F);
								grad_x.convertTo(grad_x,CV_64F);
								grad_y.convertTo(grad_y,CV_64F);
								cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, grad );

								phase(grad_x, grad_y, orientation, true);
								orientation.convertTo(orientation,CV_32S);

								// transform angle to yaw angle
								for(int i=0; i<orientation.rows; i++) {
																for(int j = 0; j<orientation.cols; j++) {
																								int& current = orientation.at<int>(i,j);
																								// shift CW to CCW angle
																								current = 360 -current;
																								// transform to yaw
																								current = makeYaw(current);
																								// flip by 90 deg CCW
																								current = correctYawAngle(current, 90);
																								// yaw now points in the direction of steepest decline
																}
								}

								//remove useless information
								grad = cv::abs(grad);
								grad.convertTo(grad,CV_8UC1);
								cv::bitwise_not(grad, grad);
								cv::threshold(grad, grad, 254, 255, CV_THRESH_BINARY);
								cv::Mat mask = cv::Mat::zeros(grad.rows, grad.cols, CV_32S);
								cv::bitwise_and(orientation, mask, orientation, grad);

								// transform global point to roi point
								//cv::Point roiPoint(midEdge.x - roi.x, midEdge.y - roi.y);
								cv::Point roiPoint(centroid.x - roi.x, centroid.y - roi.y);
								// extract phase angle information at roiPoint
								yaw = orientation.at<int>(roiPoint);

								//draw for testing
								//cv::rectangle(map, roi, cv::Scalar(255, 0, 0), 1, 1);
}

int Contour::makeYaw(const int angle) const {
								int yaw = angle;
								if(yaw<-360) {
																yaw += 360;
								}else if(yaw>180) {
																yaw -= 360;
								}
								return yaw;
}
int Contour::correctYawAngle(const int theta, const int increment) const {
								int yaw = 0;
								int angle = theta + increment;
								if (angle > 180.0) {
																yaw = -180.0 - (180.0 - angle);
								} else if (angle < -180.0) {
																yaw = 180.0 - (-180.0 - angle);
								} else {
																yaw = angle;
								}
								return yaw;
}
