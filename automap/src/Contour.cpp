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
Contour::Contour(const Contour& other) : vectorOfPoints(other.vectorOfPoints), map(other.map), centroid(other.centroid),
								midEdge(other.midEdge), yaw(other.yaw), length(other.length), score(other.score){
}

void Contour::initContour(){
								calcCentroid();
								calcMidEdge();
								calcDirectionToUnknown();
								shiftCentroid();
}
const edge& Contour::getContour(){
								return vectorOfPoints;
}

void Contour::setScore(const cv::Point& vehicle, const double& yaw, const cv::Rect& roi){
								cv::Point adjustedVehicle(vehicle.x-roi.x, vehicle.y-roi.y);
								double distance = cv::norm(adjustedVehicle-centroid);
								double dYaw = 180-std::fabs(makeYaw(yaw-this->yaw));
								//score = (180-dYaw)*(180-dYaw)* (1/ (1+distance)) *  length;
								//score = (180-dYaw)*(180-dYaw)*  length;
								score =dYaw +  dYaw*(1/ (1+(distance))) + dYaw*(length * 0.05);
}
const double Contour::getScore() const {
								return score;
}

const cv::Point& Contour::getCentroid() const {
								return centroid;
}

const cv::Point Contour::getCentroidGlobal(const cv::Rect& roi) const {
								return cv::Point(centroid.x+roi.x, centroid.y+roi.y);
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

void Contour::shiftCentroid(){
	/*
	int col = map.at<uchar>(centroid);

	double length = 1;
	int c = 0;
	cv::Point P2;

	while(col != 255 && c<30){
		double temp_yaw = correctYawAngle(yaw, 180);
		centroid.x =  (int)round(centroid.x + length * cv::cos((-temp_yaw) * CV_PI / 180.0));
		centroid.y =  (int)round(centroid.y + length * cv::sin((-temp_yaw) * CV_PI / 180.0));
		col = map.at<uchar>(centroid);
		length+=1.0;
		c++;
	}
	*/

	double temp_yaw = correctYawAngle(yaw, 180);
	centroid.x =  (int)round(centroid.x + 0.4/0.05 * cv::cos((-temp_yaw) * CV_PI / 180.0));
	centroid.y =  (int)round(centroid.y + 0.4/0.05 * cv::sin((-temp_yaw) * CV_PI / 180.0));



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
								length = 0;
								for(int i = 0; i+1<vectorOfPoints.size(); i++) {
																length += cv::norm(vectorOfPoints[i]-vectorOfPoints[i+1]);
								}
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
								cv::blur(contourMap, contourMap, cv::Size(3,3));

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

								//remove useless information
								grad = cv::abs(grad);
								grad.convertTo(grad,CV_8UC1);
								cv::bitwise_not(grad, grad);
								cv::threshold(grad, grad, 254, 255, CV_THRESH_BINARY);
								cv::Mat mask = cv::Mat::zeros(grad.rows, grad.cols, CV_32S);
								cv::bitwise_and(orientation, mask, orientation, grad);

								yaw = 0;
								double ts = 0;
								double tc = 0;
								// find average over all angles of the gradient
								for(auto current : vectorOfPoints) {
																// transform global point to roi point
																cv::Point roiPoint(current.x - roi.x, current.y - roi.y);
																int temp_yaw = orientation.at<int>(roiPoint);
																ts += cv::sin(temp_yaw*CV_PI/180);
																tc += cv::cos(temp_yaw*CV_PI/180);
								}
								// transform angle to yaw angle
								yaw = std::atan2(ts,tc)/CV_PI*180;
								yaw = -correctYawAngle(yaw, 180);

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

double Contour::makeYaw(const double angle) const {
								double yaw = angle;
								if(yaw<-360.0) {
																yaw += 360.0;
								}else if(yaw>180.0) {
																yaw -= 360.0;
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
