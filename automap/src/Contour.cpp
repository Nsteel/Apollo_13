/*
 * FloatingEdge.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#include "Contour.h"

Contour::Contour(const edge& vectorOfPoints):vectorOfPoints(vectorOfPoints) {
	calcCenter();
}
Contour::Contour(const Contour& other):vectorOfPoints(other.vectorOfPoints), center(other.center){}

const cv::Point& Contour::getCenter() const{
	return center;
}

void Contour::calcCenter(){
	cv::Point result;
	double x_mean = 0;
	double y_mean = 0;
	for(auto current : vectorOfPoints){
		x_mean += (double)current.x;
		y_mean += (double)current.y;
	}
	x_mean /= vectorOfPoints.size();
	y_mean /= vectorOfPoints.size();
	result.x= (int)x_mean;
	result.y= (int)y_mean;
	center = result;
}

