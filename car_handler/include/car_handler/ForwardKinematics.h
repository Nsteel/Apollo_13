/*
 * ForwardKinematics.h
 *
 *      Authors: Sebastian Ehmes
 *				 Nicolas Acero
 *				 Huynh-Tan Truong
 *				 Li Zhao
 */

#ifndef FORWARDKINEMATICS_H_
#define FORWARDKINEMATICS_H_

#include <utility>
#include <vector>
#include <car_handler/Eigen/Eigen/Dense>
#include <cmath>

class ForwardKinematics {
public:
	ForwardKinematics(double k);
	double flattenZeros(double value);
	std::pair<double, double> calcICC(double alpha);
	double calcRadius(std::pair<double, double> ICC);
	double calcTheta(double distance, double radius);
	Eigen::Matrix4d* calcRot(double theta, double alpha);
	Eigen::Matrix4d* calcRotWithGyro(double theta);
	Eigen::Matrix4d* calcTrans(double radius, double distance, double alpha, std::pair<double, double> ICC);
	Eigen::Matrix4d* calcTransWithGyro(double theta, double dS);
	std::vector<double> getUpdate(double alpha, double distance);
	std::vector<double> getUpdateWithGyro(double theta, double deltaS);

private:
	// k: distance between front axis and back axis
	double k;
	double PI;
	/* The following matrices represent the postion and orientation of
	 * the car as homogenous transformation matrices :
	 */
  // initT: initial position and orientation of the car
	Eigen::Matrix4d initT;
	// T: all positions and orientations of the car for every measurement
	std::vector<Eigen::Matrix4d>T;
	// prevT: previous position and orientation of the car
	Eigen::Matrix4d prevT;
	// vector consists of (x,y,z)translation and yaw-orientation in world coordinates 
	std::vector<double>currentPosition;
};

#endif /* FORWARDKINEMATICS_H_ */
