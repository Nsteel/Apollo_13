/*
 * ForwardKinematics.h
 *
 *  Created on: 24.11.2015
 *      Author: Basti
 */

#ifndef FORWARDKINEMATICS_H_
#define FORWARDKINEMATICS_H_

#include <utility>
#include <vector>
#include "Eigen/Eigen/Dense"
#include "Eigen/Eigen/LU"
#include <cmath>

class ForwardKinematics {
public:
								ForwardKinematics();
								ForwardKinematics(const ForwardKinematics &other);
								ForwardKinematics(double k);
								double flattenZeros(double value);
								double degToRad(double angle);
								double radToDeg(double angle);
								std::pair<double, double> calcICC(double alpha);
								double calcRadius(std::pair<double, double> ICC);
								double calcTheta(double distance, double radius);
								Eigen::Matrix4d* calcRot(double theta, double alpha);
								Eigen::Matrix4d* calcTrans(double radius, double distance, double alpha, std::pair<double, double> ICC);
								std::vector<double>& getUpdate(double alpha, double distance);
								double PI;

private:
								double k;

								Eigen::Matrix4d initT;
								std::vector<Eigen::Matrix4d>T;
								Eigen::Matrix4d prevT;
								std::vector<double>currentPosition;
								void init();
};

#endif /* FORWARDKINEMATICS_H_ */
