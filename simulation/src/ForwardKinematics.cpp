/*
 * ForwardKinematics.cpp
 *
 *  Created on: 24.11.2015
 *      Author: Basti
 */

#include "ForwardKinematics.h"

ForwardKinematics::ForwardKinematics() : k(0.0){
								init();
}
ForwardKinematics::ForwardKinematics(const ForwardKinematics& other) : k(other.k){
								PI = other.PI;
								initT = other.initT;
								T = other.T;
								prevT = other.prevT;
								currentPosition = other.currentPosition;
}

ForwardKinematics::ForwardKinematics(double k) : k(k) {
								init();
}

void ForwardKinematics::init(){
								PI = std::acos(-1);
								initT = Eigen::Matrix4d::Identity();
								T.push_back(Eigen::Matrix4d::Identity());
								prevT = initT;
								currentPosition=std::vector<double>(3,0);
}
std::pair<double, double> ForwardKinematics::calcICC(double alpha){
								double x = -k/std::tan(alpha);
								double y = -k;
								return std::make_pair(x,y);
}
double ForwardKinematics::flattenZeros(double value){
								return (std::fabs(value)<0.000001) ? 0 : value;
}
double ForwardKinematics::degToRad(double angle){
								return angle/180.0*PI;
}
double ForwardKinematics::radToDeg(double angle){
								return angle/PI*180.0;
}

double ForwardKinematics::calcRadius(std::pair<double, double> ICC){
								return std::sqrt(ICC.first*ICC.first+ICC.second*ICC.second);
}
double ForwardKinematics::calcTheta(double distance, double radius){
								return distance/radius;
}
Eigen::Matrix4d* ForwardKinematics::calcRot(double theta, double alpha){
								Eigen::Matrix4d* rot = new Eigen::Matrix4d();
								*rot = Eigen::Matrix4d::Identity();

								if(alpha>0) {
																(*rot)(0,0)=flattenZeros(std::cos(theta));
																(*rot)(0,1)=flattenZeros(-std::sin(theta));
																(*rot)(1,0)=flattenZeros(std::sin(theta));
																(*rot)(1,1)=flattenZeros(std::cos(theta));
								}else{
																(*rot)(0,0)=flattenZeros(std::cos(-theta));
																(*rot)(0,1)=flattenZeros(-std::sin(-theta));
																(*rot)(1,0)=flattenZeros(std::sin(-theta));
																(*rot)(1,1)=flattenZeros(std::cos(-theta));
								}

								return rot;

}
Eigen::Matrix4d* ForwardKinematics::calcTrans(double radius, double distance, double alpha, std::pair<double, double> ICC){
								Eigen::Matrix4d* trans = new Eigen::Matrix4d();
								*trans = Eigen::Matrix4d::Identity();
								double x;
								double y;

								if(alpha==0) {
																x=0;
																y=distance;
								}else{
																double theta = calcTheta(distance, radius);

																if(alpha>0) {
																								//Anmerkung: vorher stand alpha+theta
																								x = radius*std::cos(theta+alpha)+ICC.first;
																								y = radius*std::sin(theta+alpha)+ICC.second;
																}else{
																								//Anmerkung: vorher stand PI+alpha-theta
																								x = radius*std::cos(PI-theta+alpha)+ICC.first;
																								y = radius*std::sin(PI-theta+alpha)+ICC.second;
																}
								}


								//(*trans)(0,3)= (std::fabs(x)<0.000001)?0:x;
								//(*trans)(1,3)= (std::fabs(y)<0.000001)?0:y;
								(*trans)(0,3)=flattenZeros(x);
								(*trans)(1,3)=flattenZeros(y);

								return trans;

}
std::vector<double>& ForwardKinematics::getUpdate(double alpha, double distance){
								Eigen::Matrix4d* Ti = new Eigen::Matrix4d();
								*Ti = Eigen::Matrix4d::Identity();
								alpha = flattenZeros(alpha);

								if(alpha==0) {
																Ti= calcTrans(0, distance, alpha, std::make_pair(0,0));
								}else{
																std::pair<double, double> ICC = calcICC(alpha);
																double radius = calcRadius(ICC);
																double theta = calcTheta(distance, radius);

																Ti = calcRot(theta, alpha);
																Eigen::Matrix4d* trans = calcTrans(radius, distance, alpha, ICC);
																*Ti = (*trans)*(*Ti);
																delete trans;
								}

								//Eigen::Matrix4d Tn = T.back()*Ti;
								//T.push_back(Tn);

								Eigen::Matrix4d Tn = prevT*(*Ti);
								delete Ti;

								prevT = Tn;

								double x = Tn(0,3);
								double y = Tn(1,3);
								double e_x1 = Tn(0,0);
								double e_x2 = Tn(1,0);
								// Angle in Degrees
								// double angle = std::atan2(e_x2,e_x1)/(2*PI)*360
								// Angle in rad
								double angle = std::atan2(e_x2,e_x1);

								currentPosition[0]=flattenZeros(x);
								currentPosition[1]=flattenZeros(y);
								currentPosition[2]=angle;

								return currentPosition;
}
