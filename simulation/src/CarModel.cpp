/*
 * CarModel.cpp
 *
 *  Created on: 11.05.2016
 *      Author: Basti
 */

#include "CarModel.h"

CarModel::CarModel(const double dAxis, const ros::Time& time) : lastUpdate(time) {
								// init kinematic model
								fwdKin = ForwardKinematics(dAxis);
								// set steering to quasi neutral
								setSteering(8);
								// start with stationary state
								velocity = 0;
								distance = 0;
								timeStep = 0;
								pose=std::vector<double>(3,0);
								angularVelocity = 0;

}

const pose_ptr CarModel::getUpdate(const int newSteering, const int newSpeed, const ros::Time& time){
								timeStep = (time-lastUpdate).toNSec()/NSEC_PER_SEC;
								//ROS_INFO("timestep: %lf", timeStep);
								lastUpdate = time;
								double ds = velocity * timeStep;
								//double ds = speedToVelocity(newSpeed) * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
								speedToVelocity(newSpeed);
								setAngularVelocity(oldYaw, pose[2]);
								setSteering(newSteering);
								return std::make_shared<std::vector<double> >(pose);
}

const pose_ptr CarModel::getUpdateTwist(const twist_msg cmdVel, const ros::Time& time){
								timeStep = (time-lastUpdate).toNSec()/NSEC_PER_SEC;
								lastUpdate =time;
								double ds = velocity * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
								velocity = cmdVel.linear.x;
								setAngularVelocity(oldYaw, pose[2]);
								angleToSteering(fwdKin.radToDeg(cmdVel.angular.z));
								//steeringAngle = fwdKin.radToDeg(cmdVel.angular.z);
								//ROS_INFO("x: %lf / y: %lf / th: %lf / cmdVelx: %lf/ cmdAngz: %lf", pose[0], pose[1], pose[2], cmdVel.linear.x, cmdVel.angular.z);
								return std::make_shared<std::vector<double> >(pose);
}

const double CarModel::getDistance() const {
								return distance;
}

const double CarModel::getVelocity() const {
								return velocity;
}

const int CarModel::getSteering() const {
								return steering;
}
const double CarModel::getSteeringAngle() const {
								return steeringAngle;
}

const double CarModel::getAngularVelocity() const {
								return angularVelocity;
}

void CarModel::angleToSteering(const double alpha){
								if(alpha>=angleArray[0]) {
																setSteering(-50);
																return;
								}else if(alpha<=angleArray[100]) {
																setSteering(50);
																return;
								}else{
																for(int i = 1; i <= 100; i++) {
																								if(alpha < angleArray[i-1] && alpha >= angleArray[i]) {
																																setSteering(i-50);
																																return;
																								}
																}
								}
}

void CarModel::speedToVelocity(const int speed) {
								if(speed<=-10) {
																velocity = velocityArray[0];
								}else if(speed>=10) {
																velocity = velocityArray[20];
								}else{
																velocity = velocityArray[10+speed];
								}
}



void CarModel::setAngularVelocity(const double yaw0, const double yaw1){
								double dTh = yaw1-yaw0;
								if(dTh<-fwdKin.PI) {
																dTh += 2*fwdKin.PI;
								}else if(dTh>fwdKin.PI) {
																dTh -= 2*fwdKin.PI;
								}
								angularVelocity = dTh/timeStep;
}

void CarModel::setSteering(const int steering){
								if(steering>50) {
																this->steering = 50;
																steeringToAngle();
								}else if(steering < -50) {
																this->steering = -50;
																steeringToAngle();
								}else{
																this->steering = steering;
																steeringToAngle();
								}
}


void CarModel::steeringToAngle(){
								steeringAngle = angleArray[steering+50];
}
