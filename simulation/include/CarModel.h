/*
 * CarModel.h
 *
 *  Created on: 11.05.2016
 *      Author: Basti
 */

#ifndef CARMODEL_H_
#define CARMODEL_H_

#include "ForwardKinematics.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <memory.h>
#include <ros/ros.h>

typedef geometry_msgs::Twist twist_msg;
typedef std::shared_ptr<std::vector<double> > pose_ptr;

static const double NSEC_PER_SEC = 1000000000.0;

static const double angleArray[101]={25.0, 24.6, 24.2, 23.8, 23.4, 23.0, 22.6, 22.2, 21.8, 21.4,
																																					21.0, 20.6, 20.2, 19.8, 19.4, 19.0, 18.6, 18.2, 17.8, 17.4,
																																					17.0, 16.5, 16.0, 15.5, 15.0, 14.5, 14.0, 13.5, 13.0, 12.5,
																																					12.0, 11.7, 11.4, 11.1, 10.8, 10.5, 10.2, 9.9, 9.6, 9.3,
																																					9.0, 8.7, 8.4, 8.1, 7.8, 7.5, 7.2, 6.9, 6.6, 6.3,
																																					6.0, 5.2, 4.4, 3.6, 2.8, 2.0, 1.2, 0.0, -0.0, -1.2,
																																					-2.0, -2.3, -2.6, -2.9, -3.2, -3.5, -3.8, -4.1, -4.4, -4.7,
																																					-5.0, -5.4, -5.8, -6.2, -6.6, -7.0, -7.4, -7.8, -8.2, -8.6,
																																					-9.0, -9.4, -9.8, -10.2, -10.6, -11.0, -11.4, -11.8, -12.2, -12.6,
																																					-13.0, -13.5, -14.0, -14.5, -15.0, -15.5, -16.0, -16.5, -17.0, -17.5, -18.0};
static const double velocityArray[21]={-2.0, -1.8, -1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0,
																																							0.2,  0.4,  0.6,  0.8,  1.0,  1.2,  1.4,  1.6,  1.8,  2.0};

class CarModel {
public:
								CarModel(const double dAxis, const ros::Time& time);

								const pose_ptr getUpdate(const int newSteering, const int newSpeed, const ros::Time& time);
								const pose_ptr getUpdateTwist(const twist_msg cmd_vel, const ros::Time& time);

								const int getSteering() const;
								const double getSteeringAngle() const;
								const double getVelocity() const;
								const double getAngularVelocity() const;
								const double getDistance() const;
private:
								ForwardKinematics fwdKin;
								ros::Time lastUpdate;
								int steering;
								double steeringAngle;
								double timeStep;
								double velocity;
								double distance;
								std::vector<double> pose;
								double angularVelocity;


								void steeringToAngle();
								void angleToSteering(const double alpha);
								void setSteering(const int steering);
								void speedToVelocity(const int speed);
								void setAngularVelocity(const double yaw0, const double yaw1);
};

#endif /* CARMODEL_H_ */
