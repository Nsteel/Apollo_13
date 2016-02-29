/*
 * ForwardKinematics.cpp
 *
 *      Authors: Sebastian Ehmes
 *				 Nicolas Acero
 *				 Huynh-Tan Truong
 *				 Li Zhao
 */

#include <car_handler/ForwardKinematics.h>
/*	The standard constructor for ForwardKinematics only requries the distance
 *	between front and back axis als input arguments. That means the starting
 *	postion is always (0 ,0 ,0) and orientation theta = 0.
 */
ForwardKinematics::ForwardKinematics(double k):k(k) {
	PI = std::acos(-1);
	initT = Eigen::Matrix4d::Identity();
	T.push_back(Eigen::Matrix4d::Identity());
	prevT = initT;
	currentPosition=std::vector<double>(3,0);
}
/*	This method calculates the momentaneous center of curvature for a vehicle
 *	with a certain size and ackermann-drive. The ICC is proportional to the
 *	steering angle alpha.
 */
std::pair<double, double> ForwardKinematics::calcICC(double alpha){
	double x = -k/std::tan(alpha);
	double y = -k;
	return std::make_pair(x,y);
}
/*	A utility function to remove values very close to zero, since numerical sine
 *  and cosine functions sometimes do not equal 0 for certain radiant values.
 */
double ForwardKinematics::flattenZeros(double value){
	return (std::fabs(value)<0.000001)?0:value;
}
/*	Since ackermann driven cars always travel on circular lines (except when they
 *  are driving straight forward), this method calculates the radius of those circular
 *  trajectories, with the ICC as center.
*/
double ForwardKinematics::calcRadius(std::pair<double, double> ICC){
	return std::sqrt(ICC.first*ICC.first+ICC.second*ICC.second);
}
/*	This calculates how many radians the car has moved around the ICC with
 *  a certain radius.
 */
double ForwardKinematics::calcTheta(double distance, double radius){
	return distance/radius;
}
/*  A Method that calculates the homogenous transformation
 *  matrix to rotate from S_i to S_0 (S means koordinates / S_0 are world coordinates).
 *  It basically finds a way how the current coordinates need to be rotated to fit
 *  in the global context, based on driven distance and steering values.
 */
Eigen::Matrix4d* ForwardKinematics::calcRot(double theta, double alpha){
	Eigen::Matrix4d* rot = new Eigen::Matrix4d();
	*rot = Eigen::Matrix4d::Identity();

	if(alpha>0){
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
/*	Here we do not need to make assumptions about the cars orientation, with the help
 *  of steering values. In this method only the Gyroscopes measurements are used
 *  to rotate our car correctly.
*/
Eigen::Matrix4d* ForwardKinematics::calcRotWithGyro(double theta){
	Eigen::Matrix4d* rot = new Eigen::Matrix4d();
	*rot = Eigen::Matrix4d::Identity();

		(*rot)(0,0)=flattenZeros(std::cos(theta));
		(*rot)(0,1)=flattenZeros(-std::sin(theta));
		(*rot)(1,0)=flattenZeros(std::sin(theta));
		(*rot)(1,1)=flattenZeros(std::cos(theta));


	return rot;

}
/*  A Method that calculates the homogenous transformation
 *  matrix to translate from S_i to S_0 (S means koordinates / S_0 are world coordinates).
 *  It basically finds a way how the current coordinates need to be shifted to fit
 *  in the global contextm, based on driven distance and steering values.
 */
Eigen::Matrix4d* ForwardKinematics::calcTrans(double radius, double distance, double alpha, std::pair<double, double> ICC){
	Eigen::Matrix4d* trans = new Eigen::Matrix4d();
	*trans = Eigen::Matrix4d::Identity();
	double x;
	double y;

	if(alpha==0){
		x=0;
		y=distance;
	}else{
		double theta = calcTheta(distance, radius);

		if(alpha>0){
			x = radius*std::cos(theta+alpha)+ICC.first;
			y = radius*std::sin(theta+alpha)+ICC.second;
		}else{
			x = radius*std::cos(PI-theta+alpha)+ICC.first;
			y = radius*std::sin(PI-theta+alpha)+ICC.second;
		}
	}
	(*trans)(0,3)=flattenZeros(x);
	(*trans)(1,3)=flattenZeros(y);

	return trans;

}
/*	Here we do not need to make assumptions about the cars position, with the help
 *  of steering values. In this method only the Gyroscopes measurements are used
 *  to move our car correctly, since were driving on circles.
*/
Eigen::Matrix4d* ForwardKinematics::calcTransWithGyro(double theta, double dS){
	Eigen::Matrix4d* trans = new Eigen::Matrix4d();
	*trans = Eigen::Matrix4d::Identity();

	double x = std::sin(theta)*dS;
	double y = std::cos(theta)*dS;

	(*trans)(0,3)=flattenZeros(x);
	(*trans)(1,3)=flattenZeros(y);

	return trans;

}
/* Here all of the functions from above are combined (except the ones that calculate
 * with Gyroscope values). In this Method we take the old position as our new base
 * coordinates and add our new driven path and rotation. Then transformation matrices
 * are used to transform the new coordinates in world coordinates.
 */
std::vector<double> ForwardKinematics::getUpdate(double alpha, double distance){
		Eigen::Matrix4d* Ti = new Eigen::Matrix4d();
		*Ti = Eigen::Matrix4d::Identity();

		if(alpha==0){
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

		Eigen::Matrix4d Tn = prevT*(*Ti);
		delete Ti;

		prevT = Tn;

		double x = Tn(0,3);
		double y = Tn(1,3);
		double e_x1 = Tn(0,0);
		double e_x2 = Tn(1,0);
		double angle = std::atan2(e_x2,e_x1)/(2*PI)*360;

		currentPosition[0]=flattenZeros(x);
		currentPosition[1]=flattenZeros(y);
		currentPosition[2]=angle;

		return currentPosition;
	}
	/* Here all of the functions from above are combined (except the ones that don't calculate
	 * with Gyroscope values). In this Method we take the old position as our new base
	 * coordinates and add our new driven path and rotation. Then transformation matrices
	 * are used to transform the new coordinates in world coordinates.
	 */
	std::vector<double> ForwardKinematics::getUpdateWithGyro(double theta, double deltaS){
			Eigen::Matrix4d* Ti = new Eigen::Matrix4d();
			*Ti = Eigen::Matrix4d::Identity();

			double deltaTheta = theta - currentPosition[2]*PI/180;

			Ti = calcRotWithGyro(deltaTheta);
			Eigen::Matrix4d* trans = calcTransWithGyro(deltaTheta, deltaS);
			*Ti = (*trans)*(*Ti);

			Eigen::Matrix4d Tn = prevT*(*Ti);
			delete Ti;

			prevT = Tn;

			double x = Tn(0,3);
			double y = Tn(1,3);
			double e_x1 = Tn(0,0);
			double e_x2 = Tn(1,0);
			double angle = std::atan2(e_x2,e_x1)/(2*PI)*360;

			currentPosition[0]=flattenZeros(x);
			currentPosition[1]=flattenZeros(y);
			currentPosition[2]=angle;

			return currentPosition;
		}
