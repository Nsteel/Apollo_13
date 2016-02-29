/*
 * Speedometer.h
 *
 *      Authors: Sebastian Ehmes
 *				 Nicolas Acero
 *				 Huynh-Tan Truong
 *				 Li Zhao
 */


#include <cmath>
#include <ros/ros.h>
#include <list>
#include <string>
#include <vector>

class Speedometer {

public:
	Speedometer();
	Speedometer(const Speedometer& other);
	Speedometer(double initialTime);
	std::vector<double> updateArduino(double currentDistance,double currentTime);

private:
	//	last time an update was requested
	double lastUpdate;
	//  driven distance since the last update
	double lastDistance;
	//  vector contains: driven distance / unused / momentary velocity / driven distance since last update / time since last update in ms
	std::vector<double> currentValues;
	//  last time velocity was updated
	double lastSpeedUpdate;
	//  driven distance since last velocity update
	double lastSpeedDistance;

};
