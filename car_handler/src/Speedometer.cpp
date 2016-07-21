/*
 * Speedometer.cpp
 *
 *      Authors: Sebastian Ehmes
 *         Nicolas Acero
 *         Huynh-Tan Truong
 *         Li Zhao
 */

#include <car_handler/Speedometer.h>
// Standard constructor
Speedometer::Speedometer(){};
// Copy constructor
Speedometer::Speedometer(const Speedometer & other):
lastUpdate(other.lastUpdate), lastDistance(other.lastDistance), currentValues(other.currentValues),
lastSpeedUpdate(other.lastSpeedUpdate), lastSpeedDistance(other.lastSpeedDistance){};
/* Since we use the Arduino, only an initial time is required, the initial distance is always zero when
 * the Arduino is reset.
 */
Speedometer::Speedometer(double initialTime):
	lastUpdate(initialTime){
	lastDistance=0;
	currentValues = std::vector<double>(5);
};
/* With the absolute driven Distance being supplied by the Arduino, this method can calculate the driven
 * distances between update intervals and the velocity of the car.
 */
std::vector<double> Speedometer::updateArduino(double currentDistance, double currentTime){
	double dT = currentTime-lastUpdate;
  double dTs = currentTime-lastSpeedUpdate;
  double dS = (currentDistance - lastDistance);

  currentValues[0]=currentDistance;
  currentValues[3]=dS;
  currentValues[4]=dT;
  lastDistance=currentDistance;
  lastUpdate=currentTime;

  // The time inervals are limited to a minimum of 300ms for speed updates, otherwise most of the speed values
  // might be zero.
  if(dTs>=0.3){
    double dSs = (currentDistance - lastSpeedDistance);
    double dV = dSs/dTs;
    currentValues[2]=dV;
    lastSpeedDistance = currentDistance;
    lastSpeedUpdate = currentTime;
  }


	return currentValues;
}
