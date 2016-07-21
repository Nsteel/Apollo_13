/*
 * PlannerException.h
 *
 *  Created on: Jul 13, 2016
 *      Author: sebastian
 */

#ifndef PLANNEREXCEPTION_H_
#define PLANNEREXCEPTION_H_

#include <exception>
#include <string>

class PlannerException: public std::exception {
public:
	PlannerException(std::string& msg);
	const char* what() const throw();
private:
	std::string msg;
};

#endif /* PLANNEREXCEPTION_H_ */
