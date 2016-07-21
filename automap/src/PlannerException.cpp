/*
 * PlannerException.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: sebastian
 */

#include "PlannerException.h"

PlannerException::PlannerException(std::string& msg):msg(msg) {

}
const char* PlannerException::what() const throw(){
	return msg.c_str();
}
