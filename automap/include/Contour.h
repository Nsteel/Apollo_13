/*
 * FloatingEdge.h
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#ifndef CONTOUR_H_
#define CONTOUR_H_

#include <vector>
#include <cv.h>

typedef std::vector<cv::Point> edge;

class Contour {
public:
	Contour(const edge& vectorOfPoints);
	Contour(const Contour& other);
	const cv::Point& getCenter() const;

private:
	edge vectorOfPoints;
	cv::Point center;

	void calcCenter();
};

#endif /* CONTOUR_H_ */
